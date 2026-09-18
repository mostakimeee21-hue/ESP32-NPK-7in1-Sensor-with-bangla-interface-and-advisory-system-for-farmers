/*
 * ==========================================================================
 *  মাটি পরীক্ষা ও কৃষক পরামর্শ ব্যবস্থা  (ESP32)
 *  7-in-1 Soil Sensor (Modbus RTU) + SSD1306 OLED (সম্পূর্ণ বাংলা) + SIM800L SMS
 * ==========================================================================
 *
 *  কাজ:
 *    ১. RS485 সেন্সর থেকে আর্দ্রতা, তাপমাত্রা, পিএইচ, ইসি, N, P, K পড়ে
 *    ২. OLED এ সব লেখা বাংলা বিটম্যাপে দেখায় (BanglaBitmap.h)
 *    ৩. প্রতিটি উপাদানের থ্রেশহোল্ডের সাথে মিলিয়ে পরামর্শ তৈরি করে
 *    ৪. SIM800L দিয়ে কৃষকের ফোনে বাংলা SMS (UCS2/ইউনিকোড) পাঠায়
 *
 *  ফাইল:
 *    Soil_NPK_GSM_Bangla.ino   <- এই ফাইল
 *    BanglaBitmap.h            <- বাংলা বিটম্যাপ লাইব্রেরি (একই ফোল্ডারে রাখুন)
 *
 *  তারের সংযোগ (সেন্সরের পিন আগের মতোই আছে):
 *        সেন্সর (RS485)  ->  Serial2 : RX = GPIO 16 , TX = GPIO 17   <- অপরিবর্তিত
 *        SIM800L         ->  Serial1 : RX = GPIO 26 , TX = GPIO 27
 *    দুটি আলাদা UART লাগে বলে GSM মডিউলটি ২৬/২৭ এ বসবে। মডিউল এখনো না লাগালেও
 *    কোড চলবে — setup এ AT দিয়ে মডিউল খোঁজা হয়, না পেলে পরামর্শ শুধু
 *    সিরিয়াল মনিটরে ছাপা হবে, OLED স্বাভাবিকভাবেই চলতে থাকবে।
 *    পরে SIM800L লাগালে আলাদা ২A ক্ষমতার ৪.০V সাপ্লাই দিন, ESP32 এর 3.3V নয়।
 * ==========================================================================
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "BanglaBitmap.h"

/* ======================= ১. পিন ও হার্ডওয়্যার সেটিং ======================= */

#define SOIL_RX_PIN   16        // RS485 মডিউলের RO -> ESP32 (আগের মতোই)
#define SOIL_TX_PIN   17        // RS485 মডিউলের DI <- ESP32 (আগের মতোই)
#define SOIL_DE_PIN   -1        // DE/RE পিন (অটো-ডিরেকশন মডিউল হলে -1 রাখুন)
#define SOIL_BAUD     4800

#define SIM_RX_PIN    26        // SIM800L এর TX -> ESP32
#define SIM_TX_PIN    27        // SIM800L এর RX <- ESP32 (ভোল্টেজ ডিভাইডার দিন)
#define SIM_BAUD      9600

#define BUTTON_PIN    25        // ম্যানুয়ালি SMS পাঠানোর বোতাম (GND এর সাথে), না থাকলে -1

HardwareSerial soilSerial(2);   // সেন্সর — GPIO 16/17
HardwareSerial sim800(1);       // SIM800L — GPIO 26/27

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

/* ======================= ২. কৃষকের নম্বর ও সময় ======================= */

/*  যে নম্বরে SMS যাবে (কৃষকের ফোন)। SIM800L এ যে সিম ঢোকাবেন সেটি হবে
 *  পাঠানোর সিম — তার নম্বর কোডে লিখতে হয় না, শুধু সিমে SMS ব্যালান্স
 *  থাকতে হবে এবং PIN লক বন্ধ থাকতে হবে।                                */
const char* FARMER_PHONE = "+8801777368728";

#define ADVISORY_INTERVAL_MS   (6UL * 60UL * 60UL * 1000UL)  // ৬ ঘণ্টা পরপর
#define SAMPLES_BEFORE_ADVICE  5      // গড় করার জন্য কয়টি পাঠ নেবে
#define SMS_MAX_CHARS          65     // ইউনিকোড SMS এ ৭০ অক্ষরের সীমা

/* ======================= ৩. থ্রেশহোল্ড (এখানে বদলান) ======================= */
/*  মান বাংলাদেশের সাধারণ মাটির জন্য (BARC সার সুপারিশ নির্দেশিকা ঘেঁষা)।
 *  ফসল ও অঞ্চলভেদে কৃষি কর্মকর্তার পরামর্শে এগুলো ঠিক করে নিন।            */

#define TH_N_LOW        60      // মিগ্রা/কেজি — এর নিচে হলে ইউরিয়া
#define TH_N_HIGH      150

#define TH_P_LOW        12      // মিগ্রা/কেজি — এর নিচে হলে টিএসপি
#define TH_P_HIGH       40

#define TH_K_LOW        60      // মিগ্রা/কেজি — এর নিচে হলে এমওপি
#define TH_K_HIGH      200

#define TH_PH_ACID      5.5     // এর নিচে অম্ল মাটি -> ডলোচুন
#define TH_PH_ALKALI    8.0     // এর উপরে ক্ষারীয় মাটি -> জিপসাম

#define TH_MOIST_LOW    20.0    // % — এর নিচে সেচ দিতে হবে
#define TH_MOIST_HIGH   80.0    // % — এর উপরে জমে থাকা পানি সরান

#define TH_EC_HIGH      2000    // uS/cm — লবণাক্ততার ঝুঁকি
#define TH_TEMP_LOW     15.0    // ডিগ্রি সে.
#define TH_TEMP_HIGH    25.0

/* ======================= ৪. সেন্সর ভ্যারিয়েবল ======================= */

const byte soil7in1_query[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
byte response[19];

float moisture = 0.0, temperature = 0.0, ph = 0.0;
int   ec = 0, nitrogen = 0, phosphorus = 0, potassium = 0;
bool  sensorOk = false;

// গড় করার জন্য জমা রাখা
float accM = 0, accT = 0, accPh = 0;
long  accEc = 0, accN = 0, accP = 0, accK = 0;
int   sampleCount = 0;

unsigned long lastPageChange = 0;
unsigned long lastAdvisory   = 0;
int  currentPage = 0;
bool gsmReady = false;         // SIM800L সাড়া দিচ্ছে কি না
int  smsStatus = 0;            // 0 = কিছু না, 1 = পাঠাচ্ছে, 2 = গেছে, 3 = যায়নি
bool firstAdviceSent = false;

/* ======================= ৫. Modbus RTU CRC ও সেন্সর পাঠ ======================= */

uint16_t modbusCRC(const byte *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1) { crc >>= 1; crc ^= 0xA001; }
      else         { crc >>= 1; }
    }
  }
  return crc;
}

void readSensor() {
  while (soilSerial.available()) soilSerial.read();

  if (SOIL_DE_PIN >= 0) digitalWrite(SOIL_DE_PIN, HIGH);   // পাঠানোর মোড
  soilSerial.write(soil7in1_query, sizeof(soil7in1_query));
  soilSerial.flush();
  if (SOIL_DE_PIN >= 0) digitalWrite(SOIL_DE_PIN, LOW);    // শোনার মোড

  unsigned long t0 = millis();
  int index = 0;
  while (millis() - t0 < 400 && index < (int)sizeof(response)) {
    if (soilSerial.available()) response[index++] = soilSerial.read();
  }

  sensorOk = false;
  if (index >= 19 && response[0] == 0x01 && response[1] == 0x03) {
    // CRC যাচাই — ভুল ডেটার ভিত্তিতে যেন সার সুপারিশ না যায়
    uint16_t crcCalc = modbusCRC(response, 17);
    uint16_t crcRecv = response[17] | (response[18] << 8);
    if (crcCalc != crcRecv) return;

    moisture    = ((response[3]  << 8) | response[4])  * 0.1;
    temperature = ((response[5]  << 8) | response[6])  * 0.1;
    ec          =  (response[7]  << 8) | response[8];
    ph          = ((response[9]  << 8) | response[10]) * 0.1;
    nitrogen    =  (response[11] << 8) | response[12];
    phosphorus  =  (response[13] << 8) | response[14];
    potassium   =  (response[15] << 8) | response[16];

    if (temperature > 100.0) {                       // ঋণাত্মক তাপমাত্রা
      temperature = (((response[5] << 8) | response[6]) - 65536) * 0.1;
    }
    sensorOk = true;

    // গড়ের জন্য জমা
    accM += moisture; accT += temperature; accPh += ph;
    accEc += ec; accN += nitrogen; accP += phosphorus; accK += potassium;
    if (sampleCount < 1000) sampleCount++;
  }
}

/* ======================= ৬. বাংলা লেখা <-> SMS হেলপার ======================= */

/* ইংরেজি অঙ্ককে বাংলা অঙ্কে বদলানো: 62 -> ৬২ */
String toBanglaDigits(long value) {
  const char* bn[10] = {"০","১","২","৩","৪","৫","৬","৭","৮","৯"};
  String s = String(value);
  String out = "";
  for (unsigned int i = 0; i < s.length(); i++) {
    if (s[i] >= '0' && s[i] <= '9') out += bn[s[i] - '0'];
    else if (s[i] == '-') out += "-";
  }
  return out;
}

String toBanglaDigits(float value, uint8_t decimals) {
  char buf[16];
  dtostrf(value, 0, decimals, buf);
  const char* bn[10] = {"০","১","২","৩","৪","৫","৬","৭","৮","৯"};
  String out = "";
  for (char *p = buf; *p; p++) {
    if (*p >= '0' && *p <= '9') out += bn[*p - '0'];
    else if (*p == '.')         out += "."; 
    else if (*p == '-')         out += "-";
  }
  return out;
}

/* UTF-8 লেখা -> UCS2 (UTF-16BE) হেক্স স্ট্রিং — SIM800L এ বাংলা পাঠানোর জন্য */
String utf8ToUcs2Hex(const String &s) {
  String hex = "";
  const char *p = s.c_str();
  const char *hexChars = "0123456789ABCDEF";
  while (*p) {
    uint32_t cp = 0;
    uint8_t c = (uint8_t)*p;
    if (c < 0x80)              { cp = c;          p += 1; }
    else if ((c & 0xE0) == 0xC0) { cp = ((c & 0x1F) << 6) | (p[1] & 0x3F); p += 2; }
    else if ((c & 0xF0) == 0xE0) { cp = ((c & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F); p += 3; }
    else                        { cp = '?';       p += 4; }   // BMP এর বাইরে (ইমোজি) বাদ
    if (cp > 0xFFFF) cp = '?';
    hex += hexChars[(cp >> 12) & 0xF];
    hex += hexChars[(cp >>  8) & 0xF];
    hex += hexChars[(cp >>  4) & 0xF];
    hex += hexChars[ cp        & 0xF];
  }
  return hex;
}

/* AT কমান্ডের উত্তরের জন্য অপেক্ষা */
bool waitFor(const char *token, unsigned long timeout) {
  unsigned long t0 = millis();
  String buf = "";
  while (millis() - t0 < timeout) {
    while (sim800.available()) {
      buf += (char)sim800.read();
      if (buf.indexOf(token) != -1) return true;
      if (buf.indexOf("ERROR") != -1) return false;
      if (buf.length() > 300) buf = buf.substring(buf.length() - 100);
    }
    delay(10);
  }
  return false;
}

void sendCommand(const char* cmd, int waitMs = 500) {
  sim800.println(cmd);
  delay(waitMs);
  while (sim800.available()) sim800.read();
}

/* SIM800L আসলে লাগানো আছে কি না দেখা */
bool detectGsm() {
  for (int i = 0; i < 3; i++) {
    while (sim800.available()) sim800.read();
    sim800.println("AT");
    if (waitFor("OK", 1500)) return true;
  }
  return false;
}

/* একটি বাংলা SMS পাঠানো (সর্বোচ্চ ৭০ অক্ষর) */
bool sendOneBanglaSMS(const String &phone, const String &text) {
  if (!gsmReady) {                       // মডিউল নেই — শুধু সিরিয়ালে দেখানো
    Serial.println("[GSM নেই] পাঠানো হতো: " + text);
    return false;
  }
  sim800.println("AT+CMGF=1");            delay(200);
  sim800.println("AT+CSCS=\"UCS2\"");     delay(200);
  sim800.println("AT+CSMP=17,167,0,8");   delay(200);  // ইউনিকোড ডেটা কোডিং
  while (sim800.available()) sim800.read();

  // UCS2 মোডে নম্বরটাও হেক্স করে পাঠাতে হয়
  sim800.print("AT+CMGS=\"");
  sim800.print(utf8ToUcs2Hex(phone));
  sim800.println("\"");

  if (!waitFor(">", 8000)) {
    Serial.println("[SMS] প্রম্পট আসেনি");
    sim800.write(27);                      // ESC — কমান্ড বাতিল
    return false;
  }

  sim800.print(utf8ToUcs2Hex(text));
  sim800.write(26);                        // Ctrl+Z

  bool ok = waitFor("+CMGS", 20000);
  Serial.println(ok ? "[SMS] পাঠানো হয়েছে" : "[SMS] ব্যর্থ");
  return ok;
}

/* লম্বা লেখা হলে ভেঙে কয়েকটি SMS — কোডপয়েন্ট ধরে, শব্দের ফাঁকে ভাঙা হয় */
bool sendBanglaSMS(const String &phone, const String &text) {
  // কোডপয়েন্টের তালিকা বানানো
  int total = 0;
  for (unsigned int i = 0; i < text.length(); ) {
    uint8_t c = (uint8_t)text[i];
    i += (c < 0x80) ? 1 : ((c & 0xE0) == 0xC0 ? 2 : ((c & 0xF0) == 0xE0 ? 3 : 4));
    total++;
  }
  if (total <= SMS_MAX_CHARS) return sendOneBanglaSMS(phone, text);

  bool allOk = true;
  unsigned int pos = 0;
  int part = 1;
  while (pos < text.length()) {
    unsigned int i = pos;
    int count = 0, lastSpace = -1;
    while (i < text.length() && count < SMS_MAX_CHARS - 6) {   // "(১/২) " এর জায়গা
      uint8_t c = (uint8_t)text[i];
      int len = (c < 0x80) ? 1 : ((c & 0xE0) == 0xC0 ? 2 : ((c & 0xF0) == 0xE0 ? 3 : 4));
      if (c == ' ') lastSpace = i;
      i += len; count++;
    }
    unsigned int cut = (i < text.length() && lastSpace > (int)pos) ? lastSpace : i;
    String chunk = "(" + toBanglaDigits((long)part) + ") " + text.substring(pos, cut);
    if (!sendOneBanglaSMS(phone, chunk)) allOk = false;
    pos = cut;
    while (pos < text.length() && text[pos] == ' ') pos++;
    part++;
    delay(4000);                            // পরপর SMS এর মাঝে বিরতি
  }
  return allOk;
}

/* ======================= ৭. পরামর্শ তৈরি (থ্রেশহোল্ড লজিক) ======================= */

#define MAX_ADVICE 8
String adviceList[MAX_ADVICE];
int adviceCount = 0;

void addAdvice(const String &msg) {
  if (adviceCount < MAX_ADVICE) adviceList[adviceCount++] = msg;
}

void buildAdvisory(float m, float t, float p, int e, int n, int ph_, int k) {
  adviceCount = 0;

  /* --- নাইট্রোজেন --- */
  if (n < TH_N_LOW) {
    addAdvice("নাইট্রোজেন কম (" + toBanglaDigits((long)n) +
              " মিগ্রা/কেজি)। বিঘাপ্রতি ১২-১৪ কেজি ইউরিয়া উপরি প্রয়োগ করুন।");
  } else if (n > TH_N_HIGH) {
    addAdvice("নাইট্রোজেন বেশি (" + toBanglaDigits((long)n) +
              ")। এবার ইউরিয়া দেবেন না, গাছ নেতিয়ে পড়তে পারে।");
  }

  /* --- ফসফরাস --- */
  if (ph_ < TH_P_LOW) {
    addAdvice("ফসফরাস কম (" + toBanglaDigits((long)ph_) +
              " মিগ্রা/কেজি)। জমি তৈরির সময় বিঘাপ্রতি ৮-১০ কেজি টিএসপি দিন।");
  } else if (ph_ > TH_P_HIGH) {
    addAdvice("ফসফরাস যথেষ্ট আছে, এবার টিএসপি কমিয়ে দিন।");
  }

  /* --- পটাশিয়াম --- */
  if (k < TH_K_LOW) {
    addAdvice("পটাশিয়াম কম (" + toBanglaDigits((long)k) +
              " মিগ্রা/কেজি)। বিঘাপ্রতি ৬-৮ কেজি এমওপি সার প্রয়োগ করুন।");
  } else if (k > TH_K_HIGH) {
    addAdvice("পটাশিয়াম বেশি আছে, এমওপি এবার দেওয়ার দরকার নেই।");
  }

  /* --- পিএইচ --- */
  if (p < TH_PH_ACID) {
    addAdvice("মাটি অম্ল, পিএইচ " + toBanglaDigits(p, 1) +
              "। বিঘাপ্রতি ৩০-৪০ কেজি ডলোচুন ছিটিয়ে চাষ দিন।");
  } else if (p > TH_PH_ALKALI) {
    addAdvice("মাটি ক্ষারীয়, পিএইচ " + toBanglaDigits(p, 1) +
              "। জিপসাম ও জৈব সার (গোবর/কম্পোস্ট) প্রয়োগ করুন।");
  }

  /* --- আর্দ্রতা --- */
  if (m < TH_MOIST_LOW) {
    addAdvice("মাটিতে রস কম (" + toBanglaDigits(m, 1) +
              " শতাংশ)। আজই জমিতে সেচ দিন।");
  } else if (m > TH_MOIST_HIGH) {
    addAdvice("জমিতে পানি বেশি (" + toBanglaDigits(m, 1) +
              " শতাংশ)। নালা কেটে অতিরিক্ত পানি বের করে দিন।");
  }

  /* --- ইসি / লবণাক্ততা --- */
  if (e > TH_EC_HIGH) {
    addAdvice("মাটিতে লবণ বেশি (ইসি " + toBanglaDigits((long)e) +
              ")। মিঠা পানি দিয়ে জমি ধুয়ে দিন, জৈব সার বাড়ান।");
  }

  /* --- তাপমাত্রা --- */
  if (t < TH_TEMP_LOW) {
    addAdvice("মাটির তাপমাত্রা কম (" + toBanglaDigits(t, 1) +
              " ডিগ্রি)। এখন সার দিলে কাজ কম হবে, রোদ ওঠা পর্যন্ত অপেক্ষা করুন।");
  } else if (t > TH_TEMP_HIGH) {
    addAdvice("মাটি গরম (" + toBanglaDigits(t, 1) +
              " ডিগ্রি)। বিকেলে সেচ দিন, খড় বা কচুরিপানা দিয়ে মালচিং করুন।");
  }

  if (adviceCount == 0) {
    addAdvice("আপনার জমির মাটি এখন ভালো আছে, বাড়তি সার লাগবে না।");
  }
}

/* পাঠানোর জন্য পুরো বার্তা: প্রথমে মান, তারপর পরামর্শ */
String buildSummary(float m, float t, float p, int e, int n, int ph_, int k) {
  String s = "মাটি পরীক্ষার ফল:\n";
  s += "নাইট্রোজেন " + toBanglaDigits((long)n) + ", ফসফরাস " + toBanglaDigits((long)ph_) +
       ", পটাশ " + toBanglaDigits((long)k) + " মিগ্রা/কেজি\n";
  s += "পিএইচ " + toBanglaDigits(p, 1) + ", রস " + toBanglaDigits(m, 1) + "%";
  return s;
}

void sendAdvisoryToFarmer() {
  smsStatus = 1;
currentPage=9;
u8g2.clearBuffer();
pageSmsStatus();
  float m = accM / sampleCount, t = accT / sampleCount, p = accPh / sampleCount;
  int   e = accEc / sampleCount, n = accN / sampleCount;
  int   f = accP / sampleCount,  k = accK / sampleCount;

  buildAdvisory(m, t, p, e, n, f, k);

  Serial.println(gsmReady ? "\n=== কৃষককে পাঠানো হচ্ছে ===" 
                          : "\n=== পরামর্শ (GSM মডিউল লাগানো নেই) ===");
  bool ok = sendBanglaSMS(FARMER_PHONE, buildSummary(m, t, p, e, n, f, k));
  delay(4000);

  for (int i = 0; i < adviceCount; i++) {
    Serial.println(adviceList[i]);
    if (!sendBanglaSMS(FARMER_PHONE, adviceList[i])) ok = false;
    delay(4000);
  }

  smsStatus = gsmReady ? (ok ? 2 : 3) : 0;
  lastAdvisory = millis();
  firstAdviceSent = true;

  // গড়ের হিসাব রিসেট
  accM = accT = accPh = 0; accEc = accN = accP = accK = 0; sampleCount = 0;
}

/* ======================= ৮. OLED পেজ (সব বাংলা) ======================= */

void pageTitle() {
  bnDrawCentered(u8g2, 4, BN_TITLE_SOIL);
  u8g2.drawHLine(4, 34, 120);
  if (!sensorOk) bnDrawCentered(u8g2, 44, BN_MSG_NODATA);
  else           bnDrawCentered(u8g2, 44, BN_ST_OK);
}

/* একটি মান দেখানোর সাধারণ ছক: উপরে লেবেল, নিচে বাংলা অঙ্ক, ডানে একক */
void drawValuePage(const unsigned char *lbl, int lw, int lh,
                   float value, uint8_t decimals,
                   const unsigned char *unit, int uw, int uh) {
  u8g2.drawXBMP((128 - lw) / 2, 1, lw, lh, lbl);
  u8g2.drawHLine(4, lh + 4, 120);

  int numW = bnNumberWidth(value, decimals, true);
  int total = numW + 4 + uw;
  int x = (128 - total) / 2;
  bnDrawNumber(u8g2, x, 32, value, decimals, true);
  u8g2.drawXBMP(x + numW + 4, 32 + BN_BIG_H - uh, uw, uh, unit);
}

void pageMoisture()  { drawValuePage(BN_LBL_MOISTURE, BN_LBL_MOISTURE_W, BN_LBL_MOISTURE_H,
                                     moisture, 1, BN_UNIT_PCT, BN_UNIT_PCT_W, BN_UNIT_PCT_H); }
void pageTemp()      { drawValuePage(BN_LBL_TEMP, BN_LBL_TEMP_W, BN_LBL_TEMP_H,
                                     temperature, 1, BN_UNIT_CELSIUS, BN_UNIT_CELSIUS_W, BN_UNIT_CELSIUS_H); }
void pageEC()        { drawValuePage(BN_LBL_EC, BN_LBL_EC_W, BN_LBL_EC_H,
                                     ec, 0, BN_UNIT_USCM, BN_UNIT_USCM_W, BN_UNIT_USCM_H); }
void pageNitrogen()  { drawValuePage(BN_LBL_N, BN_LBL_N_W, BN_LBL_N_H,
                                     nitrogen, 0, BN_UNIT_MGKG, BN_UNIT_MGKG_W, BN_UNIT_MGKG_H); }
void pagePhosphorus(){ drawValuePage(BN_LBL_P, BN_LBL_P_W, BN_LBL_P_H,
                                     phosphorus, 0, BN_UNIT_MGKG, BN_UNIT_MGKG_W, BN_UNIT_MGKG_H); }
void pagePotassium() { drawValuePage(BN_LBL_K, BN_LBL_K_W, BN_LBL_K_H,
                                     potassium, 0, BN_UNIT_MGKG, BN_UNIT_MGKG_W, BN_UNIT_MGKG_H); }

void pagePH() {
  bnDrawCentered(u8g2, 1, BN_LBL_PH);
  u8g2.drawHLine(4, BN_LBL_PH_H + 4, 120);
  bnDrawNumberCentered(u8g2, 32, ph, 1, true);
}

/* পুষ্টি উপাদানের অংশ শুরুর শিরোনাম পেজ */
void pageNutrientTitle() {
  bnDrawCentered(u8g2, 12, BN_TITLE_NUTRIENT);
  u8g2.drawHLine(4, 46, 120);
}

/* SMS অবস্থা */
void pageSmsStatus() {
  bnDrawCentered(u8g2, 2, BN_TITLE_ADVICE);
  u8g2.drawHLine(4, 30, 120);
  if (smsStatus == 1)      bnDrawCentered(u8g2, 40, BN_MSG_SENDING);
  else if (smsStatus == 2) bnDrawCentered(u8g2, 40, BN_MSG_SENT);
  else if (smsStatus == 3) bnDrawCentered(u8g2, 40, BN_MSG_FAIL);
  else                     bnDrawCentered(u8g2, 40, BN_LBL_PHONE);
}

/* ======================= ৯. setup / loop ======================= */

void setup() {
  Serial.begin(115200);
  delay(500);

  if (SOIL_DE_PIN >= 0) { pinMode(SOIL_DE_PIN, OUTPUT); digitalWrite(SOIL_DE_PIN, LOW); }
  if (BUTTON_PIN >= 0)  pinMode(BUTTON_PIN, INPUT_PULLUP);

  soilSerial.begin(SOIL_BAUD, SERIAL_8N1, SOIL_RX_PIN, SOIL_TX_PIN);
  sim800.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);

  u8g2.begin();
  u8g2.clearBuffer();
  bnDrawCentered(u8g2, 20, BN_TITLE_SOIL);
  u8g2.sendBuffer();

  delay(3000);                            // SIM800L নেটওয়ার্কে ঢোকার সময়

  Serial.println("\n--- মাটি পরীক্ষা ও পরামর্শ ব্যবস্থা ---");

  gsmReady = detectGsm();
  if (gsmReady) {
    sendCommand("ATE0", 300);
    sendCommand("AT+CPIN?", 500);
    sendCommand("AT+CSQ", 500);
    sendCommand("AT+CREG?", 500);
    sendCommand("AT+CMGF=1", 300);
    sendCommand("AT+CNMI=2,2,0,0,0", 300);
    sendCommand("AT+CLIP=1", 300);
    Serial.println("SIM800L পাওয়া গেছে। SMS যাবে: " + String(FARMER_PHONE));
  } else {
    Serial.println("SIM800L পাওয়া যায়নি — শুধু OLED ও সিরিয়ালে পরামর্শ দেখানো হবে।");
  }

  Serial.println("প্রস্তুত। সিরিয়ালে 'SEND' লিখলে এখনই পরামর্শ তৈরি হবে।");
  lastAdvisory = millis();
  firstAdviceSent=true;
}

void loop() {
  readSensor();

  /* --- পেজ বদল --- */
  if (millis() - lastPageChange >= 2500) {
    currentPage = (currentPage + 1) % 10;
    lastPageChange = millis();
  }

  u8g2.clearBuffer();
  switch (currentPage) {
    case 0: pageTitle();            break;
    case 1: pageMoisture();         break;
    case 2: pageTemp();             break;
    case 3: pagePH();               break;
    case 4: pageEC();               break;
    case 5: pageNutrientTitle();    break;
    case 6: pageNitrogen();         break;
    case 7: pagePhosphorus();       break;
    case 8: pagePotassium();        break;
    case 9: pageSmsStatus();        break;
  }
  u8g2.sendBuffer();

  /* --- SMS পাঠানোর শর্ত --- */
  bool timeUp     = (millis() - lastAdvisory >= ADVISORY_INTERVAL_MS);
  bool enoughData = (sampleCount >= SAMPLES_BEFORE_ADVICE);
  bool pressed    = (BUTTON_PIN >= 0 && digitalRead(BUTTON_PIN) == LOW);

  if (enoughData && (timeUp || pressed || !firstAdviceSent)) {
    sendAdvisoryToFarmer();
  }

  /* --- সিরিয়াল কমান্ড: SEND / CALL: / SMS: --- */
  if (Serial.available()) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.equalsIgnoreCase("SEND")) {
      if (sampleCount == 0) readSensor();
      if (sampleCount > 0) sendAdvisoryToFarmer();
      else Serial.println("সেন্সর থেকে ডেটা পাওয়া যায়নি।");
    } else if (in.startsWith("SMS:")) {
      int sp = in.indexOf(':', 4);
      if (sp != -1) sendBanglaSMS(in.substring(4, sp), in.substring(sp + 1));
    } else {
      sim800.println(in);
    }
  }

  /* --- SIM800L এর উত্তর সিরিয়ালে দেখানো --- */
  while (sim800.available()) {
    String raw = sim800.readStringUntil('\n');
    String clean = "";
    for (unsigned int i = 0; i < raw.length(); i++)
      if (raw[i] >= 32 && raw[i] <= 126) clean += raw[i];
    if (clean.length()) Serial.println(clean);
  }

  delay(80);
}
