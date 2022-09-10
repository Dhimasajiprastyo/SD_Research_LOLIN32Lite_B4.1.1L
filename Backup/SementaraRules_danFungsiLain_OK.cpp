#pragma region include_libraries
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <RtcDS3231.h>
#include <ArduinoJson.h>
#include <Adafruit_ADS1015.h>
#include <menu.h>
#include <LiquidCrystal_I2C.h>
#include <SPIFFS.h>
#include <ArduinoHttpClient.h>
#include <SimpleKalmanFilter.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "SD_Op_D.h"
#include "string.h"
#include <esp_task_wdt.h>

#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include "setup.cpp"
#pragma endregion include_libraries

//===========================PENTING==============================================
#define __NO_SYCN__ //NO NEED TO SYNC TIME EVERY HARD RESET
//===========================PENTING==============================================

#ifdef _ACTIVATE_SENSOR_SERIAL1_
#define sensor_serial1 Serial1
#endif

#ifdef _USE_PZEM_
#include <PZEM004Tv30.h>
// #if defined (_DIRECT_PZEM_)
// // #define pzem_serial Serial1
// #elif defined (_USE_SLAVE_)
// #define slave_serial Serial1
// #endif
#endif

#ifdef _USE_ULTRASONIC_FLOW_METER_
#include <ModbusMaster.h>
#define flow4_addr 4
#define flow5_addr 5
#define flow6_addr 6
// #define flow_register 1436
#define flow_register 0
#define cubic_register 112
// #if defined (_DIRECT_ULTRASONIC_FLOW_METER_)
// // #define flow_serial Serial1
// #elif defined (_USE_SLAVE_)
// #define slave_serial Serial1
// #endif
#endif

#pragma region general_define
#ifndef DEVICE_ID
#define DEVICE_ID "tester"
#endif
#ifndef DEFAULT_TOPIC
#define DEFAULT_TOPIC "device/tester"
#endif
#ifndef STATUS_TOPIC
#define STATUS_TOPIC "device/tester/status"
#endif

#define DEVICE_TYPE 2
#if DEVICE_TYPE == 2
#define SDA_PIN 13
#define SCK_PIN 22
#define RTC_SQW_PIN 33
#elif DEVICE_TYPE == 1
#define SDA_PIN 23
#define SCK_PIN 19
#define RTC_SQW_PIN 33
#endif

#define BUTTON1 34
#define BUTTON2 35
#define BUTTON3 32
#define BAUDRATE 115200
#define GSM_BAUDRATE 9600
#define I2C_ADC0_ADDRESS 0x48
#define I2C_ADC1_ADDRESS 0x49
#define I2C_DS3231_ADDRESS 0x68
#define I2C_LCD_ADDRESS 0x27
#define TOUCH_TRESHOLD 50
#define BUTTON_BITMASK 0xD00000000
// #define TINY_GSM_MODEM_SIM800
#define DEBOUNCE_TIME 50
#define WAKE_UP_TIME 25000
#define PRINT_ANALOG_TIMEOUT 60000
#define CONFIG_FILENAME "/config.txt"
#define RULES_FILENAME "/rules.txt"
#define DEF_RULES_FLAG "/def_rules.txt"
#define DATA_FILENAME "/data.txt"
#define DEVICE_INFO_FILENAME "/device_info.txt"
#define APN "internet"
#define APN_USERNAME ""
#define APN_PASSWORD ""
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define constBattery 3.378
#define DEFAULT_WATCHDOG_TIMER 300 // 5MENIT
#define DEFAULT_BATTERY_SAFE 11.1 //11 VOLT
#define DEFAULT_TIME_FOR_BROKER 180 //3 MENIT
#define ANS_STATUS_LENGTH 770 //Jumlah karakter answer status broker, sebelumnya 750
#define LIMIT_SIMILAR_MESSAGE 8 //mencegah pesan yang sama dari retain message untuk diproses dalam 1 sesi komunikasi
#define DEFAULT_SYNCTIME_INTERVAL 8
#pragma endregion general_define

#define SD_SS 14
#define SIM_RING 27
#ifdef _USE_A1_FOR_WATER_LEVEL_
#define SW_AWLR_PWR 25
#endif
//=========================================================================

#pragma region object_generation
// Define instance here---------------------------------------------------
Adafruit_ADS1115 ADC0(I2C_ADC0_ADDRESS);
Adafruit_ADS1115 ADC1(I2C_ADC1_ADDRESS);
RtcDS3231<TwoWire> rtc(Wire);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Menu menu;
TinyGsm modem(Serial2);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
HttpClient httpTime(client, "worldtimeapi.org", 80);

#ifdef _USE_PZEM_
// pzem(&slave_serial);
/**
   level tegangan serial master(ESP32) : 3.3v
   level tegangan serial slave (arduino nano) :3.3v
   Regulator tegangan untuk PZEM harus disesuaikan sesuai dengan master yang digunakan
   PZEM(TX)------(RX)ESP
   PZEM(RX)------(TX)ESP
*/
PZEM004Tv30 pzem(&sensor_serial1);
PZEM004Tv30 pzem1(&sensor_serial1, 1);
PZEM004Tv30 pzem2(&sensor_serial1, 2);
PZEM004Tv30 pzem3(&sensor_serial1, 3);
#endif

#ifdef _USE_ULTRASONIC_FLOW_METER_
ModbusMaster flow_modbus4;
#ifdef _USE_FLOW_ID_5_
ModbusMaster flow_modbus5;
#endif
#ifdef _USE_FLOW_ID_6_
ModbusMaster flow_modbus6;
#endif
#endif

SDOP sdop; //SD operation
#pragma endregion object_generation

#pragma region enum_generation
enum opsi_broker {
  cmd_ready,
  cmd_wait_me,
  cmd_clear_pend,
  cmd_clear_sent,
  cmd_sync_time,
  cmd_count_pend,
  cmd_count_sent,
  cmd_def_settings,
  cmd_close
};

enum cmd_code {
  code_host, code_software, code_tes_kirim, code_status, code_sync_time,
  code_battery, code_imei, code_signal, code_sd_clr_pend, code_sd_clr_sent,
  code_wait_me, code_set_limit_waitme, code_sd_count_pend, code_sd_count_sent,
  code_recount_sent, code_recount_pend, code_apply_def_settings, code_set_sd_limit_access,
  code_cancle_recount_sent, code_cancle_recount_pend, code_write_rules, code_print_rules,
  code_recheck_sd, code_close_cmd, code_not_found
};
#pragma endregion enum_generation

#pragma region global_variables
// Define Global Variable here
RTC_DATA_ATTR uint8_t logCount = 0;
RTC_DATA_ATTR uint16_t recorded_aprox = 0;
RTC_DATA_ATTR uint16_t pending_aprox = 0;
RTC_DATA_ATTR bool alreadyLog = false;
RTC_DATA_ATTR bool badSignal_directSend = false;
RTC_DATA_ATTR bool already_sync = false;
RTC_DATA_ATTR uint16_t cnt_for_auto_restart = 0;
RTC_DATA_ATTR uint8_t next_ambil_data = 0;
RTC_DATA_ATTR uint8_t next_kirim_data = 0;

byte wakeUpFlag = 0;
bool bAlarmOne = false;
bool bAlarmTwo = false;
bool SIMon = false;
uint8_t selected_apn = 0;
bool filePending = false;
bool BatterySafeMode = false;
bool got_message = false;
bool wait_for_broker = false;
bool wait_broker_initiated = true;
uint32_t start_time_for_broker = 0;
uint8_t time_broker = 0;
bool wdt_is_set = false;
uint8_t cmd_message = 0;
uint16_t count_wait_me = 0;
uint16_t limit_wait_me = 1200; //in second
uint16_t int_limit_wait_me = limit_wait_me / 18;
String string_buffer;
uint8_t sending_count = 0;
uint16_t sd_limit_access = 25;
bool recount_sent = 0;
bool recount_pend = 0;
int8_t prev_cmd_code = -1;
uint8_t similar_message = 0;
unsigned long lcdRefresh;
bool disableSD_temporary = false;
bool rules_exist = false;
#pragma endregion global_variables

/**
   format array sensor dari slave
  //v1  I1  p1  Wh1 f1  pf1 |v2   I2  p2  Wh2 f2  pf2 |v3   I3  p3  Wh3 f3  pf3 |flw  h   tur ph  cl  misc1 misc2 misc3
  //0   1   2   3   4   5   |6    7   8   9   10  11  |12   13  14  15  16  17  |18   19  20  21  22  23    24    25
*/
#ifdef _DIRECT_PZEM_
//float pzem_value[7];
#endif

//-----------opsi APN --------------------------------------------------------------------------
// String apn_alias[8] = {"default", "Telkom1", "Telkom2", "Tri1", "Tri2", "Indosat1", "Indosat2", "Indosat3"};
// const char* apn_name[8] = {APN, "internet", "telkomsel", "3gprs", "3data", "indosatgprs", "indosatooredoo.com", "mms.satelindogprs.com"};
// const char* apn_usr[8] = {APN_USERNAME, "wap", "wap", "3gprs", "", "indosat", "indosat", "satmms"};
// const char* apn_pswd[8] = {APN_PASSWORD, "wap123", "wap123", "3gprs", "", "indosat", "indosatgprs", "satmm"};
//-----------opsi APN -------------------------------------------------------------------------

#pragma region server_name_settings
//char *brokerStaging = "34.101.187.36";
const char *brokerStaging = "pakis-mqtt.sumpahpalapa.com";
const char *brokerProduction = "pakis-mqtt.pdampintar.id";
// char *brokerStaging = "34.87.0.141";
// char *brokerProduction = "34.87.0.141";
// const char *brokerStaging = "pakis-mqtt.iotsakti.id";
// const char *brokerProduction = "pakis-mqtt.iotsakti.id";
#pragma endregion server_name_settings

#pragma region new_lcd_characters
byte Lock[8] = {
  0b01110,
  0b10001,
  0b10001,
  0b11111,
  0b11011,
  0b11011,
  0b11111,
  0b00000
};

byte enter[8] = { //atau burst
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b00100,
  0b00100,
  0b01110,
  0b00100
};

byte safe[8] = {
  0b01110,
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111
};

byte sd[8] = {
  0b01111,
  0b01111,
  0b10001,
  0b01001,
  0b10001,
  0b10001,
  0b10001,
  0b11111
};

#ifdef _SET_FLOW_ELECTRIC_ICON_
byte electric[8] = {
  0b11111,
  0b00110,
  0b01100,
  0b11111,
  0b00011,
  0b00110,
  0b01100,
  0b11111
};
#else
byte electric[8] = {
  0b00011,
  0b00110,
  0b01100,
  0b11111,
  0b00011,
  0b00110,
  0b01100,
  0b11000
};
#endif


#pragma endregion new_lcd_characters

#pragma region new_structures
struct Rules {
  float p1, p2, p3, o1, o2, o3, wl, fl, v1, i1, c1, v2, i2, c2, v3, i3, c3;
  //-----------operator rule
  uint8_t op1, op2, op3, oo1, oo2, oo3, owl, ofl, ov1, oi1, oc1, ov2, oi2, oc2, ov3, oi3, oc3;
};
Rules rules;

struct Config {
  char host[34];
  int port;
  int kirimDataInterval;
  int ambilDataInterval;
  int timezone;
  bool brokerSelector;
  uint8_t selected_apn;
  uint16_t SD_store_limit;
  // bool listening;
  float battery_safe;
  uint8_t minimum_signal;
  uint16_t time_for_broker;
  uint16_t watchdog_timer;
  uint8_t max_send;
  bool useSD;
  uint16_t SD_attempt;
  // uint8_t burst_send_h_on;
  // uint8_t burst_send_h_off;
  uint8_t cnt_send_from_spiffs;
  uint8_t sync_time_interval;
  uint8_t log_cnt_auto_restart;
};
Config config;

struct DeviceInfo {
  char software_version[10];
  bool sd_failure;
  char my_dn[13];
  char my_location[40];
};
DeviceInfo device_info;
#pragma endregion new_structures

//const size_t payloadSize = JSON_OBJECT_SIZE(5) * 3;
// const size_t payloadSize = JSON_OBJECT_SIZE(25) + 300; //ini oke dan aman
#ifdef _USE_PZEM_
const size_t payloadSize = JSON_OBJECT_SIZE(25);
#endif
#ifndef _USE_PZEM_
const size_t payloadSize = JSON_OBJECT_SIZE(7) + 210;
#endif

// const size_t arraySize = JSON_ARRAY_SIZE(6) * 10; //gak dipake dimana mana
// uint32_t lastReconnectAttempt = 0;
uint32_t waitingData = 0;





//-------------------------------------------------------------------------------------------------------
void stopWDT() {
  esp_task_wdt_delete(NULL); //unsubscribe wdt------
  esp_task_wdt_deinit(); //deinit wdt----------
}

void startWDT(uint16_t limitTime) {
  esp_task_wdt_init(limitTime, true); //enable panic so ESP32 restarts limit 5 menit jika terjadi kegagalan koneksi SD
  esp_task_wdt_add(NULL); //add current thread to WDT watch
}

void internalRTCwakeup(uint16_t t_wakeup) {
  esp_sleep_enable_timer_wakeup(t_wakeup * uS_TO_S_FACTOR);
  Serial.printf("Set internalRTC: %is after sleep\n", t_wakeup);
}

int8_t search_word(String& find_word, String& source) {
  uint8_t check = 0;
  uint16_t total_scan = source.length();
  uint16_t i, j;
  int8_t index_found = -1;
  bool found = true;

  check = find_word.length();
  const char* c_find_word = find_word.c_str();

  Serial.printf("-Searching word-\n%s:%i \nSize source:%i\n", c_find_word, check, total_scan);

  if (total_scan < check) {
    Serial.println("ref kata lebih sedikit!\n");
    return -1;
  }

  for (i = 0; i < total_scan; i++) {
    found = true;
    for (j = 0; j < check; j++) {
      if (find_word[j] != source[j + i]) {
        j = check;
        found = false;
      }
    }
    if (found) {
      index_found = i;
      i = total_scan + 1;
      break;
    }
  }
  Serial.printf("-SEARCH_DONE(%i)-\n", index_found);
  return index_found;
}


const char* brokerSelectorF(bool selector)
{
  /* if (selector) {
    return brokerProduction;
    } else {
    return brokerStaging;
    } */

#ifdef _USE_STAGING_
  return brokerStaging; //Apapun bool selectornya, yang direturn tetap staging server
#else
  if (selector)
    return brokerProduction;
  else
    return brokerStaging;
#endif
}

/**
   Get Analog value from ADS1115 over I2C
   if analog address less then 4 it should be get from ADC0
   @param address analog address 0-7
   @return int 16 bit og analog value
*/
int16_t readI2CAnalog(byte address) {
  Adafruit_ADS1115 ADC = address > 3 ? ADC1 : ADC0;
  address = address % 4;
  return ADC.readADC_SingleEnded(address);
}

/**
   convert from analog 16 bit to voltage
   gain mode default yang digunkan 6.144v 16bit(15bit), 6.144v/32.768(15bit) = 0.1875V per bit
   @param analog analog 16 bit
   @return float data of voltage
*/
float convertAnalogToVolt(int16_t analog) {
  float out = (analog * 0.1875) / 1000;
  out = out < 0 ? 0 : out;
  return out;
}

/**
   get battery voltage
   @return float voltage of battery
*/
float getBatteryVoltage() {
  float v;
  int16_t analogRead = readI2CAnalog(3);
  v = constBattery * (convertAnalogToVolt(analogRead));
  return v;
}

/**
   Get date time with format YYYY/MM/DD,HH:MM:SS+ZZ
   @param dt Typedef struct RtcDateTime
   @return Datetime in string
*/
String getStringDateTime(byte timezone) {
  RtcDateTime dt = rtc.GetDateTime();
  char datetime[23];
  snprintf_P(datetime, sizeof(datetime), PSTR("%04u/%02u/%02u,%02u:%02u:%02u+%02u"),
             dt.Year(),
             dt.Month(),
             dt.Day(),
             dt.Hour(),
             dt.Minute(),
             dt.Second(),
             timezone
            );
  return datetime;
}

void setCurrentTime() {
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  rtc.SetDateTime(compiled);
}

void setCurrentTime(uint32_t unixtime) {
  RtcDateTime compiled = RtcDateTime(unixtime);
  rtc.SetDateTime(compiled);
}

uint32_t syncTimeUnixTime() {
  if ((config.timezone < 7) || (config.timezone > 9)) {
    Serial.println("BUKAN TIMEZONE INDONESIA");
    lcd.clear(); lcd.print("TimezoneInvalid");
    delay(2000);
    config.timezone = DEFAULT_TIMEZONE;
  }
  String area; area.reserve(29);
  if (config.timezone == 7) {
    area = "/api/timezone/Asia/Jakarta";
  } else if (config.timezone == 8) {
    area = "/api/timezone/Asia/Makassar";
  } else if (config.timezone == 9) {
    area = "/api/timezone/Asia/Jayapura";
  }
  const char* c_area = area.c_str();
  Serial.println(c_area);
  // int err = httpTime.get(TIMEZONE == 7 ? "/api/timezone/Asia/Jakarta" : "/api/timezone/Asia/Makassar");
  int err = httpTime.get(c_area);
  if (err != 0 ) return 0;
  DynamicJsonDocument doc(512);
  String strTime = httpTime.responseBody();
  Serial.println("--HTTPtime_responseBody--");
  Serial.println(strTime);
  Serial.println("--HTTPtime_responseBody--");
  deserializeJson(doc, strTime);
  return doc["unixtime"];
}

double getGPIOWakeUp() {
  uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
  return (log(GPIO_reason)) / log(2);
}

/**
   0 = left
   1 = right
   2 = enter
*/
int8_t wait_button(bool hold) {
  int8_t btn = -1;
  while (1) {
    btn = menu.buttonEvent();
    if (btn >= 0) {
      break;
    }
    if (!hold) {
      break;
    }
    delay(100);
  }
  return btn;
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : wakeUpFlag = 1; break;
    case ESP_SLEEP_WAKEUP_EXT1 : wakeUpFlag = 2; break;
    case ESP_SLEEP_WAKEUP_TIMER : wakeUpFlag = 3; break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

/**
   print ke lcd dengan refresh rate
   @param y1 character pada baris pertama
   @param y2 character pada baris kedua
*/
char y1_past[16];
char y2_past[16];
void lcdPrint(const char y1[16], const char y2[16]) {
  if (strcmp(y1, y1_past) || strcmp(y2, y2_past)) {
    if (millis() - lcdRefresh > 150) {
      lcdRefresh = millis();
      lcd.clear();
      lcd.home();
      lcd.print(y1);
      lcd.setCursor(0, 1);
      lcd.print(y2);
    }
    strcpy(y1_past, y1);
    strcpy(y2_past, y2);
  }
}

int16_t filterAnalog(byte analogPin) {
  SimpleKalmanFilter kalman(2, 2, 0.1);
  int16_t buffer;
  for (int i = 0; i < 20; i++) {
    buffer = kalman.updateEstimate(readI2CAnalog(analogPin));
    delay(25);
  }
  return buffer;
}

/**
   sesuaikan readI2CAnalog(x)
   ganti x dengan port adc yg digunakan
*/
#ifdef _FIND_AWLR_VOLTAGE_
float getAWLRvoltage(uint8_t AnalogPin) {
  float v;
  int16_t analogRead = filterAnalog(AnalogPin);
  v = convertAnalogToVolt(analogRead);
  return v;
}

float getWaterHeight(uint8_t AnalogPin) {
  float v = getAWLRvoltage(AnalogPin);
  Serial.println(v);
  // float h_cm = (v-0.10)*105;
  // float h_cm = (v - 0.08) * 105;
  //  float h_cm = (v-0.47925) * 266.7647;
  float h_cm = (v - 0.48) * 266.1765;
  Serial.println(h_cm);
  // float h_cm = (v-0.48) * 266.1765;
  // 260 = (2.59-0.105)*x
  // 2.6 = 2.59 - x
  return h_cm;
}
#endif

float getPressureValue(uint8_t analog_port, int16_t adc) {
  int16_t adc_pressure = filterAnalog(analog_port);
  if (adc > 0) {
    adc_pressure = adc;
  }
  float bar_pressure = ((convertAnalogToVolt(adc_pressure) - 0.5) * 250) / 100;
  return bar_pressure;
}

float pressure_plus_offset(float pressure, float offset) {
  float sum = pressure + offset;
  if (pressure >= 0) {
    return sum;
  } else {
    return 0;
  }
}

void printData() {
  // int16_t buffer[4];
  int16_t adc_volt;

  while (1) {
    int buttonEvt = menu.buttonEvent();
    if (buttonEvt > -1) break;

    adc_volt = filterAnalog(3);
    // buffer[0] = filterAnalog(3);
    // buffer[1] = filterAnalog(0);
    // buffer[2] = filterAnalog(1);
    // buffer[3] = filterAnalog(2);
    // float battery = convertAnalogToVolt(buffer[0]) * 3.35;
    float battery = convertAnalogToVolt(adc_volt) * 3.35;
    float batPersentage = ((battery - 10) / 4) * 100;
    // float pressure1 = ((convertAnalogToVolt(buffer[1]) - 0.5) * 250) / 100;
    // float pressure2 = ((convertAnalogToVolt(buffer[2]) - 0.5) * 250) / 100;
    // float pressure3 = ((convertAnalogToVolt(buffer[3]) - 0.5) * 250) / 100;
    float pressure1 = getPressureValue(0, 0);
    float pressure2 = getPressureValue(1, 0);
    float pressure3 = getPressureValue(2, 0);
    if (rules_exist) {
      pressure1 = pressure_plus_offset(pressure1, rules.o1);
      pressure2 = pressure_plus_offset(pressure2, rules.o2);
      pressure3 = pressure_plus_offset(pressure3, rules.o3);
    } else {
      pressure1 = pressure_plus_offset(pressure1, _OFFSET_P1_);
      pressure2 = pressure_plus_offset(pressure2, _OFFSET_P2_);
      pressure3 = pressure_plus_offset(pressure3, _OFFSET_P3_);
    }
    // pressure1 = pressure1 >= 0 ? (pressure1 + _OFFSET_P1_) : 0; // edit + .. untuk offset
    // pressure2 = pressure2 >= 0 ? (pressure2 + _OFFSET_P2_) : 0;
    // pressure3 = pressure3 >= 0 ? (pressure3 + _OFFSET_P3_) : 0;

    char bufferLCD[2][16];
    sprintf(bufferLCD[0], "Bat:%.02f/%.01f", battery, batPersentage);
    sprintf(bufferLCD[1], "P:%.01f %.01f %.01f", pressure1, pressure2, pressure3); //nanti dicoment aja
    lcd.clear();
    lcd.home();
    lcd.print(bufferLCD[0]);
    lcd.setCursor(0, 1);
    lcd.print(bufferLCD[1]);
    delay(500);
  }
}

/* void printAnalog() {
  byte selector = 0;
  int16_t buffer[4];
  char charBuffer1[16];
  char charBuffer2[16];
  float voltage[4];
  unsigned long timeout = millis();
  lcd.clear();

  while (1) {
    if (millis() - timeout > PRINT_ANALOG_TIMEOUT) break;
    int buttonEvt = menu.buttonEvent();
    menuAction action = static_cast<menuAction>(buttonEvt);
    if (action == MENU_MOVE_LEFT || action == MENU_MOVE_RIGHT) {
      selector++;
      selector = selector > 1 ? 0 : selector;
    } else if (action == MENU_ENTER) {
      break;
    }

    if (selector == 0) {
      for (int i = 0 ; i < 4; i++) {
        buffer[i] =  readI2CAnalog(i);
        voltage[i] = convertAnalogToVolt(buffer[i]);
      }
      sprintf(charBuffer1, "A0:%.1f   A1:%.1f", voltage[0], voltage[1]);
      sprintf(charBuffer2, "A2:%.1f   A3:%.1f", voltage[2], voltage[3]);
    } else {
      for (int i = 0 ; i < 4; i++) {
        buffer[i] =  readI2CAnalog(i + 4);
        voltage[i] = convertAnalogToVolt(buffer[i]);
      }
      sprintf(charBuffer1, "A4:%.1f   A5:%.1f", voltage[0], voltage[1]);
      sprintf(charBuffer2, "A6:%.1f   A7:%.1f", voltage[2], voltage[3]);

    }
    lcdPrint(charBuffer1, charBuffer2);
    strcpy(charBuffer1, "");
    strcpy(charBuffer2, "");
  }
  }
*/

void interactiveInputInterval(const char* title, int &initialValue, byte cnt, String unit) {
  const char* c_unit = unit.c_str();

  lcd.clear();
  lcd.home();
  lcd.print(title);
  lcd.setCursor(0, 1);
  lcd.print("<" + String(initialValue) + ">");

  while (1) {
    int buttonEvt = menu.buttonEvent();
    menuAction action = static_cast<menuAction>(buttonEvt);
    char bufferChar[16];
    sprintf(bufferChar, "< %d%s >", initialValue, c_unit);
    lcdPrint(title, bufferChar);
    if (action == MENU_MOVE_LEFT) {
      initialValue -= cnt;
      initialValue = initialValue < 0 ? 0 : initialValue;
    } else if (action == MENU_MOVE_RIGHT) {
      initialValue += cnt;
    } else if (action == MENU_ENTER) {
      lcd.setCursor(15, 1);
      lcd.write(byte(0));
      delay(1000);
      break;
    }
  }
}

bool cek_battery_safe() {
  float v_bat = getBatteryVoltage();
  Serial.print("VBAT:");
  Serial.println(v_bat);
  if (v_bat <= config.battery_safe) {
    BatterySafeMode = true;
    return true;
  } else {
    BatterySafeMode = false;
    return false;
  }
}

/*
  bool cek_burst_mode(uint8_t on, uint8_t off) {
  bool mode = false;
  RtcDateTime dt = rtc.GetDateTime();
  //--------------cek apakah dalam safe mode atau tidak----------------

  if (on > off) { //contoh 23 - 8
    if ((dt.Hour() >= on) || (dt.Hour() < off)) {
      mode = true;
    } else {
      mode = false;
    }
  } else if (on < off) { //contoh 10 - 20 14 15
    if ((dt.Hour() >= on) && (dt.Hour() < off)) {
      mode = true;
    } else {
      mode = false;
    }
  }
  if (cek_battery_safe()) {
    mode = false;
  }
  burstMode = mode;
  return mode;
  }
*/

bool SDok() {
  bool sdok = config.useSD && (~device_info.sd_failure);
  Serial.print("SD_OK:");
  Serial.println(sdok);
  return sdok;
}

void print_mode() {
  if (cek_battery_safe()) {
    lcd.setCursor(14, 0); lcd.write(byte(2));
  }
  // if (cek_burst_mode(config.burst_send_h_on, config.burst_send_h_off)) {
  //   lcd.setCursor(13, 0); lcd.write(byte(1));
  // }
  if (SDok()) {
    lcd.setCursor(15, 0); lcd.write(byte(3));
  }

  lcd.setCursor(15, 1);
  if (!config.brokerSelector) {
    lcd.print("X");
    lcd.setCursor(14, 1);
  }
#ifdef _USE_PZEM_
  lcd.write(byte(4));
#endif
}

void lcdPrintCurrentDate() {
  RtcDateTime now = rtc.GetDateTime();
  uint8_t csq = modem.getSignalQuality();
  float bat = getBatteryVoltage();
  char buff[2][16];
  sprintf(buff[0], "%d/%d/%d %d", now.Day(), now.Month(), now.Year(), csq);
  sprintf(buff[1], "%d:%d:%d %.1fV", now.Hour(), now.Minute(), now.Second(), bat);
  lcdPrint(buff[0], buff[1]);
  print_mode();
}

byte i2cHealthCheck(byte address) {
  byte buffer;

  Wire.beginTransmission(address);
  buffer = Wire.endTransmission();

  return buffer;
}

void printStatus() {
  byte ADC0_Health = i2cHealthCheck(I2C_ADC0_ADDRESS);
  byte ADC1_Health = i2cHealthCheck(I2C_ADC1_ADDRESS);
  byte RTC_Health = i2cHealthCheck(I2C_DS3231_ADDRESS);
  //byte LCD_Health = i2cHealthCheck(I2C_LCD_ADDRESS);
  char lcdBuffer[2][16];
  sprintf(lcdBuffer[0], "ADC0:%s ADC1:%s", ADC0_Health == 0 ? "OK" : "Er", ADC1_Health == 0 ? "OK" : "Er");
  sprintf(lcdBuffer[1], "RTC:%s", RTC_Health == 0 ? "OK" : "Er");
  //sprintf(lcdBuffer[1], "RTC:%s LCD:%s", RTC_Health == 0 ? "OK" : "Er", LCD_Health == 0 ? "OK" : "Er");
  lcd.clear();
  lcd.home();
  lcd.print(lcdBuffer[0]);
  lcd.setCursor(0, 1);
  lcd.print(lcdBuffer[1]);
  unsigned long timeout = millis();
  bool selector = false;
  while (1) {
    int buttonEvt = menu.buttonEvent();
    if (millis() - timeout > PRINT_ANALOG_TIMEOUT) break;

    if (millis() - timeout > 3000) {
      if (selector) {
        lcdPrint(lcdBuffer[0], lcdBuffer[1]);
      } else {
        RtcTemperature temp = rtc.GetTemperature();
        float suhu = temp.AsFloatDegC();
        char buffer[16];
        sprintf(buffer, "T:%.1fC", suhu);
        lcdPrint(buffer, "");
      }
      timeout = millis();
      selector = !selector;
    }

    if (buttonEvt > -1) break;
  }
}

void cpyHostTxt(bool brokerSelector) {
  if (brokerSelector) {
    strncpy(config.host, brokerProduction, (strlen(brokerProduction) + 1));
  } else {
    strncpy(config.host, brokerStaging, (strlen(brokerStaging) + 1));
  }
}

bool readRules(fs::FS &fs, Rules &rules) {
  fs::File file = fs.open(RULES_FILENAME);
  if (!file || file.isDirectory()) {
    return false;
  }
  const size_t cap = 17 * JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(20) + 280; //hasil hitung dari https://arduinojson.org/v5/assistant/
  // StaticJsonDocument<cap * 4> doc;
  StaticJsonDocument<cap> doc;
  DeserializationError error = deserializeJson(doc, file);
  if (error) return false;

  rules.op1 = doc["p1"][0];
  rules.p1 = doc["p1"][1];
  rules.op2 = doc["p2"][0];
  rules.p2 = doc["p2"][1];
  rules.op3 = doc["p3"][0];
  rules.p3 = doc["p3"][1];

  rules.oo1 = doc["o1"][0];
  rules.o1 = doc["o1"][1];
  rules.oo2 = doc["o2"][0];
  rules.o2 = doc["o2"][1];
  rules.oo3 = doc["o3"][0];
  rules.o3 = doc["o3"][1];

  rules.owl = doc["wl"][0];
  rules.wl = doc["wl"][1];
  rules.ofl = doc["fl"][0];
  rules.fl = doc["fl"][1];

  rules.ov1 = doc["v1"][0];
  rules.v1 = doc["v1"][1];
  rules.oi1 = doc["i1"][0];
  rules.i1 = doc["i1"][1];
  rules.oc1 = doc["c1"][0];
  rules.c1 = doc["c1"][1];

  rules.ov2 = doc["v2"][0];
  rules.v2 = doc["v2"][1];
  rules.oi2 = doc["i2"][0];
  rules.i2 = doc["i2"][1];
  rules.oc2 = doc["c2"][0];
  rules.c2 = doc["c2"][1];

  rules.ov3 = doc["v3"][0];
  rules.v3 = doc["v3"][1];
  rules.oi3 = doc["i3"][0];
  rules.i3 = doc["i3"][1];
  rules.oc3 = doc["c3"][0];
  rules.c3 = doc["c3"][1];

  file.close();
  return true;
}

bool write_new_rules_from_broker(String& fname, String& new_rules) {
  const char* c_file_name = fname.c_str();
  const char* c_new_rules = new_rules.c_str();

  if (SPIFFS.exists(c_file_name)) {
    sdop.deleteFile(SPIFFS, c_file_name); //delete dulu file rules yg lama
  }

  mqtt.publish(STATUS_TOPIC, "{\"ans\":\"saving_new_rules\"}");

  bool write_ok = sdop.writeFile(SPIFFS, c_file_name, c_new_rules);
  if (write_ok) {
    if(SPIFFS.exists(DEF_RULES_FLAG)){sdop.deleteFile(SPIFFS,DEF_RULES_FLAG);}
    Serial.printf("%s updtd!\n", c_file_name);
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"RULES_UPDATED!\"}");
  } else {
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"FAILED!\"}");
  }
  delay(2000);
  String updated_rules; updated_rules.reserve(380);
  updated_rules = sdop.readFile(SPIFFS, c_file_name, 0);
  c_new_rules = updated_rules.c_str();
  Serial.print("NEW_RULES:");
  Serial.println(c_new_rules);
  //mqtt.publish(STATUS_TOPIC, c_new_config);
}


bool readConfig(fs::FS &fs, Config &config) {
  fs::File file = fs.open(CONFIG_FILENAME);
  if (!file || file.isDirectory()) {
    return false;
  }
  const size_t cap = JSON_OBJECT_SIZE(18) + 400;
  // StaticJsonDocument<cap * 4> doc;
  StaticJsonDocument<cap> doc;
  DeserializationError error = deserializeJson(doc, file);
  if (error) return false;

  config.port = doc["port"] | 1883;
  config.ambilDataInterval = doc["ambilData"];
  config.kirimDataInterval = doc["kirimData"];
  // config.lastSetAlarm0 = doc["lastSetAlarm0"];
  // config.lastSetAlarm1 = doc["lastSetAlarm1"];
  config.timezone = doc["timezone"];
  config.battery_safe = doc["battery_safe"];
  config.minimum_signal = doc["minimum_signal"];
  config.time_for_broker = doc["time_for_broker"];
  config.watchdog_timer = doc["watchdog_timer"];
  config.useSD = doc["useSD"];
  config.max_send = doc["max_send"];
  config.SD_attempt = doc["SD_attempt"];
  config.SD_store_limit = doc["SD_store_limit"];
  config.SD_attempt > 75 ? config.SD_attempt = 75 : config.SD_attempt;
  config.SD_attempt < 10 ? config.SD_attempt = 10 : config.SD_attempt;
  //config.SD_store_limit < 0 ? config.SD_store_limit = 0 : config.SD_store_limit;
  // config.burst_send_h_on = doc["burst_send_h_on"];
  // config.burst_send_h_off = doc["burst_send_h_off"];
  config.cnt_send_from_spiffs = doc["cnt_send_from_spiffs"];
  config.sync_time_interval = doc["sync_time_interval"];
  config.log_cnt_auto_restart = doc["log_cnt_auto_restart"];

#ifdef _USE_DEF_SERVER_
  config.brokerSelector = true;
#else
  config.brokerSelector = doc["brokerSelector"];
#endif

  config.selected_apn = doc["selected_apn"];
  cpyHostTxt(config.brokerSelector);
  // if(config.brokerSelector){
  //   strncpy(config.host,brokerProduction,(strlen(brokerProduction)+1));
  // }else{
  //   strncpy(config.host,brokerStaging,(strlen(brokerStaging)+1));
  // }

  // strlcpy(
  //   config.host,
  //   doc["host"] | "pakis-mosquitto.sumpahpalapa.com",
  //   sizeof(config.host)
  // );

  file.close();
  config.ambilDataInterval <= 0 ? config.ambilDataInterval = 3 : config.ambilDataInterval;
  config.kirimDataInterval <= 0 ? config.kirimDataInterval = 30 : config.kirimDataInterval;
  config.watchdog_timer > 300 ? config.watchdog_timer = 300 : config.watchdog_timer; //max 300s
  config.watchdog_timer <= 0 ? config.watchdog_timer = DEFAULT_WATCHDOG_TIMER : config.watchdog_timer;
  config.battery_safe <= 0 ? config.battery_safe = DEFAULT_BATTERY_SAFE : config.battery_safe;
  config.time_for_broker <= 0 ? config.time_for_broker = DEFAULT_TIME_FOR_BROKER : config.time_for_broker;
  config.time_for_broker > 300 ? config.time_for_broker = 300 : config.time_for_broker; //maksimal 270s
  config.sync_time_interval <= 3 ? config.sync_time_interval = 0 : config.sync_time_interval;
  return true;
}

bool writeConfig(fs::FS &fs, Config &config) {
  fs.remove(CONFIG_FILENAME);
  fs::File file = fs.open(CONFIG_FILENAME, "w");
  if (!file) return false;

  const size_t cap = JSON_OBJECT_SIZE(18) + 400;
  // StaticJsonDocument<cap * 4> doc;
  StaticJsonDocument<cap> doc;
  config.ambilDataInterval <= 0 ? config.ambilDataInterval = 3 : config.ambilDataInterval;
  config.kirimDataInterval <= 0 ? config.kirimDataInterval = 30 : config.kirimDataInterval;
  config.time_for_broker < 40 ? config.time_for_broker = 40 : config.time_for_broker; //minimal 180 detik
  config.time_for_broker > 300 ? config.time_for_broker = 300 : config.time_for_broker; //maksimal 300s
  config.watchdog_timer > 300 ? config.watchdog_timer = 300 : config.watchdog_timer; //max 300s
  config.watchdog_timer <= 0 ? config.watchdog_timer = DEFAULT_WATCHDOG_TIMER : config.watchdog_timer;
  config.SD_attempt > 75 ? config.SD_attempt = 75 : config.SD_attempt;
  config.SD_attempt < 10 ? config.SD_attempt = 10 : config.SD_attempt;
  //config.SD_store_limit < 0 ? config.SD_store_limit = 0 : config.SD_store_limit;
  config.sync_time_interval <= 2 ? config.sync_time_interval = 0 : config.sync_time_interval;

  doc["host"] = config.host;
  doc["port"] = config.port;
  doc["ambilData"] = config.ambilDataInterval;
  doc["kirimData"] = config.kirimDataInterval;
  // doc["lastSetAlarm0"] = config.lastSetAlarm0;
  // doc["lastSetAlarm1"] = config.lastSetAlarm1;
  doc["timezone"] = config.timezone;
  doc["battery_safe"] = config.battery_safe;
  doc["minimum_signal"] = config.minimum_signal;
  doc["time_for_broker"] = config.time_for_broker;
  doc["watchdog_timer"] = config.watchdog_timer;
  doc["useSD"] = config.useSD;
  doc["max_send"] = config.max_send;
  doc["SD_attempt"] = config.SD_attempt;
  doc["SD_store_limit"] = config.SD_store_limit;
  // doc["burst_send_h_on"] = config.burst_send_h_on;
  // doc["burst_send_h_off"] = config.burst_send_h_off;
  doc["cnt_send_from_spiffs"] = config.cnt_send_from_spiffs;
  doc["sync_time_interval"] = config.sync_time_interval;
  doc["log_cnt_auto_restart"] = config.log_cnt_auto_restart;

#ifdef _USE_DEF_SERVER_
  doc["brokerSelector"] = true;
#else
  doc["brokerSelector"] = config.brokerSelector;
#endif

  doc["selected_apn"] = config.selected_apn;

  if (serializeJson(doc, file) == 0) return false;
  file.close();
  return true;
}

bool read_device_info(fs::FS &fs, DeviceInfo &device_info) {
  fs::File file = fs.open(DEVICE_INFO_FILENAME);
  if (!file || file.isDirectory()) {
    return false;
  }
  const size_t cap = JSON_OBJECT_SIZE(4) + 140;
  StaticJsonDocument<cap> doc;
  DeserializationError error = deserializeJson(doc, file);

  if (error) return false;
  //FIELDS JSON-----------------------------------------

  strlcpy(device_info.software_version, doc["software_version"], sizeof(device_info.software_version));
  strlcpy(device_info.my_dn, doc["my_dn"], sizeof(device_info.my_dn));
  strlcpy(device_info.my_location, doc["my_location"], sizeof(device_info.my_location));
  device_info.sd_failure = doc["sd_failure"];
  //FIELDS JSON-----------------------------------------
  file.close();
  return true;
}

bool write_device_info(fs::FS &fs, DeviceInfo &device_info) {
  fs.remove(DEVICE_INFO_FILENAME);

  fs::File file = fs.open(DEVICE_INFO_FILENAME, "w");
  if (!file) return false;

  const size_t cap = JSON_OBJECT_SIZE(4) + 140;
  StaticJsonDocument<cap> doc;
  doc["software_version"] = device_info.software_version;
  doc["my_dn"] = device_info.my_dn;
  doc["my_location"] = device_info.my_location;
  doc["sd_failure"] = device_info.sd_failure;
  //doc["ussd"] = device_info.my_ussd;

  if (serializeJson(doc, file) == 0) return false;

  file.close();
  return true;
}

bool SIM800SleepEnable() {
  Serial2.print("AT+CSCLK=2\r\n");
  delay(500);
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  SIMon = false;
  return false;
}

bool SIM800SleepDisable() {
  Serial2.print("AT\r\n");
  delay(500);
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  Serial2.print("AT+CSCLK=0\r\n");
  delay(500);
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  SIMon = true;
  return true;
}

/*
  void set_burst_hour() {
  uint8_t i = 0;
  uint8_t btn = 3;
  int8_t on_off[2];
  on_off[0] = config.cnt_send_from_spiffs;
  // on_off[1] = config.burst_send_h_off;

  lcd.clear();
  lcd.backlight();

  while (i < 1) {
    lcd.clear();
    lcd.print("On=Off:Disabled");
    lcd.setCursor((i * 8), 1); lcd.print(">"); //sdfsdf
    lcd.setCursor(1, 1); lcd.print("On:");
    // lcd.setCursor(9, 1); lcd.print("Off:");
    lcd.setCursor(3, 1); lcd.print(on_off[0]);
    // lcd.setCursor(12, 1); lcd.print(on_off[1]);

    btn = menu.buttonEvent();
    i == 0 ? lcd.setCursor(0, 3) : lcd.setCursor(0, 12);
    btn == 0 ? on_off[i]-- : on_off[i];
    btn == 1 ? on_off[i]++ : on_off[i];
    on_off[i] > 60 ? on_off[i] = 0 : on_off[i];
    on_off[i] < 0 ? on_off[i] = 60 : on_off[i];
    btn == 2 ? i++ : i ;
    delay(50);
  }

  config.cnt_send_from_spiffs = on_off[0];
  // config.burst_send_h_off = on_off[1];

  if (writeConfig(SPIFFS, config)) {
    lcd.clear();
    lcd.print("NewConfig Saved!");
  }
  else {
    lcd.clear();
    lcd.print("Failed to Save!");
  }
  delay(1000);
  lcd.clear();
  }
*/

bool wait_for_signal(uint16_t second, uint8_t minimumSignal) {
  if (!SIMon) { //jika sim lupa dienable--------
    SIM800SleepDisable();
    lcd.clear();
    lcd.print("WakingUpSIM800..");
    delay(2000);
  }
  uint8_t csq_1 = 0; //Checking signal before sync time--------------
  bool getSignal = true;
  unsigned long timeout = millis();
  while ((csq_1 < minimumSignal) || (csq_1 > 80)) {
    csq_1 = modem.getSignalQuality();
    lcd.clear();
    lcd.printf("Signal:%i [%i]", csq_1, minimumSignal);
    lcd.setCursor(0, 1);
    // lcd.print(apn_name[config.selected_apn]);
    lcd.print(APN);
    if (millis() - timeout > (second * 1000)) { //limit waktu nunggu sinyal adalah 1 menit
      lcd.setCursor(0, 0);
      lcd.print("Timeout!!!");
      getSignal = false;
      break;
    }
    delay(300);
  }
  return getSignal;
}

/*
void cek_apn() {
  SIM800SleepDisable();
  lcd.clear();
  int8_t i = 7;
  int8_t btn = 1;
  bool index_menu = 1;
  while (index_menu == 1) {

    if (btn == 0) { //tombol kiri
      --i < 0 ? i = 7 : i;
    } else if (btn == 1) { //tombol kanan
      ++i > 7 ? i = 0 : i;
    } else if (btn == 2) { //tombol ok
      lcd.clear();
      lcd.print("exit  set  cncl");
      lcd.setCursor(0, 1);
      lcd.print("<-           ->");
      lcd.setCursor(7, 1);
      lcd.write(byte(1));

      while (1) {
        btn = menu.buttonEvent();
        if (btn == 0) { //exit
          index_menu = 0;
          btn = -1;
          lcd.clear();
          lcd.print("To main menu...");
          delay(2000);
          break;
        } else if (btn == 1) { //cancle
          btn = 1;
          lcd.clear();
          lcd.print("To APN menu...");
          delay(2000);
          break;
        } else if (btn == 2) { //OK
          //selected_apn=i;
          lcd.clear();
          lcd.print("Setting new APN");
          lcd.setCursor(0, 1);
          // lcd.print(apn_name[i]);
          lcd.print(APN);
          delay(1000);
          break;
        }
      }
      if (btn == 2) {
        config.selected_apn = i;
        writeConfig(SPIFFS, config);
        modem.restart(); //perbedaan prog ram d

        lcd.clear();
        lcd.print("Connecting...");
        lcd.setCursor(0, 1);
        // lcd.print(apn_name[config.selected_apn]);
        lcd.print(APN);
        // modem.gprsConnect(apn_name[config.selected_apn], apn_usr[config.selected_apn], apn_pswd[config.selected_apn]);
        modem.gprsConnect(APN, APN_USERNAME, APN_PASSWORD);

        if (!modem.isNetworkConnected()) {
          lcd.clear();
          // lcd.print(apn_alias[config.selected_apn]);
          lcd.print(APN);
          lcd.setCursor(0, 1);
          lcd.print("fail!");
          delay(2000);
        }
        if (modem.isNetworkConnected()) {
          uint8_t sig;
          while (1) {
            sig = modem.getSignalQuality();
            lcd.clear();
            // lcd.print(apn_alias[config.selected_apn]);
            lcd.print(APN);
            lcd.setCursor(0, 1);
            lcd.print("Signal:");
            lcd.print(sig);
            if (menu.buttonEvent() > -1) {
              break;
            }
            delay(200);
          }
        }
      }
    }

    if (btn > -1) {
      lcd.clear();
      lcd.print(i);
      lcd.print(".APN:");
      // lcd.print(apn_alias[i]);
      // lcd.setCursor(0, 1);
      // lcd.print(apn_name[i]);
      lcd.print(APN);
    }

    btn = menu.buttonEvent();
    delay(50);
  }
  SIM800SleepEnable();
}
*/

bool deleteFile(fs::FS &fs, String filename) {
  return fs.remove(filename);
}

byte setupModem(const char *broker) { //koneksi ke internet
  startWDT(config.watchdog_timer);

  modem.restart(); //Perbedaan dengan software versi 8d

  Serial.println("Connecting to network (_f/SetupModem)");
  // Serial.println(apn_name[config.selected_apn]);
  // Serial.println(apn_usr[config.selected_apn]);
  // Serial.println(apn_pswd[config.selected_apn]);
  Serial.println(APN);
  Serial.println(APN_USERNAME);
  Serial.println(APN_PASSWORD);
  lcd.clear();
  lcd.print("Setup modem:");
  lcd.setCursor(0, 1);
  // if (!modem.gprsConnect(apn_name[config.selected_apn], apn_usr[config.selected_apn], apn_pswd[config.selected_apn])) {
  if (!modem.gprsConnect(APN, APN_USERNAME, APN_PASSWORD)) {
    Serial.println("fail!");
    lcd.print("Failed!");
    delay(1000);
    lcd.clear();
    return 1;
  }
  if (modem.isNetworkConnected()) {
    Serial.println("Network connected (_f/SetupModem)");
    lcd.print("Connected!");
    delay(1000);
  }
  mqtt.setServer(broker, 1883);
  stopWDT();
  return 0;
}

#ifdef _ACTIVATE_SENSOR_SERIAL1_
void sensor_serial_set(bool flow) {
  if (flow) {
    // sensor_serial1.end();
    sensor_serial1.begin(9600, SERIAL_8N1, 15, 2);
  } else {
    // sensor_serial1.end();
    sensor_serial1.begin(9600, SERIAL_8N1, 25, 26);
  }
}
#endif

//"note":"0X / 1= / 2< / 3> / 4<= / 5>= / 6!="
template <typename T> bool cek_anomali(T rev_val, T val, uint8_t op) {
  if (op == 1) { // ==
    if (val == rev_val) {
      return true;
    }
  } else if (op == 2) { // <
    if (val < rev_val) {
      return true;
    }
  } else if (op == 3) { // >
    if (val > rev_val) {
      return true;
    }
  } else if (op == 4) { // <=
    if (val <= rev_val) {
      return true;
    }
  } else if (op == 5) { //>=
    if (val >= rev_val) {
      return true;
    }
  } else if (op == 6) { //!=
    if (val != rev_val) {
      return true;
    }
  }
  return false;
}

#ifdef _USE_ULTRASONIC_FLOW_METER_
float read_flow(ModbusMaster flow_modbus){
  // sensor_serial1.updateBaudRate(19200)
  float flow_val = 0;
  uint8_t result = 0, datasize = 2; //datasize 2 byte
  flow_modbus.clearTransmitBuffer();
  flow_modbus.clearResponseBuffer();
  result = flow_modbus.readHoldingRegisters(flow_register, datasize); //0 start address for flow data, 2 is total number of register to be rad
  Serial.print("ResultFlow:"); Serial.println(result);
  if (result == flow_modbus.ku8MBSuccess){
    uint8_t j;
    uint16_t buf[datasize];
    Serial.println("Success! Processing...");
    for (j = 0; j < datasize; j++)
    {
      buf[j] = flow_modbus.getResponseBuffer(j);
      Serial.print(buf[j]);
      Serial.print(" ");
    }
    Serial.println("<- done");
    memcpy(&flow_val, &buf, sizeof(float));
    Serial.print("Flow m3/h:");
    Serial.println(flow_val, 6);
    Serial.print("Flow l/s:");
    flow_val = flow_val / 3.6;
    Serial.println(flow_val, 6);
    return flow_val;
  }else{
    Serial.println("Reading Flow Failed!");
    return 0;
  }
  delay(100);
}

float read_kubikasi(ModbusMaster flow_modbus){
  float kubik;
  uint8_t result = 0, datasize = 2; 
  flow_modbus.clearTransmitBuffer();
  flow_modbus.clearResponseBuffer();
  result = flow_modbus.readHoldingRegisters(cubic_register, datasize); //0 start address for flow data, 2 is total number of register to be rad
  Serial.print("ResultKubik:"); Serial.println(result);
  if (result == flow_modbus.ku8MBSuccess){
    uint8_t j;
    uint16_t buf[datasize];
    Serial.println("Success! Processing...");
    for (j = 0; j < datasize; j++)
    {
      buf[j] = flow_modbus.getResponseBuffer(j);
      Serial.print(buf[j]);
      Serial.print(" ");
    }
    Serial.println("<- done");
    memcpy(&kubik, &buf, sizeof(float));
    Serial.print("Volume m3:");
    Serial.println(kubik, 2);
    return kubik;
  }else{
    Serial.println("Reading cubic Failed!");
    return 0;
  }
}
#endif

// StaticJsonDocument<payloadSize> serializeData(Config configs) { //PENGAMBILAN DATA SENSOR-----------
String serializeData(Config configs, bool manual) { //PENGAMBILAN DATA SENSOR-----------
  bool anomali = false;
  Serial.println("Collecting data!");
  lcd.clear();
  lcd.print("Collecting data!");
#ifdef _USE_ULTRASONIC_FLOW_METER_
#ifdef _DYNAMIC_SERIAL_PORT_PIN_
  sensor_serial_set(1);
  delay(100);
#endif
  float flow_value = 0;
  float kubikasi = 0;
#endif
  SIM800SleepDisable();
  uint8_t i, xdata = 10;
  delay(200);
  uint16_t batrei = filterAnalog(3); //kalau step up dinyalakan sebelum ambil ambil data analog ups, nilai ups selalu 0
  int P1 = filterAnalog(0);

  if (rules_exist) {
    float bar_p1 = getPressureValue(0, P1);
    if (rules.oo1 == 7) {
      bar_p1 = pressure_plus_offset(bar_p1, rules.o1);
    } else if (rules.oo1 == 8) {
      bar_p1 = pressure_plus_offset(bar_p1, -rules.o1);
    }
    Serial.print("P1:");
    Serial.println(bar_p1);
    anomali = (anomali || cek_anomali <uint16_t> (rules.p1, bar_p1, rules.op1));
    Serial.println(anomali);
  }

#ifdef _USE_ALL_PRESSURE_
  int P2 = filterAnalog(1);
  int P3 = filterAnalog(2);

  if (rules_exist) {
    float bar_p2 = getPressureValue(0, P2);
    if (rules.oo2 == 7) {
      bar_p2 = pressure_plus_offset(bar_p2, rules.o2);
    } else if (rules.oo2 == 8) {
      bar_p2 = pressure_plus_offset(bar_p2, -rules.o2);
    }
    anomali = (anomali || cek_anomali <uint16_t> (rules.p2, bar_p2, rules.op2)); //jangan lupa konversi tekanannya dulu, karena masih raw adc

    float bar_p3 = getPressureValue(0, P3);
    if (rules.oo3 == 7) {
      bar_p3 = pressure_plus_offset(bar_p3, rules.o3);
    } else if (rules.oo3 == 8) {
      bar_p3 = pressure_plus_offset(bar_p3, -rules.o3);
    }
    anomali = (anomali || cek_anomali <uint16_t> (rules.p3, bar_p3, rules.op3)); //jangan lupa konversi tekanannya dulu, karena masih raw adc
    Serial.print("P2:");
    Serial.println(bar_p2);
    Serial.print("P3:");
    Serial.println(bar_p3);
  }
#endif

#ifdef _USE_A1_FOR_WATER_LEVEL_
  digitalWrite(SW_AWLR_PWR, 1);
  delay(1000);
  float h_awlr = getWaterHeight(1);
  delay(500);
  digitalWrite(SW_AWLR_PWR, 0);
  anomali = (rules_exist && (anomali || cek_anomali <float> (rules.wl, h_awlr, rules.owl)));
#endif

#ifdef _USE_ULTRASONIC_FLOW_METER_
  flow_value = read_flow(flow_modbus4);
  kubikasi = read_kubikasi(flow_modbus4);
  anomali = (rules_exist && (anomali || cek_anomali <float> (rules.fl, flow_value, rules.ofl)));
#endif

#ifdef _USE_PZEM_
#ifdef _DYNAMIC_SERIAL_PORT_PIN_
  sensor_serial_set(0);
  delay(100);
#endif

  Serial.print("Rec PZEM");
  //Data from PZEM--------------------------
  float i1 = pzem1.current();
  isnan(i1) ? i1 = 0 : i1;

  float i2 = pzem2.current();
  isnan(i2) ? i2 = 0 : i2;

  float i3 = pzem3.current();
  isnan(i3) ? i3 = 0 : i3;

  float f1 = pzem1.frequency();
  isnan(f1) ? f1 = 0 : f1;

  float f2 = pzem2.frequency();
  isnan(f2) ? f2 = 0 : f2;

  float f3 = pzem3.frequency();
  isnan(f3) ? f3 = 0 : f3;

  float v1 = pzem1.voltage();
  isnan(v1) ? v1 = 0 : v1;

  float v2 = pzem2.voltage();
  isnan(v2) ? v2 = 0 : v2;

  float v3 = pzem3.voltage();
  isnan(v3) ? v3 = 0 : v3;

  float pf1 = pzem1.pf();
  isnan(pf1) ? pf1 = 0 : pf1;

  float pf2 = pzem2.pf();
  isnan(pf2) ? pf2 = 0 : pf2;

  float pf3 = pzem3.pf();
  isnan(pf3) ? pf3 = 0 : pf3;

  float wh1 = pzem1.energy();
  isnan(wh1) ? wh1 = 0 : wh1;

  float wh2 = pzem2.energy();
  isnan(wh2) ? wh2 = 0 : wh2;

  float wh3 = pzem3.energy();
  isnan(wh3) ? wh3 = 0 : wh3;
  //Data from PZEM--------------------------
  if(rules_exist){
    anomali = (anomali || cek_anomali <float> (rules.v1, v1, rules.ov1));
    anomali = (anomali || cek_anomali <float> (rules.v2, v2, rules.ov2));
    anomali = (anomali || cek_anomali <float> (rules.v3, v3, rules.ov3));

    anomali = (anomali || cek_anomali <float> (rules.i1, i1, rules.oi1));
    anomali = (anomali || cek_anomali <float> (rules.i2, i2, rules.oi2));
    anomali = (anomali || cek_anomali <float> (rules.i3, i3, rules.oi3));

    anomali = (anomali || cek_anomali <float> (rules.c1, pf1, rules.oc1));
    anomali = (anomali || cek_anomali <float> (rules.c2, pf2, rules.oc2));
    anomali = (anomali || cek_anomali <float> (rules.c3, pf3, rules.oc3));
  }
#endif

  if (anomali) {
    badSignal_directSend = true; //untuk pengiriman langsung ketika terjadi anomali
    lcd.clear(); lcd.print("ANOMALI!!!");
    delay(1000);
  }

  uint16_t signal = 0;
  uint8_t sig;
  if (manual) {
    signal = 33;
  } else {
    for (i = 0; i < xdata; i++) {
      sig = modem.getSignalQuality();
      if (sig <= 60) {
        signal += sig;
      }
      delay(200);
    }

    signal = signal / xdata; //rata2 sinyal
  }
  if(anomali){
    signal += 40;
  }

  StaticJsonDocument<payloadSize> doc;
  // doc["date"] = getStringDateTime(configs.timezone);
  //2022/01/03,16:39:10+08
  String dateTime; dateTime.reserve(25);
  dateTime = getStringDateTime(configs.timezone);
  const char* c_dateTime = dateTime.c_str();
  doc["date"] = c_dateTime;
  doc["UPS"] = batrei;
  doc["dfPressure1"] = P1;
#ifdef _USE_ALL_PRESSURE_
  doc["dfPressure2"] = P2;
  doc["dfPressure3"] = P3;
#endif

#ifdef _USE_PZEM_
  doc["V1"] = v1;
  doc["V2"] = v2;
  doc["V3"] = v3;
  doc["I1"] = i1;
  doc["I2"] = i2;
  doc["I3"] = i3;
  doc["PF1"] = pf1;
  doc["PF2"] = pf2;
  doc["PF3"] = pf3;
  doc["Wh1"] = wh1;
  doc["Wh2"] = wh2;
  doc["Wh3"] = wh3;
  doc["f1"] = f1;
  doc["f2"] = f2;
  doc["f3"] = f3;
#endif

#ifdef _USE_ULTRASONIC_FLOW_METER_
  doc["flow"] = flow_value;
  doc["water_h"] = kubikasi;
#endif

#ifdef _USE_A1_FOR_WATER_LEVEL_
  doc["water_h"] = h_awlr;
#endif

  doc["Signal"] = signal;
  /*
    doc["water_h"] = h_water;
    doc["Stanmeter"] = kubikasi;
    doc["Light"] = light; */

  // return doc;
  char cekdata[payloadSize];
  serializeJson(doc, cekdata); //Hasil di monitor : {"date":"2021/08/13,16:18:03+08","UPS":3504,"dfPressure1":1892,"dfPressure2":1955,"dfPressure3":1859}
  String log_result; log_result.reserve(payloadSize + 1);
  log_result = cekdata;
  // Serial.println(log_result);
  Serial.println("====");
  if (anomali) {
    Serial.println("ANOMALI!!!");
  }
  return log_result;
}

String report_text(String& add_text) {
  //{"ans": "BAK2GUYANGAN Penida_B3.6.8j_(5/30|13-14|LBr)-"}
  float bat = getBatteryVoltage();
  char s_bat[6];
  sprintf(s_bat, "%.02fV", bat);
  String buffer; buffer.reserve(102);
  buffer = "{\"ans\":\"";
  if(SPIFFS.exists(DEF_RULES_FLAG)){
    buffer += "R!_";
  }
  if (!SDok()) {
    buffer += "SD";
    if (device_info.sd_failure) {
      buffer += "X";
    } else {
      buffer += "0";
    }
    buffer += "_";
  }
  buffer += device_info.my_location;
  buffer += "_";
  buffer += device_info.software_version;
  buffer += "_";
  buffer += "("; //
  buffer += (String)config.ambilDataInterval;
  buffer += "/";
  buffer += (String)config.kirimDataInterval;
  buffer += "|nxA:";
  // buffer += (String)config.cnt_send_from_spiffs;
  buffer += (String)next_ambil_data;
  buffer += "/nxK:";
  buffer += (String)next_kirim_data;
  buffer += "/ar:";
  buffer += (String)cnt_for_auto_restart;
  // buffer += "|";
  // config.listening ? buffer += "L" : buffer += "l";
  buffer += "|P";
  buffer += (String)pending_aprox;
  buffer += "/S";
  buffer += (String)recorded_aprox;
  buffer += "|";
  buffer += s_bat;
  buffer += ")_";
  buffer += add_text;
  buffer += "\"}";
  return buffer;
}


void update_sd_total_files(bool sent, uint16_t new_total) {
  String total; total.reserve(28);
  String filename; filename.reserve(21);
  if (sent) {
    recorded_aprox = new_total;
    filename = "/total_recorded.txt";
    total = "{\"total_recorded\":";
  } else {
    pending_aprox = new_total;
    filename = "/total_pending.txt";
    total = "{\"total_pending\":";
  }

  total += (String)new_total;
  total += "}";
  const char* c_filename = filename.c_str();
  const char* c_total = total.c_str();
  Serial.printf(" Will save %s:%s\n", c_filename, c_total);
  sdop.writeFile(SD, c_filename, c_total);
  sdop.readFile(SD, c_filename, 1);
}

uint16_t read_sd_approx_sent_pend(bool sent) {
  uint16_t total_approx = 0;
  String filename; filename.reserve(21);
  String index_json; index_json.reserve(15);
  if (sent) {
    filename = "/total_recorded.txt";
    index_json = "total_recorded";
  } else {
    filename = "/total_pending.txt";
    index_json = "total_pending";
  }
  const char* c_filename = filename.c_str();
  const char* c_index_json = index_json.c_str();

  if (SD.exists(c_filename)) {
    Serial.print("--READING "); Serial.println(c_filename);
    String number; number.reserve(30); //{\"total_recorded\":25800}
    number = sdop.readFile(SD, c_filename, 0);
    const char* c_number = number.c_str();
    StaticJsonDocument<45> doc;
    DeserializationError error = deserializeJson(doc, c_number);
    if (error) {
      sent ? total_approx = recorded_aprox : total_approx = pending_aprox;
    } else {
      total_approx = doc[c_index_json].as<uint16_t>();
      Serial.printf("[%s:%i]\n", c_index_json, total_approx);
    }
  } else {
    sent ? total_approx = recorded_aprox : total_approx = pending_aprox;
  }
  return total_approx;
}

// boolean mqttConnect(const char *broker, bool init_wdt, bool mqtt_connected) { //JANGAN LUPA KASI WATCHDOG TIMER DI SINI, KRN BISA SAJA MACET SAAT BACA SD----
boolean mqttConnect(const char *broker, bool init_wdt, bool mqtt_connected, bool sd_halt) { //JANGAN LUPA KASI WATCHDOG TIMER DI SINI, KRN BISA SAJA MACET SAAT BACA SD----
  if (init_wdt) {
    //------------SET WDT--------------JAGA2 KALAU HANG-------------
    startWDT(config.watchdog_timer);
    //------------SET WDT--------------JAGA2 KALAU HANG-------------
    sending_count = 0;
  } else {
    sending_count++;
  }

  if (sending_count > 4) {
    sending_count = 0;
    lcd.clear(); lcd.print("Limit achieved!");
    lcd.setCursor(0, 1); lcd.print("Send later!");
    stopWDT();
    delay(500);
    wait_for_broker = true;
    got_message = false;
    start_time_for_broker = millis();
    return false;
  }

  lcd.backlight();
  lcd.clear();
  lcd.print("Sending process!");
  uint8_t limitSend = config.max_send;
  limitSend > 50 ? limitSend = 50 : limitSend;
  uint16_t jumlah_file_pending = 0;
  uint16_t total_recorded = 0;
  uint16_t total_pending = 0;
  bool useSD = SDok();
  if (sd_halt) {
    useSD = false;
    lcd.clear();
    lcd.print("SD_TMP_DSBLD!");
    delay(1000);
  }

  if (useSD) {
    jumlah_file_pending = sdop.countFile(SD, "/pend", 50);
    if (!(SD.exists("/sent"))) {
      sdop.createDir(SD, "/sent");
    }

    total_recorded = read_sd_approx_sent_pend(1);
    total_pending = read_sd_approx_sent_pend(0);

  } else {
    jumlah_file_pending = 1;
  }
  Serial.printf("Total recorded +- :%u\n", total_recorded);
  Serial.printf("Total pending +- :%u\n", total_pending);
  uint8_t i = 0;
  uint16_t failedPost = 0;
  wait_for_broker = true;
  wait_broker_initiated = true;

  if (jumlah_file_pending == 0) {
    Serial.println("NO FILE TO SEND! _f/mqtt_cnct");
    filePending = false;
    lcd.clear();
    lcd.print("No Pending file!");
    delay(500);
    return false; //jika tidak ada file pending, maka balik-------
  }

  lcd.clear();
  lcd.printf("Pending : %i", jumlah_file_pending);
  lcd.setCursor(0, 1); lcd.print("Connecting MQTT!");
  Serial.print("(_f/mqtt_cnct)Connecting to:");
  Serial.print(broker);
  delay(1000);

  boolean mqtt_status;
  if (mqtt_connected) {
    mqtt_status = true;
  } else {
    mqtt_status = mqtt.connect(DEVICE_ID);
  }

  esp_task_wdt_reset(); //RESET WATCHDOG TIMER-------------RESET DULU DI SINI BIAR ADA TAMBAHAN WAKTU

  if (mqtt_status == false) {
    Serial.println(" fail");
    lcd.clear();
    lcd.print("Failed2Connect!");
    delay(1000);
    return false; //Jangan lanjutkan MQTT konek
  }

  Serial.println(" success");
  String statustxt; statustxt.reserve(10);
  statustxt = "Connected";
  string_buffer = report_text(statustxt);
  const char* status_txt = string_buffer.c_str();
  mqtt.setKeepAlive(15);
  mqtt.publish(STATUS_TOPIC, status_txt); //kasi tau mqtt dulu kalo kita connected

  uint8_t willsend;
  if (jumlah_file_pending > config.max_send) {
    willsend = config.max_send;
  } else {
    willsend = jumlah_file_pending;
  }

  lcd.clear();
  lcd.printf("Send:%i/Pend:%i", willsend, jumlah_file_pending);

  //---------------------ITERASI PENGULANGAN PENGIRIMAN DATA---------------------
  lcd.setCursor(0, 1);
  lcd.print("Sent:");
  for (i = 0; i < willsend; i++) {
    //Update status jumlah yang akan dikirim----------------
    String isi_date = "{\"ans\":\"";
    isi_date += getStringDateTime(config.timezone);
    isi_date += "|sending:";
    isi_date += (String)(willsend - i);
    isi_date += "\"}";
    const char* number_send_status = isi_date.c_str();
    mqtt.publish(STATUS_TOPIC, number_send_status);
    //Update status jumlah yang akan dikirim----------------
    lcd.setCursor(5, 1);
    lcd.print(i + 1); //jumlah file terkirim

    File root = SD.open("/pend");
    File file = root.openNextFile();

    String file_pending; file_pending.reserve(22);
    file_pending = file.name();// /pend/10102359.txt
    String new_name; new_name.reserve(19);
    new_name = "/sent";
    new_name = new_name += file_pending.substring(5, 18);
    const char* nama_file_pending = file_pending.c_str();
    const char* new_name_log = new_name.c_str();

    String isi_file_pending = "";
    lcd.print("-");
    if (useSD) {
      isi_file_pending = sdop.readFile(SD, nama_file_pending, 1);
      Serial.println("from SD");
      lcd.write(byte(3)); //print icon SD card
    } else {
      isi_file_pending = sdop.readFile(SPIFFS, DATA_FILENAME, 1);
      Serial.println("from SPIFFS");
      lcd.print("ESP "); //print icon SD card
    }

    const char* data_toMQTT = isi_file_pending.c_str(); //ori
    Serial.printf("Isi:%s\n", data_toMQTT);
    bool post_result = mqtt.publish(DEFAULT_TOPIC, data_toMQTT); //kirim ke MQTT
    // Serial.print("postResult1:"); Serial.println(post_result);
    if (!post_result) {
      delay(500);
      mqtt.setKeepAlive(8);
      post_result = mqtt.publish(DEFAULT_TOPIC, data_toMQTT); //kirim ke MQTT sekali lagi
    }
    // Serial.print("postResult2:"); Serial.println(post_result);
    if (post_result) { //jika berhasil dikirim ke server, jika gagal biarkan ulang proses dari awal
      esp_task_wdt_reset(); //RESET WATCHDOG TIMER-------------RESET DULU DI SINI BIAR ADA TAMBAHAN WAKTU
      lcd.setCursor(15, 1); lcd.write(byte(1)); //icon panah ke atas

      if (useSD) {
        Serial.printf("SENT: %s \n", nama_file_pending);
        Serial.printf("New name:%s \n", new_name_log);
        bool write_log_SD = false;

        uint8_t move_attempt = 0;
        while (!write_log_SD && (move_attempt < 1)) { //jika gagal simpan ke log luar di SD ulangi sampai 3x
          if (SD.exists(new_name_log)) {
            Serial.print("File is exist in /sent, ");
            if (sdop.deleteFile(SD, nama_file_pending)) { //hapus file di pending
              lcd.setCursor(15, 1); lcd.print("x"); //karakter x artinya berhasil dihapus
            }
            else {
              lcd.setCursor(15, 1); lcd.write(248); //karakter x garis atas
            }
          } else {
            write_log_SD = sdop.renameFile(SD, nama_file_pending, new_name_log);
            if (!write_log_SD) {
              i = willsend;
              lcd.setCursor(15, 1); lcd.write(254); //karakter :O
            } else {
              total_recorded++;
              total_pending--;
              lcd.setCursor(10, 1); lcd.print("rec.");
            }
          }
          move_attempt++;
        }
      } else {
        Serial.println("SENT: Data in SPIFFS!");
      }

      Serial.println("-----");
    } else {
      failedPost++;
      // Serial.print("FailedPost:"); Serial.println(failedPost);
    }
    delay(300);
  }

  if (useSD) {
    update_sd_total_files(1, total_recorded);
    update_sd_total_files(0, total_pending);
    Serial.printf("Recorded aprox:%u\n", recorded_aprox);
    Serial.printf("Pending aprox:%i\n", pending_aprox);
  }

  lcd.clear();
  if (failedPost == 0) {
    filePending = false;
    lcd.print("All data sent!");
    delay(1000);
  } else {
    filePending = true;
    lcd.print("Failed:");
    lcd.print(failedPost);
    delay(1000);
  }

  stopWDT();
  mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Sending DONE!\"}");
  lcd.clear();
  Serial.println("Sending data done!");
  got_message = true;
  start_time_for_broker = millis();
  return mqtt.connected();
}

void setAlarmOne(int intervalSendingData) {
  Config config;
  bool bConfig = readConfig(SPIFFS, config);
  rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth);
  RtcDateTime now = rtc.GetDateTime();
  if (bConfig) {
    writeConfig(SPIFFS, config);
  }
  RtcDateTime alarmTime = now + (intervalSendingData * 60);
  DS3231AlarmOne alarm1(
    alarmTime.Day(),
    alarmTime.Hour(),
    alarmTime.Minute(),
    alarmTime.Second(),
    DS3231AlarmOneControl_HoursMinutesSecondsMatch);
  rtc.SetAlarmOne(alarm1);
  rtc.LatchAlarmsTriggeredFlags();
  internalRTCwakeup((60 * config.ambilDataInterval) + 15);
  next_kirim_data = alarmTime.Minute();
  Serial.println("Reset alarm1");
}

//----------SET ALARM TWO ORIGINAL-----------------
// void setAlarmTwo(int intervalGetData) {
//   Config config;
//   bool bConfig = readConfig(SPIFFS, config);
//   rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth);
//   RtcDateTime now = rtc.GetDateTime();
//   if (bConfig) {
//     writeConfig(SPIFFS, config);
//   }
//   RtcDateTime alarmTime = now + (intervalGetData * 60);
//   DS3231AlarmTwo alarm2(
//     alarmTime.Day(),
//     alarmTime.Hour(),
//     alarmTime.Minute(),
//     DS3231AlarmTwoControl_HoursMinutesDayOfMonthMatch);
//   rtc.SetAlarmTwo(alarm2);
//   rtc.LatchAlarmsTriggeredFlags();
//   internalRTCwakeup((60 * config.ambilDataInterval) + 30);
//   Serial.println("Reset alarm2");
// }
//---------------------------------------------------
void setAlarmTwo(int intervalGetData) {
  Config config;
  bool bConfig = readConfig(SPIFFS, config);
  rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth);
  RtcDateTime now = rtc.GetDateTime();
  if (bConfig) {
    writeConfig(SPIFFS, config);
  }
  uint8_t next_minute = 0, mod_minute = 0;
  mod_minute = now.Minute() % config.ambilDataInterval;
  next_minute = now.Minute() + (config.ambilDataInterval - mod_minute);
  if (next_minute > 59) {
    next_minute = 0;
  }
  // Serial.print("now_minute:");
  // Serial.println(now.Minute());
  // Serial.print("mod_minute:");
  // Serial.println(mod_minute);
  Serial.print("next_minute:");
  Serial.println(next_minute);
  DS3231AlarmTwo alarm2(
    0,
    0,
    next_minute,
    DS3231AlarmTwoControl_MinutesMatch);
  rtc.SetAlarmTwo(alarm2);
  rtc.LatchAlarmsTriggeredFlags();
  internalRTCwakeup((60 * config.ambilDataInterval) + 15);
  next_ambil_data = next_minute;
  Serial.println("Reset alarm2");
  if ((next_kirim_data < next_ambil_data) || (next_ambil_data == 0)) { //mekanisme untuk mencegah alarm 1 terjadi lebih dulu,
    uint8_t dif = 0;                                                   //karena kalau alarm1 mepet maka waktu alarm2 selanjutnya akan bergeser
    if (next_ambil_data == 0) {
      dif = 60 - next_kirim_data;
    } else {
      dif = next_ambil_data - next_kirim_data;
    }

    if (dif <= 1) {
      uint8_t add_t_kirim = 0;
      if (next_ambil_data == 0) {
        add_t_kirim = 60 + 1 - now.Minute();
      } else {
        add_t_kirim = next_ambil_data + 1 - now.Minute(); //sdfsdf; //yg ini menitnya tambah 1 aja biar aman, yg penting alarm 1 belakangan
      }
      setAlarmOne(add_t_kirim);
    }
  }

}

bool cekSD(uint8_t attempt) {
  //------ATEMPT connection to SD card--------------------------------
  digitalWrite(SD_SS, 0); //umpan sinyal slave select

  attempt <= 0 ? attempt = 3 : attempt;
  bool SDcheck = false;
  uint16_t limit_reconnect = 0;
  SDcheck = SD.begin(SD_SS);
  if (!SDcheck) {
    Serial.println("Card Mount Failed");
    lcd.clear();
    lcd.print("SD Failed!!!");
    lcd.setCursor(0, 1);
    lcd.print("Reconnect:");
    lcd.setCursor(11, 0); lcd.print("[");
    lcd.print(attempt); lcd.print("]");

    while (!SDcheck) {
      if (!SDcheck) {
        SDcheck = SD.begin(SD_SS);
        lcd.setCursor(10, 1);
        lcd.print(limit_reconnect);
        lcd.setCursor(13, 1); //lcd.print(limit_attemptSD);
        limit_reconnect++;
        delay(100);
        digitalWrite(SD_SS, 0);
        delay(200);
      }
      if ((limit_reconnect % 10) == 0) {
        digitalWrite(SD_SS, 1);
        delay(500);
        digitalWrite(SD_SS, 0);
        delay(500);
        sdop.cardInfo();
      }

      if (limit_reconnect > attempt) {
        lcd.clear(); lcd.print("  SD FAILED!!!");
        lcd.setCursor(0, 1); lcd.print("Reset Power & SD");
        device_info.sd_failure = true;
        write_device_info(SPIFFS, device_info);
        delay(1000);
        return false;
      }

    }
    digitalWrite(SD_SS, 0);
  }
  return SDcheck;
}

void printSDdisabled() {
  lcd.clear(); lcd.print("SD Disabled!");
}

uint16_t clr_sent_pend(bool sent, uint8_t total_delete, uint8_t auto_delete) {
  if (SDok()) {
    String dir; dir.reserve(6);
    sent ? dir = "/sent" : dir = "/pend";
    const char* c_dir = dir.c_str();
    uint16_t number = sdop.countFile(SD, c_dir, 101);
    lcd.clear(); lcd.print("Files:");
    number > 100 ? lcd.print("100+") : lcd.print(number);

    delay(1000);
    int8_t btn = -1;
    if (auto_delete) {
      btn = 0;
    } else {
      lcd.setCursor(0, 1); lcd.printf("<Clear%i> <No>", total_delete);
      btn = wait_button(1);
    }

    if (btn == 0) {
      lcd.clear();
      lcd.print("Deleting ");
      sent == 1 ? lcd.print("s") : lcd.print("p");
      total_delete == 0 ? lcd.print("all") : lcd.print(total_delete);
      lcd.print("..");
      delay(500);
      uint16_t deleted = sdop.deleteManyFile(SD, c_dir, total_delete);
      if (deleted > 0) {
        lcd.clear(); lcd.printf("Deleted:%i", deleted);
        logCount = 0;
      } else {
        lcd.clear(); lcd.print ("Deleted:0/Failed");
      }
      delay(1000);

      if (sent) {
        int16_t new_recorded_aprox = recorded_aprox - deleted;
        new_recorded_aprox < 0 ? new_recorded_aprox = 0 : new_recorded_aprox;
        update_sd_total_files(1, new_recorded_aprox);
      } else {
        int16_t new_pending_aprox = pending_aprox - deleted;
        new_pending_aprox < 0 ? new_pending_aprox = 0 : new_pending_aprox;
        update_sd_total_files(0, new_pending_aprox);
      }
      return deleted;
    } else if (btn > 0) {
      lcd.clear(); lcd.print("Cancled!");
      delay(1000);
      return 0;
    }
  } else {
    printSDdisabled();
    delay(1000);
    return 0;
  }
  return 1;
}

void sync_time(bool set_alarm) {
  lcd.clear();
  lcd.backlight();
  lcd.home();
  lcd.print("Ajd & Sync time");
  lcd.setCursor(0, 1);
  lcd.print(DEVICE_ID);
  SIM800SleepDisable();
  setupModem(brokerSelectorF(config.brokerSelector));
  lcd.clear();
  lcd.print("Checking Time...");
  uint32_t timeFromServer = syncTimeUnixTime();
  Serial.print("(_f/sync) Time from server::> ");
  Serial.println(timeFromServer);
  if (timeFromServer != 0) {
    setCurrentTime(timeFromServer - (31536000 * 30) + (config.timezone * 3600) - 604800);
    already_sync = true;
    lcd.clear();
    lcd.print("Time from server");
  } else {
    lcd.clear();
    lcd.print("NO time received");
  }

  if (set_alarm) {
    Serial.print("(_f/sync) Set Alarm two to: ");
    Serial.println(config.ambilDataInterval);
    setAlarmTwo(config.ambilDataInterval);
    delay(1000);
    Serial.print("(_f/sync) Set Alarm one to: ");
    Serial.println(config.kirimDataInterval);
    setAlarmOne(config.kirimDataInterval);
  }
}

// void recountPend() {
void recountPend(uint16_t limited) {
  if (SDok()) {
    stopWDT();
    startWDT(600);
    if (!(SD.exists("/pend"))) {
      sdop.createDir(SD, "/pend");
    }
    lcd.clear(); lcd.print("RECOUNT PEND...");
    if (limited > 0) {
      pending_aprox = sdop.countFile(SD, "/pend", limited);
    } else {
      pending_aprox = sdop.countFile(SD, "/pend", 0);
    }
    update_sd_total_files(0, pending_aprox);

    if (pending_aprox > 5000) { //VERSI N----MLIHAT KASUS DI SEMA AGUNG HASIL PEMBACAAN SD NYA TERLALU BESAR PADAHAL ASLINYA HANYA SEDIKIT
      // config.useSD = false; hghjfghf;
      device_info.sd_failure = true;
      // writeConfig(SPIFFS, config);
      write_device_info(SPIFFS, device_info);
      lcd.clear(); lcd.print("Error!");
      lcd.setCursor(0, 1); lcd.printf("/pend:%i", pending_aprox);
    } else {
      lcd.clear(); lcd.print("Done!");
      lcd.setCursor(0, 1); lcd.printf("/pend:%i", pending_aprox);
    }

    stopWDT();
  } else {
    printSDdisabled();
  }
}

void recountSent(uint8_t limited) {
  if (SDok()) {
    stopWDT();
    startWDT(600);
    if (!(SD.exists("/sent"))) {
      sdop.createDir(SD, "/sent");
    }
    lcd.clear(); lcd.print("RECOUNT SENT...");
    if (limited > 0) {
      recorded_aprox = sdop.countFile(SD, "/sent", limited);
    } else {
      recorded_aprox = sdop.countFile(SD, "/sent", 0);
    }
    update_sd_total_files(1, recorded_aprox);
    if (recorded_aprox > 5000) { //VERSI N----MLIHAT KASUS DI SEMA AGUNG
      device_info.sd_failure = true;
      write_device_info(SPIFFS, device_info);
      lcd.clear(); lcd.print("Error!");
      lcd.setCursor(0, 1); lcd.printf("/sent:%i", recorded_aprox);
    } else {
      lcd.clear(); lcd.print("Done!");
      lcd.setCursor(0, 1); lcd.printf("/sent:%i", recorded_aprox);
    }
    stopWDT();
  } else {
    printSDdisabled();
  }
}

void log_toSD(bool manual) { //JANGAN LUPA KASI WATCHDOG TIMER KARENA MUNGKIN SAJA BISA HANG SAAT AKSES SD
  //------------SET WDT--------------JAGA2 KALAU HANG-------------
  startWDT(config.watchdog_timer);
  //------------SET WDT--------------JAGA2 KALAU HANG-------------
  RtcDateTime dt = rtc.GetDateTime();
  uint8_t hour = dt.Hour();
  uint8_t day = dt.Day();
  uint8_t month = dt.Month();
  uint8_t year = dt.Year();
  uint8_t minute = dt.Minute();

  //cek auto sync time
  uint8_t mod_hour;
  if (config.sync_time_interval <= 2) {
    mod_hour = hour;
  } else {
    mod_hour = hour % config.sync_time_interval;
  }

  //Serial.printf("1ModHour:%i, hasSync?%i\n",mod_hour,already_sync);
  if ((mod_hour > 0) && (already_sync)) { //mencegah supaya dalam jam yang sama tidak terus2an sync
    already_sync = false;
  }
  Serial.printf("ModHour:%i, hasSync?%i\n", mod_hour, already_sync);
  lcd.setCursor(0, 1); lcd.printf("sync:%i,mod:%i", already_sync, mod_hour);
  delay(1000);

  // if ((year > 2100) || (month > 12) || (day > 31) || (hour > 23) || (minute > 59) || (config.timezone != DEFAULT_TIMEZONE)) { //sync time jika tanggal tidak wajar
  if ((year > 2050) || (month > 12) || (day > 31) || (hour > 23) || (minute > 59)) { //sync time jika tanggal tidak wajar
    sync_time(0);
    config.brokerSelector = true; //sdfsdf;
    writeConfig(SPIFFS, config);
  }

  if ((config.sync_time_interval > 2) && (!already_sync)) {
    if (mod_hour == 0) {
      config.brokerSelector = true; //sdfsdf;
      sync_time(0);
    }
    delay(500);
  }

  //-------Posisi SERIALIZE_DATA yang baru------------------------
  Serial.println("[LOG2SD]-SerializeData:");
  String dataResult; dataResult.reserve(payloadSize + 1);
  dataResult = serializeData(config, manual);
  const char* data = dataResult.c_str();
  Serial.println(data);
  Serial.println("----");
  delay(500);
  //-------Posisi SERIALIZE_DATA yang baru------------------------

  //cek apakah waktunya untuk disable sd sementara atau tidak
  uint8_t mod_disbl_sd_tmp = 0;
  if (config.cnt_send_from_spiffs == 0) {
    config.cnt_send_from_spiffs = 6;
    writeConfig(SPIFFS, config);
  }
  mod_disbl_sd_tmp = cnt_for_auto_restart % config.cnt_send_from_spiffs;
  Serial.println(mod_disbl_sd_tmp);
  if (mod_disbl_sd_tmp == 0) {
    disableSD_temporary = true;
    lcd.setCursor(0, 1); lcd.print("RecheckSD|");
    delay(1000);
    if (cekSD(8)) {
      if (device_info.sd_failure) {
        device_info.sd_failure = false;
        write_device_info(SPIFFS, device_info);
        setAlarmTwo(config.ambilDataInterval);
      }
    } else {
      if (device_info.sd_failure) {
        device_info.sd_failure = true;
        write_device_info(SPIFFS, device_info);
        setAlarmTwo(config.ambilDataInterval);
      }
    }
  } else {
    disableSD_temporary = false;
  }

  if (SDok() && (!disableSD_temporary)) {
    disableSD_temporary = false;
  } else {
    disableSD_temporary = true;
  }

  // if (config.useSD || device_info.sd_failure || (!disableSD_temporary)) {
  if (!disableSD_temporary) {
    lcd.setCursor(0, 1); lcd.print("CheckingSD|");
    delay(200);
    if (!cekSD(3)) {
      disableSD_temporary = true;
    }
  }

  // disableSD_temporary = false;
  if (!disableSD_temporary) { //cek jumlah pending apakah anomali (65535) atau tidak,
    recountPend(500);
    lcd.clear(); lcd.print("P:");
    lcd.print(pending_aprox);
    delay(800);
    if (pending_aprox >= 500) {
      disableSD_temporary = true;
      clr_sent_pend(0, 50, 1);
    }
    lcd.print("/");
    lcd.print(disableSD_temporary);
    delay(800);

    //------------hitung isi folder sent
    recountSent(200); //untuk counting mandiri oleh alat, recount dibatasi supaya tidak makan waktu
    if ((recorded_aprox > config.SD_store_limit) && (config.SD_store_limit > 0)) { //cek kalo batas penyimpanan data terdeteksi
      if (recorded_aprox >= 200) {
        clr_sent_pend(1, 200, 1); //ngk perlu update recorded aprox
        delay(800);
        lcd.clear();
        lcd.print("OVER_LIMIT!");
      } else {
        recorded_aprox = recorded_aprox - clr_sent_pend(1, (config.max_send * 2), 1);
        delay(800);
        lcd.clear();
        lcd.print("Below 200 OK");
      }
      delay(800);
      lcd.clear();
      lcd.print("New total rec:");
      lcd.setCursor(0, 1); lcd.print(recorded_aprox);
      delay(800);
      lcd.clear();
    }
  }
  //-------Posisi SERIALIZE_DATA sebelumnya------------------------
  //-------Posisi SERIALIZE_DATA sebelumnya------------------------
  if (!disableSD_temporary) {
    //-----------Generate nama file baru----------
    char file_name[19]; // /pend/xxxxxxxx.txt
    const char* fname = file_name;
    //snprintf_P(file_name, sizeof(file_name), PSTR("/pend/%04u%02u%02u_%02u%02u_%02u.txt"),
    snprintf_P(file_name, sizeof(file_name), PSTR("/pend/%02u%02u%02u%02u.txt"),
               dt.Month(),
               dt.Day(),
               dt.Hour(),
               dt.Minute()
              );
    Serial.println(fname);
    lcd.clear();
    lcd.backlight();

    if (!(SD.exists(fname))) {
      sdop.writeFile(SD, fname, data); //Buat file baru dan Tulis data ke SD folder pending
      pending_aprox++;
      update_sd_total_files(0, pending_aprox);
      lcd.print("Recorded2SD!");
      String isi_fileSD = sdop.readFile(SD, fname, 0); //Baca isi file di SD
      const char* isi_from_string = isi_fileSD.c_str(); //membuat const char* dari string!!!!!
      Serial.printf("Saved2pending: %s \n", isi_from_string);
    } else {
      lcd.print("Exist!");
      Serial.printf("%s EXIST!", fname);
    }
    !filePending ? filePending = true : filePending ; //kalo ngk ada file pending, jadikan status ada file pending
    delay(500);
  } else {

    // if (!(SPIFFS.exists(DATA_FILENAME))) {
    if (SPIFFS.exists(DATA_FILENAME)) {
      sdop.deleteFile(SPIFFS, DATA_FILENAME);
      lcd.clear();
      lcd.print("SPIFFS Deleted");
      delay(1000);
    }
    sdop.writeFile(SPIFFS, DATA_FILENAME, data);
    lcd.clear();
    lcd.print("Rec2SPIFFS!");
    String isi_dataSPIFFS = sdop.readFile(SPIFFS, DATA_FILENAME, 0); //Baca isi file di SD
    const char* isi_from_string = isi_dataSPIFFS.c_str(); //membuat const char* dari string!!!!!
    Serial.printf("Saved2SPIFFS: %s \n", isi_from_string);
    delay(500);
  }

  esp_task_wdt_reset(); //RESET WATCHDOG TIMER-------------
  stopWDT();
  lcd.clear();
  lcd.print("Checking status!");
  //cek burst send mode--------------------
  // cek_burst_mode(config.burst_send_h_on, config.burst_send_h_off); //lansung seting burstMode
  //cek burst send mode--------------------

  if (!disableSD_temporary) {
    //Jika jumlah data pending lebih banyak dari jumlah data yang seharusnya, langsung kirim, kalau tidak nanti numpuk banyak
    Serial.println("inLOG2SD!");
    Serial.printf("Kirim:%i, Ambil:%i\n", config.kirimDataInterval, config.ambilDataInterval);
    config.kirimDataInterval == 0 ? config.kirimDataInterval = 30 : config.kirimDataInterval;
    config.ambilDataInterval == 0 ? config.ambilDataInterval = 5 : config.ambilDataInterval;
    uint8_t max_pending_data = (config.kirimDataInterval / config.ambilDataInterval);
    uint8_t aprox_pending_data = sdop.countFile(SD, "/pend", (max_pending_data + 5 ));
    if (aprox_pending_data > max_pending_data) {
      badSignal_directSend = true;
      lcd.clear(); lcd.print("More than");
      lcd.setCursor(0, 1); lcd.print ("it should!!!");
    }
  }

  if (disableSD_temporary) { //kalau burst mode aktif atau tidak menggunakan SD card, jadi langsung kirim
    badSignal_directSend = true; //pinjam bad signal untuk memicu pengiriman saat burst mode
    Serial.println("burst/notuseSD/DsblSDtmp");
  }
  //Cek apakah ada hutang kirim atau tidak-----------------
  bool okToSend = false;
  uint8_t ix = 0;
  for (ix = 0; ix < 5; ix++) {
    cek_battery_safe();
    if (BatterySafeMode) {
      lcd.clear();
      lcd.print("SAFE MODE ON!!!");
      Serial.println("SAFEMODE!");
      delay(80);
    } else {
      lcd.clear();
      lcd.print("BAT OK!");
    }
  }
  lcd.clear();
  lcd.print("Starting SIM800");
  if (badSignal_directSend && (!BatterySafeMode)) {
    Serial.println("Prv.badsig/drctSend_f/Log");
    SIM800SleepDisable();
    delay(1000);
    if (!setupModem(brokerSelectorF(config.brokerSelector))) { //1 = failed, 0=ok
      okToSend = true;
      badSignal_directSend = false;
    }
  }

  if (okToSend) { //jika sinyal !badSignal_directSend dari alarm one atau file owe ada, dan sudah bisa konek internet, maka langsung kirim data hutang
    logCount = 0;
    Serial.println("SigOK_Sending_f/Log");
    SIMon = 1;
    if (wait_for_signal(60, config.minimum_signal)) { //tunggu sinyal 8 hingga 1 menit
      mqtt.setBufferSize(1024);
      Serial.print("BROKER:");
      Serial.println(config.brokerSelector);
      //------------------------CUKUP UPAYA KIRIM SEKALI SAJA, Kalau gagal nanti coba lagi-----------------
      bool sent = mqttConnect(brokerSelectorF(config.brokerSelector), 1, 0, disableSD_temporary); //langsung kirim data
      if (!sent) {
        Serial.print("DiscntMQTT&GPRS_f/Log");

        String statustxt; statustxt.reserve(10);
        statustxt = "Discnnctd";
        string_buffer = report_text(statustxt);
        const char* status_txt = string_buffer.c_str();
        mqtt.publish(STATUS_TOPIC, status_txt);
        mqtt.disconnect();
        if (!modem.gprsDisconnect()) {
          Serial.println("failed!_f/Log");
        }
        Serial.println("OK!_f/Log");
        delay(200);
        SIM800SleepEnable();
        wait_for_broker = false; //gak usah komunikasi
        delay(200);
      }
      //------------------------CUKUP UPAYA KIRIM SEKALI SAJA, Kalau gagal nanti coba lagi-----------------
    }
  }

  cnt_for_auto_restart++;
  if (((cnt_for_auto_restart % 24) == 0) && (config.log_cnt_auto_restart > 0)) { //mekanisme auto hard reset ketika tidak ada sinyal selama periode tertentu
    if (!(wait_for_signal(30, config.minimum_signal))) {
      lcd.clear(); lcd.print("Hard RST!");
      pinMode(SIM_RING, OUTPUT);
      digitalWrite(SIM_RING, 0);
      delay(10000);
    }
  }
  // if ((cnt_for_auto_restart >= config.log_cnt_auto_restart) && (config.log_cnt_auto_restart > 0)) {
    if (cnt_for_auto_restart >= (1440 / config.ambilDataInterval)) { //hard reset 1x 24 jam tapi random
      lcd.clear(); lcd.print("Hard RST!");
      pinMode(SIM_RING, OUTPUT);
      digitalWrite(SIM_RING, 0);
      delay(10000);
    // stopWDT();
    // lcd.clear(); lcd.print("Self Restart!");
    // Serial.println("SELF_RESTART!");
    // startWDT(5);
    // while (1) {}
  }
  delay(400);
}



void testKirim() { //Fungsi tes kirim----------------
  lcd.clear();
  lcd.home();
  lcd.print("Test kirim...");
  badSignal_directSend = true;
  SIM800SleepDisable();
  mqtt.setBufferSize(1024);
  log_toSD(true);
  // mqttConnect(brokerSelectorF(config.brokerSelector), 1, 0, disableSD_temporary);
}

String config_and_devinfo_status(bool cmd) {
  //{"ans":[{"date":"2021/10/14,05:57:50+07"},{"host":"34.87.0.141","port":1883,"ambilData":8,"kirimData":32,"lastSetAlarm0":687505630,"lastSetAlarm1":687506103,"timezone":7,"safe_h_on":0,"safe_h_off":0,"battery_safe":11.2,"minimum_signal":5,"time_for_broker":180,"watchdog_timer":360,"useSD":true,"max_send":25,"SD_attempt":75,"burst_send_h_on":18,"burst_send_h_off":15,"brokerSelector":true,"selected_apn":0},{"software_version":"B3.6.8d","my_dn":"DN83","my_location":"Gamping Sleman","sd_failure":false}]}
  //reserve 750
  String isi_status; isi_status.reserve(ANS_STATUS_LENGTH);
  isi_status = "{\"ans\":[{\"date\":\"";
  isi_status += getStringDateTime(config.timezone);
  isi_status += "\"},";
  if (cmd) {
    isi_status += sdop.readFile(SPIFFS, CONFIG_FILENAME, 0); //Baca isi file di SD
    isi_status += ",";
    isi_status += sdop.readFile(SPIFFS, DEVICE_INFO_FILENAME, 0);
  } else {
    isi_status += "{\"";
    isi_status += "tz:";
    isi_status += (String)config.timezone;
    isi_status += "|t4b:";
    isi_status += (String)config.time_for_broker;
    isi_status += "|uSD:";
    isi_status += (String)config.useSD;
    isi_status += "|SDl:";
    isi_status += (String)config.SD_store_limit;
    isi_status += "|spf:";
    isi_status += (String)config.cnt_send_from_spiffs;
    isi_status += "|syc:";
    isi_status += (String)config.sync_time_interval;
    isi_status += "|rst:";
    isi_status += (String)config.log_cnt_auto_restart;
    isi_status += "\"}";
  }
  isi_status += "]}";
  //Serial.println(isi_status);
  return isi_status;
}

uint16_t get_int_json_message(String& message, int16_t if_error) {
  uint16_t result_value;
  const char* source = message.c_str();
  StaticJsonDocument<100> doc;
  DeserializationError error = deserializeJson(doc, source);
  if (error) {
    result_value = if_error;
  } else {
    result_value = doc["value"].as<int>();
  }
  return result_value;
}

void close_communication() {
  start_time_for_broker = 0;
  wait_for_broker = false;
  wait_broker_initiated = true;
  time_broker = 0;
  cmd_message = cmd_close;
}

void write_custom_config_from_broker(String& fname, String& new_config) {
  const char* c_file_name = fname.c_str();
  const char* c_new_config = new_config.c_str();
  mqtt.publish(STATUS_TOPIC, "{\"ans\":\"updating_custom_config\"}");

  bool write_ok = sdop.writeFile(SD, c_file_name, c_new_config);
  if (write_ok) {
    Serial.printf("%s updated!\n", c_file_name);
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"UPDATED!\"}");
  } else {
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"FAILED!\"}");
  }
  delay(2000);
  String new_custom_config; new_custom_config.reserve(75);
  new_custom_config = sdop.readFile(SD, c_file_name, 0);
  c_new_config = new_custom_config.c_str();
  mqtt.publish(STATUS_TOPIC, c_new_config);
}

bool check_similar_cmd(int8_t cmd, uint8_t limit_similar_msg) {
  prev_cmd_code == cmd ? similar_message ++ : similar_message = 0;
  limit_similar_msg == 0 ? limit_similar_msg = LIMIT_SIMILAR_MESSAGE : limit_similar_msg;
  if (similar_message >= limit_similar_msg) {
    got_message = true;
    return true;
  } else {
    if ((prev_cmd_code == code_wait_me) && (cmd != cmd_wait_me)) {
      cmd_message = cmd_ready;
    }
    prev_cmd_code = cmd;
    got_message = false;
    return false;
  }
}

void parsing_message(String& message) {
  //-------------------------COMMAND-------
  if (prev_cmd_code != code_wait_me) {
    cmd_message = cmd_ready;
  }

  String find; find.reserve(20);
  find = "host";
  Serial.println(message);
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_host, 2)) {
      return;
    }
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"updating_device_config\"}");
    const char* new_config = message.c_str();
    sdop.writeFile(SPIFFS, CONFIG_FILENAME, new_config);
    readConfig(SPIFFS, config);
    Serial.println("New config:");
    sdop.readFile(SPIFFS, CONFIG_FILENAME, 0);
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"device_config_updated\"}");
    return;
  }

  find = "software";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_software, 2)) {
      return;
    }
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"updating_device_info\"}");
    const char* new_config = message.c_str();
    sdop.writeFile(SPIFFS, DEVICE_INFO_FILENAME, new_config);
    read_device_info(SPIFFS, device_info);
    Serial.println("New Device Info:");
    sdop.readFile(SPIFFS, DEVICE_INFO_FILENAME, 0);
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"device_info_updated\"}");
    return;
  }

  find = "tes_kirim";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_tes_kirim, 4)) {
      return;
    }
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"preparing_tes_kirim\"}");
    mqtt.disconnect();
    testKirim();
    wait_broker_initiated = true;
    return;
  }

  find = "status";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_status, 2)) {
      return;
    }
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"preparing_status\"}");
    String status; status.reserve(ANS_STATUS_LENGTH);
    status = config_and_devinfo_status(1);
    const char* status_char = status.c_str();
    Serial.println(status_char);
    mqtt.setKeepAlive(15);
    mqtt.publish(STATUS_TOPIC, status_char);
    Serial.println("PUBLISHED!");
    return;
  }

  find = "sync_time";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_sync_time, 2)) {
      return;
    }
    cmd_message = cmd_sync_time;
    return;
  }

  find = "battery";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_battery, 2)) {
      return;
    }
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Checking battery voltage\"}");
    float v = getBatteryVoltage();
    char vc[25];
    sprintf(vc, "{\"ans\":\"Battery:%.02fV\"}", v);
    mqtt.publish(STATUS_TOPIC, vc);
    return;
  }

  find = "imei";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_imei, 2)) {
      return;
    }
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Checking IMEI\"}");
    String imei; imei.reserve(29);
    imei = "{\"ans\":\"IMEI:" ;
    imei += modem.getIMEI();
    imei += "\"}";
    const char* cimei = imei.c_str();
    Serial.println(imei);
    mqtt.publish(STATUS_TOPIC, cimei);
    return;
  }

  find = "signal";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_signal, 0)) {
      return;
    }
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Checking signal\"}");
    uint8_t i = 0, sig = 0;
    for (i = 0; i < 10; i++) {
      sig = modem.getSignalQuality();
      delay(100);
    }
    char sc[20];
    sprintf(sc, "{\"ans\":\"Signal:%i\"}", sig);
    Serial.println(sc);
    mqtt.publish(STATUS_TOPIC, sc);
    return;
  }

  find = "sd_clr_pend";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_sd_clr_pend, 0)) {
      return;
    }
    if (SDok()) {
      cmd_message = cmd_clear_pend;
    }
    return;
  }

  find = "sd_clr_sent";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_sd_clr_sent, 0)) {
      return;
    }
    if (SDok()) {
      cmd_message = cmd_clear_sent;
    }
    return;
  }

  find = "wait_me";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_wait_me, 2)) {
      return;
    }
    if (cmd_message != cmd_wait_me) {
      cmd_message = cmd_wait_me;
      int_limit_wait_me = limit_wait_me / 18;
    }
    return;
  }

  find = "set_limit_waitme";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_set_limit_waitme, 3)) {
      return;
    }
    limit_wait_me = get_int_json_message(message, 1200);
    int_limit_wait_me = limit_wait_me / 18;
    String newlimit; newlimit.reserve(36);
    newlimit = "{\"ans\":\"New limit waitme:" ;
    newlimit += (String)limit_wait_me;
    newlimit += "\"}";
    const char* c_newlimit = newlimit.c_str();
    Serial.println(newlimit);
    mqtt.publish(STATUS_TOPIC, c_newlimit);
    return;
  }

  find = "sd_count_pend";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_sd_count_pend, 0)) {
      return;
    }
    if (SDok()) {
      cmd_message = cmd_count_pend;
    }
    return;
  }

  find = "sd_count_sent";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_sd_count_sent, 0)) {
      return;
    }
    if (SDok()) {
      cmd_message = cmd_count_sent;
    }
    return;
  }

  find = "recount_sent";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_recount_sent, 0)) {
      return;
    }
    recount_sent = true;
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Recount SEND is set!\"}");
    delay(1000);
    return;
  }

  find = "cancle_recount_sent";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_cancle_recount_sent, 0)) {
      return;
    }
    recount_sent = false;
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Recount SEND is CANCLED!\"}");
    delay(1000);
    return;
  }

  find = "recount_pend";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_recount_pend, 0)) {
      return;
    }
    recount_pend = true;
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Recount PEND is set!\"}");
    delay(1000);
    return;
  }

  find = "cancle_recount_pend";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_cancle_recount_pend, 0)) {
      return;
    }
    recount_pend = false;
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Recount PEND is CANCLED!\"}");
    delay(1000);
    return;
  }


  find = "apply_def_settings";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_apply_def_settings, 2)) {
      return;
    }
    cmd_message = cmd_def_settings;
    return;
  }

  find = "set_sd_limit_access";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_set_sd_limit_access, 4)) {
      return;
    }
    sd_limit_access = get_int_json_message(message, -1);
    if (sd_limit_access == -1) {
      mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Error deseraializing value! number=max_send\"}");
      sd_limit_access = config.max_send;
    } else {
      String msg; msg.reserve(20);
      msg = "{\"ans\":\"Number:";
      msg += (String)sd_limit_access;
      msg += "\"}";
      const char* number = msg.c_str();
      mqtt.publish(STATUS_TOPIC, number);
    }
    return;
  }
  //-----------------------------writing new custom config------------- /rules.txt
  find = "/rules.txt";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_write_rules, 3)) {
      return;
    }
    write_new_rules_from_broker(find, message);
    return;
  }

  find = "print_rules";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_print_rules, 2)) {
      return;
    }
    String rules_txt; rules_txt.reserve(380);
    rules_txt = sdop.readFile(SPIFFS, RULES_FILENAME, 0);
    const char* c_rules_txt = rules_txt.c_str();
    mqtt.publish(STATUS_TOPIC, c_rules_txt);
    return;
  }
  //-----------------------------writing new custom config-------------

  find = "recheck_sd";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_recheck_sd, 3)) {
      return;
    }
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Rechecking SD (42atmp)\"}");
    if (cekSD(42)) {
      mqtt.publish(STATUS_TOPIC, "{\"ans\":\"SD CONNECTED!\"}");
    } else {
      mqtt.publish(STATUS_TOPIC, "{\"ans\":\"SD FAILED!\"}");
    }
  }

  find = "restart_device";
  if (search_word(find, message) > -1) {
    stopWDT();
    lcd.clear(); lcd.print("CMD_RESTART!");
    Serial.println("CMD_RESTART!");
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Will restart in 10s!\"}");
    startWDT(10);
    while (1) {

    }
  }

  find = "close_cmd";
  if (search_word(find, message) > -1) {
    if (check_similar_cmd(code_close_cmd, 1)) {
      return;
    }
    close_communication();
    got_message = true;
    return;
  }

  if (prev_cmd_code != code_wait_me) {
    prev_cmd_code = code_not_found;
    mqtt.publish(STATUS_TOPIC, "{\"ans\":\"CMD NOT FOUND!\"}");
  }
  return;
}


void mqttCallback(char* topic, byte* payload, unsigned int len) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  uint16_t i = 0;
  String message; message.reserve(800);
  for (i = 0; i < len; i++) { //construct a string from byte payload from broker
    message += char(payload[i]);
  }
  parsing_message(message);
  if (got_message) { //diubah dari "close_cmd" untuk selesai dari koneksi broker
    got_message = false;
  } else {
    got_message = true;
  }
}

void cek_spiffs() {
  uint8_t totalFile = 0;
  uint16_t tByte = SPIFFS.totalBytes();
  Serial.println(tByte);
  lcd.clear(); lcd.backlight();
  lcd.printf("All:%i", tByte);
  uint16_t uByte = SPIFFS.usedBytes();
  Serial.println(uByte);
  lcd.setCursor(0, 1);
  lcd.printf("Use:%i", uByte);
  File root = SPIFFS.open("/");

  File file = root.openNextFile();
  while (file) {
    Serial.print("FILE: ");
    Serial.println(file.name());
    totalFile++;
    file = root.openNextFile();
  }
  root.close();
  lcd.setCursor(9, 1); lcd.printf("f:%i", totalFile);
  wait_button(1);
}

void set_battery_safe() {
  float v_baterai;
  int8_t btn = -1;
  float bat = config.battery_safe;

  while (btn != 2) {
    v_baterai = getBatteryVoltage();
    lcd.clear();
    lcd.printf("Battery:%f", v_baterai);
    btn = menu.buttonEvent();
    btn == 0 ? bat -= 0.1f : bat;
    btn == 1 ? bat += 0.1f : bat;
    bat > 14.0 ? bat = 14.0 : bat;
    bat < 10.8 ? bat = 10.8 : bat;
    lcd.setCursor(0, 1); lcd.printf(">Safe on:%f", bat);
    delay(100);
  }
  config.battery_safe = bat;
  if (writeConfig(SPIFFS, config)) {
    lcd.clear();
    lcd.print("NewConfig Saved!");
  }
  else {
    lcd.clear();
    lcd.print("Failed to Save!");
  }
  delay(1000);
  lcd.clear();
}


void minimum_signal() {
  int8_t btn = -1;
  int8_t min_sig = config.minimum_signal;
  while (btn != 2) {
    lcd.clear();
    lcd.printf("Signal:%i", modem.getSignalQuality());
    btn = menu.buttonEvent();
    btn == 0 ? min_sig-- : min_sig;
    btn == 1 ? min_sig++ : min_sig;
    min_sig > 30 ? min_sig = 0 : min_sig;
    min_sig < 0 ? min_sig = 30 : min_sig;
    lcd.setCursor(0, 1); lcd.printf(">Min_Sig:%i", min_sig);
    delay(100);
  }
  config.minimum_signal = min_sig;
  if (writeConfig(SPIFFS, config)) {
    lcd.clear();
    lcd.print("NewConfig Saved!");
  }
  else {
    lcd.clear();
    lcd.print("Failed to Save!");
  }
  delay(1000);
  lcd.clear();
}

void cek_data_sd() {
  lcd.print("Result:"); lcd.setCursor(0, 1);
  lcd.printf("pnd:%i ", pending_aprox);
  lcd.printf("rec:%i", recorded_aprox);
  wait_button(1);
}

bool begin_spiffs() {
  esp_task_wdt_init(300, true); //enable panic so ESP32 restarts limit 5 menit jika terjadi kegagalan koneksi SD
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  lcd.clear();
  lcd.backlight();
  lcd.print("Starting SPIFFS!");
  bool status = true;
  uint8_t i = 0;
  //------------START SPIFFS--------------------------------
  if (!SPIFFS.begin(true)) {
    lcd.setCursor(0, 1);
    lcd.printf("Error! %i", i);

    delay(3000);
  }
  if (!status) { //jika gagal, kontak mqtt bahwa spiffs error-------
    esp_task_wdt_delete(NULL); //unsubscribe wdt------
    esp_task_wdt_deinit(); //deinit wdt----------
    return false;
  } else {
    lcd.setCursor(0, 1);
    lcd.print("Success!!!");
    delay(300);
    lcd.clear();
  }
  esp_task_wdt_delete(NULL); //unsubscribe wdt------
  esp_task_wdt_deinit(); //deinit wdt----------
  delay(300);
  return status;
  //-----------------------------------------
}

void setDefSettings() {
  lcd.clear(); lcd.print("Reset config...");
  String myloc; myloc.reserve(40);
  myloc = DEVICE_LOCATION;
  String mydn; mydn.reserve(12);
  mydn = DEVICE_ID;
  const char *c_myloc = myloc.c_str();
  const char *c_mydn = mydn.c_str();
  Serial.println(c_myloc);

  config.time_for_broker = 90;
  config.watchdog_timer = 210;
  config.SD_attempt = 15;
  config.max_send = 15;
  config.cnt_send_from_spiffs = 6; // setiap 6 kali pengambilan data, alat akan disable temporary SD dan mengirimkan data dari SPIFFS, sehingga bisa listening k broker

  config.SD_store_limit = 100; //(3hari)
  config.ambilDataInterval = 5;
  config.kirimDataInterval = 30;
  config.battery_safe = DEFAULT_BATTERY_SAFE;
  config.minimum_signal = 5;
  config.sync_time_interval = DEFAULT_SYNCTIME_INTERVAL; // sync di jam 12 malam saja sekali
  config.log_cnt_auto_restart = 240;

  strlcpy(device_info.software_version, SOFTWARE_VERSION, (strlen(SOFTWARE_VERSION) + 1));
  strlcpy(device_info.my_dn, c_mydn, (strlen(c_mydn) + 1));
  strncpy(device_info.my_location, c_myloc, (strlen(c_myloc) + 1));

  // strlcpy(device_info.my_dn, DEVICE_ID, 14);
  writeConfig(SPIFFS, config);
  write_device_info(SPIFFS, device_info);
  read_device_info(SPIFFS, device_info);
  setAlarmTwo(config.ambilDataInterval);
  Serial.println("AFTERDEFF:");
  Serial.println(device_info.software_version);
  Serial.println(device_info.my_dn);
  Serial.println(device_info.my_location);
  lcd.clear(); lcd.print("Done!");
  delay(500);
}

void update_config_report() {
  bool bUpdateConfig = writeConfig(SPIFFS, config);
  lcd.clear(); lcd.print("Save config:");
  lcd.setCursor(0, 1);
  if (!bUpdateConfig) {
    lcd.print("Error!");
  } else {
    lcd.print("Success!");
  }
}

void write_normal_config() {
  String normal_config; normal_config.reserve(75);
  //{"ans":"/config_normal.txt","ambil":4,"kirim":12,"time_for_broker":150}
  normal_config = "{\"ans\":\"/config_normal.txt\",\"ambil\":";
  normal_config += (String)config.ambilDataInterval;
  normal_config += ",\"kirim\":";
  normal_config += (String)config.kirimDataInterval;
  normal_config += ",\"time_for_broker\":";
  normal_config += (String)config.time_for_broker;
  normal_config += "}";
  const char* c_normal_config = normal_config.c_str();
  sdop.writeFile(SD, "/config_normal.txt", c_normal_config);
  sdop.readFile(SD, "/config_normal.txt", 1);
}

void setup() {
  string_buffer.reserve(100);
  count_wait_me = 0;
  prev_cmd_code = -1;
  sd_limit_access = 25;
  similar_message = 0;
  cmd_message = cmd_ready;
  pinMode(SD_SS, OUTPUT);
  pinMode(SIM_RING, INPUT_PULLUP);
  digitalWrite(SD_SS, 0);

#ifdef _USE_A1_FOR_WATER_LEVEL_
  pinMode(SW_AWLR_PWR, OUTPUT);
  digitalWrite(SW_AWLR_PWR, 0);
#endif

#ifndef _FOR_WATER
  // digitalWrite(27, 0);
#endif

  #pragma region serial_begin_for_addon
#ifdef _USE_SLAVE_
  slave_serial.begin(9600);
#endif

#ifdef _ACTIVATE_SENSOR_SERIAL1_
  // #define sensor_serial1 Serial1
  // sensor_serial1.begin(9600, SERIAL_8N1, 15, 2);
#ifdef _DYNAMIC_SERIAL_PORT_PIN_
  sensor_serial_set(1);
#endif
#endif

#ifdef _USE_PZEM_
#ifdef _DIRECT_PZEM_
  // pzem_serial.begin(9600, SERIAL_8N1, 15, 2);
#endif
#endif

#ifdef _USE_ULTRASONIC_FLOW_METER_
  // flow_serial.begin(9600, SERIAL_8N1, 15, 2);
  flow_modbus4.begin(flow4_addr, sensor_serial1);
  #ifdef _USE_FLOW_ID_5_
  flow_modbus5.begin(flow5_addr, sensor_serial1);
  #endif
  #ifdef _USE_FLOW_ID_6_
  flow_modbus6.begin(flow6_addr, sensor_serial1);
  #endif
#endif
#pragma endregion serial_begin_for_addon

  Serial.begin(BAUDRATE);
  Serial2.begin(GSM_BAUDRATE, SERIAL_8N1);

  Wire.begin(SDA_PIN, SCK_PIN);
  Wire.setClock(50000L);
  btStop(); // turn off bluetooth
  WiFi.mode(WIFI_MODE_NULL); // turn of wifi
  ADC0.begin();
  ADC1.begin();
  rtc.Begin();
  rtc.Enable32kHzPin(false);
  menu.buttonBegin();
  lcd.init();
  delay(100);
  print_wakeup_reason();

  uint32_t scl = Wire.getClock();
  Serial.printf("I2C_SCL:%i\n", scl);

  //buat char baru untuk LCD----------
  lcd.createChar(0, Lock);
  lcd.createChar(1, enter);
  lcd.createChar(2, safe);
  lcd.createChar(3, sd);
  lcd.createChar(4, electric);
  //buat char baru untuk LCD----------

  lcd.backlight();
  lcd.clear();
  lcd.print("REBOOT!");
  delay(1000);
  lcd.clear();
  //--------------------------------END SETUP DASAR------------------------------

  if (!SPIFFS.begin(true)) {
    lcd.setCursor(0, 1);
    lcd.print("SPIFFS Error!");
    delay(2000);
  }
  //---------------------------------READ CONFIG----------------------------
  bool bConfig = readConfig(SPIFFS, config); //Baca konfigurasi
  if (!bConfig) {
    Serial.println("Config file not found ! (_f/Setup)");
    Config setupConfig;
#ifdef _USE_DEF_SERVER_
    strlcpy(setupConfig.host, brokerProduction, sizeof(setupConfig.host));
#else
    // strlcpy(setupConfig.host, brokerStaging, sizeof(setupConfig.host));
    strncpy(setupConfig.host, brokerProduction, (strlen(brokerProduction) + 1));
#endif
    setupConfig.port = 1883;
    setupConfig.kirimDataInterval = 30;
    setupConfig.ambilDataInterval = 5;
    setupConfig.timezone = DEFAULT_TIMEZONE;
    setupConfig.brokerSelector = true;
    setupConfig.battery_safe = DEFAULT_BATTERY_SAFE;
    setupConfig.selected_apn = 0;
    setupConfig.cnt_send_from_spiffs = 6;
    setupConfig.minimum_signal = 7;
    setupConfig.time_for_broker = DEFAULT_TIME_FOR_BROKER;
    setupConfig.watchdog_timer = DEFAULT_WATCHDOG_TIMER;
    setupConfig.max_send = 8;
    setupConfig.useSD = true;
    setupConfig.SD_attempt = 15;
    setupConfig.log_cnt_auto_restart = 260;
    setupConfig.sync_time_interval = 8;

    bool bSetupConfig = writeConfig(SPIFFS, setupConfig);
    if (!bSetupConfig) {
      Serial.println("Failed to write config (_f/Setup)");
    } else {
      bConfig = readConfig(SPIFFS, config);
      if (!bConfig) {
        Serial.println("Failed load config after setup (_f/Setup)");
      }
    }
  }

  if (config.SD_attempt == 0) {
    config.SD_attempt = 25;
    writeConfig(SPIFFS, config);
  }

  if (config.max_send == 0) {
    config.max_send = 25;
    writeConfig(SPIFFS, config);
  }

  if (config.timezone > 20 || config.timezone < -20) {
    config.timezone = 0;
  }
  Serial.println("===CONFIG===");
  Serial.printf("Env: %s\n", config.brokerSelector ? "Production" : "Staging");
  Serial.printf("Host: %s \n", config.host);
  Serial.printf("Port: %d \n", config.port);
  Serial.printf("Timezone: %d \n", config.timezone);
  Serial.printf("Ambil Data Interval: %d \n", config.ambilDataInterval);
  Serial.printf("Kirim Data Interval: %d \n", config.kirimDataInterval);
  Serial.printf("Time for broker (s): %i\n", config.time_for_broker);
  Serial.printf("Watch dog timer (s): %i\n", config.watchdog_timer);
  Serial.printf("Max send: %i\n", config.max_send);
  Serial.printf("Max SD attempt: %i\n", config.SD_attempt);
  Serial.println("==END CONFIG==");
  //----------------------------------------------END READ CONFIG---------------------
  Serial.println(getStringDateTime(config.timezone));
  //----------------------READ DEVICE INFO FROM SPIFFS---------------------

  bool bdevinfo = read_device_info(SPIFFS, device_info);
  // bool bdevinfo = false;
  if (!bdevinfo) {
    Serial.println("Device info file not found !(/Setup)");
    DeviceInfo setupDeviceInfo;
    String myloc; myloc.reserve(40);
    myloc = DEVICE_LOCATION;
    String mydn; mydn.reserve(12);
    mydn = DEVICE_ID;
    const char *c_myloc = myloc.c_str();
    const char *c_mydn = mydn.c_str();
    Serial.println(c_myloc);
    Serial.println(c_mydn);

    setupDeviceInfo.sd_failure = true;
    strlcpy(setupDeviceInfo.software_version, SOFTWARE_VERSION, (strlen(SOFTWARE_VERSION) + 1));
    strlcpy(setupDeviceInfo.my_dn, c_mydn, (strlen(c_mydn) + 1));
    strncpy(setupDeviceInfo.my_location, c_myloc, (strlen(c_myloc) + 1));
    Serial.println("IN_STRLCPY-0----");
    Serial.println(setupDeviceInfo.software_version);
    Serial.println(setupDeviceInfo.my_location);
    Serial.println(setupDeviceInfo.my_dn);
    bool bsetupDeviceInfo = write_device_info(SPIFFS, setupDeviceInfo);
    if (!bsetupDeviceInfo) {
      Serial.println("Failed to write Device Info (_f/Setup)");
    } else {
      bsetupDeviceInfo = read_device_info(SPIFFS, setupDeviceInfo);
      Serial.println("WriteDevInfo_OK----");
      Serial.println(setupDeviceInfo.my_location);
      if (!bsetupDeviceInfo) {
        Serial.println("Failed load Device Info after setup (_f/Setup)");
      }
    }
  }
  Serial.println("NewDevInfo__DONE----");
  read_device_info(SPIFFS, device_info);
  Serial.println(device_info.my_location);

  // uint8_t compare = strcmp(device_info.software_version, SOFTWARE_VERSION);
  // Serial.printf("Cmp software version: %i\n", compare);
  // if (compare > 0) {
  //   Serial.println(device_info.software_version);
  //   Serial.println(SOFTWARE_VERSION);
  //   strlcpy(device_info.software_version, SOFTWARE_VERSION, 10);
  //   write_device_info(SPIFFS, device_info);
  // }

  Serial.println("===DEV INFO===");
  Serial.printf("Software: %s\n", device_info.software_version);
  Serial.printf("DN      : %s\n", device_info.my_dn);
  Serial.printf("Location: %s\n", device_info.my_location);
  Serial.printf("SD Fail : %i\n", device_info.sd_failure);
  Serial.println("==END DEV INFO==");
  //----------------------END READ DEVICE INFO FROM SPIFFS---------------------

  //----------------------READ RULES-------------------------------------------
  if (SPIFFS.exists("/rules.txt")) {
    rules_exist = readRules(SPIFFS,rules);
    String text_rules; text_rules.reserve(380);
    Serial.println("RULES_EXIST_SETUP");
    lcd.clear(); lcd.print("RULES_EXIST");
    text_rules = sdop.readFile(SPIFFS, RULES_FILENAME, 1);
    // Serial.println(text_rules);
    delay(1000);
  } else {
    String text_rules; text_rules.reserve(380);
    // text_rules =
    const char *c_rueles = "{\"rules\":\"/rules.txt\","
                           "\"n1\":\"o1(offsetP1),c1(PF1)\","
                           "\"n2\":\"index0:0X/1=/2</3>/4<=/5>=/6!=/7+/8-\","
                           "\"p1\":[0, 10.2],"
                           "\"p2\":[0, 14.3],"
                           "\"p3\":[0, 12.2],"
                           "\"o1\":[0, 0.1],"
                           "\"o2\":[0, 0.1],"
                           "\"o3\":[0, 0.1],"
                           "\"wl\":[0, 504.6],"
                           "\"fl\":[0, 14.6],"
                           "\"v1\":[0, 200.3],"
                           "\"i1\":[0, 60.5],"
                           "\"c1\":[0, 0.54],"
                           "\"v2\":[0, 567.3],"
                           "\"i2\":[0, 34.9],"
                           "\"c2\":[0, 0.65],"
                           "\"v3\":[0, 200.3],"
                           "\"i3\":[0, 50.5],"
                           "\"c3\":[0, 0.53]}";
    lcd.clear(); lcd.print("NO_RULES");
    delay(2000);
    // sdlkfjsdf; //lanjut dari sini
    if (sdop.writeFile(SPIFFS, RULES_FILENAME, c_rueles)) {
      rules_exist = true;
      sdop.writeFile(SPIFFS,DEF_RULES_FLAG,"0");
      lcd.clear(); lcd.print("RULES_WRITEN!");
      text_rules = sdop.readFile(SPIFFS, RULES_FILENAME, 1);
      // Serial.println(text_rules);
      delay(1000);
    } else {
      rules_exist = false;
      lcd.clear(); lcd.print("RULES_FAILED!");
      Serial.println("RULES_FAILED!");
      delay(1000);
    }
  }
  //-------------------END READ RULES------------------------------------------

  Serial.printf("UseSD:%i\n", config.useSD);
  if (SDok()) {
    if (cekSD(config.SD_attempt)) {
      device_info.sd_failure = false;
      if (device_info.sd_failure) {
        write_device_info(SPIFFS, device_info);
      }
      lcd.backlight();
      lcd.clear();
      lcd.print("SD Connected!");
      delay(500);

      //CEK SD CARD TYPE---------------
      String SDtype = sdop.cardInfo();
      lcd.backlight();
      lcd.clear();
      lcd.print(SDtype); //SDHC [pending]
      //END CEK------------------------

      //----------CEK PENDING DATA di SD-------------------------------
      lcd.setCursor(0, 1);
      lcd.print("Reading SD!");
      recorded_aprox = read_sd_approx_sent_pend(1);
      pending_aprox = read_sd_approx_sent_pend(0);

      Serial.printf("Pending approx:%i \n", pending_aprox);
      Serial.printf("Recorded approx:%i \n", recorded_aprox);
      lcd.setCursor(9, 0);

      lcd.setCursor(0, 1);
      lcd.printf("pnd:%i rec:%i", pending_aprox, recorded_aprox);

      Serial.printf("Total LOG---->: %i \n", logCount);
      Serial.printf("Bool Log : %i \n", alreadyLog);
      delay(500);
      //----------CEK PENDING DATA di SD-------------------------------
      print_mode();
      delay(500);
    } else {
      device_info.sd_failure = true;
      write_device_info(SPIFFS, device_info);
    }
  }

  Serial.printf("IntervalBaca:%i, IntervalKirim:%i\n", config.ambilDataInterval, config.kirimDataInterval);
  config.ambilDataInterval == 0 ? config.ambilDataInterval = 5 : config.ambilDataInterval;
  config.kirimDataInterval == 0 ? config.kirimDataInterval = 30 : config.ambilDataInterval;
  if (logCount > (config.kirimDataInterval / config.ambilDataInterval)) { //log sudah banyak, tapi kenapa alarm one tidak dipanggil?
    Serial.println("*** ALARM 1? ***");
    internalRTCwakeup((60 * config.ambilDataInterval) + 15);
    bAlarmOne = true;
    lcd.clear();
    lcd.backlight();
    setAlarmOne(config.kirimDataInterval);
    lcd.print("RST_Alarm1:");
    lcd.setCursor(0, 1); lcd.printf("%imnt", config.kirimDataInterval);
    delay(1000);
    lcd.clear();
  }
  if (wakeUpFlag == 3) {
    internalRTCwakeup((60 * config.ambilDataInterval) + 15); //set alarm sendiri untuk jaga2
    Serial.println("*--*INTERNAL RTC!!!!*--*");
    lcd.backlight();
    lcd.clear();
    lcd.print("Internal RTC!");
    delay(1000);
    lcd.clear();
    if (!alreadyLog) { //mungkin terjadi sesuatu shg tidak ngelog
      setAlarmTwo(config.ambilDataInterval);
      //bAlarmTwo = true;
      lcd.print("RST_Alarm2");
      lcd.setCursor(0, 1); lcd.printf("%imnt", config.ambilDataInterval);
      delay(1000);
      lcd.clear();
    }
    bAlarmTwo = true;
    alreadyLog = false;
    // Parse command
  } else if (wakeUpFlag == 1) {
    Serial.print("SQW send trigger >> :");
    DS3231AlarmFlag flag = rtc.LatchAlarmsTriggeredFlags();
    Serial.println(flag);
    if (flag & DS3231AlarmFlag_Alarm1) {
      Serial.println("from Alarm One (_f/Setup)");
      setAlarmOne(config.kirimDataInterval); //set alarm1 dan 2
      //setAlarmTwo(config.ambilDataInterval);
      bAlarmOne = true;
      Serial.println(bAlarmOne);
    } else if (flag & DS3231AlarmFlag_Alarm2) {
      bAlarmTwo = true;
      Serial.println("from Alarm Two (_f/Setup)");
      setAlarmTwo(config.ambilDataInterval);
    }
  } else if (wakeUpFlag == 2) {
    Serial.print("Trigger from button: ");
    Serial.println(getGPIOWakeUp(), 0);
    // menu.buttonBegin();
    unsigned long wakeUpTime = millis();
    bool initialFlag = false;
    lcd.backlight();
    // lcd.clear(); lcd.print("Preparing Menu!");
    // delay(1000);
    startWDT(180);
    while (1) { //------HANDLE MENU UTAMA--------------------------------------
      int buttonEvt = menu.buttonEvent();
      bool selected = false;
      if (buttonEvt > -1 || initialFlag) {
        initialFlag = true;
        menuAction action = static_cast<menuAction>(buttonEvt);
        String buffer = menu.dispatchMenu(action);
        char test[16];
        buffer.toCharArray(test, 16);
        lcdPrint(test, "<              >");
        if (buffer == "#test$") {
          selected = true;
          menu.levelMenu = 0;
          testKirim();
        } else if (buffer == "#data$") {
          selected = true;
          menu.levelMenu = 0;
          delay(300);
          printData();
          lcd.clear();
          //cek_spiffs();
          /* } else if (buffer == "#analog$") {
            selected = true;
            menu.levelMenu = 0;
            printAnalog(); */
        } else if (buffer == "#status$") {
          selected = true; menu.levelMenu = 0;
          printStatus();
        } else if (buffer == "#apn$") {
          selected = true; menu.levelMenu = 0;
          // delay(300);
          // cek_apn();
          // delay(300);
        } else if (buffer == "#disablesd$") {
          selected = true; menu.levelMenu = 1;
          delay(300);
          config.useSD = false;
          device_info.sd_failure = false;
          writeConfig(SPIFFS, config);
          write_device_info(SPIFFS, device_info);
          lcd.clear();
          lcd.print("SD Disabled!");
          // lcd.setCursor(0, 1);
          // if (device_info.sd_failure) {
          //   lcd.print("SD_FAILED!");
          // } else {
          //   lcd.print("SD_OK");
          // }
          delay(1000);
        } else if (buffer == "#enablesd$") {
          selected = true; menu.levelMenu = 1;
          delay(300);
          config.useSD = true;
          device_info.sd_failure = false;
          writeConfig(SPIFFS, config);
          write_device_info(SPIFFS, device_info);
          lcd.clear();
          lcd.print("SD will Enabled");
          lcd.setCursor(0, 1);
          lcd.print("after RESTART!");
          delay(1000);
        } else if (buffer == "#pzemset$") {
          selected = true; menu.levelMenu = 0;
          uint8_t result, datasize = 2; //datasize 2

#ifdef _USE_A1_FOR_WATER_LEVEL_
          uint8_t result, datasize = 2; //datasize 2
          digitalWrite(SW_AWLR_PWR, 1);
          lcd.clear(); lcd.print("Preparing AWLR!");
          delay(500);
          uint8_t btn = wait_button(0);
          float v = 0, water_h = 0;
          while (btn != 2) {
            v = getAWLRvoltage(1);
            water_h = getWaterHeight(1);
            lcd.clear();
            lcd.print("V:"); lcd.print(v);
            lcd.setCursor(0, 1);
            lcd.print("H(cm):"); lcd.print(water_h);
            btn = wait_button(0);
            delay(200);
          }
          digitalWrite(SW_AWLR_PWR, 0);
          lcd.clear(); lcd.print("AWLR exit!");
          delay(2000);
#endif

#ifdef _USE_ULTRASONIC_FLOW_METER_
          // while(!modbusok){
#ifdef _DYNAMIC_SERIAL_PORT_PIN_
          sensor_serial_set(1);
          delay(100);
#endif
          float flow_value = 0;
          flow_value = read_flow(flow_modbus4);
          delay(100);
          wait_button(1);
#endif
#ifdef _SET_PZEM_ADDR_
#ifdef _DYNAMIC_SERIAL_PORT_PIN_
          sensor_serial_set(0);
          delay(100);
#endif
          selected = true; menu.levelMenu = 0;
          int initialValue = 0;
          interactiveInputInterval("PZEM CMD:", initialValue, 1, "cmd");
          if (initialValue == 10) {
            //erase counted energy
            pzem.resetEnergy();
            lcd.clear();
            lcd.print("Energy is reset");
            delay(1000);
          } else if (initialValue == 1) {
            //set connected pzem with address 1
            pzem.setAddress(1);
            lcd.clear();
            lcd.print("PZEM addr 1 set");
            delay(1000);
          } else if (initialValue == 2) {
            //set connected pzem with address 2
            pzem.setAddress(2);
            lcd.clear();
            lcd.print("PZEM addr 2 set");
            delay(1000);
          } else if (initialValue == 3) {
            //set connected pzem with address 3
            pzem.setAddress(3);
            lcd.clear();
            lcd.print("PZEM addr 3 set");
            delay(1000);
          } else if (initialValue == 0) {
            //check available address
            Serial.println("-----------------");
            // uint8_t address = pzem.getAddress();
            // lcd.clear();
            // lcd.printf("Add:%i", address);
            Serial.println("---___---___");
            pzem.search();
            delay(2000);
          } else if (initialValue == 6) {
            int8_t btn = -1;
            float v1;
            while (btn < 0) {
              v1 = pzem1.voltage();
              isnan(v1) ? v1 = 0 : v1;
              lcd.clear();
              lcd.printf("V1:%.02f", v1);
              btn = wait_button(0);
              delay(500);
            }
            lcd.clear(); lcd.print("Done!");
            delay(1000);
          } else if (initialValue == 7) {
            int8_t btn = -1;
            float v1;
            while (btn < 0) {
              v1 = pzem2.voltage();
              isnan(v1) ? v1 = 0 : v1;
              lcd.clear();
              lcd.printf("V2:%.02f", v1);
              btn = wait_button(0);
              delay(500);
            }
            lcd.clear(); lcd.print("Done!");
            delay(1000);
          } else if (initialValue == 8) {
            int8_t btn = -1;
            float v1;
            while (btn < 0) {
              v1 = pzem3.voltage();
              isnan(v1) ? v1 = 0 : v1;
              lcd.clear();
              lcd.printf("V3:%.02f", v1);
              btn = wait_button(0);
              delay(500);
            }
            lcd.clear(); lcd.print("Done!");
            delay(1000);
          }
          else {
            lcd.clear();
            lcd.print("NO CMD!");
            delay(1000);
            //do nothing
          }

          /* delay(300);
            if (config.useSD) {
            cek_data_sd();
            } else {
            printSDdisabled();
            }
            delay(300); */
#endif
        } else if (buffer == "#rcntsent$") {
          selected = true; menu.levelMenu = 1;
          delay(300);
          recountSent(0);
          wait_button(1);
          delay(300);
        } else if (buffer == "#rcntpend$") {
          selected = true; menu.levelMenu = 1;
          delay(300);
          recountPend(0);
          wait_button(1);
          delay(300);
        } else if (buffer == "#clrpend$") {
          selected = true; menu.levelMenu = 1;
          delay(300);
          lcd.clear(); lcd.print("Preparing...");
          clr_sent_pend(0, 50, 0);
        } else if (buffer == "#clrsent$") {
          selected = true; menu.levelMenu = 1;
          delay(300);
          lcd.clear(); lcd.print("Preparing...");
          clr_sent_pend(1, 50, 0);
        } else if (buffer == "#bat$") {
          selected = true; menu.levelMenu = 0;
          delay(300);
          set_battery_safe();
          delay(300);
        } else if (buffer == "#sig$") {
          selected = true; menu.levelMenu = 0;
          delay(300);
          minimum_signal();
          delay(300);
        } else if (buffer == "#burst$") {
          selected = true; menu.levelMenu = 0;
          delay(300);
          // set_burst_hour();
          // delay(300);
        } else if (buffer == "#defset$") {
          selected = true; menu.levelMenu = 0;
          sdop.deleteFile(SPIFFS, RULES_FILENAME);
          setDefSettings();
          delay(500);
        } else if (buffer == "#exit$") {
          selected = true;
          buttonEvt = -1;
          initialFlag = false;
          menu.reset();
          delay(500);
        } else if (buffer == "#back$") {
          selected = true;
          //menu.reset();
          menu.levelMenu = 0;
          delay(500);
        } else if (buffer == "#spiffs$") {
          selected = true; menu.levelMenu = 0;
          delay(500);
          cek_spiffs();
          delay(500);
        } else if (buffer == "#ambil_data$") {
          selected = true; menu.levelMenu = 1;

          int initialValue = config.ambilDataInterval;
          interactiveInputInterval("Ambil Data (T)", initialValue, 1, "menit");
          config.ambilDataInterval = initialValue;
          bool bUpdateConfig = writeConfig(SPIFFS, config);
          lcd.clear();
          lcd.home();
          lcd.print("Save config:");
          lcd.setCursor(0, 1);
          if (!bUpdateConfig) {
            lcd.print("Error!");
          } else {
            lcd.print("Success!");
          }
          setAlarmTwo(config.ambilDataInterval);
          write_normal_config();
          delay(1000);
        } else if (buffer == "#kirim_data$") {
          selected = true; menu.levelMenu = 1;

          int initialValue = config.kirimDataInterval;
          interactiveInputInterval("Kirim Data (T)", initialValue, 1, "menit");
          config.kirimDataInterval = initialValue;
          bool bUpdateConfig = writeConfig(SPIFFS, config);
          lcd.clear();
          lcd.home();
          lcd.print("Save config:");
          lcd.setCursor(0, 1);
          if (!bUpdateConfig) {
            lcd.print("Error!");
          } else {
            lcd.print("Success!");
          }
          setAlarmOne(config.kirimDataInterval);
          write_normal_config();
          delay(1000);
        } else if (buffer == "#timezone$") {
          selected = true; menu.levelMenu = 1;
          int initialValue = config.timezone;
          interactiveInputInterval("Zona waktu:", initialValue, 1, "jam");
          config.timezone = initialValue;
          bool bUpdateConfig = writeConfig(SPIFFS, config);
          lcd.clear();
          lcd.home();
          lcd.print("Save config:");
          lcd.setCursor(0, 1);
          if (!bUpdateConfig)
          {
            lcd.print("Error!");
          }
          else
          {
            lcd.print("Success!");
          }
          //TIME CONSUMMING SYNC---------------------------
          delay(1000);
          sync_time(1);
        } else if (buffer == "#server$") {
          selected = true; menu.levelMenu = 1;

#ifdef _USE_DEF_SERVER_
          int initialValue = 1;
          interactiveInputInterval("Server[defProd]:", initialValue, 0, "");
          config.brokerSelector = true;
#else
          int initialValue = int(config.brokerSelector);
          interactiveInputInterval("Server:", initialValue, 1, "");
          config.brokerSelector = bool(initialValue);
          cpyHostTxt(config.brokerSelector);
#endif

          bool bUpdateConfig = writeConfig(SPIFFS, config);
          lcd.clear();
          lcd.home();
          lcd.print("Save config:");
          lcd.setCursor(0, 1);
          if (!bUpdateConfig)
          {
            lcd.print("Error!");
          }
          else
          {
            lcd.print("Success!");
          }
          delay(1000);
        }
        if (selected) { //reset timer dan menu
          // menu.levelMenu = 0;
          esp_task_wdt_reset();
          wakeUpTime = millis();
        }
        if (buffer == "break" || buffer == "undefined") break;
      } else if (!initialFlag) {
        lcdPrintCurrentDate();
        // if(!config.brokerSelector){sdfsdf;
        //   lcd.setCursor(15, 1); lcd.print("X");
        // }
#ifdef _USE_STAGING_
        lcd.setCursor(15, 1); lcd.print("X");
#endif
      }

      if (millis() - wakeUpTime > WAKE_UP_TIME) break;
    } //END------HANDLE MENU--------------------------------------
    stopWDT();
  } else {//Pertama kali hidup------------
    //SIM800SleepDisable();  //
    lcd.backlight();
    lcd.clear();
    lcd.print(HARDWARE_VERSION);
    lcd.setCursor(0, 1);
    lcd.print(SOFTWARE_VERSION);
    delay(1000);
    lcd.clear();
    lcd.print(DEVICE_LOCATION);
    lcd.setCursor(0, 1);
    lcd.print(DEVICE_ID);
    delay(1000);
    SIMon = 0;

#ifndef __NO_SYCN__ //Pertama kali hidup------------
    // RtcDateTime dt = rtc.GetDateTime();
    // if (dt.Year() < 2021) {
    //TIME CONSUMING COMMAND---------------------------------------------------------
    wait_for_signal(60, config.minimum_signal);
    delay(1500);
    //Checking signal before sync time--------------
    lcd.clear();
    lcd.home();
    lcd.print("Sync time...");
    lcd.setCursor(0, 1);
    lcd.print(DEVICE_ID);

    setupModem(brokerSelectorF(config.brokerSelector));
    lcd.clear();
    lcd.print("Checking Time...");
    uint32_t timeFromServer = syncTimeUnixTime();
    modem.gprsDisconnect();
    SIM800SleepEnable();
    Serial.print("(_f/Setup1st)Time from server::> ");
    Serial.println(timeFromServer);
    if (timeFromServer != 0) {
      setCurrentTime(timeFromServer - (31536000 * 30) + (config.timezone * 3600) - 604800);
      lcd.clear();
      lcd.print("Time from server");
    } else {
      lcd.clear();
      lcd.print("NO time received");
    }
    delay(500);
    //END TIME CONSUMING COMMAND---------------------------------------------------------
    //}
#endif

    Serial.printf("(_f/Setup) Set Alarm two to: %i \n", config.ambilDataInterval);
    setAlarmTwo(config.ambilDataInterval);
    delay(100);
    Serial.printf("(_f/Setup) Set Alarm one to: %i \n", config.kirimDataInterval);
    setAlarmOne(config.kirimDataInterval);

    bool bWrite = deleteFile(SPIFFS, DATA_FILENAME);
    if (!bWrite) Serial.println("failed to write data (_f/Setup)");

#ifdef _SEND_DATA_ON_HARD_RESET_
    uint8_t t_cnt = 0;
    int8_t btn = -1;
    bool send_on_hard_reset = true;
    lcd.clear(); lcd.print("PressBtn=Cancle");
    // sdfsdf;
    while (t_cnt < 10) {
      btn = wait_button(0);
      // Serial.println(btn);
      lcd.setCursor(t_cnt, 1); lcd.print("=");
      delay(500);
      if (btn >= 0) {
        send_on_hard_reset = false;
        t_cnt = 11;
        lcd.clear(); lcd.print("Cancled!");
        delay(500);
      }
      t_cnt++;
    }
    if (send_on_hard_reset) {
      lcd.clear(); lcd.print("SENDING TEST!");
      delay(500);
      testKirim();
    }
#endif

    delay(100);
    lcd.clear();
  }
  wakeUpFlag = 0;

  pinMode(RTC_SQW_PIN, INPUT_PULLUP);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0); // change manually
  esp_sleep_enable_ext1_wakeup(BUTTON_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);

  Serial.println(getStringDateTime(config.timezone));
  lcd.clear();
  delay(200);

  //---------------------------------when alarm triggered
  if (bAlarmOne) {
    lcd.clear(); lcd.print("Alarm 1!");
    delay(200);
    Serial.println("In alarm one");
    internalRTCwakeup((60 * config.ambilDataInterval) + 15);

    // if ((!config.useSD) && (device_info.sd_failure)) { //kalo useSD=false, dan SDfailure=true cek sd card lagi, siapa tau udah mau
    //   if (cekSD(config.SD_attempt / 2)) {
    //     config.useSD = true;
    //     writeConfig(SPIFFS, config);
    //     lcd.clear(); lcd.print("SD Reconnected!");
    //   } else {
    //     lcd.clear(); lcd.print("SD Failed!");
    //   }
    //   delay(500);
    // }

    logCount = 0; //penanda kalau alarm 1 sudah pernah dipanggil
    cek_battery_safe();
    // uint8_t pending = sdop.countFile(SD, "/pend", 1);
    if (SDok()) {
      recountPend(100);
    } else {
      pending_aprox = 0;
    }
    if (!BatterySafeMode && (pending_aprox > 0)) {
      SIM800SleepDisable();
      if (wait_for_signal(60, config.minimum_signal)) { //true = dapat sinyal, false = tdk dpt sinyal target

        badSignal_directSend = false;
        Serial.println(APN);
        // Serial.println(apn_name[config.selected_apn]);

        byte notconnected = setupModem(brokerSelectorF(config.brokerSelector)); //1 = failed, 0=ok
        if (notconnected) {
          badSignal_directSend = true;
        }
        if ((sdop.countFile(SD, "/pend", 1)) > 0) { //cek ada file pending atau tidak
          filePending = true;
        }

        mqtt.setBufferSize(1024);
      } else {
        badSignal_directSend = true;
      }
    } else if (pending_aprox <= 0) {
      Serial.println(">>>NO PENDING!<<<");
      lcd.clear();
      lcd.print("Alarm 1 CANCLED!");
      lcd.setCursor(0, 1); lcd.print("NO PENDING!");
    } else {
      Serial.println(">>>SAFE MODE!<<<");
      lcd.clear();
      lcd.print("Alarm 1 CANCLED!");
      lcd.setCursor(0, 1); lcd.print("IN SAFE MODE!");
    }
    delay(500);
  } else if (bAlarmTwo) {
    lcd.clear(); lcd.print("Alarm 2!");
    lcd.backlight();
    delay(500);
    Serial.println("-----Alarm two (_f/Setup)-----");
    log_toSD(false);
    alreadyLog = true; //penanda kalau alarm 2 sudah pernah dipanggil
    logCount++;
    internalRTCwakeup((60 * config.ambilDataInterval) + 15);
  }
  delay(100);
  waitingData = millis();
}

void loop() {
  Serial.println("LOOP");
  if (bAlarmOne && (!badSignal_directSend) && filePending && (!BatterySafeMode)) { //jika ada alarm one dan ada signal dan ada file pending, maka lakukan pengiriman
    Serial.printf("Alarm One:%u \n", bAlarmOne);
    Serial.printf("Signal:%u \n", (!badSignal_directSend));
    Serial.printf("Pending Exist:%u \n", filePending);
    Serial.printf("Safe mode off:%u \n", (!BatterySafeMode));

    uint8_t trial = 0;
    for (trial = 0; trial < 3; trial++) { //Coba 5 kali kirim
      if (mqttConnect(brokerSelectorF(config.brokerSelector), 1, 0, disableSD_temporary)) { //Langsung kirim data
        trial = 6;
        lcd.clear();
        lcd.print("Success!");
        delay(500);
      } else {
        mqtt.disconnect();
      }
    }
    bAlarmOne = false;
  } else if (wait_for_broker) { //--------------------JIKA DISURUH MENUNGGU PESAN DARI BROKER-------------
    if (!wait_broker_initiated) {
      if (!wdt_is_set) {
        //------------SET WDT--------------JAGA2 KALAU HANG-------------
        startWDT(config.watchdog_timer);
        //------------SET WDT--------------JAGA2 KALAU HANG-------------
        mqtt.setCallback(mqttCallback);
        mqtt.setBufferSize(1024); //sudah diset di alarm one atau tes kirim
        wdt_is_set = true;
      }

      lcd.clear(); lcd.print("Preparing!"); delay(100);
      bool mqtt_connected = mqtt.connect(DEVICE_ID);
      if (!mqtt_connected) { //while
        mqtt_connected = mqtt.connect(DEVICE_ID);
        delay(100);
      }

      uint16_t time_now = (millis() - start_time_for_broker) / 1000;
      String status; status.reserve(18);
      if (cmd_message == cmd_wait_me) {
        status = (String)count_wait_me;
        status += "/";
        status += (String)int_limit_wait_me;
        status += "_WAITING!";
      } else {
        status = (String)time_now;
        status += "/";
        status += (String)config.time_for_broker;
        status += "...";
      }
      string_buffer = report_text(status);
      const char* reconect_txt = string_buffer.c_str();
      mqtt.publish(STATUS_TOPIC, reconect_txt);

      mqtt.subscribe(CMD_TOPIC);
      mqtt.loop();
      Serial.println("Subs to Def Topic!");

      wait_broker_initiated = true;
      mqtt.setKeepAlive(20);
    } else {
      mqtt.loop();
      Serial.printf("Limit:%i - Now:", (config.time_for_broker * 1000));
      Serial.println((millis() - start_time_for_broker));
      if (cmd_message != cmd_wait_me) {
        count_wait_me = 0; //reset limit waitme
      }

      if (got_message) {
        esp_task_wdt_reset(); //RESET WATCHDOG TIMER-------------RESET DULU DI SINI BIAR ADA TAMBAHAN WAKTU
        if (cmd_message == cmd_sync_time) {
          mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Sync time...\"}");
          mqtt.setKeepAlive(15);
          delay(1000);
          sync_time(0);
          lcd.clear(); lcd.print("Sync Time done!");
          delay(500);
        } else if (cmd_message == cmd_clear_pend) {
          mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Clear some files in SD/pend...\"}");
          uint16_t deleted = clr_sent_pend(0, sd_limit_access, 1);
          String answer; answer.reserve(20);
          answer = "{\"ans\":\"Deltd pend:";
          answer += (String)deleted;
          answer += "\"}";
          const char* c_answer = answer.c_str();
          mqtt.publish(STATUS_TOPIC, c_answer);
          delay(500);
        } else if (cmd_message == cmd_clear_sent) {
          mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Clear some files in SD/sent...\"}");
          uint16_t deleted = clr_sent_pend(1, sd_limit_access, 1);
          String answer; answer.reserve(20);
          answer = "{\"ans\":\"Deltd sent:";
          answer += (String)deleted;
          answer += "\"}";
          const char* c_answer = answer.c_str();
          mqtt.publish(STATUS_TOPIC, c_answer);
          delay(500);
        } else if (cmd_message == cmd_count_pend) {
          mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Counting files in SD/pend...\"}");
          uint16_t files = sdop.countFile(SD, "/pend", sd_limit_access);
          files > 100 ? files = 100 : files;
          String answer; answer.reserve(34);
          answer = "{\"ans\":\"Pending:";
          answer += (String)files;
          if (files >= sd_limit_access) {
            answer += "/+";
          }
          answer += "|Approx:";
          answer += (String)pending_aprox;
          answer += "\"}";
          const char* c_answer = answer.c_str();
          mqtt.publish(STATUS_TOPIC, c_answer);
          delay(500);
        } else if (cmd_message == cmd_count_sent) {
          mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Counting files in SD/sent...\"}");
          uint16_t files = sdop.countFile(SD, "/sent", sd_limit_access);
          files > 100 ? files = 100 : files;
          String answer; answer.reserve(34);
          answer = "{\"ans\":\"Sent:";
          answer += (String)files;
          if (files >= sd_limit_access) {
            answer += "/+";
          }
          answer += "|Approx:";
          answer += (String)recorded_aprox;
          answer += "\"}";
          const char* c_answer = answer.c_str();
          mqtt.publish(STATUS_TOPIC, c_answer);
          delay(500);
        } else if (cmd_message == cmd_def_settings) {
          mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Applying default settings...\"}");
          setDefSettings();
          delay(500);
          mqtt.publish(STATUS_TOPIC, "{\"ans\":\"Default settings IS SET!\"}");
        }

        mqtt.setKeepAlive(15);
        delay(200);
        lcd.clear();
        start_time_for_broker = millis();
        got_message = false;
        time_broker = 19; // diskoneknya lewat sini aja// diskoneknya lewat sini aja
      }

      if (time_broker > 18) {
        time_broker = 0;
        wait_broker_initiated = false;
        int_limit_wait_me = limit_wait_me / 18;
        if ((cmd_message == cmd_wait_me) && (count_wait_me < int_limit_wait_me)) { //67 = 20menit / 18s
          count_wait_me++;
          start_time_for_broker = millis();
          esp_task_wdt_reset();
        } else if (count_wait_me >= int_limit_wait_me) {
          close_communication();
        } else {
          // if (!config.listening) { //cek apa perlu always listening atau tidak, setidaknya wait broker initiaded true, supaya hanya saat alarm one ini dieksekusi, alarm 2 tidak
          //   if ((!(cek_burst_mode(config.burst_send_h_on, config.burst_send_h_off))) && (config.burst_send_h_on != config.burst_send_h_off)) { //jika tidak dalam burst mode, maka jangan komunikasi, jika burst mode off maka komunikasi terus
          //     //wait_for_broker = false;
          //     close_communication();
          //     Serial.println("I'M NOT LISTENING BROKER!");
          //   }
          // } else {
          count_wait_me = 0;
          mqtt.disconnect();
          delay(1500); //kasi waktu buat diskonek
          // }
        }
        Serial.printf("CMD_MSG:%i\n", cmd_message);
      }

      if (wait_button(0) > -1) {
        lcd.backlight();
        lcd.clear(); lcd.print("End listening?");
        lcd.setCursor(0, 1); lcd.print("No");
        lcd.setCursor(7, 1); lcd.print("Yes");
        int8_t btn = -1;
        uint8_t limit_btn = 0;
        while (1) {
          limit_btn++;
          btn = wait_button(0);
          if (btn == 0) {
            break;
          }
          else if (btn == 2) {
            close_communication();
            break;
          }
          if (limit_btn > 50) { //8s
            break;
          }
          delay(100);
        }
        lcd.clear();
      }

      if ((millis() - start_time_for_broker) > (config.time_for_broker * 1000)) { //waktu habis
        close_communication();
        lcd.clear(); lcd.print("TIMEOUT!");
      } else {
        time_broker++;
        lcd.clear();
        if (cmd_message == cmd_wait_me) {
          lcd.print("I'M WAITING!!!");
          lcd.setCursor(8, 1); lcd.printf("%i/%i", count_wait_me, int_limit_wait_me);
        } else {
          lcd.print("Listening!");
        }
        lcd.setCursor(0, 1); lcd.printf("[%i/18]", time_broker);
      }
      delay(1000);
    }

    //--------------------JIKA DISURUH MENUNGGU PESAN DARI BROKER-------------
  } else {
    bool recount = false;
    if (recount_sent || recount_pend) {
      recount = true;
    }

    lcd.clear();
    lcd.backlight(); lcd.print("disconnecting...");

    if (cmd_message == cmd_close) {  //BUAT PESAN DI MQTT BROKER KALAU KITA AKAN DISKONEK, DAN AKAN KONEK LAGI NANTI-------------
      stopWDT();

      wait_for_broker = false;
      mqtt.unsubscribe(CMD_TOPIC);

      // bool in_burst_mode = cek_burst_mode(config.burst_send_h_on, config.burst_send_h_off);
      // bool burst_mode_off = false;
      // config.burst_send_h_off == config.burst_send_h_on ? burst_mode_off = true : burst_mode_off = false;
      // bool rapid = false;
      // if (in_burst_mode || (!config.useSD)) {
      //   rapid = true;
      // }

      String closetxt; closetxt.reserve(40);
      //closed:2021/10/14,05:21:02+07-<10/30|15-21|LBR>
      closetxt = "CLOSED_";
      //closetxt += getStringDateTime(config.timezone);
      //closetxt += "_";
      if (recount_sent) {
        closetxt += "s";
      }
      if (recount_pend) {
        closetxt += "p";
      }
      // {"ans":"SDX_PENINJOAN S1_B3.6.8f_closed:2021/10/14,05:21:02+07-<10/30|15-21|LBR>-sp"}
      string_buffer = report_text(closetxt);
      const char* exp_session = string_buffer.c_str();
      mqtt.publish(CMD_TOPIC, exp_session);

      String status; status.reserve(ANS_STATUS_LENGTH);
      status = config_and_devinfo_status(0);
      const char* status_char = status.c_str();
      mqtt.publish(STATUS_TOPIC, status_char);

      mqtt.setKeepAlive(15);
      delay(2000);
      mqtt.disconnect();
      Serial.println("END CONNECTION W/ BROKER!");
      //BUAT PESAN DI MQTT BROKER KALAU KITA AKAN DISKONEK, DAN AKAN KONEK LAGI NANTI-------------
    }

    //Disconnect semua koneksi dan suruh tidur SIM800
    Serial.print("Discnt MQTT&GPRS_f/1loop");
    if (SIMon) {
      if (!modem.gprsDisconnect()) {
        Serial.println("failed!_f/1loop");
      }
    }

    Serial.println("OK!_f/1loop");
    delay(100);
    SIM800SleepEnable();
    delay(100);

    //Eksekusi perintah recount data SD jika ada
    if (SDok() && recount) {
      if (recount_pend) {
        recountPend(0);
        recount_pend = false;
        delay(1000);
      }
      if (recount_sent) {
        recountSent(0);
        recount_sent = false;
        delay(1000);
      }
    }
    internalRTCwakeup((60 * config.ambilDataInterval) + 15);
    Serial.println("Sleep!_f/1loop");
    lcd.clear();
    lcd.print("GoingToSleep!");
    delay(200);
    lcd.noBacklight();
    lcd.clear();
    lcdPrintCurrentDate(); // ---[5/12/60]---
    // lcd.setCursor(10, 0); lcd.printf("|%i", recorded_aprox);
    lcd.setCursor(10, 0); lcd.printf("|K%i", next_kirim_data);
    lcd.setCursor(0, 1); lcd.printf("%i/%i|A%i p%i|", config.ambilDataInterval, config.kirimDataInterval, next_ambil_data, pending_aprox);
    // lcd.setCursor(0, 1); lcd.printf("%i/%i/%i p%i|", config.ambilDataInterval, config.kirimDataInterval, config.time_for_broker, pending_aprox);
    Serial.println(device_info.my_location);
    Serial.println(config.host);
    Serial.println(next_ambil_data);
    esp_deep_sleep_start();
  }
}
