#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <myconfig.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>
#include <EEPROM.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h>

// -------------------- PCF8574 Button Pins (bit positions 0..7) --------------------
#define L_BUTTON      P0
#define C_BUTTON      P1
#define R_BUTTON      P2
#define GATE_BUTTON   P3
#define PUMP_BUTTON   P4
#define FAN_BUTTON    P5
#define FEEDER_BUTTON P6

// -------------------- PCF8574 Relay Pins (bit positions 0..7) --------------------
#define GATE_RELAY    P0
#define PUMP_RELAY    P1
#define FAN_RELAY     P2

// -------------------- GPIO Pins --------------------
#define FEEDER_PIN         4
#define MOTION_SENSOR_PIN  5

// -------------------- SD SPI Pins (check your ESP32-S3 wiring!) --------------------
#define SD_CS   10
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK  12

#define PCF8574_ADDRESS_1 0x20
#define PCF8574_ADDRESS_2 0x21

#define PUMP_ACTIVE_DUR               3500
#define FEEDER_ACTIVE_DUR             2500
#define DEBOUNCE_DELAY_MS             50
#define PCF_READ_INTERVAL_MS          30
#define SENSOR_READ_INTERVAL_MS       2000
#define UI_REFRESH_INTERVAL_MS        200
#define SENSOR_DATA_PATCH_INTERVAL_MS 6000

#define TIMEZONE_OFFSET_SEC         (7 * 3600)
#define NTP_SYNC_INTERVAL           (6UL * 60UL * 60UL * 1000UL)
#define RTC_DRIFT_THRESHOLD_SEC     5
#define MIN_SYNC_INTERVAL_MS        (6UL * 60UL * 60UL * 1000UL)
#define WIFI_TIMEOUT                20000UL

#define MAX_SCHEDULES   8

#define EEPROM_SIZE 96
#define SSID_ADDR 0
#define PASS_ADDR 32

// Default thresholds
#define DEF_HIGH_TEMP_THRESHOLD 31
#define DEF_LOW_TEMP_THRESHOLD  20
#define DEF_HIGH_HUM_THRESHOLD  75
#define DEF_LOW_HUM_THRESHOLD   45
#define MOTION_ALERT_ENABLED    false

#define DEF_SEND_DATA_PERIODICALLY  false

#define LCD_ROW 2
#define LCD_COL 16

PCF8574 pcf1(PCF8574_ADDRESS_1);
PCF8574 pcf2(PCF8574_ADDRESS_2);
RTC_DS3231 rtc;
SPIClass spiSD(FSPI);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", TIMEZONE_OFFSET_SEC, NTP_SYNC_INTERVAL);
WebServer server(80);
WiFiClientSecure fbClient;
HTTPClient http;
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
LiquidCrystal_I2C lcd(0x27, LCD_COL, LCD_ROW);

enum ScheduleType : uint8_t { SCHED_FEEDER, SCHED_WATER };

enum MenuID : uint8_t {
  MENU_SHOW_DATA = 0,
  MENU_CHECK_SCHEDULE,
  MENU_MANUAL_SCHEDULE,
  MENU_COUNT
};

struct ScheduleSlot {
  bool enabled;
  ScheduleType type;
  char time[6];   // "HH:MM"
  bool executed;  // runtime-only
};

struct ActuatorTimer {
  bool active;
  unsigned long startTime;
  unsigned long duration;
};

ActuatorTimer pumpTimer   = { false, 0, PUMP_ACTIVE_DUR };
ActuatorTimer feederTimer = { false, 0, FEEDER_ACTIVE_DUR };
ScheduleSlot schedules[MAX_SCHEDULES];

bool rtcInitialized = false;
bool sdInitialized = false;
bool ntpInitialized = false;
bool isConfigMode = false;
bool scheduleResetDone = false;
bool ahtInitialized = false;
bool bmpInitialized = false;

bool isSendDataPeriodically = DEF_SEND_DATA_PERIODICALLY;

byte buttonState        = 0b01111111;
byte lastButtonState    = 0b01111111;
byte rawButtonState     = 0b01111111;
byte stableButtonState  = 0b01111111;
byte lastRawButtonState = 0b01111111;

unsigned long lastDebounceTime = 0;
unsigned long lastPCFReadTime = 0;
unsigned long lastNTPSyncMillis = 0;
unsigned long connectStart = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastUIUpdate = 0;
unsigned long lastDataPatch = 0;

byte actuatorState = 0b00000111; // bit=1 means relay OFF (active LOW hardware)

// WiFi creds
char ssid[32];
char pass[64];

// Sensor values
float ahtTemperature = 0.0;
float ahtHumidity = 0.0;
float bmpPressure = 0.0;
float bmpAltitude = 0.0;
bool motionDetected = false;

// Threshold storage
float tempHighThreshold = DEF_HIGH_TEMP_THRESHOLD;
float tempLowThreshold = DEF_LOW_TEMP_THRESHOLD;
float humHighThreshold = DEF_HIGH_HUM_THRESHOLD;
float humLowThreshold = DEF_LOW_HUM_THRESHOLD;

bool tempHighAlertActive = false;
bool tempLowAlertActive = false;
bool humHighAlertActive = false;
bool humLowAlertActive = false;
bool motionAlertActive = false;

const char* menuNames[MENU_COUNT] = {
  "Show Data",
  "Check Schedule",
  "Manual Schedule"
};

volatile int8_t currentMenu = 0;
bool menuDirty = true;
bool showDataScreen = false; // NEW: Show sensor values when MENU_SHOW_DATA selected & confirmed

// -------------------- Helpers: Active-LOW relay logic --------------------
static inline void setRelayOff(uint8_t relayBitPos) {
  actuatorState |= _BV(relayBitPos);   // bit=1 means OFF
}
static inline void setRelayOn(uint8_t relayBitPos) {
  actuatorState &= ~_BV(relayBitPos);  // bit=0 means ON
}

void applyRelayState(void) {
  // active LOW relays:
  // actuatorState bit=1 -> write HIGH (OFF), bit=0 -> write LOW (ON)
  pcf2.digitalWrite(GATE_RELAY, (actuatorState & _BV(GATE_RELAY)) ? HIGH : LOW);
  pcf2.digitalWrite(PUMP_RELAY, (actuatorState & _BV(PUMP_RELAY)) ? HIGH : LOW);
  pcf2.digitalWrite(FAN_RELAY,  (actuatorState & _BV(FAN_RELAY))  ? HIGH : LOW);
}

// -------------------- Forward declarations --------------------
void initializeButtons(void);
void initializeRelays(void);
void initializeFeeder(void);
void initializeAHT(void);
void initializeBMP(void);
void initializeMotionSensor(void);
void initializeSensors(void);
void initializeDisplay(void);
void initializeRTC(void);
void initializeSDCardReader(void);
void initializeNTP(void);

void readRawButtonInput(void);
void updateButtonInput(void);
void checkButtonStateChange(void);
void checkRelayActivity(void);
void checkSerialCommand(void);
void checkScheduleExecution(void);
void checkAndSyncRTCOnBoot(void);
void resetScheduleExecutionAtMidnight(void);
void printCurrentTime(void);
void printSchedule(void);
void loadScheduleFromSDCard(void);
ScheduleType parseScheduleType(const char* str);
bool syncRTCWithNTP(bool force = false);

void printScheduleFromFirebase(void);
bool loadScheduleFromFirebaseToRAM(void);

void checkAlerts(void);
void showAlertStatus(void);
void handleMenuNavigation(int direction);
void renderMenuUI(void);
void renderDataUI(void);
void updateDisplayUI(void);
bool saveScheduleToSDCard(void);

void sendSensorDataToFirebase(void);
void sendSensorDataPeriodically(void);

void updateSensorThresholdFromFirebase(void);
void checkSensorThreshold(void);
void toggleSendDataToFirebase(void);

void readEEPROM(void);
void writeEEPROM(const char* newSsid, const char* newPass);
bool connectWiFi(void);
String configPage(void);
void startConfigAP(void);

void startPump(void);
void stopPump(void);
void startFeeder(void);
void stopFeeder(void);

void toggleGate(void);
void toggleFan(void);

void readAHTdata(void);
void readBMPdata(void);
void readMotionSensorData(void);
void readSensorsData(void);
void showSensorsData(void);
void printSerialCommandList(void);

static inline bool looksLikeValidEEPROMString(const char* s, size_t maxLen) {
  // Basic sanity: not 0xFF-filled, not empty, printable-ish
  if (!s) return false;
  if ((uint8_t)s[0] == 0xFF) return false;
  if (s[0] == '\0') return false;
  for (size_t i = 0; i < maxLen && s[i] != '\0'; i++) {
    char c = s[i];
    if (c < 32 || c > 126) return false;
  }
  return true;
}

// -------------------- Setup/Loop --------------------
void setup() {
  Serial.begin(115200);
  delay(50);

  // IMPORTANT: init I2C bus first
  Wire.begin();

  initializeButtons();
  initializeRelays();
  initializeFeeder();
  initializeSensors();
  initializeDisplay();

  // WiFi STA setup (we use EEPROM credentials)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  readEEPROM();

  if (looksLikeValidEEPROMString(ssid, sizeof(ssid)) && looksLikeValidEEPROMString(pass, sizeof(pass))) {
    Serial.println(F("Found saved WiFi credentials"));
    if (!connectWiFi()) {
      Serial.println(F("WiFi failed, entering config mode"));
      isConfigMode = true;
      startConfigAP();
      return;
    }
  } else {
    Serial.println(F("No valid WiFi credentials found"));
    isConfigMode = true;
    startConfigAP();
    return;
  }

  initializeNTP();
  initializeRTC();
  checkAndSyncRTCOnBoot();

  initializeSDCardReader();
  loadScheduleFromSDCard();

  Serial.println(F("Setup completed."));
  Serial.println(F("Commands: press buttons or send 'i' for list."));
}

void loop() {
  readRawButtonInput();
  updateButtonInput();

  checkRelayActivity();
  checkScheduleExecution();
  checkSerialCommand();

  readSensorsData();
  sendSensorDataPeriodically();

  if (isConfigMode) {
    server.handleClient();
  }

  updateDisplayUI();
}

// -------------------- Init functions --------------------
void initializeButtons(void) {
  // PCF inputs often need HIGH written to enable pull-up behavior
  pcf1.pinMode(L_BUTTON, INPUT);      pcf1.digitalWrite(L_BUTTON, HIGH);
  pcf1.pinMode(C_BUTTON, INPUT);      pcf1.digitalWrite(C_BUTTON, HIGH);
  pcf1.pinMode(R_BUTTON, INPUT);      pcf1.digitalWrite(R_BUTTON, HIGH);
  pcf1.pinMode(GATE_BUTTON, INPUT);   pcf1.digitalWrite(GATE_BUTTON, HIGH);
  pcf1.pinMode(PUMP_BUTTON, INPUT);   pcf1.digitalWrite(PUMP_BUTTON, HIGH);
  pcf1.pinMode(FAN_BUTTON, INPUT);    pcf1.digitalWrite(FAN_BUTTON, HIGH);
  pcf1.pinMode(FEEDER_BUTTON, INPUT); pcf1.digitalWrite(FEEDER_BUTTON, HIGH);

  if (!pcf1.begin()) {
    Serial.println(F("ERROR: Could not initialize buttons' PCF8574!"));
    while (1) delay(100);
  }
  Serial.println(F("buttons' PCF8574 initialized successfully."));
}

void initializeRelays(void) {
  pcf2.pinMode(GATE_RELAY, OUTPUT);
  pcf2.pinMode(PUMP_RELAY, OUTPUT);
  pcf2.pinMode(FAN_RELAY, OUTPUT);

  if (!pcf2.begin()) {
    Serial.println(F("ERROR: Could not initialize relays' PCF8574!"));
    while (1) delay(100);
  }

  // All relays OFF (active LOW -> write HIGH)
  setRelayOff(GATE_RELAY);
  setRelayOff(PUMP_RELAY);
  setRelayOff(FAN_RELAY);
  applyRelayState();

  Serial.println(F("relays' PCF8574 initialized successfully."));
}

void initializeFeeder(void) {
  pinMode(FEEDER_PIN, OUTPUT);
  digitalWrite(FEEDER_PIN, LOW); // OFF
  Serial.println(F("feeder initialized successfully."));
}

void initializeMotionSensor(void) {
  pinMode(MOTION_SENSOR_PIN, INPUT);
  Serial.println(F("motion sensor initialized successfully."));
}

void initializeRTC(void) {
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    while (1) delay(100);
  }

  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, setting time from compile time"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  rtcInitialized = true;
  Serial.println(F("RTC initialized successfully."));
}

void initializeSDCardReader(void) {
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println(F("ERROR: Could not initialize SD Card Reader!"));
    sdInitialized = false;
    return;
  }
  sdInitialized = true;
  Serial.println(F("SD Card Reader initialized successfully."));
}

void initializeNTP(void) {
  timeClient.setUpdateInterval(3600000); // 1 hour
  timeClient.setTimeOffset(TIMEZONE_OFFSET_SEC);
  timeClient.begin();

  Serial.print(F("Waiting for NTP first update..."));
  int attempts = 0;
  while (!timeClient.update() && attempts < 20) {
    delay(500);
    Serial.print(F("."));
    attempts++;
  }

  if (attempts >= 20 || !timeClient.isTimeSet()) {
    Serial.println(F(" FAILED."));
    Serial.print(F("WiFi status: "));
    Serial.println(WiFi.status());
    ntpInitialized = false;
    return;
  }

  Serial.println(F(" OK"));
  ntpInitialized = true;
  lastNTPSyncMillis = millis();
  Serial.println(F("NTP Client initialized successfully."));
}

void initializeAHT(void) {
  if (!aht.begin()) {
    Serial.println(F("ERROR: Could not initialize AHT20 sensor!"));
    ahtInitialized = false;
    return;
  }
  ahtInitialized = true;
  Serial.println(F("AHT20 sensor initialized successfully."));
}

void initializeBMP(void) {
  if (!bmp.begin(0x77)) {
    Serial.println(F("ERROR: Could not initialize BMP280 sensor!"));
    bmpInitialized = false;
    return;
  }
  bmpInitialized = true;
  Serial.println(F("BMP280 sensor initialized successfully."));
}

void initializeSensors(void) {
  initializeAHT();
  initializeBMP();
  initializeMotionSensor(); // FIX: was missing
}

void initializeDisplay(void) {
  lcd.init();
  lcd.backlight();
  Serial.println(F("LCD Display initialized successfully."));
}

// -------------------- Buttons --------------------
void readRawButtonInput(void) {
  unsigned long now = millis();
  if (now - lastPCFReadTime < PCF_READ_INTERVAL_MS) return;
  lastPCFReadTime = now;

  rawButtonState = 0;
  rawButtonState |= pcf1.digitalRead(L_BUTTON)      ? _BV(L_BUTTON)      : 0;
  rawButtonState |= pcf1.digitalRead(C_BUTTON)      ? _BV(C_BUTTON)      : 0;
  rawButtonState |= pcf1.digitalRead(R_BUTTON)      ? _BV(R_BUTTON)      : 0;
  rawButtonState |= pcf1.digitalRead(GATE_BUTTON)   ? _BV(GATE_BUTTON)   : 0;
  rawButtonState |= pcf1.digitalRead(PUMP_BUTTON)   ? _BV(PUMP_BUTTON)   : 0;
  rawButtonState |= pcf1.digitalRead(FAN_BUTTON)    ? _BV(FAN_BUTTON)    : 0;
  rawButtonState |= pcf1.digitalRead(FEEDER_BUTTON) ? _BV(FEEDER_BUTTON) : 0;
}

void updateButtonInput(void) {
  if (rawButtonState != lastRawButtonState) {
    lastDebounceTime = millis();
    lastRawButtonState = rawButtonState;
  }

  if (millis() - lastDebounceTime >= DEBOUNCE_DELAY_MS) {
    if (stableButtonState != rawButtonState) {
      buttonState = rawButtonState;
      checkButtonStateChange();
      stableButtonState = rawButtonState;
    }
  }
}

void checkButtonStateChange(void) {
  if (buttonState == lastButtonState) return;

  // L
  if ((buttonState & _BV(L_BUTTON)) != (lastButtonState & _BV(L_BUTTON))) {
    if (!(buttonState & _BV(L_BUTTON))) {
      Serial.println(F("L Button Pressed."));
      handleMenuNavigation(-1);
    }
  }

  // C (use as “select/toggle view”)
  if ((buttonState & _BV(C_BUTTON)) != (lastButtonState & _BV(C_BUTTON))) {
    if (!(buttonState & _BV(C_BUTTON))) {
      Serial.println(F("C Button Pressed."));
      if (currentMenu == MENU_SHOW_DATA) {
        showDataScreen = !showDataScreen;
        menuDirty = true;
      }
      // You can extend: MENU_MANUAL_SCHEDULE triggers a manual action, etc.
    }
  }

  // R
  if ((buttonState & _BV(R_BUTTON)) != (lastButtonState & _BV(R_BUTTON))) {
    if (!(buttonState & _BV(R_BUTTON))) {
      Serial.println(F("R Button Pressed."));
      handleMenuNavigation(+1);
    }
  }

  // Gate
  if ((buttonState & _BV(GATE_BUTTON)) != (lastButtonState & _BV(GATE_BUTTON))) {
    if (!(buttonState & _BV(GATE_BUTTON))) {
      Serial.println(F("Gate Button Pressed."));
      toggleGate();
    }
  }

  // Pump
  if ((buttonState & _BV(PUMP_BUTTON)) != (lastButtonState & _BV(PUMP_BUTTON))) {
    if (!(buttonState & _BV(PUMP_BUTTON))) {
      Serial.println(F("Pump Button Pressed."));
      startPump();
    }
  }

  // Fan
  if ((buttonState & _BV(FAN_BUTTON)) != (lastButtonState & _BV(FAN_BUTTON))) {
    if (!(buttonState & _BV(FAN_BUTTON))) {
      Serial.println(F("Fan Button Pressed."));
      toggleFan();
    }
  }

  // Feeder
  if ((buttonState & _BV(FEEDER_BUTTON)) != (lastButtonState & _BV(FEEDER_BUTTON))) {
    if (!(buttonState & _BV(FEEDER_BUTTON))) {
      Serial.println(F("Feeder Button Pressed."));
      startFeeder();
    }
  }

  lastButtonState = buttonState;
}

// -------------------- Actuators --------------------
void checkRelayActivity(void) {
  unsigned long now = millis();
  if (pumpTimer.active && (now - pumpTimer.startTime >= pumpTimer.duration)) stopPump();
  if (feederTimer.active && (now - feederTimer.startTime >= feederTimer.duration)) stopFeeder();
}

void startPump(void) {
  if (pumpTimer.active) return;
  pumpTimer.active = true;
  pumpTimer.startTime = millis();

  setRelayOn(PUMP_RELAY);   // FIX: keep actuatorState consistent
  applyRelayState();

  Serial.println(F("Pump ON"));
}

void stopPump(void) {
  pumpTimer.active = false;

  setRelayOff(PUMP_RELAY);
  applyRelayState();

  Serial.println(F("Pump OFF"));
}

void startFeeder(void) {
  if (feederTimer.active) return;
  feederTimer.active = true;
  feederTimer.startTime = millis();

  // Safer than analogWrite for demo: ON/OFF
  digitalWrite(FEEDER_PIN, HIGH);

  Serial.println(F("Feeder ON"));
}

void stopFeeder(void) {
  feederTimer.active = false;
  digitalWrite(FEEDER_PIN, LOW);
  Serial.println(F("Feeder OFF"));
}

void toggleGate(void) {
  actuatorState ^= _BV(GATE_RELAY);
  applyRelayState();
  Serial.println(F("Gate toggled"));
}

void toggleFan(void) {
  actuatorState ^= _BV(FAN_RELAY);
  applyRelayState();
  Serial.println(F("Fan toggled"));
}

// -------------------- Time/Schedule --------------------
void printCurrentTime(void) {
  DateTime now = rtc.now();
  Serial.print(F("Current DateTime: "));
  Serial.print(now.year()); Serial.print('/');
  Serial.print(now.month()); Serial.print('/');
  Serial.print(now.day()); Serial.print(' ');
  Serial.print(now.hour()); Serial.print(':');
  Serial.print(now.minute()); Serial.print(':');
  Serial.print(now.second());
  Serial.println();
}

ScheduleType parseScheduleType(const char* str) {
  if (strcmp(str, "FEEDER") == 0) return SCHED_FEEDER;
  return SCHED_WATER;
}

void loadScheduleFromSDCard(void) {
  if (!sdInitialized) {
    Serial.println(F("SD not initialized. Cannot load schedule."));
    return;
  }

  if (!SD.exists("/schedule.csv")) {
    Serial.println(F("No schedule.csv found."));
    return;
  }

  File file = SD.open("/schedule.csv", FILE_READ);
  if (!file) {
    Serial.println(F("Failed to open schedule.csv"));
    return;
  }

  for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
    schedules[i].enabled = false;
    schedules[i].executed = false;
    schedules[i].time[0] = '\0';
  }

  // skip header line
  file.readStringUntil('\n');

  uint8_t index = 0;
  char line[32];

  while (file.available() && index < MAX_SCHEDULES) {
    int len = file.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';

    char* token = strtok(line, ",");
    if (!token) continue;
    schedules[index].enabled = atoi(token) != 0;

    token = strtok(NULL, ",");
    if (!token) continue;
    schedules[index].type = parseScheduleType(token);

    token = strtok(NULL, ",");
    if (!token) continue;
    strncpy(schedules[index].time, token, sizeof(schedules[index].time));
    schedules[index].time[5] = '\0';

    schedules[index].executed = false;
    index++;
  }

  file.close();

  Serial.print(F("Loaded "));
  Serial.print(index);
  Serial.println(F(" schedules from SD."));
}

void printSchedule(void) {
  Serial.println(F("=== Schedule List ==="));
  for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
    if (!schedules[i].enabled) continue;
    Serial.print(F("#")); Serial.print(i + 1); Serial.print(F(" | "));
    Serial.print(schedules[i].type == SCHED_FEEDER ? F("FEEDER") : F("WATER"));
    Serial.print(F(" | "));
    Serial.println(schedules[i].time);
  }
}

void checkScheduleExecution(void) {
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck < 1000) return;
  lastCheck = millis();

  DateTime now = rtc.now();
  char currentTime[6];
  snprintf(currentTime, sizeof(currentTime), "%02d:%02d", now.hour(), now.minute());

  if (now.hour() == 0 && now.minute() == 0) {
    resetScheduleExecutionAtMidnight();
  } else {
    scheduleResetDone = false;
  }

  for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
    if (!schedules[i].enabled) continue;
    if (schedules[i].executed) continue;
    if (strcmp(schedules[i].time, currentTime) == 0) {
      Serial.print(F("Executing schedule #"));
      Serial.println(i + 1);

      if (schedules[i].type == SCHED_FEEDER) startFeeder();
      else startPump();

      schedules[i].executed = true;
    }
  }
}

void resetScheduleExecutionAtMidnight(void) {
  if (scheduleResetDone) return;
  Serial.println(F("Midnight reached. Resetting schedule executed flags."));
  for (uint8_t i = 0; i < MAX_SCHEDULES; i++) schedules[i].executed = false;
  scheduleResetDone = true;
}

// -------------------- NTP sync --------------------
bool syncRTCWithNTP(bool force) {
  if (!ntpInitialized) {
    Serial.println(F("NTP not initialized."));
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print(F("WiFi not connected. Status: "));
    Serial.println(WiFi.status());
    return false;
  }
  if (!force && millis() - lastNTPSyncMillis < MIN_SYNC_INTERVAL_MS) {
    Serial.println(F("NTP sync skipped (interval)."));
    return false;
  }

  Serial.println(F("Syncing RTC with NTP..."));
  bool updated = timeClient.forceUpdate();

  if (!updated || !timeClient.isTimeSet()) {
    Serial.println(F("NTP update failed."));
    return false;
  }

  time_t ntpTime = timeClient.getEpochTime();
  DateTime rtcTime = rtc.now();
  long drift = labs((long)(ntpTime - rtcTime.unixtime()));

  Serial.print(F("RTC drift: "));
  Serial.print(drift);
  Serial.println(F(" sec"));

  if (!force && drift < RTC_DRIFT_THRESHOLD_SEC) {
    Serial.println(F("RTC drift acceptable. No update."));
    return false;
  }

  rtc.adjust(DateTime(ntpTime));
  lastNTPSyncMillis = millis();
  Serial.println(F("RTC updated from NTP."));
  printCurrentTime();
  return true;
}

void checkAndSyncRTCOnBoot(void) {
  Serial.println(F("Checking RTC against NTP on boot..."));

  if (!rtcInitialized) {
    Serial.println(F("RTC not initialized, skipping."));
    return;
  }
  if (!ntpInitialized) {
    Serial.println(F("NTP not available, using RTC time."));
    printCurrentTime();
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("WiFi not connected, skipping NTP check."));
    printCurrentTime();
    return;
  }
  if (!timeClient.isTimeSet()) {
    Serial.println(F("NTP time not set yet."));
    return;
  }

  time_t ntpTime = timeClient.getEpochTime();
  DateTime rtcTime = rtc.now();
  long drift = labs((long)(ntpTime - rtcTime.unixtime()));

  Serial.print(F("RTC vs NTP drift on boot: "));
  Serial.print(drift);
  Serial.println(F(" sec"));

  if (drift >= RTC_DRIFT_THRESHOLD_SEC) {
    Serial.println(F("Drift exceeds threshold. Updating RTC from NTP."));
    rtc.adjust(DateTime(ntpTime));
    lastNTPSyncMillis = millis();
  }
  printCurrentTime();
}

// -------------------- EEPROM/WiFi config --------------------
void readEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(SSID_ADDR, ssid);
  EEPROM.get(PASS_ADDR, pass);
  EEPROM.end();

  ssid[sizeof(ssid) - 1] = '\0';
  pass[sizeof(pass) - 1] = '\0';
}

void writeEEPROM(const char* newSsid, const char* newPass) {
  EEPROM.begin(EEPROM_SIZE);
  memset(ssid, 0, sizeof(ssid));
  memset(pass, 0, sizeof(pass));

  strncpy(ssid, newSsid, sizeof(ssid) - 1);
  strncpy(pass, newPass, sizeof(pass) - 1);

  EEPROM.put(SSID_ADDR, ssid);
  EEPROM.put(PASS_ADDR, pass);
  EEPROM.commit();
  EEPROM.end();
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  Serial.print("Connecting to WiFi");
  connectStart = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    if (millis() - connectStart > WIFI_TIMEOUT) {
      Serial.println("\nWiFi Timeout!");
      return false;
    }
  }

  Serial.println("\nConnected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  return true;
}

String configPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>Smart Pet Cage WiFi Setup</title></head>
<body>
  <h2>WiFi Configuration</h2>
  <form action="/save" method="POST">
    SSID:<br>
    <input type="text" name="ssid"><br>
    Password:<br>
    <input type="password" name="pass"><br><br>
    <input type="submit" value="Save">
  </form>
  <form action="/retry" method="post">
    <button type="submit">Retry Saved WiFi</button>
  </form>
</body>
</html>
)rawliteral";
}

void startConfigAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32S3_SmartPetCage");

  Serial.println("AP Mode Started");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() {
    server.send(200, "text/html", configPage());
  });

  server.on("/save", HTTP_POST, []() {
    String newSsid = server.arg("ssid");
    String newPass = server.arg("pass");

    writeEEPROM(newSsid.c_str(), newPass.c_str());

    server.send(200, "text/html", "<h3>Saved! Rebooting...</h3>");
    delay(1500);
    ESP.restart();
  });

  server.on("/retry", HTTP_POST, []() {
    server.send(200, "text/plain", "Rebooting to retry saved WiFi.");
    delay(300);
    ESP.restart();
  });

  server.begin();
}

// -------------------- Firebase schedule read --------------------
static String withAuth(String url) {
  if (strlen(FIREBASE_AUTH) > 0) {
    url += "?auth=";
    url += FIREBASE_AUTH;
  }
  return url;
}

void printScheduleFromFirebase(void) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[Firebase] WiFi not connected."));
    return;
  }

  fbClient.setInsecure();
  const char* types[] = { "feeder", "water" };

  for (uint8_t t = 0; t < 2; t++) {
    Serial.println();
    Serial.print(F("=== "));
    Serial.print(types[t]);
    Serial.println(F(" schedules ==="));

    for (uint8_t i = 1; i <= 4; i++) {
      String url = String(FIREBASE_URL) + "/schedules/" + types[t] + "/" + i + ".json";
      url = withAuth(url);

      http.begin(fbClient, url);
      int httpCode = http.GET();

      if (httpCode != HTTP_CODE_OK) {
        Serial.print(F("#")); Serial.print(i);
        Serial.print(F(" ERROR ")); Serial.println(httpCode);
        http.end();
        continue;
      }

      String payload = http.getString();
      http.end();

      bool enabled = payload.indexOf("\"enabled\":\"true\"") >= 0;

      String time = "--:--";
      int timePos = payload.indexOf("\"time\"");
      if (timePos != -1) {
        int colon = payload.indexOf(":", timePos);
        int q1 = payload.indexOf("\"", colon + 1);
        int q2 = payload.indexOf("\"", q1 + 1);
        if (q1 != -1 && q2 != -1) {
          String extracted = payload.substring(q1 + 1, q2);
          if (extracted.length() == 5) time = extracted;
        }
      }

      Serial.print(F("#")); Serial.print(i);
      Serial.print(F(" | "));
      Serial.print(enabled ? F("ENABLED") : F("DISABLED"));
      Serial.print(F(" | "));
      Serial.println(time);
    }
  }
  Serial.println(F("[Firebase] Schedule read complete."));
}

bool loadScheduleFromFirebaseToRAM(void) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[Firebase] WiFi not connected"));
    return false;
  }

  fbClient.setInsecure();
  Serial.println(F("[Firebase] Loading schedules into RAM..."));

  for (int i = 0; i < MAX_SCHEDULES; i++) {
    schedules[i].enabled = false;
    schedules[i].executed = false;
    schedules[i].time[0] = '\0';
  }

  const char* types[] = { "feeder", "water" };

  for (int t = 0; t < 2; t++) {
    ScheduleType schedType = (t == 0) ? SCHED_FEEDER : SCHED_WATER;

    for (int slot = 1; slot <= 4; slot++) {
      String url = String(FIREBASE_URL) + "/schedules/" + types[t] + "/" + slot + ".json";
      url = withAuth(url); // FIX: auth was missing in your original RAM loader

      http.begin(fbClient, url);
      int code = http.GET();

      if (code != HTTP_CODE_OK) {
        Serial.print(F("[Firebase] HTTP error "));
        Serial.println(code);
        http.end();
        continue;
      }

      String payload = http.getString();
      http.end();

      bool enabled = payload.indexOf("\"enabled\":\"true\"") >= 0;

      String time = "--:--";
      int timePos = payload.indexOf("\"time\"");
      if (timePos != -1) {
        int colon = payload.indexOf(":", timePos);
        int q1 = payload.indexOf("\"", colon + 1);
        int q2 = payload.indexOf("\"", q1 + 1);
        if (q1 != -1 && q2 != -1) {
          String extracted = payload.substring(q1 + 1, q2);
          if (extracted.length() == 5) time = extracted;
        }
      }

      int index = (schedType == SCHED_FEEDER ? 0 : 4) + (slot - 1);

      schedules[index].enabled = enabled;
      schedules[index].type = schedType;
      strncpy(schedules[index].time, time.c_str(), 6);
      schedules[index].time[5] = '\0';
      schedules[index].executed = false;

      Serial.print(F("[RAM] "));
      Serial.print(types[t]);
      Serial.print(F(" #"));
      Serial.print(slot);
      Serial.print(F(" | "));
      Serial.print(enabled ? F("ENABLED") : F("DISABLED"));
      Serial.print(F(" | "));
      Serial.println(time);
    }
  }

  Serial.println(F("[Firebase] RAM schedule load complete"));
  return true;
}

// -------------------- Sensors/Alerts --------------------
void readAHTdata(void) {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  ahtTemperature = temp.temperature;
  ahtHumidity = humidity.relative_humidity;
}

void readBMPdata(void) {
  bmpPressure = bmp.readPressure() / 100.0F;
  bmpAltitude = bmp.readAltitude();
}

void readMotionSensorData(void) {
  motionDetected = digitalRead(MOTION_SENSOR_PIN);
}

void readSensorsData(void) {
  if (millis() - lastSensorReadTime < SENSOR_READ_INTERVAL_MS) return;
  lastSensorReadTime = millis();

  // FIX: do not block motion reading if aht/bmp failed
  if (ahtInitialized) readAHTdata();
  if (bmpInitialized) readBMPdata();
  readMotionSensorData();

  checkAlerts();
}

void checkAlerts(void) {
  if (ahtInitialized) {
    if (ahtTemperature >= tempHighThreshold) {
      if (!tempHighAlertActive) {
        Serial.print(F("[ALERT] Temp HIGH: "));
        Serial.print(ahtTemperature);
        Serial.println(F(" C"));
        tempHighAlertActive = true;
      }
    } else tempHighAlertActive = false;

    if (ahtTemperature <= tempLowThreshold) {
      if (!tempLowAlertActive) {
        Serial.print(F("[ALERT] Temp LOW: "));
        Serial.print(ahtTemperature);
        Serial.println(F(" C"));
        tempLowAlertActive = true;
      }
    } else tempLowAlertActive = false;

    if (ahtHumidity >= humHighThreshold) {
      if (!humHighAlertActive) {
        Serial.print(F("[ALERT] Humidity HIGH: "));
        Serial.print(ahtHumidity);
        Serial.println(F(" %"));
        humHighAlertActive = true;
      }
    } else humHighAlertActive = false;

    if (ahtHumidity <= humLowThreshold) {
      if (!humLowAlertActive) {
        Serial.print(F("[ALERT] Humidity LOW: "));
        Serial.print(ahtHumidity);
        Serial.println(F(" %"));
        humLowAlertActive = true;
      }
    } else humLowAlertActive = false;
  }

  if (MOTION_ALERT_ENABLED && motionDetected) {
    if (!motionAlertActive) {
      Serial.println(F("[ALERT] Motion detected!"));
      motionAlertActive = true;
    }
  } else motionAlertActive = false;
}

void showSensorsData(void) {
  Serial.print(F("Temp: ")); Serial.print(ahtTemperature); Serial.println(F(" C"));
  Serial.print(F("Hum : ")); Serial.print(ahtHumidity);    Serial.println(F(" %"));
  Serial.print(F("Pres: ")); Serial.print(bmpPressure);    Serial.println(F(" hPa"));
  Serial.print(F("Alt : ")); Serial.print(bmpAltitude);    Serial.println(F(" m"));
  Serial.print(F("Motion: ")); Serial.println(motionDetected ? "TRUE" : "FALSE");
}

// -------------------- Firebase data PATCH --------------------
void sendSensorDataToFirebase(void) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[Firebase] WiFi not connected, aborting data send."));
    return;
  }

  fbClient.setInsecure();
  String url = String(FIREBASE_URL) + "/data.json";
  url = withAuth(url);

  String payload = "{";
  payload += "\"temperature\":" + String(ahtTemperature, 2) + ",";
  payload += "\"humidity\":"    + String(ahtHumidity, 2) + ",";
  payload += "\"pressure\":"    + String(bmpPressure, 2) + ",";
  payload += "\"altitude\":"    + String(bmpAltitude, 2) + ",";
  payload += "\"motion\":"      + String(motionDetected ? "true" : "false") + ",";
  payload += "\"timestamp\":"   + String(rtc.now().unixtime());
  payload += "}";

  http.begin(fbClient, url);
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.PATCH(payload);

  if (httpCode > 0) {
    Serial.print(F("[Firebase] PATCH code: "));
    Serial.println(httpCode);
    if (httpCode == HTTP_CODE_OK) {
      Serial.println(F("[Firebase] Sensor data updated."));
    } else {
      Serial.println(http.getString());
    }
  } else {
    Serial.print(F("[Firebase] PATCH failed: "));
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}

void sendSensorDataPeriodically(void) {
  if (!isSendDataPeriodically) return;
  if (millis() - lastDataPatch < SENSOR_DATA_PATCH_INTERVAL_MS) return;
  lastDataPatch = millis();
  sendSensorDataToFirebase();
}

// -------------------- Threshold functions (kept as-is, optional to improve later) --------------------
void updateSensorThresholdFromFirebase(void) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[Threshold] WiFi not connected"));
    return;
  }

  fbClient.setInsecure();
  String url = String(FIREBASE_URL) + "/threshold.json";
  url = withAuth(url);

  Serial.println(F("[Threshold] Fetching threshold..."));
  http.begin(fbClient, url);
  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK) {
    Serial.print(F("[Threshold] HTTP error "));
    Serial.println(httpCode);
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  if (payload.length() < 10) {
    Serial.println(F("[Threshold] Empty payload"));
    return;
  }

  int tPos = payload.indexOf("\"temperature\"");
  if (tPos != -1) {
    tempLowThreshold  = payload.substring(payload.indexOf("\"low\":", tPos) + 6).toFloat();
    tempHighThreshold = payload.substring(payload.indexOf("\"high\":", tPos) + 7).toFloat();
  }

  int hPos = payload.indexOf("\"humidity\"");
  if (hPos != -1) {
    humLowThreshold  = payload.substring(payload.indexOf("\"low\":", hPos) + 6).toFloat();
    humHighThreshold = payload.substring(payload.indexOf("\"high\":", hPos) + 7).toFloat();
  }

  Serial.println(F("[Threshold] Updated."));
  checkSensorThreshold();
}

void checkSensorThreshold(void) {
  Serial.println("[CMD] Threshold Check");
  Serial.print("Temperature: High="); Serial.print(tempHighThreshold);
  Serial.print(" C, Low="); Serial.print(tempLowThreshold); Serial.println(" C");
  Serial.print("Humidity: High="); Serial.print(humHighThreshold);
  Serial.print("%, Low="); Serial.print(humLowThreshold); Serial.println("%");
}

void toggleSendDataToFirebase(void) {
  isSendDataPeriodically = !isSendDataPeriodically;
  Serial.print(F("[CMD] Periodic send = "));
  Serial.println(isSendDataPeriodically ? "TRUE" : "FALSE");
}

void showAlertStatus(void) {
  Serial.println("=== Alert Status ===");
  Serial.print("Temp High: "); Serial.println(tempHighAlertActive ? "ACTIVE" : "INACTIVE");
  Serial.print("Temp Low : "); Serial.println(tempLowAlertActive ? "ACTIVE" : "INACTIVE");
  Serial.print("Hum High : "); Serial.println(humHighAlertActive ? "ACTIVE" : "INACTIVE");
  Serial.print("Hum Low  : "); Serial.println(humLowAlertActive ? "ACTIVE" : "INACTIVE");
  Serial.print("Motion   : "); Serial.println(motionAlertActive ? "ACTIVE" : "INACTIVE");
}

// -------------------- LCD UI --------------------
void handleMenuNavigation(int direction) {
  currentMenu += direction;
  if (currentMenu < 0) currentMenu = MENU_COUNT - 1;
  else if (currentMenu >= MENU_COUNT) currentMenu = 0;

  // If user navigates away, go back to menu view
  showDataScreen = false;
  menuDirty = true;
}

void renderMenuUI(void) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("< MENU >");

  lcd.setCursor(0, 1);
  const char* title = menuNames[currentMenu];
  int len = strlen(title);
  int padding = (LCD_COL - len) / 2;
  for (int i = 0; i < padding; i++) lcd.print(" ");
  lcd.print(title);
}

void renderDataUI(void) {
  lcd.clear();

  // Row 0: T/H
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(ahtTemperature, 1);
  lcd.print("C H:");
  lcd.print(ahtHumidity, 0);
  lcd.print("%");

  // Row 1: Motion + Pump state
  lcd.setCursor(0, 1);
  lcd.print("M:");
  lcd.print(motionDetected ? "1" : "0");
  lcd.print(" P:");
  bool pumpOn = (actuatorState & _BV(PUMP_RELAY)) ? false : true;
  lcd.print(pumpOn ? "ON " : "OFF");
  lcd.print(isSendDataPeriodically ? " FB" : " --");
}

void updateDisplayUI(void) {
  unsigned long now = millis();
  if (!menuDirty && (now - lastUIUpdate < UI_REFRESH_INTERVAL_MS)) return;

  lastUIUpdate = now;
  menuDirty = false;

  if (currentMenu == MENU_SHOW_DATA && showDataScreen) renderDataUI();
  else renderMenuUI();
}

// -------------------- Serial commands --------------------
void printSerialCommandList(void) {
  Serial.println(F("=== Serial Command List ==="));
  Serial.println(F("1 - Toggle Gate Relay"));
  Serial.println(F("2 - Activate Pump Relay"));
  Serial.println(F("3 - Toggle Fan Relay"));
  Serial.println(F("4 - Activate Feeder"));
  Serial.println(F("t - Print Current Time"));
  Serial.println(F("s - Print Schedule"));
  Serial.println(F("x - Set RTC to 2026-01-01 23:59:30 (test)"));
  Serial.println(F("n - Sync RTC with NTP (force)"));
  Serial.println(F("f - Print schedule from Firebase"));
  Serial.println(F("r - Load schedule from Firebase to RAM"));
  Serial.println(F("l - Load schedule from SD"));
  Serial.println(F("w - Save schedule to SD"));
  Serial.println(F("y - Show sensors data"));
  Serial.println(F("o - Toggle periodic data send to Firebase"));
  Serial.println(F("p - Send sensor data to Firebase (once)"));
  Serial.println(F("h - Show threshold"));
  Serial.println(F("H - Fetch threshold from Firebase"));
  Serial.println(F("a - Show alert status"));
  Serial.println(F("i - Print this list"));
}

void checkSerialCommand(void) {
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1': toggleGate(); break;
      case '2': startPump(); break;
      case '3': toggleFan(); break;
      case '4': startFeeder(); break;
      case 't': printCurrentTime(); break;
      case 's': printSchedule(); break;
      case 'x':
        rtc.adjust(DateTime(2026, 1, 1, 23, 59, 30));
        Serial.println(F("RTC manually set for testing."));
        break;
      case 'n': syncRTCWithNTP(true); break;
      case 'f': printScheduleFromFirebase(); break;
      case 'r': loadScheduleFromFirebaseToRAM(); break;
      case 'l': loadScheduleFromSDCard(); break;
      case 'w': saveScheduleToSDCard(); break;
      case 'y': showSensorsData(); break;
      case 'i': printSerialCommandList(); break;
      case 'o': toggleSendDataToFirebase(); break;
      case 'p': sendSensorDataToFirebase(); break;
      case 'h': checkSensorThreshold(); break;
      case 'H': updateSensorThresholdFromFirebase(); break;
      case 'a': showAlertStatus(); break;
      default: break;
    }
  }
}

// -------------------- Save schedule to SD --------------------
bool saveScheduleToSDCard(void) {
  if (!sdInitialized) {
    Serial.println(F("[SD] SD not initialized."));
    return false;
  }
  if (SD.exists("/schedule.csv")) SD.remove("/schedule.csv");

  File file = SD.open("/schedule.csv", FILE_WRITE);
  if (!file) {
    Serial.println(F("[SD] Failed to open schedule.csv for writing."));
    return false;
  }

  file.println(F("enabled,type,time"));
  uint8_t count = 0;
  for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
    if (!schedules[i].enabled) continue;
    file.print('1'); file.print(',');
    file.print(schedules[i].type == SCHED_FEEDER ? F("FEEDER") : F("WATER"));
    file.print(',');
    file.println(schedules[i].time);
    count++;
  }
  file.close();

  Serial.print(F("[SD] Saved "));
  Serial.print(count);
  Serial.println(F(" schedules."));
  return true;
}