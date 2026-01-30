#include <Arduino.h>
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
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include "esp_heap_caps.h"
#include "Dog_Sound_Detector_inferencing.h"
#include "esp_task_wdt.h"

#define L_BUTTON      P0
#define C_BUTTON      P1
#define R_BUTTON      P2
#define GATE_BUTTON   P3
#define PUMP_BUTTON   P4
#define FAN_BUTTON    P5
#define FEEDER_BUTTON P6

#define GATE_RELAY    P0
#define PUMP_RELAY    P1
#define FAN_RELAY     P2
#define FEEDER_PIN    4
#define MOTION_SENSOR_PIN 5
#define RGB_LED       48
#define LIMIT_SWITCH  15

#define WIFI_CONNECT_IND    P3
#define SD_CARD_IND         P4
#define DOG_BARK_IND        P5
#define FIREBASE_RX_IND     P6
#define FIREBASE_TX_IND     P7

#define FOOD_SENSOR_ECHO_PIN 47
#define FOOD_SENSOR_TRIG_PIN 21

#define WATER_SENSOR_ECHO_PIN 17
#define WATER_SENSOR_TRIG_PIN 18

#define MICROPHONE_PIN  6

#define SD_CS   10
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK  12

#define PCF8574_ADDRESS_1 0x20
#define PCF8574_ADDRESS_2 0x21

#define PUMP_ACTIVE_DUR               3500
#define FEEDER_ACTIVE_DUR             1500
#define DEBOUNCE_DELAY_MS             50      // Debounce delay for button press
#define PCF_READ_INTERVAL_MS          30
#define SENSOR_READ_INTERVAL_MS       1000
#define UI_REFRESH_INTERVAL_MS        200
#define SENSOR_DATA_PATCH_INTERVAL_MS 6000    // Keep the system responsive
#define COMMAND_POLL_INTERVAL_MS      5000
#define ACTUATOR_CONTROL_COOLDOWN     5000    // 5s cooldown
#define HEARTBEAT_INTERVAL_MS        10000   // 10s

#define TIMEZONE_OFFSET_SEC         (7 * 3600)
#define NTP_SYNC_INTERVAL           (6UL * 60UL * 60UL * 1000UL)
#define RTC_DRIFT_THRESHOLD_SEC     5
#define MIN_SYNC_INTERVAL_MS        (6UL * 60UL * 60UL * 1000UL) // 6 hours
#define WIFI_TIMEOUT                20000UL     // 20 seconds

#define MAX_SCHEDULES   8

#define EEPROM_SIZE 96
#define SSID_ADDR 0
#define PASS_ADDR 32

// Default Alert System Threshold
#define DEF_HIGH_TEMP_THRESHOLD 31
#define DEF_LOW_TEMP_THRESHOLD  20
#define DEF_HIGH_HUM_THRESHOLD  75
#define DEF_LOW_HUM_THRESHOLD   45
#define MOTION_ALERT_ENABLED    false   // We don't need to alert this for now
#define LOW_FOOD_THRESHOLD      20
#define HIGH_FOOD_THRESHOLD     80
#define LOW_WATER_THRESHOLD     20
#define HIGH_WATER_THRESHOLD    80

#define DEF_SEND_DATA_PERIODICALLY  true
#define DEF_POLL_CMD_PERIODICALLY   true

#define LCD_ROW 2
#define LCD_COL 16

#define NUM_PIXELS 1

#define SLIDESHOW_SCREEN_DURATION_MS  3000   // Auto-advance every 3 seconds
#define SLIDESHOW_SCREEN_COUNT        6 

#define MIC_TASK_STACK    32768  // bytes (adjust if needed)
#define MIC_TASK_PRIO     1
#define MIC_TASK_CORE     0

#define SAMPLE_RATE       20000          // 20 kHz
#define SAMPLE_LENGTH     EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
static_assert(SAMPLE_LENGTH == 20000, "SAMPLE_LENGTH expected 20000");

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
Adafruit_NeoPixel rgb(NUM_PIXELS, RGB_LED, NEO_GRB + NEO_KHZ800);
Servo feeder;

enum ScheduleType : uint8_t {
  SCHED_FEEDER,
  SCHED_WATER
};

enum SlideshowScreen : uint8_t {
    SLIDE_TEMP_HUM = 0,
    SLIDE_PRESSURE_ALT,
    SLIDE_WATER_LEVEL,
    SLIDE_FOOD_LEVEL,
    SLIDE_MOTION_GATE,
    SLIDE_SYSTEM_INFO,
    SLIDE_COUNT
};

enum ScheduleViewMode : uint8_t {
    SCHED_VIEW_MAIN_MENU = 0,
    SCHED_VIEW_FEEDER_LIST,
    SCHED_VIEW_WATER_LIST,
    SCHED_VIEW_FEEDER_DETAIL,
    SCHED_VIEW_WATER_DETAIL,
    SCHED_VIEW_BACK_OPTION,
    SCHED_VIEW_COUNT
};

enum AlertThresholdViewMode : uint8_t {
    ALERT_VIEW_MAIN_MENU = 0,
    ALERT_VIEW_TEMPERATURE,
    ALERT_VIEW_HUMIDITY,
    ALERT_VIEW_WATER_LEVEL,
    ALERT_VIEW_FOOD_LEVEL,
    ALERT_VIEW_BACK_OPTION,
    ALERT_VIEW_COUNT
};

enum MenuID : uint8_t {
    MENU_SHOW_DATA = 0,
    MENU_CHECK_SCHEDULE,
    MENU_CHECK_THRESHOLD,
    MENU_COUNT
};

struct ScheduleSlot {
  bool enabled;
  ScheduleType type;
  char time[6];     // "HH:MM"
  bool executed;    // runtime-only
};

struct ActuatorTimer {
    bool active;
    unsigned long startTime;
    unsigned long duration;
};

struct Command {
    String key;
    String action;
    String target;
};

ActuatorTimer pumpTimer   = { false, 0, PUMP_ACTIVE_DUR };
ActuatorTimer feederTimer = { false, 0, FEEDER_ACTIVE_DUR };
ScheduleSlot schedules[MAX_SCHEDULES];
Command ongoingCommand = {"", "", ""};

bool rtcInitialized = false;
bool sdInitialized = false;
bool ntpInitialized = false;
bool isConfigMode = false;
bool scheduleResetDone = false;
bool ahtInitialized = false;
bool bmpInitialized = false;
bool rgbInitialized = false;

bool isSendDataPeriodically = DEF_SEND_DATA_PERIODICALLY;
bool isCommandPollPeriodically = DEF_POLL_CMD_PERIODICALLY;
bool isFullFillingActive = false;
bool isFanAutoMode = false;
bool isFanAutoControlActive = false;

bool isCommandExecuted = false;

byte buttonState = 0b01111111; // Only use 7 bit
byte lastButtonState = 0b01111111; // Only use 7 bit
byte rawButtonState = 0b01111111; // Only use 7 bit
byte stableButtonState = 0b01111111; // Only use 7 bit
byte lastRawButtonState = 0b01111111; // Only use 7 bit

unsigned long lastDebounceTime = 0;
unsigned long lastPCFReadTime = 0;
unsigned long lastNTPSyncMillis = 0;
unsigned long connectStart = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastUIUpdate = 0;
unsigned long lastDataPatch = 0;
unsigned long lastCommandPoll = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastBarkCountReset = 0;
unsigned long lastBarkDetectionTime = 0;

byte actuatorState = 0b00000111; // Only use 3 bit for now
byte lastActuatorState = 0b00000111; // Only use 3 bit for now

bool isFeederRunning = false;
bool isGateSwitchClosed = false;

char ssid[32];
char pass[64];

/* Sensors Data */
float ahtTemperature = 0.0;
float ahtHumidity = 0.0;
float bmpPressure = 0.0;
float bmpAltitude = 0.0; 
bool motionDetected = false;
int16_t micAnalogValue = 0;
float foodLevelPercent = 0.0;
float waterLevelPercent = 0.0;
float rawFoodSensorDistance = 0.0;
float rawWaterSensorDistance = 0.0;

/* Threshold Storage (Use default for now) */ 
float tempHighThreshold = DEF_HIGH_TEMP_THRESHOLD;
float tempLowThreshold = DEF_LOW_TEMP_THRESHOLD;
float humHighThreshold = DEF_HIGH_HUM_THRESHOLD;
float humLowThreshold = DEF_LOW_HUM_THRESHOLD;
float foodLowThreshold = LOW_FOOD_THRESHOLD;
float foodHighThreshold = HIGH_FOOD_THRESHOLD;
float waterLowThreshold = LOW_WATER_THRESHOLD;
float waterHighThreshold = HIGH_WATER_THRESHOLD;

float foodFullDistance = 27.0;      // cm
float foodEmptyDistance = 30.0;     // cm
float waterFullDistance = 27.0;     // cm
float waterEmptyDistance = 30.0;    // cm
float tempDecreaseTarget = 2.0f;

bool tempHighAlertActive = false;
bool tempLowAlertActive = false;
bool humHighAlertActive = false;
bool humLowAlertActive = false;
bool motionAlertActive = false;
bool foodLowAlertActive = false;
bool waterLowAlertActive = false;
bool barkAlertActive = false;
bool abnormalBehaviorAlertActive = false;

const char* menuNames[MENU_COUNT] = {
    "Show Data",
    "Check Schedule",
    "Check Threshold"
};

volatile int8_t currentMenu = 0;
bool menuDirty = true;

int8_t rgbMode = 0;

/* LCD Slideshow State */
uint8_t currentSlideshowScreen = SLIDE_TEMP_HUM;
unsigned long lastSlideshowAdvance = 0;
bool slideshowActive = false;

/* Schedule View State */
uint8_t scheduleViewMode = SCHED_VIEW_MAIN_MENU;
uint8_t selectedScheduleIndex = 0;  // For detail view
uint8_t currentScheduleIndex = 0; 
unsigned long lastScheduleViewUpdate = 0;
bool scheduleViewDirty = true;

/* Alert Threshold View State */
uint8_t alertThresholdViewMode = ALERT_VIEW_MAIN_MENU;
uint8_t selectedAlertIndex = 0;  // For cycling through thresholds
unsigned long lastAlertViewUpdate = 0;
bool alertViewDirty = true;

// ---- shared status (protected with spinlock) ----
volatile bool mic_ready = false;
float mic_level_rms = 0.0f;    // normalized RMS [0..1]
uint8_t mic_level = 0;         // scaled 0..100 for human/UI
float bark_confidence = 0.0f;  // classifier probability for 'bark' class
bool  bark_detected = false;   // derived boolean (thresholded)

portMUX_TYPE micMux = portMUX_INITIALIZER_UNLOCKED; // protect writes/reads

// ---- runtime calibration (for MAX9814 gain floating) ----
static float ambient_rms_ema = 0.0f;   // exponential moving average of 'quiet' RMS
const float AMBIENT_ALPHA = 0.01f;     // smoothing factor (lower -> slower)

// choose thresholds for dB->level mapping (tune later)
const float MIN_REL_DB = -50.0f; // corresponds to very quiet relative to ambient
const float MAX_REL_DB = 20.0f;  // loudest you expect (above ambient)

const float BARK_CONF_THRESHOLD = 0.65f; // bark detection threshold

// ---- audio buffer ----
static int16_t audio_buffer[SAMPLE_LENGTH]; // 20000 int16_t
static float *ei_input_global = nullptr;

uint16_t barkCount;
uint16_t barkCountThreshold = 6;
unsigned long BARK_COUNT_RESET_PERIOD = 60000; // Clean per minute

// Default Value
unsigned long feederEnableDuration = FEEDER_ACTIVE_DUR;
unsigned long pumpEnableDuration = PUMP_ACTIVE_DUR;

// System Config (Firebase)
String petName = "My Pet";

void initializeButtons (void);
void initializeRelays (void);
void initializeFeeder (void); 
void initializeAHT (void);
void initializeBMP (void);
void initializeMotionSensor (void);
void initializeSensors (void);
void initializeDisplay (void);
void initializeRTC (void);
void initializeSDCardReader (void);
void initializeWiFi (void);
void initializeNTP (void);
void initializeRGB (void);
void initializeMic (void);
void initializeGateLimitSwitch (void);

void readRawButtonInput (void);
void updateButtonInput (void);
void checkButtonStateChange (void);
void checkRelayActivity (void);
void checkSerialCommand (void);
void checkScheduleExecution (void);
void checkAndSyncRTCOnBoot(void);
void resetScheduleExecutionAtMidnight (void);
void printCurrentTime (void);
void printSchedule (void);
void printSDCardInfo (void);
void loadScheduleFromSDCard (void);
ScheduleType parseScheduleType (const char* str);
bool syncRTCWithNTP (bool force = false);
bool updateScheduleFromCloud (void);
void applyRelayState (void);
void printScheduleFromFirebase (void);
bool loadScheduleFromFirebaseToRAM (void);
void printSerialCommandList (void);
void checkAlerts (void);
void showAlertStatus (void);
void handleMenuNavigation (int direction);
void renderMenuUI (void);
void updateDisplayUI (void);
bool saveScheduleToSDCard (void);
void sendSensorDataToFirebase (void);
void sendSensorDataPeriodically (void);
void updateSensorThresholdFromFirebase (void);
void checkSensorThreshold (void);
void toggleSendDataToFirebase (void);
void showActuatorState (void);
bool fetchCommandFromFirebase(void);
void fetchCommandFromFirebasePeriodically (void);
void toggleCommandPolling (void);
void executeCommand (const String& action, const String& target);
bool deleteCommandFromFirebase (const String& commandKey);
void syncSchedule (void);
void syncSensorThreshold (void);
void processCommand (void);
void cleanOngoingCommand (void);
void loadDefaultSchedule (void);
bool sendLogToFirebase (
    const String& category,
    const String& event,
    const String& target,
    int value,
    const String& source,
    const String& reason
);
void logEventToSDCard (
    const char* category,
    const char* source,
    const char* action,
    const char* value = ""
);
void sendActuatorStateToFirebase (void);

void readEEPROM();
void writeEEPROM(const char* newSsid, const char* newPass);
bool connectWiFi();
String configPage();
void startConfigAP();

void startPump (void);
void stopPump (void);
void startFeeder (void);
void stopFeeder (void);

void toggleGate (void);
void toggleFan (void);
void enableGate (void);
void enableFan (void);
void disableGate (void);
void disableFan (void);

void readAHTdata (void);
void readBMPdata (void);
void readMotionSensorData (void);
void readMicData (void);
void readLimitSwitchData (void);
void readSensorsData (void);
void showSavedAHTdata (void);
void showSavedBMPdata (void);
void showMotionSensorData (void);
void showMicData (void);
void showGateSwitchData (void);
void showSensorsData (void);
void checkOccupationThresholds (void);
void checkFanAutoControl (void);
void displaySensorsDataOnLCD (void);
void updateRGBMode (void);
void watchdog (void);                   // Additional features (develop later)
void sendDeviceHeartbeat (void);        // Additional features (develop later)

// In Development
void initializeFoodLevelSensor (void);
void initializeWaterLevelSensor (void);
void readRawFoodSensorDistance (void);
void readRawWaterSensorDistance (void);
void checkFoodLevel (void);
void checkWaterLevel (void);
void showFoodLevelData (void);
void showWaterLevelData (void);

// Actuator Control Helper Function
void setRelayOn (uint8_t relayBitPos);
void setRelayOff (uint8_t relayBitPos);
void toggleRelay (uint8_t relayBitPos);

void handleRoot (void);
void handleSave (void);
void handleRetry (void);
void printHeap (void);
String getTimestamp();

String withAuth(String url);

void startSensorDataSlideshow(void);
void updateSensorDataSlideshow(void);
void nextSlideshowScreen(void);
void previousSlideshowScreen(void);
void displaySlideshowScreen(uint8_t screenID);
void displaySlideTemperatureHumidity(void);
void displaySlidePressureAltitude(void);
void displaySlideWaterLevel(void);
void displaySlideFoodLevel(void);
void displaySlideMotionGate(void);
void displaySlideSystemInfo(void);

void enterScheduleViewMenu(void);
void exitScheduleViewMenu(void);
void updateScheduleViewMenu(void);
void displayScheduleViewMenu(void);
void displayScheduleMainMenu(void);
void displayFeederScheduleList(void);
void displayWaterScheduleList(void);
void displayScheduleDetail(uint8_t index);
void navigateScheduleMenu(int direction);
void selectScheduleItem(void);
void goBackScheduleView(void);
void displayBackOption(void);

void enterAlertThresholdMenu(void);
void exitAlertThresholdMenu(void);
void updateAlertThresholdView(void);
void displayAlertThresholdMenu(void);
void displayAlertMainMenu(void);
void displayTemperatureThreshold(void);
void displayHumidityThreshold(void);
void displayWaterLevelThreshold(void);
void displayFoodLevelThreshold(void);
void displayAlertBackOption(void);
void navigateAlertMenu(int direction);
void selectAlertItem(void);

void processMenuSelection(void);
void displayConfigMenu(void);

void startMicTask();
void micTask(void* pvParameters);
float computeRMSFromBuffer(const int16_t *buf, size_t len);
void normalizeToFloatBuffer(const int16_t *in, size_t len, float *out);
int ei_get_data(size_t offset, size_t length, float *out_ptr);
void appendMicInferenceToJSON(String &json);
float mic_get_rms();
uint8_t mic_get_level();
float mic_get_bark_confidence();
bool mic_is_bark_detected();
void readMicValuesExample();
void displayMicrophoneNoisePage();
void displayBarkPage();

void getConfiguration (void);
void updateBarkCounter (void);

void setup () {
    Serial.begin(115200);
    initializeButtons();
    initializeRelays();
    initializeFeeder();
    initializeSensors();
    initializeMic();
    initializeDisplay();
    initializeRGB();
    initializeRTC();
    initializeSDCardReader();

    startMicTask();

    /* WiFi Setup */ 
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(100);
    
    readEEPROM();

    if (strlen(ssid) > 0) {
        Serial.println(F("Found saved WiFi credentials"));
        if (connectWiFi()) {
            Serial.print(F("WiFi connected using stored credentials: "));
            Serial.println(ssid);
        } else {
            Serial.println(F("WiFi failed, entering config mode"));
            isConfigMode = true;
            startConfigAP();
            displayConfigMenu();
            return; // stop normal boot
        }
    } else {
        Serial.println(F("No WiFi credentials found"));
        isConfigMode = true;
        startConfigAP();
        displayConfigMenu();
        return;
    }

    initializeNTP();
    checkAndSyncRTCOnBoot();
    syncSchedule();
    syncSensorThreshold();
    getConfiguration();

    Serial.println(F("Setup completed."));
    Serial.println(F("Try to tap the buttons or send \'i\' to check available commands."));
}

void loop () {
    readRawButtonInput();
    updateButtonInput();
    checkRelayActivity();
    checkScheduleExecution();
    checkSerialCommand();
    readSensorsData();
    sendSensorDataPeriodically();
    fetchCommandFromFirebasePeriodically();
    updateSensorDataSlideshow(); 
    sendDeviceHeartbeat();
    checkOccupationThresholds();
    checkFanAutoControl();

    if (isConfigMode) {
        server.handleClient();
        delay(5);
    }

    updateDisplayUI();
}

void initializeButtons (void) {
    pcf1.pinMode(L_BUTTON, INPUT);
    pcf1.pinMode(C_BUTTON, INPUT);
    pcf1.pinMode(R_BUTTON, INPUT);
    pcf1.pinMode(GATE_BUTTON, INPUT);
    pcf1.pinMode(PUMP_BUTTON, INPUT);
    pcf1.pinMode(FAN_BUTTON, INPUT);
    pcf1.pinMode(FEEDER_BUTTON, INPUT);

    if (!pcf1.begin()) {
        Serial.println(F("ERROR: Could not initialize buttons\' PCF8574! Check wiring, I2C address, SDA/SCL connections and power."));
        while (1) delay(100);
    }

    Serial.println(F("buttons\' PCF8574 initialized successfully."));
}
void initializeRelays (void) {
    pcf2.pinMode(GATE_RELAY, OUTPUT);
    pcf2.pinMode(PUMP_RELAY, OUTPUT);
    pcf2.pinMode(FAN_RELAY, OUTPUT);

    pcf2.pinMode(WIFI_CONNECT_IND, OUTPUT);
    pcf2.pinMode(SD_CARD_IND, OUTPUT);
    pcf2.pinMode(DOG_BARK_IND, OUTPUT);
    pcf2.pinMode(FIREBASE_RX_IND, OUTPUT);
    pcf2.pinMode(FIREBASE_TX_IND, OUTPUT);

    if (!pcf2.begin()) {
        Serial.println(F("ERROR: Could not initialize relays\' PCF8574! Check wiring, I2C address, SDA/SCL connections and power."));
        while (1) delay(100);
    }

    // Set relay state to low on beginning.
    pcf2.digitalWrite(GATE_RELAY, HIGH);
    pcf2.digitalWrite(PUMP_RELAY, HIGH);
    pcf2.digitalWrite(FAN_RELAY, HIGH);

    pcf2.digitalWrite(WIFI_CONNECT_IND, HIGH);
    pcf2.digitalWrite(SD_CARD_IND, HIGH);
    pcf2.digitalWrite(DOG_BARK_IND, HIGH);
    pcf2.digitalWrite(FIREBASE_RX_IND, HIGH);
    pcf2.digitalWrite(FIREBASE_TX_IND, HIGH);

    Serial.println(F("relays\' PCF8574 initialized successfully."));
}
void initializeFeeder (void) {
    feeder.setPeriodHertz(50);
    feeder.attach(FEEDER_PIN);
    feeder.write(180); // Initial position
    Serial.println(F("feeder initialized successfully."));
}
void initializeRTC (void) {
    if (!rtc.begin()) {
        Serial.println(F("Couldn't find RTC"));
        while (1) delay(100);
    }

    if (rtc.lostPower()) {
        Serial.println(F("RTC lost power, setting the time!"));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    rtcInitialized = true;
    Serial.println(F("RTC initialized successfully."));
}
void initializeSDCardReader (void) {
    spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, spiSD)) {
        Serial.println(F("ERROR: Could not initialize SD Card Reader! Check wiring and connections."));
        sdInitialized = false;
        return;
    }

    sdInitialized = true;
    Serial.println(F("SD Card Reader initialized successfully."));
}
void initializeWiFi (void) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print(F("Connecting to WiFi"));
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(F("."));
    }
    Serial.println();
    Serial.print(F("Connected to WiFi. IP address: "));
    Serial.println(WiFi.localIP());
}
void initializeNTP (void) {
    timeClient.setUpdateInterval(3600000);  // 1 hour between syncs
    timeClient.setTimeOffset(TIMEZONE_OFFSET_SEC);
    timeClient.begin();
    
    Serial.print(F("Waiting for NTP first update..."));
    int attempts = 0;
    while (!timeClient.update() && attempts < 10) {
        delay(500);
        Serial.print(F("."));
        attempts++;
    }
    
    if (attempts >= 10) {
        Serial.println(F(" FAILED after 5 seconds."));
        Serial.println(F("WiFi status: "));
        Serial.println(WiFi.status());
        ntpInitialized = false;
        return;
    }
    
    Serial.println(F(" OK"));
    ntpInitialized = true;
    lastNTPSyncMillis = millis();
    Serial.println(F("NTP Client initialized successfully."));
}
void initializeAHT (void) {
    if (!aht.begin()) {
        Serial.println(F("ERROR: Could not initialize AHT20 sensor! Check wiring and connections."));
        return;
    }
    ahtInitialized = true;
    Serial.println(F("AHT20 sensor initialized successfully."));
}
void initializeBMP (void) {
    if (!bmp.begin(0x77)) {
        Serial.println(F("ERROR: Could not initialize BMP280 sensor! Check wiring and connections."));
        return;
    }
    bmpInitialized = true;
    Serial.println(F("BMP280 sensor initialized successfully."));
}
void initializeMotionSensor (void) {
    pinMode(MOTION_SENSOR_PIN, INPUT);
    Serial.println(F("Motion sensor initialized successfully."));
}
void initializeSensors (void) {
    initializeAHT();
    initializeBMP();
    initializeMotionSensor();
    initializeFoodLevelSensor();
    initializeWaterLevelSensor();
    initializeGateLimitSwitch();
}
void initializeDisplay (void) {
    lcd.init();
    lcd.backlight();
    Serial.println(F("LCD Display initialized successfully."));
}
void initializeRGB (void) {
    if (!rgb.begin()) {
        Serial.println(F("ERROR: Could not initialize RGB LED! Check wiring and connections."));
        return;
    }
    rgb.setBrightness(100);
    rgb.clear();
    rgb.show(); // Initialize all pixels to 'off'

    rgbInitialized = true;

    Serial.println(F("RGB LED initialized successfully."));
}
void initializeMic (void) {
    // Analog pin setup if needed
    Serial.println(F("Microphone initialized successfully."));
}
void initializeFoodLevelSensor (void) {
    pinMode(FOOD_SENSOR_TRIG_PIN, OUTPUT);
    pinMode(FOOD_SENSOR_ECHO_PIN, INPUT);
    Serial.println(F("Food Level Sensor initialized successfully."));
}
void initializeWaterLevelSensor (void) {
    pinMode(WATER_SENSOR_TRIG_PIN, OUTPUT);
    pinMode(WATER_SENSOR_ECHO_PIN, INPUT);
    Serial.println(F("Water Level Sensor initialized successfully."));
}
void initializeGateLimitSwitch (void) {
    pinMode(LIMIT_SWITCH, INPUT);
    Serial.println("Limit Switch initialized successfully.");
}

void readRawButtonInput (void) {
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

void updateButtonInput (void) {
    // Serial.print(rawButtonState);
    // Serial.print(", ");
    // Serial.println(lastRawButtonState);

    if (rawButtonState != lastRawButtonState) {
        // Serial.println(F("Button state changed, resetting debounce timer."));
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

void checkButtonStateChange (void) {
    if (buttonState != lastButtonState) {
        if ((buttonState & _BV(L_BUTTON)) != (lastButtonState & _BV(L_BUTTON))) {
            if (!(buttonState & _BV(L_BUTTON))) {
                Serial.println(F("L Button Pressed."));
                if (slideshowActive) {
                    previousSlideshowScreen();
                } else if (scheduleViewMode != SCHED_VIEW_MAIN_MENU) {
                    navigateScheduleMenu(-1);
                } else if (alertThresholdViewMode != ALERT_VIEW_MAIN_MENU) {
                    navigateAlertMenu(-1);
                } else {
                    handleMenuNavigation(-1);
                }
            } else {
                Serial.println(F("L Button Released."));
            }
        }

        if ((buttonState & _BV(C_BUTTON)) != (lastButtonState & _BV(C_BUTTON))) {
            if (!(buttonState & _BV(C_BUTTON))) {
                Serial.println(F("C Button Pressed."));
                if (scheduleViewMode != SCHED_VIEW_MAIN_MENU) {
                    selectScheduleItem();
                } else if (alertThresholdViewMode != ALERT_VIEW_MAIN_MENU) {
                    selectAlertItem();
                } else {
                    processMenuSelection();
                }
            } else {
                Serial.println(F("C Button Released."));
            }
        }

        if ((buttonState & _BV(R_BUTTON)) != (lastButtonState & _BV(R_BUTTON))) {
            if (!(buttonState & _BV(R_BUTTON))) {
                Serial.println(F("R Button Pressed."));
                if (slideshowActive) {
                    nextSlideshowScreen();
                } else if (scheduleViewMode != SCHED_VIEW_MAIN_MENU) {
                    navigateScheduleMenu(+1);
                } else if (alertThresholdViewMode != ALERT_VIEW_MAIN_MENU) {
                    navigateAlertMenu(+1);
                } else {
                    handleMenuNavigation(+1);
                }
            } else {
                Serial.println(F("R Button Released."));
            }
        }

        if ((buttonState & _BV(GATE_BUTTON)) != (lastButtonState & _BV(GATE_BUTTON))) {
            if (!(buttonState & _BV(GATE_BUTTON))) {
                Serial.println(F("Gate Button Pressed."));
                toggleGate();
                sendActuatorStateToFirebase();
                sendLogToFirebase("actuator", "gate_toggle", "gate", actuatorState & _BV(GATE_RELAY) ? 0 : 1, "device", "manual_button");
            } else {
                Serial.println(F("Gate Button Released."));
            }
        }

        if ((buttonState & _BV(PUMP_BUTTON)) != (lastButtonState & _BV(PUMP_BUTTON))) {
            if (!(buttonState & _BV(PUMP_BUTTON))) {
                Serial.println(F("Pump Button Pressed."));
                startPump();
                bool pumpState = actuatorState & _BV(PUMP_RELAY) ? 0 : 1;
                sendActuatorStateToFirebase();
                sendLogToFirebase("actuator", "pump_on", "pump", pumpState, "device", "manual_button");
            } else {
                Serial.println(F("Pump Button Released."));
            }
        }

        if ((buttonState & _BV(FAN_BUTTON)) != (lastButtonState & _BV(FAN_BUTTON))) {
            if (!(buttonState & _BV(FAN_BUTTON))) {
                Serial.println(F("Fan Button Pressed."));
                toggleFan();
                sendActuatorStateToFirebase();
                sendLogToFirebase("actuator", "fan_toggle", "fan", actuatorState & _BV(FAN_RELAY) ? 0 : 1, "device", "manual_button");
            } else {
                Serial.println(F("Fan Button Released."));
            }
        }

        if ((buttonState & _BV(FEEDER_BUTTON)) != (lastButtonState & _BV(FEEDER_BUTTON))) {
            if (!(buttonState & _BV(FEEDER_BUTTON))) {
                Serial.println(F("Feeder Button Pressed."));
                startFeeder();
                sendActuatorStateToFirebase();
                sendLogToFirebase("actuator", "feeder_on", "feeder", isFeederRunning, "device", "manual_button");
            } else {
                Serial.println(F("Feeder Button Released."));
            }
        }

        lastButtonState = buttonState;
    }
}

void checkRelayActivity (void) {
    unsigned long now = millis();

    if (!isFullFillingActive) {
        // Pump timer
        if (pumpTimer.active && (now - pumpTimer.startTime >= pumpTimer.duration)) {
            stopPump();
            sendActuatorStateToFirebase();
        }

        // Feeder timer
        if (feederTimer.active && (now - feederTimer.startTime >= feederTimer.duration)) {
            stopFeeder();
            sendActuatorStateToFirebase();
        }
    }
    
}

void checkSerialCommand (void) {
    while (Serial.available() > 0) {
        char cmd = Serial.read();

        switch (cmd) {
            case '1': 
                toggleGate();
                Serial.println(F("Gate Relay Toggled."));
                sendActuatorStateToFirebase();
                sendLogToFirebase("actuator", "gate_toggle", "gate", actuatorState & _BV(GATE_RELAY) ? 0 : 1, "device", "manual_serial");
                break;
            case '2':
                startPump();
                Serial.println(F("Pump Relay Activated."));
                sendActuatorStateToFirebase();
                sendLogToFirebase("actuator", "pump_on", "pump", actuatorState & _BV(PUMP_RELAY) ? 0 : 1, "device", "manual_serial");
                break;
            case '3':
                toggleFan();
                Serial.println(F("Fan Relay Toggled."));
                sendActuatorStateToFirebase();
                sendLogToFirebase("actuator", "fan_toggle", "fan", actuatorState & _BV(FAN_RELAY) ? 0 : 1, "device", "manual_serial");
                break;
            case '4':
                startFeeder();
                Serial.println(F("Feeder Activated."));
                sendActuatorStateToFirebase();
                sendLogToFirebase("actuator", "feeder_on", "feeder", isFeederRunning, "device", "manual_serial");
                break;
            case 't':
                printCurrentTime();
                break;
            case 's':
                printSchedule();
                break;
            case 'x':
                rtc.adjust(DateTime(2026, 1, 1, 23, 59, 30));
                Serial.println(F("RTC manually set to 23:59 for testing."));
                break;
            case 'n':
                syncRTCWithNTP(true);
                break;
            case 'u':
                if (isConfigMode) {
                    Serial.println(F("Config mode active. Cloud sync blocked."));
                    break;
                }

                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println(F("WiFi not connected. Cannot update schedule."));
                    break;
                }

                Serial.println(F("Updating schedule from cloud..."));
                updateScheduleFromCloud();
                break;
            case 'f':
                printScheduleFromFirebase();
                break;
            case 'r':
                loadScheduleFromFirebaseToRAM();
                break;
            case 'l':
                loadScheduleFromSDCard();
                break;
            case 'w':
                saveScheduleToSDCard();
                break;
            case 'y':
                showSensorsData();
                break;
            case '?':
            case 'i':
                printSerialCommandList();
                break;
            case 'o':
                toggleSendDataToFirebase();
                break;
            case 'p':
                Serial.println(F("Sending sensor data to Firebase..."));
                sendSensorDataToFirebase();
                break;
            case 'h':
                checkSensorThreshold();
                break;
            case 'H':
                updateSensorThresholdFromFirebase();
                break;
            case 'a':
                showAlertStatus();
                break;
            case 'g':
                showActuatorState();
                break;
            case 'c':
                fetchCommandFromFirebase();
                break;
            case 'C':
                toggleCommandPolling();
                break;
            case '.':
                updateRGBMode();
                break;
            case 'z':
            case 'Z': {
                Serial.println("Testing RGB LED...");
                rgb.setPixelColor(0, rgb.Color(255, 0, 0));
                rgb.show();
                delay(500);
                rgb.setPixelColor(0, rgb.Color(0, 255, 0));
                rgb.show();
                delay(500);
                rgb.setPixelColor(0, rgb.Color(0, 0, 255));
                rgb.show();
                delay(500);
                rgb.clear();
                rgb.show();
                Serial.println("RGB test complete");
                }
                break;
            case '/':
                processCommand();
                break;
            case 'd':  // 'd' for display/slideshow
                if (slideshowActive) {
                    slideshowActive = false;
                    lcd.clear();
                    Serial.println(F("Slideshow stopped"));
                } else {
                    startSensorDataSlideshow();
                    Serial.println(F("Sensor slideshow started. Use < and > to navigate."));
                }
                break;
            case 'A':  // 'A' for Actuator state upload
                sendActuatorStateToFirebase();
                Serial.println(F("Uploading actuator state to Firebase..."));
                break;
            case 'M':  // Toggle continuous mode
                isFullFillingActive = !isFullFillingActive;
                Serial.print(F("Continuous Mode: "));
                Serial.println(isFullFillingActive ? F("ON") : F("OFF"));
                break;
            case 'F':  // Toggle fan auto mode
                isFanAutoMode = !isFanAutoMode;
                Serial.print(F("Fan Auto Mode: "));
                Serial.println(isFanAutoMode ? F("ON") : F("OFF"));
                break;
            // case 'd':
            //     IPAddress serverIP;
            //     if (WiFi.hostByName("pool.ntp.org", serverIP)) {
            //         Serial.print(F("DNS Resolution: pool.ntp.org -> "));
            //         Serial.println(serverIP);
            //     } else {
            //         Serial.println(F("DNS Resolution failed for pool.ntp.org"));
            //     }
            //     break;
        }
    }
}

void printCurrentTime (void) {
    DateTime now = rtc.now();
    Serial.print(F("Current DateTime: "));
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
}

void printSchedule (void) {
    Serial.println(F("=== Schedule List ==="));

    for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
        if (!schedules[i].enabled) continue;

        Serial.print(F("#"));
        Serial.print(i + 1);
        Serial.print(F(" | "));

        Serial.print(schedules[i].type == SCHED_FEEDER ? F("FEEDER") : F("WATER"));
        Serial.print(F(" | "));
        Serial.println(schedules[i].time);
    }
}

void printSDCardInfo (void) {
    if (!sdInitialized) {
        Serial.println(F("SD Card not initialized."));
        return;
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.print(F("SD Card Size: "));
    Serial.print(cardSize);
    Serial.println(F(" MB"));

    uint64_t usedSize = SD.usedBytes() / (1024 * 1024);
    Serial.print(F("SD Card Used Space: "));
    Serial.print(usedSize);
    Serial.println(F(" MB"));
    Serial.print(F("SD Card Free Space: "));
    Serial.print(cardSize - usedSize);
    Serial.println(F(" MB"));
}

void startPump(void) {
    if (pumpTimer.active) return;   // prevent stacking

    pumpTimer.active = true;
    pumpTimer.startTime = millis();

    // Active LOW relay
    // actuatorState &= ~_BV(PUMP_RELAY);
    setRelayOn(PUMP_RELAY);
    applyRelayState();
    Serial.println(F("Pump ON"));
}

void stopPump(void) {
    pumpTimer.active = false;
    // pcf2.digitalWrite(PUMP_RELAY, HIGH);
    // actuatorState |= _BV(PUMP_RELAY);
    setRelayOff(PUMP_RELAY);
    applyRelayState();
    Serial.println(F("Pump OFF"));
}

void startFeeder(void) {
    if (feederTimer.active) return; // prevent stacking

    feederTimer.active = true;
    feederTimer.startTime = millis();

    // For now: LED / relay simulation
    feeder.write(0);
    isFeederRunning = true;
    Serial.println(F("Feeder ON"));
}

void stopFeeder(void) {
    feederTimer.active = false;
    feeder.write(180);
    isFeederRunning = false;
    Serial.println(F("Feeder OFF"));
}

void loadScheduleFromSDCard (void) {
    if (!sdInitialized) {
        Serial.println(F("SD Card not initialized. Cannot load schedule."));
        return;
    }

    if (!SD.exists("/schedule.csv")) {
        Serial.println(F("No schedule file found on SD Card."));
        return;
    }

    File file = SD.open("/schedule.csv", FILE_READ);
    if (!file) {
        Serial.println(F("Failed to open schedule.csv"));
        return;
    }

    Serial.println(F("Reading schedule.csv..."));

    for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
        schedules[i].enabled = false;
        schedules[i].executed = false;
    }

    uint8_t index = 0;
    char line[32];
    file.readStringUntil('\n');

    while (file.available() && index < MAX_SCHEDULES) {
        int len = file.readBytesUntil('\n', line, sizeof(line) - 1);
        line[len] = '\0';

        char* token;

        // enabled
        token = strtok(line, ",");
        if (!token) continue;
        schedules[index].enabled = atoi(token);

        // type
        token = strtok(NULL, ",");
        if (!token) continue;
        schedules[index].type = parseScheduleType(token);

        // time
        token = strtok(NULL, ",");
        if (!token) continue;
        strncpy(schedules[index].time, token, sizeof(schedules[index].time));
        schedules[index].time[5] = '\0';

        schedules[index].executed = false; // runtime only

        index++;
    }

    file.close();

    Serial.print(F("Loaded "));
    Serial.print(index);
    Serial.println(F(" schedules."));
}

ScheduleType parseScheduleType (const char* str) {
    if (strcmp(str, "FEEDER") == 0) {
        return SCHED_FEEDER;
    }
    return SCHED_WATER;
}

void checkScheduleExecution(void) {
    static uint32_t lastCheck = 0;
    if (millis() - lastCheck < 1000) return; // check once per second
    lastCheck = millis();

    DateTime now = rtc.now();

    char currentTime[6];
    snprintf(currentTime, sizeof(currentTime), "%02d:%02d", now.hour(), now.minute());

    // Reset execution flags at midnight
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

            if (schedules[i].type == SCHED_FEEDER) {
                startFeeder();
                sendLogToFirebase("actuator", "feeder_on", "feeder", 1, "device", "schedule");
            } else if (schedules[i].type == SCHED_WATER) {
                startPump();
                sendLogToFirebase("actuator", "pump_on", "pump", 1, "device", "schedule");
            }
            
            sendActuatorStateToFirebase();
            schedules[i].executed = true;
        }
    }
}

void resetScheduleExecutionAtMidnight(void) {
    if (!scheduleResetDone) {
        Serial.println(F("Midnight reached. Resetting schedules."));
        for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
            schedules[i].executed = false;
        }
        scheduleResetDone = true;
    }
}

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
    Serial.print(F("Local IP: "));
    Serial.println(WiFi.localIP());
    
    // Force update with longer timeout
    bool updated = false;
    for (int i = 0; i < 3; i++) {
        Serial.print(F("  Attempt "));
        Serial.print(i + 1);
        Serial.print(F("/3 - "));
        
        // Clear any pending packets
        while (ntpUDP.parsePacket() > 0) {
            ntpUDP.flush();
        }
        
        // Force timeClient to send NTP request
        timeClient.forceUpdate();
        delay(1500);  // Wait for response
        
        if (timeClient.isTimeSet()) {
            updated = true;
            Serial.println(F("OK - Time received"));
            break;
        }
        Serial.println(F("failed - no response"));
        delay(1000);
    }
    
    if (!updated) {
        Serial.println(F("NTP update failed after 3 attempts."));
        Serial.println(F("Possible causes:"));
        Serial.println(F("  1. DNS cannot resolve pool.ntp.org"));
        Serial.println(F("  2. UDP port 123 is blocked by firewall"));
        Serial.println(F("  3. NTP server is unreachable"));
        return false;
    }

    time_t ntpTime = timeClient.getEpochTime();
    DateTime rtcTime = rtc.now();
    time_t rtcEpoch = rtcTime.unixtime();
    long drift = abs((long)(ntpTime - rtcEpoch));

    Serial.print(F("NTP Epoch time: "));
    Serial.println(ntpTime);
    Serial.print(F("RTC drift: "));
    Serial.print(drift);
    Serial.println(F(" sec"));

    if (!force && drift < RTC_DRIFT_THRESHOLD_SEC) {
        Serial.println(F("RTC drift acceptable. No update."));
        return false;
    }

    rtc.adjust(DateTime(ntpTime));
    lastNTPSyncMillis = millis();
    Serial.println(F("RTC updated from NTP successfully."));
    printCurrentTime();
    return true;
}

void checkAndSyncRTCOnBoot(void) {
    Serial.println(F("Checking RTC against NTP on boot..."));

    if (!rtcInitialized) {
        Serial.println(F("RTC not initialized, skipping NTP check."));
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

    // Make sure NTP has valid time
    if (!timeClient.isTimeSet()) {
        Serial.println(F("NTP time not set yet."));
        return;
    }

    time_t ntpTime = timeClient.getEpochTime();
    DateTime rtcTime = rtc.now();

    time_t rtcEpoch = rtcTime.unixtime();
    long drift = abs((long)(ntpTime - rtcEpoch));

    Serial.print(F("RTC vs NTP drift on boot: "));
    Serial.print(drift);
    Serial.println(F(" sec"));

    if (drift >= RTC_DRIFT_THRESHOLD_SEC) {
        Serial.println(F("Drift exceeds threshold. Updating RTC from NTP."));
        rtc.adjust(DateTime(ntpTime));
        lastNTPSyncMillis = millis();
        printCurrentTime();
    } else {
        Serial.println(F("RTC time is acceptable. No update needed."));
        printCurrentTime();
    }
}

void readEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(SSID_ADDR, ssid);
  EEPROM.get(PASS_ADDR, pass);
  EEPROM.end();
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

String configPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>🏠 Smart Pet Cage WiFi Setup</title></head>
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

void startConfigAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32S3_SmartPetCage");

    Serial.println("AP Mode Started");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/retry", HTTP_POST, handleRetry);


    server.begin();
}

bool updateScheduleFromCloud(void) {
    Serial.println(F("[Cloud] Fetching schedule..."));

    // 1. Fetch data from cloud (HTTP / Firebase / etc)
    //    For now, simulate success
    bool fetchOK = true;

    if (!fetchOK) {
        Serial.println(F("[Cloud] Fetch failed."));
        return false;
    }

    // 2. Write to SD (schedule.csv)
    if (!sdInitialized) {
        Serial.println(F("[Cloud] SD not initialized."));
        return false;
    }

    File file = SD.open("/schedule.csv", FILE_WRITE);
    if (!file) {
        Serial.println(F("[Cloud] Failed to open schedule.csv"));
        return false;
    }

    // Example content (replace later)
    file.println("enabled,type,time");
    file.println("1,FEEDER,07:00");
    file.println("1,WATER,12:00");

    file.close();

    Serial.println(F("[Cloud] Schedule saved to SD."));

    // 3. Reload into RAM
    loadScheduleFromSDCard();

    Serial.println(F("[Cloud] Schedule update complete."));
    return true;
}

void applyRelayState (void) {
    pcf2.digitalWrite(GATE_RELAY,   (actuatorState & _BV(GATE_RELAY)) ? HIGH : LOW);
    pcf2.digitalWrite(PUMP_RELAY,   (actuatorState & _BV(PUMP_RELAY)) ? HIGH : LOW);
    pcf2.digitalWrite(FAN_RELAY,    (actuatorState & _BV(FAN_RELAY))  ? HIGH : LOW);
}

void printScheduleFromFirebase(void) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[Firebase] WiFi not connected."));
        return;
    }

    fbClient.setInsecure(); // HTTPS

    const char* types[] = { "feeder", "water" };

    for (uint8_t t = 0; t < 2; t++) {
        Serial.println();
        Serial.print(F("=== "));
        Serial.print(types[t]);
        Serial.println(F(" schedules ==="));

        for (uint8_t i = 1; i <= 4; i++) {
            String url = String(FIREBASE_URL)
                       + "/schedules/"
                       + types[t]
                       + "/"
                       + i
                       + ".json";

            if (strlen(FIREBASE_AUTH) > 0) {
                url += "?auth=";
                url += FIREBASE_AUTH;
            }

            http.begin(fbClient, url);
            int httpCode = http.GET();

            if (httpCode != HTTP_CODE_OK) {
                Serial.print(F("#"));
                Serial.print(i);
                Serial.print(F(" ERROR "));
                Serial.println(httpCode);
                http.end();
                continue;
            }

            String payload = http.getString();
            http.end();

            // Expected: {"enabled":"true","time":"07:00"}
            bool enabled = payload.indexOf("\"enabled\":\"true\"") > 0;

            int timePos = payload.indexOf("\"time\"");
            String time = "--:--";

            if (timePos != -1) {
                int colon = payload.indexOf(":", timePos);
                int q1 = payload.indexOf("\"", colon + 1);
                int q2 = payload.indexOf("\"", q1 + 1);

                if (q1 != -1 && q2 != -1) {
                    String extracted = payload.substring(q1 + 1, q2);
                    if (extracted.length() > 0) {
                        time = extracted;
                    }
                }
            }


            Serial.print(F("#"));
            Serial.print(i);
            Serial.print(F(" | "));
            Serial.print(enabled ? F("ENABLED") : F("DISABLED"));
            Serial.print(F(" | "));
            Serial.println(time);
        }
    }

    Serial.println();
    Serial.println(F("[Firebase] Schedule read complete."));
}

bool loadScheduleFromFirebaseToRAM(void) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[Firebase] WiFi not connected"));
        return false;
    }

    fbClient.setInsecure(); // HTTPS

    Serial.println(F("[Firebase] Loading schedules into RAM..."));

    // Clear RAM first
    for (int i = 0; i < MAX_SCHEDULES; i++) {
        schedules[i].enabled = false;
        schedules[i].executed = false;
        schedules[i].time[0] = '\0';
    }

    const char* types[] = { "feeder", "water" };

    for (int t = 0; t < 2; t++) {
        ScheduleType schedType = (t == 0) ? SCHED_FEEDER : SCHED_WATER;

        for (int slot = 1; slot <= 4; slot++) {
            String url = String(FIREBASE_URL) +
                         "/schedules/" + types[t] +
                         "/" + slot + ".json";

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

            // ---------- Parse ENABLED ----------
            bool enabled = payload.indexOf("\"enabled\":\"true\"") > 0;

            // ---------- Parse TIME ----------
            String time = "--:--";
            int timePos = payload.indexOf("\"time\"");
            if (timePos != -1) {
                int colon = payload.indexOf(":", timePos);
                int q1 = payload.indexOf("\"", colon + 1);
                int q2 = payload.indexOf("\"", q1 + 1);

                if (q1 != -1 && q2 != -1) {
                    String extracted = payload.substring(q1 + 1, q2);
                    if (extracted.length() == 5) {
                        time = extracted;
                    }
                }
            }

            // ---------- Write to RAM ----------
            int index = (schedType == SCHED_FEEDER ? 0 : 4) + (slot - 1);

            schedules[index].enabled = enabled;
            schedules[index].type = schedType;
            strncpy(schedules[index].time, time.c_str(), 6);
            schedules[index].executed = false;

            // ---------- Debug ----------
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

void printSerialCommandList (void) {
    // Printing available serial commands
    Serial.println(F("=== Serial Command List ==="));
    Serial.println(F("1 - Toggle Gate Relay"));
    Serial.println(F("2 - Activate Pump Relay"));
    Serial.println(F("3 - Toggle Fan Relay"));
    Serial.println(F("4 - Activate Feeder"));
    Serial.println(F("t - Print Current Time"));
    Serial.println(F("s - Print Schedule"));
    Serial.println(F("n - Sync RTC with NTP"));
    Serial.println(F("u - Update Schedule from Cloud"));
    Serial.println(F("f - Print Schedule from Firebase"));
    Serial.println(F("r - Load Schedule from Firebase to RAM"));
    Serial.println(F("y - Show Sensors\' data"));
    Serial.println(F("w - Force write schedule to SD Card"));
    Serial.println(F("l - Load Schedule from SD Card to RAM"));
    Serial.println(F("o - Toggle periodic data send to Firebase"));
    Serial.println(F("p - Send sensor data to Firebase"));
    Serial.println(F("h - Show sensor threshold"));
    Serial.println(F("H - Force sync sensor threshold with Firebase"));
    Serial.println(F("a - Show alert status"));
    Serial.println(F("g - Show actuator state"));
    Serial.println(F("c - Force Fetch command from Firebase"));
    Serial.println(F("C - Toggle periodic command polling"));
    Serial.println(F(". - Update RGB mode"));
    Serial.println(F("z - Test RGB"));
    Serial.println(F("/ - Force Fetch & Process Command"));
    Serial.println(F("d - Start/Stop Sensor Data Slideshow on LCD"));
    Serial.println(F("A - Upload Actuator State to Firebase"));
    Serial.println(F("M - Toggle full-filling mode"));
    Serial.println(F("F - Toggle fan auto mode"));
    Serial.println(F("i - Print this Command List"));
}

void readAHTdata (void) {
    if (!ahtInitialized) return;
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    ahtTemperature = temp.temperature;
    ahtHumidity = humidity.relative_humidity;
}

void readBMPdata (void) {
    if (!bmpInitialized) return;
    bmpPressure = bmp.readPressure() / 100.0F; // Convert to hPa
    bmpAltitude = bmp.readAltitude();
}

void readMotionSensorData (void) {
    motionDetected = digitalRead(MOTION_SENSOR_PIN);
}

void readMicData (void) {
    micAnalogValue = analogRead(MICROPHONE_PIN);
}

void readLimitSwitchData (void) {
    isGateSwitchClosed = digitalRead(LIMIT_SWITCH);
}

void readSensorsData (void) {
    if (millis() - lastSensorReadTime < SENSOR_READ_INTERVAL_MS) return;
    lastSensorReadTime = millis();
    readAHTdata();
    readBMPdata();
    readMotionSensorData();
    readRawWaterSensorDistance();
    readRawFoodSensorDistance();
    readMicData();
    readLimitSwitchData();
    checkWaterLevel();
    checkFoodLevel();

    checkAlerts();
}

void showSavedAHTdata (void) {
    Serial.print(F("AHT20 Temperature: "));
    Serial.print(ahtTemperature);
    Serial.println(F(" °C"));

    Serial.print(F("AHT20 Humidity: "));
    Serial.print(ahtHumidity);
    Serial.println(F(" %"));
}

void showSavedBMPdata (void) {
    Serial.print(F("BMP280 Pressure: "));
    Serial.print(bmpPressure);
    Serial.println(F(" hPa"));

    Serial.print(F("BMP280 Altitude: "));
    Serial.print(bmpAltitude);
    Serial.println(F(" m"));
}

void showMotionSensorData (void) {
    Serial.print(F("Motion detected: "));
    Serial.println(motionDetected? "TRUE": "FALSE");
}

void showSensorsData (void) {
    showSavedAHTdata();
    showSavedBMPdata();
    showMotionSensorData();
    showWaterLevelData();
    showFoodLevelData();
    showMicData();
    showGateSwitchData();
}

void checkAlerts (void) {
    if (ahtInitialized) {
        /* Temperature */ 
        if (ahtTemperature >= tempHighThreshold) {
            if (!tempHighAlertActive) {
                Serial.println(F("[ALERT] Temperature TOO HIGH!"));
                Serial.print(F(" Value: "));
                Serial.print(ahtTemperature);
                Serial.println(F(" °C"));
                tempHighAlertActive = true;
                sendLogToFirebase("alert", "temperature_high", "temperature", (int)ahtTemperature, "device", "threshold_exceeded");
            }
        } else if (tempHighAlertActive) {
            tempHighAlertActive = false;
            sendLogToFirebase("alert", "temperature_normal", "temperature", (int)ahtTemperature, "device", "recovered");
        }

        if (ahtTemperature <= tempLowThreshold) {
            if (!tempLowAlertActive) {
                Serial.println(F("[ALERT] Temperature TOO LOW!"));
                Serial.print(F("  Value: "));
                Serial.print(ahtTemperature);
                Serial.println(F(" °C"));
                tempLowAlertActive = true;
                sendLogToFirebase("alert", "temperature_low", "temperature", (int)ahtTemperature, "device", "threshold_exceeded");
            }
        } else if (tempLowAlertActive) {
            tempLowAlertActive = false;
            sendLogToFirebase("alert", "temperature_normal", "temperature", (int)ahtTemperature, "device", "recovered");
        } 
        
        /* Humidity */ 
        if (ahtHumidity >= humHighThreshold) {
            if (!humHighAlertActive) {
                Serial.println(F("[ALERT] Humidity TOO HIGH!"));
                Serial.print(F("  Value: "));
                Serial.print(ahtHumidity);
                Serial.println(F(" %"));
                humHighAlertActive = true;
                sendLogToFirebase("alert", "humidity_high", "humidity", (int)ahtHumidity, "device", "recovered");
            }
        } else if (humHighAlertActive) {
            humHighAlertActive = false;
            sendLogToFirebase("alert", "humidity_normal", "humidity", (int)ahtHumidity, "device", "recovered");
        }

        if (ahtHumidity <= humLowThreshold) {
            if (!humLowAlertActive) {
                Serial.println(F("[ALERT] Humidity TOO LOW!"));
                Serial.print(F("  Value: "));
                Serial.print(ahtHumidity);
                Serial.println(F(" %"));
                humLowAlertActive = true;
                sendLogToFirebase("alert", "humidity_low", "humidity", (int)ahtHumidity, "device", "recovered");
            }
        } else if (humLowAlertActive) {
            humLowAlertActive = false;
            sendLogToFirebase("alert", "humidity_normal", "humidity", (int)ahtHumidity, "device", "recovered");
        }
    }

    /* Motion */
    if (MOTION_ALERT_ENABLED && motionDetected) {
        if (!motionAlertActive) {
            Serial.println(F("[ALERT] Motion detected inside cage!"));
            motionAlertActive = true;
        }
    } else {
        motionAlertActive = false;
    }

    if (foodLevelPercent <= foodLowThreshold) {
        if (!foodLowAlertActive) {
            Serial.println(F("[ALERT] Food level LOW!"));
            foodLowAlertActive = true;
            sendLogToFirebase("alert", "food_level_low", "food_level", (int)foodLevelPercent, "device", "threshold_exceeded");
        }
    } else if (foodLowAlertActive) {
        foodLowAlertActive = false;
        sendLogToFirebase("alert", "food_level_normal", "food_level", (int)foodLevelPercent, "device", "recovered");
    }

    if (waterLevelPercent <= waterLowThreshold) {
        if (!waterLowAlertActive) {
            Serial.println(F("[ALERT] Water level LOW!"));
            waterLowAlertActive = true;
            sendLogToFirebase("alert", "water_level_low", "water_level", (int)waterLevelPercent, "device", "threshold_exceeded");
        }
    } else if (waterLowAlertActive) {
        waterLowAlertActive = false;
        sendLogToFirebase("alert", "water_level_normal", "water_level", (int)waterLevelPercent, "device", "recovered");
    }

    updateBarkCounter();

    if (mic_is_bark_detected()) {
        if (!barkAlertActive) {
            Serial.println(F("[ALERT] Barking detected!"));
            barkAlertActive = true;
            sendLogToFirebase("alert", "barking_detected", "microphone", (int)(mic_get_bark_confidence() * 100), "device", "ai_inference");
        }
    } else {
        barkAlertActive = false;
    }

    if (barkCount >= barkCountThreshold) {
        if (!abnormalBehaviorAlertActive) {
            Serial.print(F("[ALERT] ABNORMAL BEHAVIOR DETECTED! Bark count: "));
            Serial.print(barkCount);
            Serial.print(F(" exceeds threshold: "));
            Serial.println(barkCountThreshold);
            abnormalBehaviorAlertActive = true;
            sendLogToFirebase("alert", "abnormal_behavior", "microphone", barkCount, "device", "excessive_barking");
        }
    } else if (abnormalBehaviorAlertActive && barkCount < barkCountThreshold) {
        // Only clear alert when bark count drops below threshold
        abnormalBehaviorAlertActive = false;
        Serial.print(F("[ALERT] Abnormal behavior cleared. Current bark count: "));
        Serial.println(barkCount);
        sendLogToFirebase("alert", "abnormal_behavior_cleared", "microphone", barkCount, "device", "recovered");
    }
}

void handleMenuNavigation (int direction) {
    currentMenu += direction;

    if (currentMenu < 0) {
        currentMenu = MENU_COUNT - 1;
    } else if (currentMenu >= MENU_COUNT) {
        currentMenu = 0;
    }

    menuDirty = true;
}

void renderMenuUI (void) {
    if (slideshowActive) return;

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

void updateDisplayUI (void) {
    unsigned long now = millis();

    if (scheduleViewMode != SCHED_VIEW_MAIN_MENU) {
        updateScheduleViewMenu();
        return;
    }

    if (alertThresholdViewMode != ALERT_VIEW_MAIN_MENU) {
        updateAlertThresholdView();
        return;
    }

    if (isConfigMode) return;

    if (!menuDirty && (now - lastUIUpdate < UI_REFRESH_INTERVAL_MS)) {
        return;
    }

    lastUIUpdate = now;
    menuDirty = false;

    renderMenuUI();
}

void toggleGate(void) {
    // actuatorState ^= _BV(GATE_RELAY);
    toggleRelay(GATE_RELAY);
    applyRelayState();
    Serial.println(F("Gate toggled"));
}

void toggleFan(void) {
    // actuatorState ^= _BV(FAN_RELAY);
    toggleRelay(FAN_RELAY);
    applyRelayState();
    Serial.println(F("Fan toggled"));
}

void enableFan (void) {
    setRelayOn(FAN_RELAY);
    applyRelayState();
    Serial.println(F("Fan enabled"));
}

void enableGate (void) {
    setRelayOn(GATE_RELAY);
    applyRelayState();
    Serial.println(F("Gate opened"));
}

void disableFan (void) {
    setRelayOff(FAN_RELAY);
    applyRelayState();
    Serial.println(F("Fan disabled"));
}

void disableGate (void) {
    setRelayOff(GATE_RELAY);
    applyRelayState();
    Serial.println(F("Gate closed"));
}

bool saveScheduleToSDCard (void) {
    if (!sdInitialized) {
        Serial.println(F("[SD] SD not initialized. Cannot save schedule."));
        return false;
    }

    // Delete existing file first to ensure clean write
    if (SD.exists("/schedule.csv")) {
        SD.remove("/schedule.csv");
    }

    File file = SD.open("/schedule.csv", FILE_WRITE);
    if (!file) {
        Serial.println(F("[SD] Failed to open schedule.csv for writing."));
        return false;
    }

    // Header
    file.println(F("enabled,type,time"));

    uint8_t count = 0;

    for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
        if (!schedules[i].enabled) continue;

        file.print(schedules[i].enabled ? '1' : '0');
        file.print(',');

        file.print(
            schedules[i].type == SCHED_FEEDER ? F("FEEDER") : F("WATER")
        );
        file.print(',');

        file.println(schedules[i].time);

        count++;
    }

    file.close();

    Serial.print(F("[SD] Schedule saved successfully ("));
    Serial.print(count);
    Serial.println(F(" entries)."));

    return true;
}

void sendSensorDataToFirebase (void) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[Firebase] WiFi not connected, aborting data send."));
        return;
    }

    fbClient.setInsecure();  // Firebase HTTPS

    String url = String(FIREBASE_URL) + "/data.json";
    if (strlen(FIREBASE_AUTH) > 0) {
        url += "?auth=";
        url += FIREBASE_AUTH;
    }

    // ---- Build JSON payload ----
    // Build JSON payload with proper quoting for booleans and strings
    String payload = "{";
    payload += "\"temperature\":" + String(ahtTemperature, 2) + ",";
    payload += "\"humidity\":"    + String(ahtHumidity, 2) + ",";
    payload += "\"pressure\":"    + String(bmpPressure, 2) + ",";
    payload += "\"altitude\":"    + String(bmpAltitude, 2) + ",";
    payload += "\"motion\":"      + String(motionDetected ? "true" : "false") + ","; // boolean, no quotes
    payload += "\"water_level\":" + String(waterLevelPercent, 2) + ",";
    payload += "\"food_level\":"  + String(foodLevelPercent, 2) + ",";
    payload += "\"mic_level\":"   + String(micAnalogValue) + ",";
    appendMicInferenceToJSON(payload);
    payload += "\"gate_status\":\"" + String(isGateSwitchClosed ? "closed" : "open") + "\","; // string, add quotes
    payload += "\"timestamp\":"   + String(rtc.now().unixtime());
    payload += "}";
    // Validate payload if needed before sending

    http.begin(fbClient, url);
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.PATCH(payload);

    if (httpCode > 0) {
        Serial.print(F("[Firebase] PATCH code: "));
        Serial.println(httpCode);

        if (httpCode == HTTP_CODE_OK) {
            Serial.println(F("[Firebase] Sensor data updated successfully."));
        } else {
            Serial.println(http.getString());
        }
    } else {
        Serial.print(F("[Firebase] PATCH failed: "));
        Serial.println(http.errorToString(httpCode));
    }

    http.end();
}

void sendSensorDataPeriodically (void) {
    if (millis() - lastDataPatch < SENSOR_DATA_PATCH_INTERVAL_MS || !isSendDataPeriodically) return;
    lastDataPatch = millis();
    sendSensorDataToFirebase();
}

void updateSensorThresholdFromFirebase (void) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[Threshold] WiFi not connected"));
        return;
    }

    fbClient.setInsecure();

    String url = String(FIREBASE_URL) + "/thresholds.json";
    if (strlen(FIREBASE_AUTH) > 0) {
        url += "?auth=";
        url += FIREBASE_AUTH;
    }

    Serial.println(F("[Threshold] Fetching thresholds from Firebase..."));
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

    if (payload.length() < 50) {
        Serial.println(F("[Threshold] Payload too small or empty"));
        return;
    }

    Serial.println(F("[Threshold] Successfully fetched. Parsing..."));

    // -------- Temperature --------
    int tempPos = payload.indexOf("\"temperature\"");
    if (tempPos != -1) {
        int lowPos = payload.indexOf("\"low\":", tempPos);
        int highPos = payload.indexOf("\"high\":", tempPos);
        
        if (lowPos != -1) {
            String tempLowStr = payload.substring(lowPos + 6);
            tempLowThreshold = tempLowStr.toFloat();
        }
        if (highPos != -1) {
            String tempHighStr = payload.substring(highPos + 7);
            tempHighThreshold = tempHighStr.toFloat();
        }
        Serial.print(F("[Threshold] Temperature: "));
        Serial.print(tempLowThreshold, 1);
        Serial.print(F("°C - "));
        Serial.print(tempHighThreshold, 1);
        Serial.println(F("°C"));
    }

    // -------- Humidity --------
    int humPos = payload.indexOf("\"humidity\"");
    if (humPos != -1) {
        int lowPos = payload.indexOf("\"low\":", humPos);
        int highPos = payload.indexOf("\"high\":", humPos);
        
        if (lowPos != -1) {
            String humLowStr = payload.substring(lowPos + 6);
            humLowThreshold = humLowStr.toFloat();
        }
        if (highPos != -1) {
            String humHighStr = payload.substring(highPos + 7);
            humHighThreshold = humHighStr.toFloat();
        }
        Serial.print(F("[Threshold] Humidity: "));
        Serial.print(humLowThreshold, 1);
        Serial.print(F("% - "));
        Serial.print(humHighThreshold, 1);
        Serial.println(F("%"));
    }

    // -------- Water Level --------
    int waterPos = payload.indexOf("\"water_level\"");
    if (waterPos != -1) {
        int lowPos = payload.indexOf("\"low\":", waterPos);
        int highPos = payload.indexOf("\"high\":", waterPos);
        
        if (lowPos != -1) {
            String waterLowStr = payload.substring(lowPos + 6);
            waterLowThreshold = waterLowStr.toFloat();
        }
        if (highPos != -1) {
            String waterHighStr = payload.substring(highPos + 7);
            waterHighThreshold = waterHighStr.toFloat();
        }
        Serial.print(F("[Threshold] Water Level: "));
        Serial.print(waterLowThreshold, 1);
        Serial.print(F("% - "));
        Serial.print(waterHighThreshold, 1);
        Serial.println(F("%"));
    }

    // -------- Food Level --------
    int foodPos = payload.indexOf("\"food_level\"");
    if (foodPos != -1) {
        int lowPos = payload.indexOf("\"low\":", foodPos);
        
        if (lowPos != -1) {
            String foodLowStr = payload.substring(lowPos + 6);
            foodLowThreshold = foodLowStr.toFloat();
        }
        Serial.print(F("[Threshold] Food Level Low: "));
        Serial.print(foodLowThreshold, 1);
        Serial.println(F("%"));
    }

    // -------- Bark Thresholds --------
    int barkPos = payload.indexOf("\"bark\"");
    if (barkPos != -1) {
        // Parse bark count threshold
        int countPos = payload.indexOf("\"count\":", barkPos);
        if (countPos != -1) {
            String countStr = payload.substring(countPos + 8);
            barkCountThreshold = (uint16_t)countStr.toInt();
        }
        
        // Parse cooldown
        int cooldownPos = payload.indexOf("\"cooldown_sec\":", barkPos);
        if (cooldownPos != -1) {
            String cooldownStr = payload.substring(cooldownPos + 15);
            uint32_t barkCooldownSec = (uint32_t)cooldownStr.toInt();
            BARK_COUNT_RESET_PERIOD = barkCooldownSec * 1000;
            // You can store this if needed for alert suppression
        }
        
        Serial.print(F("[Threshold] Bark: count="));
        Serial.print(barkCountThreshold);
        Serial.print(F(", reset="));
        Serial.print(BARK_COUNT_RESET_PERIOD / 1000);
        Serial.println(F("sec"));
    }

    // -------- Noise Thresholds (Microphone Level) --------
    int noisePos = payload.indexOf("\"noise\"");
    if (noisePos != -1) {
        int highPctPos = payload.indexOf("\"high_pct\":", noisePos);
        int minSecPos = payload.indexOf("\"min_sec\":", noisePos);
        
        if (highPctPos != -1) {
            String noiseHighStr = payload.substring(highPctPos + 11);
            uint8_t noiseHighPct = (uint8_t)noiseHighStr.toInt();
            // Store for noise monitoring if needed
            Serial.print(F("[Threshold] Noise High: "));
            Serial.print(noiseHighPct);
            Serial.println(F("%"));
        }
        
        if (minSecPos != -1) {
            String noiseMinStr = payload.substring(minSecPos + 10);
            uint8_t noiseMinSec = (uint8_t)noiseMinStr.toInt();
            // Store for noise monitoring duration if needed
            Serial.print(F("[Threshold] Noise Min Duration: "));
            Serial.print(noiseMinSec);
            Serial.println(F("sec"));
        }
    }

    // -------- Pressure Thresholds --------
    int pressurePos = payload.indexOf("\"pressure\"");
    if (pressurePos != -1) {
        int lowPos = payload.indexOf("\"low\":", pressurePos);
        int highPos = payload.indexOf("\"high\":", pressurePos);
        
        if (lowPos != -1) {
            String pressureLowStr = payload.substring(lowPos + 6);
            // Store pressure low threshold if you have a variable for it
            Serial.print(F("[Threshold] Pressure Low: "));
            Serial.print(pressureLowStr.toFloat(), 1);
            Serial.println(F(" hPa"));
        }
        if (highPos != -1) {
            String pressureHighStr = payload.substring(highPos + 7);
            // Store pressure high threshold if you have a variable for it
            Serial.print(F("[Threshold] Pressure High: "));
            Serial.print(pressureHighStr.toFloat(), 1);
            Serial.println(F(" hPa"));
        }
    }

    // -------- Temperature Decrease (Hysteresis for Fan) --------
    int tempDecreasePos = payload.indexOf("\"temperature_decrease\"");
    if (tempDecreasePos != -1) {
        String tempDecreaseStr = payload.substring(tempDecreasePos + 23);
        float tempDecrease = tempDecreaseStr.toFloat();
        tempDecreaseTarget = tempDecrease;
        // This value (2°C) is already hardcoded in the fan logic
        Serial.print(F("[Threshold] Temperature Decrease (Fan Hysteresis): "));
        Serial.print(tempDecrease, 1);
        Serial.println(F("°C"));
    }

    Serial.println(F("[Threshold] All thresholds updated successfully"));
    checkSensorThreshold();
}

void checkSensorThreshold (void) {
    Serial.println("[CMD] Threshold Check");
    Serial.print("Temperature: High="); Serial.print(tempHighThreshold); Serial.print(" °C, Low="); Serial.print(tempLowThreshold); Serial.println(" °C");
    Serial.print("Humidity: High="); Serial.print(humHighThreshold); Serial.print("%, Low="); Serial.print(humLowThreshold); Serial.println("%");
}

void toggleSendDataToFirebase (void) {
    Serial.println("[CMD] Toggle Periodic Data Sending ...");
    isSendDataPeriodically = !isSendDataPeriodically;
    Serial.print("> Set to "); Serial.println((isSendDataPeriodically)? "True" : "False"); 
}

void showAlertStatus (void) {
    Serial.println("=== Alert Status ===");
    Serial.print("Temp High Alert: "); Serial.println(tempHighAlertActive ? "ACTIVE" : "INACTIVE");
    Serial.print("Temp Low Alert: "); Serial.println(tempLowAlertActive ? "ACTIVE" : "INACTIVE");
    Serial.print("Hum High Alert: "); Serial.println(humHighAlertActive ? "ACTIVE" : "INACTIVE");
    Serial.print("Hum Low Alert: "); Serial.println(humLowAlertActive ? "ACTIVE" : "INACTIVE");
    Serial.print("Motion Alert: "); Serial.println(motionAlertActive ? "ACTIVE" : "INACTIVE");
}

void updateRGBMode (void) {
    if (!rgbInitialized) return;
    rgbMode = (++rgbMode) % 4;

    switch (rgbMode) {
        case 0:
            rgb.clear();
            break;
        case 1:
            rgb.setPixelColor(0, rgb.Color(255, 0, 0));
            break;
        case 2:
            rgb.setPixelColor(0, rgb.Color(0, 255, 0));
            break;
        case 3:
            rgb.setPixelColor(0, rgb.Color(0, 0, 255));
            break;
    }
    rgb.show();
    Serial.print(F("RGB mode set to "));
    Serial.println(rgbMode);
}

void displaySensorsDataOnLCD (void) {
    // To be implemented later
}

void setRelayOn (uint8_t relayBitPos) {
    actuatorState &= ~_BV(relayBitPos);
}

void setRelayOff (uint8_t relayBitPos) {
    actuatorState |= _BV(relayBitPos);
}

void toggleRelay (uint8_t relayBitPos) {
    actuatorState ^= _BV(relayBitPos);
}

void showActuatorState (void) {
    Serial.println("=== Actuator States ===");
    Serial.print("Gate Relay: "); Serial.println((actuatorState & _BV(GATE_RELAY)) ? "OFF" : "ON");
    Serial.print("Pump Relay: "); Serial.println((actuatorState & _BV(PUMP_RELAY)) ? "OFF" : "ON");
    Serial.print("Fan Relay: ");  Serial.println((actuatorState & _BV(FAN_RELAY))  ? "OFF" : "ON");
    Serial.print("Feeder: ");     Serial.println(isFeederRunning ? "ON" : "OFF");
}

String withAuth(String url) {
  if (strlen(FIREBASE_AUTH) > 0) {
    url += "?auth=";
    url += FIREBASE_AUTH;
  }
  return url;
}

bool fetchCommandFromFirebase (void) {
    if (WiFi.status() != WL_CONNECTED) return false;
    
    fbClient.setInsecure();

    String url = String(FIREBASE_URL)
        + "/commands.json?orderBy=\"$key\"&limitToFirst=1";

    if (strlen(FIREBASE_AUTH) > 0) {
        url += "&auth=";
        url += FIREBASE_AUTH;
    }

    http.begin(fbClient, url);
    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK) {
        http.end();
        return false;
    }

    String payload = http.getString();
    http.end();

    if (payload == "null") return false;

    // ---- Parse key ----
    int keyStart = payload.indexOf('{') + 1;
    int keyEnd   = payload.indexOf(':', keyStart);
    if (keyStart <= 0 || keyEnd <= 0) return false;

    String key = payload.substring(keyStart, keyEnd);
    key.replace("\"", "");

    // ---- Parse action ----
    int aPos = payload.indexOf("\"action\"");
    int aQ1  = payload.indexOf("\"", aPos + 8);
    int aQ2  = payload.indexOf("\"", aQ1 + 1);
    String action = payload.substring(aQ1 + 1, aQ2);

    // ---- Parse target ----
    int tPos = payload.indexOf("\"target\"");
    int tQ1  = payload.indexOf("\"", tPos + 8);
    int tQ2  = payload.indexOf("\"", tQ1 + 1);
    String target = payload.substring(tQ1 + 1, tQ2);

    ongoingCommand.key = key;
    ongoingCommand.action = action;
    ongoingCommand.target = target;

    Serial.println(F("[Command] Received"));
    Serial.print(F("  Key    : ")); Serial.println(key);
    Serial.print(F("  Action : ")); Serial.println(action);
    Serial.print(F("  Target : ")); Serial.println(target);
    return true;
}

void fetchCommandFromFirebasePeriodically (void) {
    unsigned long now = millis();
    if (now - lastCommandPoll < COMMAND_POLL_INTERVAL_MS || !isCommandPollPeriodically) return;
    lastCommandPoll = now;
    processCommand();
}

void toggleCommandPolling (void) {
    Serial.println("[CMD] Toggle Command Polling ...");
    isCommandPollPeriodically = !isCommandPollPeriodically;
    Serial.print("> Set to "); Serial.println((isCommandPollPeriodically)? "True" : "False");
}

void executeCommand (const String& action, const String& target) {
    if (action == "" && target == "") return;

    if (action == "reload") {
        if (target == "schedule") {
            loadScheduleFromFirebaseToRAM();
            saveScheduleToSDCard();
        }
        if (target == "threshold") {
            updateSensorThresholdFromFirebase();
        }
        if (target == "config") {
            getConfiguration();
        }
        return;
    }

    if (action == "enable") {
        if (target == "fan") {
            enableFan();
            sendLogToFirebase("actuator", "fan_on", "fan", 1, "app", "command");
        }
        if (target == "gate") {
            enableGate();
            sendLogToFirebase("actuator", "gate_on", "gate", 1, "app", "command");
        }
        if (target == "pump") {
            startPump();
            sendLogToFirebase("actuator", "pump_on", "pump", 1, "app", "command");
        }
        if (target == "feeder") {
            startFeeder();
            sendLogToFirebase("actuator", "feeder_on", "feeder", 1, "app", "command");
        }
        sendActuatorStateToFirebase();
        return;
    }

    if (action == "disable") {
        if (target == "fan") {
            disableFan();
            sendLogToFirebase("actuator", "fan_off", "fan", 0, "app", "command");
        }
        if (target == "gate") {
            disableGate();
            sendLogToFirebase("actuator", "gate_off", "gate", 0, "app", "command");
        }
        if (target == "pump") {
            stopPump();
            sendLogToFirebase("actuator", "pump_off", "pump", 0, "app", "command");
        }
        if (target == "feeder") {
            stopFeeder();
            sendLogToFirebase("actuator", "feeder_off", "feeder", 0, "app", "command");
        }
        sendActuatorStateToFirebase();
        return;
    }

    if (action == "toggle") {
        if (target == "fan") {
            toggleFan();
            sendLogToFirebase("actuator", "fan_toggle", "fan", actuatorState & _BV(FAN_RELAY) ? 0 : 1, "app", "command");
        }
        if (target == "gate") {
            toggleGate();
            sendLogToFirebase("actuator", "gate_toggle", "gate", actuatorState & _BV(GATE_RELAY) ? 0 : 1, "app", "command");
        }
        sendActuatorStateToFirebase();
        return;
    }

    Serial.println(F("[Command] Unknown action/target"));
}

bool deleteCommandFromFirebase (const String& commandKey) {
    fbClient.setInsecure();

    String url = String(FIREBASE_URL) + "/commands/" + commandKey + ".json";
    url = withAuth(url);

    http.begin(fbClient, url);
    int code = http.sendRequest("DELETE");
    http.end();

    if (code == HTTP_CODE_OK) {
        Serial.println(F("[Command] Deleted successfully"));
        return true;
    }

    Serial.print(F("[Command] Delete failed: "));
    Serial.println(code);
    return false;
}

void syncSchedule (void) {
    if (WiFi.status() != WL_CONNECTED) {
        if (sdInitialized) {
            loadScheduleFromSDCard();
        } else {
            /* Add default schedule
               Water => 09:00, 13:00, 17:00
               Food  => 09:00, 17:00
            */ 
            loadDefaultSchedule();
        }
        return;
    }

    loadScheduleFromFirebaseToRAM();
}

void syncSensorThreshold (void) {
    if (WiFi.status() != WL_CONNECTED) return;
    updateSensorThresholdFromFirebase();
}

void processCommand (void) {
    if (ongoingCommand.key != "" && ongoingCommand.action != "" && ongoingCommand.target != "") {
        // Pass. Commmand already fetched
    } else if (!fetchCommandFromFirebase()) {
        return;
    }

    executeCommand(ongoingCommand.action, ongoingCommand.target);
    isCommandExecuted = true;

    /* ACK */ 
    if (deleteCommandFromFirebase(ongoingCommand.key)) cleanOngoingCommand();
}

/* Clean Command Storage */ 
void cleanOngoingCommand (void) {
    ongoingCommand.key = "";
    ongoingCommand.action = "";
    ongoingCommand.target = "";
    isCommandExecuted = false;
}

void loadDefaultSchedule (void) {
    Serial.println(F("[Schedule] Loading default schedule..."));
    
    // Clear all schedules first
    for (int i = 0; i < MAX_SCHEDULES; i++) {
        schedules[i].enabled = false;
        schedules[i].executed = false;
        schedules[i].time[0] = '\0';
    }
    
    // Water => 09:00, 13:00, 17:00
    schedules[0].enabled = true;
    schedules[0].type = SCHED_WATER;
    strncpy(schedules[0].time, "09:00", 6);
    schedules[0].executed = false;
    
    schedules[1].enabled = true;
    schedules[1].type = SCHED_WATER;
    strncpy(schedules[1].time, "13:00", 6);
    schedules[1].executed = false;
    
    schedules[2].enabled = true;
    schedules[2].type = SCHED_WATER;
    strncpy(schedules[2].time, "17:00", 6);
    schedules[2].executed = false;
    
    // Feeder => 09:00, 17:00
    schedules[3].enabled = true;
    schedules[3].type = SCHED_FEEDER;
    strncpy(schedules[3].time, "09:00", 6);
    schedules[3].executed = false;
    
    schedules[4].enabled = true;
    schedules[4].type = SCHED_FEEDER;
    strncpy(schedules[4].time, "17:00", 6);
    schedules[4].executed = false;
    
    Serial.println(F("[Schedule] Default schedule loaded:"));
    Serial.println(F("  Water: 09:00, 13:00, 17:00"));
    Serial.println(F("  Feeder: 09:00, 17:00"));
}

void handleRoot() {
  server.send(200, "text/html", configPage());
}

void handleSave() {
  String newSsid = server.arg("ssid");
  String newPass = server.arg("pass");

  writeEEPROM(newSsid.c_str(), newPass.c_str());

  server.send(200, "text/html", "<h3>Saved! Rebooting...</h3>");
  delay(1000);
  ESP.restart();
}

void handleRetry() {
  server.send(200, "text/plain",
              "Device will reboot and attempt to reconnect using saved WiFi.");
  delay(500);
  ESP.restart();
}

void printHeap() {
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());
}

void readRawFoodSensorDistance (void) {
    digitalWrite(FOOD_SENSOR_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(FOOD_SENSOR_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(FOOD_SENSOR_TRIG_PIN, LOW);

    long duration = pulseIn(FOOD_SENSOR_ECHO_PIN, HIGH, 30000); // 30ms timeout
    
    if (duration == 0) {
        Serial.println(F("[Food Sensor] No response - Check wiring and power"));
        return;
    }
    
    rawFoodSensorDistance = duration * 0.034 / 2; // Convert to cm
}

void readRawWaterSensorDistance (void) {
    digitalWrite(WATER_SENSOR_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(WATER_SENSOR_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(WATER_SENSOR_TRIG_PIN, LOW);

    long duration = pulseIn(WATER_SENSOR_ECHO_PIN, HIGH, 30000); // 30ms timeout
    
    if (duration == 0) {
        Serial.println(F("[Water Sensor] No response - Check wiring and power"));
        return;
    }
    
    rawWaterSensorDistance = duration * 0.034 / 2; // Convert to cm
}

void checkFoodLevel (void) {
    if (foodEmptyDistance == foodFullDistance) {
        Serial.println(F("[Food] ERROR: Empty and Full distance are the same!"));
        foodLevelPercent = 0;
        return;
    }

    foodLevelPercent = 100 * (foodEmptyDistance - rawFoodSensorDistance) / (foodEmptyDistance - foodFullDistance);

    if (foodLevelPercent < 0) {
        foodLevelPercent = 0; 
        return;
    }

    if (foodLevelPercent > 100) {
        foodLevelPercent = 100;
        return;
    }
}

void checkWaterLevel (void) {
    if (waterEmptyDistance == waterFullDistance) {
        Serial.println(F("[Water] ERROR: Empty and Full distances are the same!"));
        waterLevelPercent = 0;
        return;
    }

    waterLevelPercent = 100 * (waterEmptyDistance - rawWaterSensorDistance) / (waterEmptyDistance - waterFullDistance);

    if (waterLevelPercent < 0) {
        waterLevelPercent = 0;
        return;
    }

    if (waterLevelPercent > 100) {
        waterLevelPercent = 100;
        return;
    }
}

void showFoodLevelData (void) {
    Serial.print(F("Raw Food Sensor Distance: "));
    Serial.print(rawFoodSensorDistance);
    Serial.println(F(" cm"));
    
    Serial.print(F("Food Level: "));
    Serial.print(foodLevelPercent);
    Serial.println(F(" %"));
}

void showWaterLevelData (void) {
    Serial.print(F("Raw Water Sensor Distance: "));
    Serial.print(rawWaterSensorDistance);
    Serial.println(F(" cm"));

    Serial.print(F("Water Level: "));
    Serial.print(waterLevelPercent);
    Serial.println(F(" %"));
}

void showMicData (void) {
    Serial.print(F("Microphone Analog Value: "));
    Serial.println(micAnalogValue);
}

void showGateSwitchData (void) {
    Serial.print(F("Gate Switch Status: "));
    Serial.println(isGateSwitchClosed ? "CLOSED" : "OPEN");
}

bool sendLogToFirebase (
    const String& category,
    const String& event,
    const String& target,
    int value,
    const String& source,
    const String& reason
) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[LOG] WiFi not connected"));
        return false;
    }

    DateTime now = rtc.now();

    String url = withAuth(String(FIREBASE_URL) + "/event_log.json");

    http.begin(fbClient, url);
    http.addHeader("Content-Type", "application/json");

    String payload = "{";
    payload += "\"timestamp\":" + String(now.unixtime()) + ",";
    payload += "\"datetime\":\"" + String(now.year()) + "-" +
               String(now.month()) + "-" +
               String(now.day()) + " " +
               String(now.hour()) + ":" +
               String(now.minute()) + ":" +
               String(now.second()) + "\",";
    payload += "\"source\":\"" + source + "\",";
    payload += "\"category\":\"" + category + "\",";
    payload += "\"event\":\"" + event + "\",";
    payload += "\"target\":\"" + target + "\",";
    payload += "\"value\":" + String(value) + ",";
    payload += "\"reason\":\"" + reason + "\"";
    payload += "}";

    int code = http.POST(payload);

    if (code > 0) {
        Serial.print(F("[LOG] Sent OK, code: "));
        Serial.println(code);
    } else {
        Serial.print(F("[LOG] Failed: "));
        Serial.println(http.errorToString(code));
    }

    http.end();
    return (code > 0);
}

void logEventToSDCard (
    const char* category,
    const char* source,
    const char* action,
    const char* value
) {
    if (!sdInitialized) {
        Serial.println(F("[LOG] SD not initialized, log skipped"));
        return;
    }

    File logFile = SD.open("/event_log.csv", FILE_APPEND);
    if (!logFile) {
        Serial.println(F("[LOG] Failed to open event_log.csv"));
        return;
    }

    String line =
        getTimestamp() + "," +
        category + "," +
        source + "," +
        action + "," +
        value + "\n";

    logFile.print(line);
    logFile.close();

    Serial.print(F("[LOG] "));
    Serial.print(line);
}

String getTimestamp() {
    DateTime now = rtc.now();
    char buf[20];
    snprintf(buf, sizeof(buf),
             "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    return String(buf);
}

void startSensorDataSlideshow(void) {
    slideshowActive = true;
    currentSlideshowScreen = SLIDE_TEMP_HUM;
    lastSlideshowAdvance = millis();
    displaySlideshowScreen(currentSlideshowScreen);
}

void updateSensorDataSlideshow(void) {
    if (!slideshowActive) return;
    
    unsigned long now = millis();
    if (now - lastSlideshowAdvance >= SLIDESHOW_SCREEN_DURATION_MS) {
        nextSlideshowScreen();
        lastSlideshowAdvance = now;
    }
}

void nextSlideshowScreen(void) {
    currentSlideshowScreen = (currentSlideshowScreen + 1) % SLIDE_COUNT;
    displaySlideshowScreen(currentSlideshowScreen);
    Serial.print(F("[LCD] Advanced to screen: "));
    Serial.println(currentSlideshowScreen);
}

void previousSlideshowScreen(void) {
    if (currentSlideshowScreen == 0) {
        currentSlideshowScreen = SLIDE_COUNT - 1;
    } else {
        currentSlideshowScreen--;
    }
    displaySlideshowScreen(currentSlideshowScreen);
    Serial.print(F("[LCD] Went back to screen: "));
    Serial.println(currentSlideshowScreen);
}

void displaySlideshowScreen(uint8_t screenID) {
    lcd.clear();
    
    switch (screenID) {
        case SLIDE_TEMP_HUM:
            displaySlideTemperatureHumidity();
            break;
        case SLIDE_PRESSURE_ALT:
            displaySlidePressureAltitude();
            break;
        case SLIDE_WATER_LEVEL:
            displaySlideWaterLevel();
            break;
        case SLIDE_FOOD_LEVEL:
            displaySlideFoodLevel();
            break;
        case SLIDE_MOTION_GATE:
            displaySlideMotionGate();
            break;
        case SLIDE_SYSTEM_INFO:
            displaySlideSystemInfo();
            break;
        default:
            lcd.print(F("Unknown Screen"));
            break;
    }
}

void displaySlideTemperatureHumidity(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("Temp: "));
    lcd.print(ahtTemperature, 1);
    lcd.print(F("C"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("Hum:  "));
    lcd.print(ahtHumidity, 1);
    lcd.print(F("%"));
}

void displaySlidePressureAltitude(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("Press: "));
    lcd.print(bmpPressure, 0);
    lcd.print(F("hPa"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("Alt: "));
    lcd.print(bmpAltitude, 0);
    lcd.print(F("m"));
}

void displaySlideWaterLevel(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("Water Level:"));
    
    lcd.setCursor(0, 1);
    lcd.print(waterLevelPercent, 0);
    lcd.print(F("%"));
    
    // Optional: Add a simple bar graph
    uint8_t barLength = (waterLevelPercent / 100.0) * 10;
    lcd.print(F(" ["));
    for (uint8_t i = 0; i < 10; i++) {
        lcd.print(i < barLength ? "#" : "-");
    }
    lcd.print(F("]"));
}

void displaySlideFoodLevel(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("Food Level:"));
    
    lcd.setCursor(0, 1);
    lcd.print(foodLevelPercent, 0);
    lcd.print(F("%"));
    
    // Optional: Add a simple bar graph
    uint8_t barLength = (foodLevelPercent / 100.0) * 10;
    lcd.print(F(" ["));
    for (uint8_t i = 0; i < 10; i++) {
        lcd.print(i < barLength ? "#" : "-");
    }
    lcd.print(F("]"));
}

void displaySlideMotionGate(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("Motion: "));
    lcd.print(motionDetected ? F("YES") : F("NO"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("Gate:   "));
    lcd.print(isGateSwitchClosed ? F("CLOSED") : F("OPEN"));
}

void displaySlideSystemInfo(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("WiFi: "));
    if (WiFi.status() == WL_CONNECTED) {
        lcd.print(F("OK"));
    } else {
        lcd.print(F("OFF"));
    }
    
    lcd.setCursor(0, 1);
    lcd.print(F("Free: "));
    lcd.print(ESP.getFreeHeap() / 1024);
    lcd.print(F("KB"));
}

void processMenuSelection(void) {
    if (slideshowActive) {
        slideshowActive = false;
        menuDirty = true;
        updateDisplayUI();
        return;
    }

    switch (currentMenu) {
        case MENU_SHOW_DATA:
            startSensorDataSlideshow();
            Serial.println(F("Sensor slideshow started. Use < and > to navigate."));
            break;
        case MENU_CHECK_SCHEDULE:
            enterScheduleViewMenu();
            selectScheduleItem();
            Serial.println(F("Schedule menu entered. Use L/R/C to navigate."));
            break;
        case MENU_CHECK_THRESHOLD:
            enterAlertThresholdMenu();
            selectAlertItem();
            Serial.println(F("Alert threshold menu entered. Use L/R/C to navigate."));
            break;
        default:
            break;
    }
}

void displayConfigMenu(void) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Config Menu IP:"));
    lcd.setCursor(0, 1);
    lcd.print(WiFi.softAPIP());
}

void enterScheduleViewMenu(void) {
    Serial.println(F("[Schedule Menu] Entered schedule view"));
    scheduleViewMode = SCHED_VIEW_MAIN_MENU;
    selectedScheduleIndex = 0;
    scheduleViewDirty = true;
    updateScheduleViewMenu();
}

void exitScheduleViewMenu(void) {
    Serial.println(F("[Schedule Menu] Exited schedule view"));
    scheduleViewMode = SCHED_VIEW_MAIN_MENU;
    selectedScheduleIndex = 0;
    scheduleViewDirty = true;
    menuDirty = true;
}

void updateScheduleViewMenu(void) {
    unsigned long now = millis();
    
    if (!scheduleViewDirty && (now - lastScheduleViewUpdate < UI_REFRESH_INTERVAL_MS)) {
        return;
    }
    
    lastScheduleViewUpdate = now;
    scheduleViewDirty = false;
    
    displayScheduleViewMenu();
}

void displayScheduleViewMenu(void) {
    lcd.clear();
    
    switch (scheduleViewMode) {
        case SCHED_VIEW_MAIN_MENU:
            displayScheduleMainMenu();
            break;
        case SCHED_VIEW_FEEDER_LIST:
            displayFeederScheduleList();
            break;
        case SCHED_VIEW_WATER_LIST:
            displayWaterScheduleList();
            break;
        case SCHED_VIEW_FEEDER_DETAIL:
            displayScheduleDetail(selectedScheduleIndex);
            break;
        case SCHED_VIEW_WATER_DETAIL:
            displayScheduleDetail(selectedScheduleIndex);
            break;
        default:
            break;
    }
}

void displayScheduleMainMenu(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("> SCHEDULE <"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("Feeder / Water"));
    
    Serial.println(F("[Schedule] Main menu - Use L/R to select"));
}

void displayFeederScheduleList(void) {
    // Count enabled feeder schedules
    uint8_t feederCount = 0;
    uint8_t feederIndices[MAX_SCHEDULES];
    
    for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
        if (schedules[i].enabled && schedules[i].type == SCHED_FEEDER) {
            feederIndices[feederCount] = i;
            feederCount++;
        }
    }
    
    if (feederCount == 0) {
        lcd.setCursor(0, 0);
        lcd.print(F("< Feeder >"));
        lcd.setCursor(0, 1);
        lcd.print(F("No schedules"));
        return;
    }
    
    // Wrap selection index
    if (selectedScheduleIndex >= feederCount) {
        selectedScheduleIndex = 0;
    }
    
    // Store the actual schedule index for detail view
    currentScheduleIndex = feederIndices[selectedScheduleIndex];
    uint8_t currentIdx = currentScheduleIndex;
    
    lcd.setCursor(0, 0);
    lcd.print(F("< Feeder "));
    lcd.print(selectedScheduleIndex + 1);
    lcd.print(F("/"));
    lcd.print(feederCount);
    lcd.print(F(" >"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("Time: "));
    lcd.print(schedules[currentIdx].time);
    if (schedules[currentIdx].executed) {
        lcd.print(F(" [D]"));  // D for Done
    } else {
        lcd.print(F(" [ ]"));
    }
}

void displayWaterScheduleList(void) {
    // Count enabled water schedules
    uint8_t waterCount = 0;
    uint8_t waterIndices[MAX_SCHEDULES];
    
    for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
        if (schedules[i].enabled && schedules[i].type == SCHED_WATER) {
            waterIndices[waterCount] = i;
            waterCount++;
        }
    }
    
    if (waterCount == 0) {
        lcd.setCursor(0, 0);
        lcd.print(F("< Water >"));
        lcd.setCursor(0, 1);
        lcd.print(F("No schedules"));
        return;
    }
    
    // Wrap selection index
    if (selectedScheduleIndex >= waterCount) {
        selectedScheduleIndex = 0;
    }
    
    // Store the actual schedule index for detail view
    currentScheduleIndex = waterIndices[selectedScheduleIndex];
    uint8_t currentIdx = currentScheduleIndex;
    
    lcd.setCursor(0, 0);
    lcd.print(F("< Water "));
    lcd.print(selectedScheduleIndex + 1);
    lcd.print(F("/"));
    lcd.print(waterCount);
    lcd.print(F(" >"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("Time: "));
    lcd.print(schedules[currentIdx].time);
    if (schedules[currentIdx].executed) {
        lcd.print(F(" [D]"));  // D for Done
    } else {
        lcd.print(F(" [ ]"));
    }
}

void displayScheduleDetail(uint8_t index) {
    if (index >= MAX_SCHEDULES) return;
    
    ScheduleSlot& slot = schedules[index];
    
    lcd.setCursor(0, 0);
    lcd.print(slot.type == SCHED_FEEDER ? F("FEEDER") : F("WATER"));
    lcd.print(F(" #"));
    lcd.print(index + 1);
    
    lcd.setCursor(0, 1);
    lcd.print(F("Time: "));
    lcd.print(slot.time);
    lcd.print(F(" - "));
    lcd.print(slot.enabled ? F("ON") : F("OFF"));
}

void navigateScheduleMenu(int direction) {
    switch (scheduleViewMode) {
        case SCHED_VIEW_MAIN_MENU:
            // L/R buttons don't navigate from main menu (only C does)
            break;
            
        case SCHED_VIEW_FEEDER_LIST: {
            // L button: Switch to Water mode
            if (direction < 0) {
                scheduleViewMode = SCHED_VIEW_WATER_LIST;
                selectedScheduleIndex = 0;
                scheduleViewDirty = true;
                Serial.println(F("[Schedule] Switched to Water list"));
            } else if (direction > 0) {
                // R button: Navigate through feeder schedules
                uint8_t count = 0;
                for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
                    if (schedules[i].enabled && schedules[i].type == SCHED_FEEDER) {
                        count++;
                    }
                }
                if (count > 0) {
                    selectedScheduleIndex++;
                    if (selectedScheduleIndex >= count) {
                        selectedScheduleIndex = 0;  // Circular
                    }
                    scheduleViewDirty = true;
                }
            }
            break;
        }
            
        case SCHED_VIEW_WATER_LIST: {
            // L button: Switch to Feeder mode
            if (direction < 0) {
                scheduleViewMode = SCHED_VIEW_FEEDER_LIST;
                selectedScheduleIndex = 0;
                scheduleViewDirty = true;
                Serial.println(F("[Schedule] Switched to Feeder list"));
            } else if (direction > 0) {
                // R button: Navigate through water schedules
                uint8_t count = 0;
                for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
                    if (schedules[i].enabled && schedules[i].type == SCHED_WATER) {
                        count++;
                    }
                }
                if (count > 0) {
                    selectedScheduleIndex++;
                    if (selectedScheduleIndex >= count) {
                        selectedScheduleIndex = 0;  // Circular
                    }
                    scheduleViewDirty = true;
                }
            }
            break;
        }
            
        case SCHED_VIEW_FEEDER_DETAIL:
        case SCHED_VIEW_WATER_DETAIL:
            // L/R don't navigate in detail view (only C to go back)
            break;
            
        default:
            break;
    }
}

void selectScheduleItem(void) {
    switch (scheduleViewMode) {
        case SCHED_VIEW_MAIN_MENU:
            // C button to enter Feeder list
            scheduleViewMode = SCHED_VIEW_FEEDER_LIST;
            selectedScheduleIndex = 0;
            scheduleViewDirty = true;
            Serial.println(F("[Schedule] Entered Feeder list"));
            break;
            
        case SCHED_VIEW_FEEDER_LIST:
        case SCHED_VIEW_WATER_LIST:
            // C button to view detail
            Serial.println(F("[Schedule] Viewing schedule detail"));
            scheduleViewMode = (scheduleViewMode == SCHED_VIEW_FEEDER_LIST) ? 
                               SCHED_VIEW_FEEDER_DETAIL : SCHED_VIEW_WATER_DETAIL;
            scheduleViewDirty = true;
            break;
            
        case SCHED_VIEW_FEEDER_DETAIL:
        case SCHED_VIEW_WATER_DETAIL:
            // C button to show back option
            scheduleViewMode = SCHED_VIEW_BACK_OPTION;
            scheduleViewDirty = true;
            Serial.println(F("[Schedule] Showing back option"));
            break;
            
        case SCHED_VIEW_BACK_OPTION:
            // C button to go back to main menu
            exitScheduleViewMenu();
            Serial.println(F("[Schedule] Returned to main menu"));
            break;
            
        default:
            break;
    }
}

void goBackScheduleView(void) {
    if (scheduleViewMode == SCHED_VIEW_FEEDER_DETAIL || 
        scheduleViewMode == SCHED_VIEW_WATER_DETAIL) {
        // From detail view, go back to list view
        scheduleViewMode = (scheduleViewMode == SCHED_VIEW_FEEDER_DETAIL) ? 
                          SCHED_VIEW_FEEDER_LIST : SCHED_VIEW_WATER_LIST;
        scheduleViewDirty = true;
        Serial.println(F("[Schedule] Went back to list"));
    } else if (scheduleViewMode == SCHED_VIEW_FEEDER_LIST || 
               scheduleViewMode == SCHED_VIEW_WATER_LIST) {
        // From list view, go back to main menu
        scheduleViewMode = SCHED_VIEW_MAIN_MENU;
        selectedScheduleIndex = 0;
        scheduleViewDirty = true;
        Serial.println(F("[Schedule] Went back to main menu"));
    }
}

void sendActuatorStateToFirebase (void) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[Firebase] WiFi not connected, aborting actuator state send."));
        return;
    }

    fbClient.setInsecure();  // Firebase HTTPS

    String url = String(FIREBASE_URL) + "/status.json";
    if (strlen(FIREBASE_AUTH) > 0) {
        url += "?auth=";
        url += FIREBASE_AUTH;
    }

    // ---- Build JSON payload with actuator states ----
    String payload = "{";
    payload += "\"gate\":{";
    payload += "\"state\":" + String((actuatorState & _BV(GATE_RELAY)) ? "\"off\"" : "\"on\"") + ",";
    payload += "\"raw_bit\":" + String((actuatorState & _BV(GATE_RELAY)) ? 1 : 0);
    payload += "},";
    
    payload += "\"pump\":{";
    payload += "\"state\":" + String((actuatorState & _BV(PUMP_RELAY)) ? "\"off\"" : "\"on\"") + ",";
    payload += "\"raw_bit\":" + String((actuatorState & _BV(PUMP_RELAY)) ? 1 : 0) + ",";
    payload += "\"active\":" + String(pumpTimer.active ? "true" : "false");
    payload += "},";
    
    payload += "\"fan\":{";
    payload += "\"state\":" + String((actuatorState & _BV(FAN_RELAY)) ? "\"off\"" : "\"on\"") + ",";
    payload += "\"raw_bit\":" + String((actuatorState & _BV(FAN_RELAY)) ? 1 : 0);
    payload += "},";
    
    payload += "\"feeder\":{";
    payload += "\"state\":" + String(isFeederRunning ? "\"on\"" : "\"off\"") + ",";
    payload += "\"active\":" + String(feederTimer.active ? "true" : "false");
    payload += "},";
    
    payload += "\"timestamp\":" + String(rtc.now().unixtime());
    payload += "}";

    http.begin(fbClient, url);
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.PATCH(payload);

    if (httpCode > 0) {
        Serial.print(F("[Actuator] PATCH code: "));
        Serial.println(httpCode);

        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) {
            Serial.println(F("[Actuator] State updated to Firebase successfully."));
            Serial.print(F("[Actuator] Payload: "));
            Serial.println(payload);
        } else {
            Serial.print(F("[Actuator] Unexpected response: "));
            Serial.println(http.getString());
        }
    } else {
        Serial.print(F("[Actuator] PATCH failed: "));
        Serial.println(http.errorToString(httpCode));
    }

    http.end();
}

void displayBackOption(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("< Go Back >"));
    lcd.setCursor(0, 1);
    lcd.print(F("Press C to return"));
}

void enterAlertThresholdMenu(void) {
    Serial.println(F("[Alert Menu] Entered alert threshold view"));
    alertThresholdViewMode = ALERT_VIEW_MAIN_MENU;
    selectedAlertIndex = 0;
    alertViewDirty = true;
    updateAlertThresholdView();
}

void exitAlertThresholdMenu(void) {
    Serial.println(F("[Alert Menu] Exited alert threshold view"));
    alertThresholdViewMode = ALERT_VIEW_MAIN_MENU;
    selectedAlertIndex = 0;
    alertViewDirty = true;
    menuDirty = true;
}

void updateAlertThresholdView(void) {
    unsigned long now = millis();
    
    if (!alertViewDirty && (now - lastAlertViewUpdate < UI_REFRESH_INTERVAL_MS)) {
        return;
    }
    
    lastAlertViewUpdate = now;
    alertViewDirty = false;
    
    displayAlertThresholdMenu();
}

void displayAlertThresholdMenu(void) {
    lcd.clear();
    
    switch (alertThresholdViewMode) {
        case ALERT_VIEW_MAIN_MENU:
            displayAlertMainMenu();
            break;
        case ALERT_VIEW_TEMPERATURE:
            displayTemperatureThreshold();
            break;
        case ALERT_VIEW_HUMIDITY:
            displayHumidityThreshold();
            break;
        case ALERT_VIEW_WATER_LEVEL:
            displayWaterLevelThreshold();
            break;
        case ALERT_VIEW_FOOD_LEVEL:
            displayFoodLevelThreshold();
            break;
        case ALERT_VIEW_BACK_OPTION:
            displayAlertBackOption();
            break;
        default:
            break;
    }
}

void displayAlertMainMenu(void) {
    lcd.setCursor(0, 0);
    lcd.print(F(">ALERT THRES<"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("Temp/Hum/Level"));
    
    Serial.println(F("[Alert] Main menu - Use R to navigate"));
}

void displayTemperatureThreshold(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("Temp (C)"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("H:"));
    lcd.print(tempHighThreshold, 1);
    lcd.print(F(" L:"));
    lcd.print(tempLowThreshold, 1);
}

void displayHumidityThreshold(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("Humidity (%)"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("H:"));
    lcd.print(humHighThreshold, 1);
    lcd.print(F(" L:"));
    lcd.print(humLowThreshold, 1);
}

void displayWaterLevelThreshold(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("Water Level (%)"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("L:"));
    lcd.print(waterLowThreshold, 1);
    lcd.print(F(" H:"));
    lcd.print(waterHighThreshold, 1);
}

void displayFoodLevelThreshold(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("Food Level (%)"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("L:"));
    lcd.print(foodLowThreshold, 1);
    lcd.print(F(" H:"));
    lcd.print(foodHighThreshold, 1);
}

void displayAlertBackOption(void) {
    lcd.setCursor(0, 0);
    lcd.print(F("< Go Back >"));
    
    lcd.setCursor(0, 1);
    lcd.print(F("Press C to return"));
}

void navigateAlertMenu(int direction) {
    switch (alertThresholdViewMode) {
        case ALERT_VIEW_MAIN_MENU:
            // L/R buttons don't navigate from main menu
            break;
            
        case ALERT_VIEW_TEMPERATURE:
        case ALERT_VIEW_HUMIDITY:
        case ALERT_VIEW_WATER_LEVEL:
        case ALERT_VIEW_FOOD_LEVEL:
            // R button cycles through thresholds
            if (direction > 0) {
                alertThresholdViewMode++;
                if (alertThresholdViewMode > ALERT_VIEW_FOOD_LEVEL) {
                    alertThresholdViewMode = ALERT_VIEW_TEMPERATURE;
                }
                alertViewDirty = true;
                Serial.print(F("[Alert] Switched to threshold mode: "));
                Serial.println(alertThresholdViewMode);
            }
            // L button goes back to previous threshold
            else if (direction < 0) {
                if (alertThresholdViewMode == ALERT_VIEW_TEMPERATURE) {
                    alertThresholdViewMode = ALERT_VIEW_FOOD_LEVEL;
                } else {
                    alertThresholdViewMode--;
                }
                alertViewDirty = true;
                Serial.print(F("[Alert] Switched to threshold mode: "));
                Serial.println(alertThresholdViewMode);
            }
            break;
            
        case ALERT_VIEW_BACK_OPTION:
            // L/R don't navigate in back option
            break;
            
        default:
            break;
    }
}

void selectAlertItem(void) {
    switch (alertThresholdViewMode) {
        case ALERT_VIEW_MAIN_MENU:
            // C button to enter Temperature threshold
            alertThresholdViewMode = ALERT_VIEW_TEMPERATURE;
            alertViewDirty = true;
            Serial.println(F("[Alert] Viewing Temperature threshold"));
            break;
            
        case ALERT_VIEW_TEMPERATURE:
        case ALERT_VIEW_HUMIDITY:
        case ALERT_VIEW_WATER_LEVEL:
        case ALERT_VIEW_FOOD_LEVEL:
            // C button to show back option
            alertThresholdViewMode = ALERT_VIEW_BACK_OPTION;
            alertViewDirty = true;
            Serial.println(F("[Alert] Showing back option"));
            break;
            
        case ALERT_VIEW_BACK_OPTION:
            // C button to go back to main menu
            exitAlertThresholdMenu();
            Serial.println(F("[Alert] Returned to main menu"));
            break;
            
        default:
            break;
    }
}

void startMicTask() {
  // configure ADC (Arduino wrappers), resolution & attenuation
  analogReadResolution(12);        // 12-bit ADC (0..4095)
  analogSetPinAttenuation(MICROPHONE_PIN, ADC_11db); // allow larger input swings

  // create the FreeRTOS task pinned to MIC_TASK_CORE
  xTaskCreatePinnedToCore(
    micTask,
    "MicTask",
    MIC_TASK_STACK / sizeof(StackType_t),
    nullptr,
    MIC_TASK_PRIO,
    nullptr,
    MIC_TASK_CORE
  );
  
  Serial.print(F("[MicTask] Started on core ")); Serial.println(MIC_TASK_CORE);
}

// compute RMS of signed samples (centered on zero)
float computeRMSFromBuffer(const int16_t *buf, size_t len) {
  // Convert each raw ADC sample to signed centered value.
  // For 12-bit ADC (0..4095) center = 2048.
  double sumsq = 0.0;
  const double center = 2048.0;

  for (size_t i = 0; i < len; ++i) {
    double s = ((double)buf[i] - center);
    sumsq += s * s;
  }
  double meanSq = sumsq / (double)len;
  double rms = sqrt(meanSq);

  // Normalize to 0..1 by dividing by center
  float normalized = (float)(rms / center);
  if (normalized < 0.0f) normalized = 0.0f;
  return normalized;
}

// Helper: create float normalized buffer expected by EI (range -1..1)
void normalizeToFloatBuffer(const int16_t *in, size_t len, float *out) {
  const float center = 2048.0f;
  for (size_t i = 0; i < len; ++i) {
    out[i] = ((float)in[i] - center) / center; // maps to approx [-1, 1]
  }
}

void micTask(void* pvParameters) {
  // allocate temporary float buffer for EI
  float *ei_input = (float*)heap_caps_malloc(SAMPLE_LENGTH * sizeof(float), MALLOC_CAP_8BIT);
  if (!ei_input) {
    Serial.println(F("[MicTask] Failed to allocate EI buffer!"));
    vTaskDelete(NULL);
    return;
  }
  ei_input_global = ei_input; 

  // register this task with the task watchdog
  esp_err_t wres = esp_task_wdt_add(NULL);
  if (wres != ESP_OK) {
    Serial.println(F("[MicTask] WARNING: esp_task_wdt_add failed"));
  } else {
    Serial.println(F("[MicTask] Registered with TWDT"));
  }

  // small safety delay so rest of system finishes setup
  vTaskDelay(pdMS_TO_TICKS(1000));

  const unsigned microDelay = (unsigned)(1000000UL / SAMPLE_RATE); // 50 us @ 20kHz
  const int WDT_FEED_INTERVAL_SAMPLES = 1000; // reset WDT every 1000 samples
  const int CHUNK_SIZE = 2000;               // samples per chunk (0.1s)
  const int NUM_CHUNKS = SAMPLE_LENGTH / CHUNK_SIZE; // should be 10 for 20000/2000

  // sanity check
  if (SAMPLE_LENGTH % CHUNK_SIZE != 0) {
    Serial.println(F("[MicTask] WARNING: SAMPLE_LENGTH not divisible by CHUNK_SIZE"));
  }

  for (;;) {
    int globalIndex = 0;
    int wdtCounter = 0;

    // Capture SAMPLE_LENGTH samples in CHUNK_SIZE blocks
    for (int chunk = 0; chunk < NUM_CHUNKS; ++chunk) {
      // capture one chunk
      for (int i = 0; i < CHUNK_SIZE; ++i) {
        int32_t v = analogRead(MICROPHONE_PIN); // [0..4095]
        audio_buffer[globalIndex++] = (int16_t)v;

        // precise small delay between samples
        ets_delay_us(microDelay);

        // feed TWDT regularly
        if (++wdtCounter >= WDT_FEED_INTERVAL_SAMPLES) {
          esp_task_wdt_reset();
          wdtCounter = 0;
        }
      }

      // Allow system idle and other tasks to run
      vTaskDelay(1);

      // Reset WDT again after the short delay
      esp_task_wdt_reset();
    } // end capture all chunks

    // --- Now we have the full SAMPLE_LENGTH buffer filled ---
    // Compute RMS
    float rms = computeRMSFromBuffer(audio_buffer, SAMPLE_LENGTH);

    // Update ambient baseline (EMA)
    ambient_rms_ema = (1.0f - AMBIENT_ALPHA) * ambient_rms_ema + AMBIENT_ALPHA * rms;
    float baseline = ambient_rms_ema;
    if (baseline < 1e-6f) baseline = 1e-6f;

    // Compute rel dB and scaled level
    float rel_ratio = rms / baseline;
    float rel_db = 20.0f * log10f(rel_ratio + 1e-12f);
    float clamped = (rel_db - MIN_REL_DB) / (MAX_REL_DB - MIN_REL_DB);
    if (clamped < 0.0f) clamped = 0.0f;
    if (clamped > 1.0f) clamped = 1.0f;
    uint8_t scaledLevel = (uint8_t)roundf(clamped * 100.0f);

    // Prepare EI input
    normalizeToFloatBuffer(audio_buffer, SAMPLE_LENGTH, ei_input);

    // reset WDT before heavy inference
    esp_task_wdt_reset();

    // Run classifier
    ei_signal_t signal;
    signal.total_length = SAMPLE_LENGTH;
    signal.get_data = &ei_get_data;

    ei_impulse_result_t result;
    unsigned long t0 = millis();
    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);
    unsigned long dt = millis() - t0;
    Serial.printf("[MicTask] inference dt=%lums\n", dt);

    float barkProb = 0.0f;
    if (r == EI_IMPULSE_OK) {
      barkProb = result.classification[0].value;
    } else {
      Serial.println(F("[MicTask] EI run_classifier error!"));
    }
    bool detected = (barkProb >= BARK_CONF_THRESHOLD);

    // Publish atomically
    portENTER_CRITICAL(&micMux);
      mic_level_rms = rms;
      mic_level = scaledLevel;
      bark_confidence = barkProb;
      bark_detected = detected;
      mic_ready = true;
    portEXIT_CRITICAL(&micMux);

    // Optional: light up RGB LED if bark detected
    if (detected) {
      pcf2.digitalWrite(DOG_BARK_IND, LOW);
    } else {
      pcf2.digitalWrite(DOG_BARK_IND, HIGH);
    }

    // Final WDT reset and give scheduler a chance
    esp_task_wdt_reset();
    vTaskDelay(1); // allow other tasks and IDLE to run again
  } // end for(;;)

  // cleanup (unreachable)
  esp_task_wdt_delete(NULL);
  free(ei_input);
  vTaskDelete(NULL);
}

int ei_get_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = ei_input_global[offset + i];
    }
    return 0;
}

void appendMicInferenceToJSON(String &json) {
  float rms;
  uint8_t lvl;
  float conf;
  bool bark;

  portENTER_CRITICAL(&micMux);
    rms = mic_level_rms;
    lvl = mic_level;
    conf = bark_confidence;
    bark = bark_detected;
  portEXIT_CRITICAL(&micMux);

  json += "\"mic_rms\":" + String(rms, 4) + ",";
  json += "\"mic_level\":" + String(lvl) + ",";
  json += "\"bark_confidence\":" + String(conf, 3) + ",";
  json += "\"bark_detected\":" + String(bark ? "true" : "false") + ",";
}

float mic_get_rms() {
    float rms;
    portENTER_CRITICAL(&micMux);
        rms = mic_level_rms;
    portEXIT_CRITICAL(&micMux);
    return rms;
}

uint8_t mic_get_level() {
    uint8_t lvl;
    portENTER_CRITICAL(&micMux);
        lvl = mic_level;
    portEXIT_CRITICAL(&micMux);
    return lvl;
}

float mic_get_bark_confidence() {
    float conf;
    portENTER_CRITICAL(&micMux);
        conf = bark_confidence;
    portEXIT_CRITICAL(&micMux);
    return conf;
}

bool mic_is_bark_detected() {
    bool bark;
    portENTER_CRITICAL(&micMux);
        bark = bark_detected;
    portEXIT_CRITICAL(&micMux);
    return bark;
}

void readMicValuesExample() {
  float rms_local;
  uint8_t level_local;
  float bark_local;
  bool barkBool_local;
  bool ready_local;

  portENTER_CRITICAL(&micMux);
    rms_local = mic_level_rms;
    level_local = mic_level;
    bark_local = bark_confidence;
    barkBool_local = bark_detected;
    ready_local = mic_ready;
  portEXIT_CRITICAL(&micMux);

  if (ready_local) {
    Serial.printf("[Mic] RMS=%f, Level=%u%%, BarkProb=%f, Det=%d\n",
                  rms_local, level_local, bark_local, barkBool_local ? 1 : 0);
  }
}

void displayBarkPage() {
  float rms;
  uint8_t lvl;
  float conf;
  bool bark;

  portENTER_CRITICAL(&micMux);
    rms = mic_level_rms;
    lvl = mic_level;
    conf = bark_confidence;
    bark = bark_detected;
  portEXIT_CRITICAL(&micMux);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Bark: "));
  lcd.print(bark ? "YES" : "NO");

  lcd.setCursor(0, 1);
  lcd.print(F("Conf: "));
  lcd.print(conf, 2);
}

void displayMicrophoneNoisePage() {
    float rms;
    uint8_t lvl;

    portENTER_CRITICAL(&micMux);
        rms = mic_level_rms;
        lvl = mic_level;
    portEXIT_CRITICAL(&micMux);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Mic Level: "));
    lcd.print(lvl);
    lcd.print(F("%"));

    lcd.setCursor(0, 1);
    lcd.print(F("RMS: "));
    lcd.print(rms, 3);
}

void sendDeviceHeartbeat (void) {
    if (millis() - lastHeartbeat< HEARTBEAT_INTERVAL_MS) {
        return;
    }
    lastHeartbeat = millis();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[Heartbeat] WiFi not connected, aborting heartbeat send."));
        return;
    }

    fbClient.setInsecure();  // Firebase HTTPS

    String url = String(FIREBASE_URL) + "/heartbeat.json";
    url = withAuth(url);

    String payload = "{";
    payload += "\"last_seen\":" + String(rtc.now().unixtime());
    payload += "}";

    http.begin(fbClient, url);
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.PATCH(payload);

    if (httpCode > 0) {
        Serial.print(F("[Heartbeat] PATCH code: "));
        Serial.println(httpCode);

        if (httpCode == HTTP_CODE_OK) {
            Serial.println(F("[Heartbeat] Device heartbeat sent successfully."));
        } else {
            Serial.print(F("[Heartbeat] Response: "));
            Serial.println(http.getString());
        }
    } else {
        Serial.print(F("[Heartbeat] PATCH failed: "));
        Serial.println(http.errorToString(httpCode));
    }

    http.end();
}

void checkOccupationThresholds (void) {
    // Auto-stop pump if water too high
    if (waterLevelPercent >= waterHighThreshold && pumpTimer.active) {
        Serial.println(F("[AUTO-OFF] Water level HIGH - stopping pump"));
        stopPump();
        sendLogToFirebase("actuator", "pump_auto_off", "water_level", (int)waterLevelPercent, "device", "occupation_high");
    }
    
    // Auto-stop feeder if food too high
    if (foodLevelPercent >= foodHighThreshold && isFeederRunning) {
        Serial.println(F("[AUTO-OFF] Food level HIGH - stopping feeder"));
        stopFeeder();
        sendLogToFirebase("actuator", "feeder_auto_off", "food_level", (int)foodLevelPercent, "device", "occupation_high");
    }
}

void checkFanAutoControl (void) {
    if (!isFanAutoMode) return;
    // Enable fan if temperature exceeds high threshold
    if (ahtTemperature >= tempHighThreshold && !isFanAutoControlActive) {
        Serial.print(F("[AUTO-FAN] Temperature HIGH ("));
        Serial.print(ahtTemperature, 1);
        Serial.print(F("°C >= "));
        Serial.print(tempHighThreshold, 1);
        Serial.println(F("°C) - enabling fan"));
        enableFan();
        isFanAutoControlActive = true;
        sendLogToFirebase("actuator", "fan_auto_on", "temperature", (int)ahtTemperature, "device", "threshold_exceeded");
    }
    
    // Disable fan if temperature drops 2°C below high threshold
    // Hysteresis: turn off at (tempHighThreshold - 2)
    if (ahtTemperature <= (tempHighThreshold - tempDecreaseTarget) && isFanAutoControlActive) {
        Serial.print(F("[AUTO-FAN] Temperature NORMAL ("));
        Serial.print(ahtTemperature, 1);
        Serial.print(F("°C <= "));
        Serial.print(tempHighThreshold - tempDecreaseTarget, 1);
        Serial.println(F("°C) - disabling fan"));
        disableFan();
        isFanAutoControlActive = false;
        sendLogToFirebase("actuator", "fan_auto_off", "temperature", (int)ahtTemperature, "device", "recovered");
    }
}

void getConfiguration (void) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[System] Config data cannot be loaded. Using defaults..."));
        return;
    }

    fbClient.setInsecure();

    String url = String(FIREBASE_URL) + "/config.json";
    url = withAuth(url);

    Serial.println(F("[Config] Fetching configuration from Firebase..."));
    
    http.begin(fbClient, url);
    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK) {
        Serial.print(F("[Config] HTTP error: "));
        Serial.println(httpCode);
        http.end();
        return;
    }

    String payload = http.getString();
    http.end();

    if (payload.length() < 10) {
        Serial.println(F("[Config] Payload too short or empty"));
        return;
    }

    Serial.print(F("[Config] Received payload: "));
    Serial.println(payload);

    // -------- Parse fan_auto --------
    int fanAutoPos = payload.indexOf("\"fan_auto\"");
    if (fanAutoPos != -1) {
        int colonPos = payload.indexOf(":", fanAutoPos);
        String fanAutoValue = payload.substring(colonPos + 1);
        isFanAutoMode = (fanAutoValue.indexOf("true") != -1);
        Serial.print(F("[Config] fan_auto: "));
        Serial.println(isFanAutoMode ? F("true") : F("false"));
    }

    // -------- Parse full_filling --------
    int fullFillingPos = payload.indexOf("\"full_filling\"");
    if (fullFillingPos != -1) {
        int colonPos = payload.indexOf(":", fullFillingPos);
        String fullFillingValue = payload.substring(colonPos + 1);
        isFullFillingActive = (fullFillingValue.indexOf("true") != -1);
        Serial.print(F("[Config] full_filling: "));
        Serial.println(isFullFillingActive ? F("true") : F("false"));
    }

    // -------- Parse pet_name --------
    int petNamePos = payload.indexOf("\"pet_name\"");
    if (petNamePos != -1) {
        int colonPos = payload.indexOf(":", petNamePos);
        int q1 = payload.indexOf("\"", colonPos + 1);
        int q2 = payload.indexOf("\"", q1 + 1);
        
        if (q1 != -1 && q2 != -1) {
            String extracted = payload.substring(q1 + 1, q2);
            if (extracted.length() > 0 && extracted.length() <= 32) {
                petName = extracted;
                Serial.print(F("[Config] pet_name: "));
                Serial.println(petName);
            }
        }
    }

    Serial.println(F("[Config] Configuration loaded successfully"));
}

void updateBarkCounter(void) {    
    // Reset bark counter every BARK_COUNT_RESET_PERIOD (60 seconds)
    if (millis() - lastBarkCountReset >= BARK_COUNT_RESET_PERIOD) {
        if (barkCount > 0) {
            Serial.print(F("[BARK] Counter reset. Barks in last minute: "));
            Serial.println(barkCount);
        }
        barkCount = 0;
        lastBarkCountReset = millis();
    }
    
    // Count barks: increment counter only once per bark event
    if (mic_is_bark_detected()) {
        // Only count if at least 1 second has passed since last detection
        if (millis() - lastBarkDetectionTime >= 1000) {
            barkCount++;
            lastBarkDetectionTime = millis();
            
            Serial.print(F("[BARK] Bark #"));
            Serial.print(barkCount);
            Serial.print(F(" detected. Confidence: "));
            Serial.print(mic_get_bark_confidence(), 2);
            Serial.println(F("%"));
        }
    }
}