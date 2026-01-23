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

#define WIFI_CONNECT_IND    P3
#define SD_CARD_IND         P4
#define DOG_BARK_IND        P5

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
#define FEEDER_ACTIVE_DUR             2500
#define DEBOUNCE_DELAY_MS             50      // Debounce delay for button press
#define PCF_READ_INTERVAL_MS          30
#define SENSOR_READ_INTERVAL_MS       2000
#define UI_REFRESH_INTERVAL_MS        200
#define SENSOR_DATA_PATCH_INTERVAL_MS 6000    // Keep the system responsive
#define COMMAND_POLL_INTERVAL_MS      5000
#define ACTUATOR_CONTROL_COOLDOWN     5000    // 5s cooldown

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

#define DEF_SEND_DATA_PERIODICALLY  false
#define DEF_POLL_CMD_PERIODICALLY   false

#define LCD_ROW 2
#define LCD_COL 16

#define NUM_PIXELS 1

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

enum MenuID : uint8_t {
    MENU_SHOW_DATA = 0,
    MENU_CHECK_SCHEDULE,
    MENU_MANUAL_SCHEDULE,
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

byte actuatorState = 0b00000111; // Only use 3 bit for now
byte lastActuatorState = 0b00000111; // Only use 3 bit for now

bool isFeederRunning = false;

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

float foodFullDistance = 29.0;      // cm
float foodEmptyDistance = 33.0;     // cm
float waterFullDistance = 29.0;     // cm
float waterEmptyDistance = 33.0;    // cm

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

int8_t rgbMode = 0;

void initializeButtons (void);
void initializeRelays (void);
void initializeFeeder (void); // Not yet implemented now
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
void readSensorsData (void);
void showSavedAHTdata (void);
void showSavedBMPdata (void);
void showMotionSensorData (void);
void showSensorsData (void);
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
            printHeap();
            startConfigAP();
            printHeap();
            return; // stop normal boot
        }
    } else {
        Serial.println(F("No WiFi credentials found"));
        isConfigMode = true;
        startConfigAP();
        return;
    }

    initializeNTP();
    checkAndSyncRTCOnBoot();
    loadScheduleFromSDCard();
    syncSchedule();
    syncSensorThreshold();

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

    if (!pcf2.begin()) {
        Serial.println(F("ERROR: Could not initialize relays\' PCF8574! Check wiring, I2C address, SDA/SCL connections and power."));
        while (1) delay(100);
    }

    // Set relay state to low on beginning.
    pcf2.digitalWrite(GATE_RELAY, HIGH);
    pcf2.digitalWrite(PUMP_RELAY, HIGH);
    pcf2.digitalWrite(FAN_RELAY, HIGH);

    Serial.println(F("relays\' PCF8574 initialized successfully."));
}
void initializeFeeder (void) {
    feeder.setPeriodHertz(50);
    feeder.attach(FEEDER_PIN);
    feeder.write(0); // Initial position
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
                handleMenuNavigation(-1);
            } else {
                Serial.println(F("L Button Released."));
            }
        }

        if ((buttonState & _BV(C_BUTTON)) != (lastButtonState & _BV(C_BUTTON))) {
            if (!(buttonState & _BV(C_BUTTON))) {
                Serial.println(F("C Button Pressed."));
            } else {
                Serial.println(F("C Button Released."));
            }
        }

        if ((buttonState & _BV(R_BUTTON)) != (lastButtonState & _BV(R_BUTTON))) {
            if (!(buttonState & _BV(R_BUTTON))) {
                Serial.println(F("R Button Pressed."));
                handleMenuNavigation(+1);
            } else {
                Serial.println(F("R Button Released."));
            }
        }

        if ((buttonState & _BV(GATE_BUTTON)) != (lastButtonState & _BV(GATE_BUTTON))) {
            if (!(buttonState & _BV(GATE_BUTTON))) {
                Serial.println(F("Gate Button Pressed."));
                toggleGate();
            } else {
                Serial.println(F("Gate Button Released."));
            }
        }

        if ((buttonState & _BV(PUMP_BUTTON)) != (lastButtonState & _BV(PUMP_BUTTON))) {
            if (!(buttonState & _BV(PUMP_BUTTON))) {
                Serial.println(F("Pump Button Pressed."));
                startPump();
            } else {
                Serial.println(F("Pump Button Released."));
            }
        }

        if ((buttonState & _BV(FAN_BUTTON)) != (lastButtonState & _BV(FAN_BUTTON))) {
            if (!(buttonState & _BV(FAN_BUTTON))) {
                Serial.println(F("Fan Button Pressed."));
                toggleFan();
            } else {
                Serial.println(F("Fan Button Released."));
            }
        }

        if ((buttonState & _BV(FEEDER_BUTTON)) != (lastButtonState & _BV(FEEDER_BUTTON))) {
            if (!(buttonState & _BV(FEEDER_BUTTON))) {
                Serial.println(F("Feeder Button Pressed."));
                startFeeder();
            } else {
                Serial.println(F("Feeder Button Released."));
            }
        }

        lastButtonState = buttonState;
    }
}

void checkRelayActivity (void) {
    unsigned long now = millis();

    // Pump timer
    if (pumpTimer.active && (now - pumpTimer.startTime >= pumpTimer.duration)) {
        stopPump();
    }

    // Feeder timer
    if (feederTimer.active && (now - feederTimer.startTime >= feederTimer.duration)) {
        stopFeeder();
    }
}

void checkSerialCommand (void) {
    while (Serial.available() > 0) {
        char cmd = Serial.read();

        switch (cmd) {
            case '1': 
                toggleGate();
                Serial.println(F("Gate Relay Toggled."));
                break;
            case '2':
                startPump();
                Serial.println(F("Pump Relay Activated."));
                break;
            case '3':
                toggleFan();
                Serial.println(F("Fan Relay Toggled."));
                break;
            case '4':
                startFeeder();
                Serial.println(F("Feeder Activated."));
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
    feeder.write(180);
    isFeederRunning = true;
    Serial.println(F("Feeder ON"));
}

void stopFeeder(void) {
    feederTimer.active = false;
    feeder.write(0);
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
            } else if (schedules[i].type == SCHED_WATER) {
                startPump();
            }

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

void readSensorsData (void) {
    if (millis() - lastSensorReadTime < SENSOR_READ_INTERVAL_MS) return;
    lastSensorReadTime = millis();
    readAHTdata();
    readBMPdata();
    readMotionSensorData();
    readRawWaterSensorDistance();
    checkWaterLevel();

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

    String url = String(FIREBASE_URL) + "/threshold.json";
    if (strlen(FIREBASE_AUTH) > 0) {
        url += "?auth=";
        url += FIREBASE_AUTH;
    }

    Serial.println(F("[Threshold] Fetching threshold from Firebase..."));
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

    // -------- Temperature --------
    int tLow = payload.indexOf("\"temperature\"");
    if (tLow != -1) {
        tempLowThreshold  = payload.substring(payload.indexOf("\"low\":", tLow) + 6).toFloat();
        tempHighThreshold = payload.substring(payload.indexOf("\"high\":", tLow) + 7).toFloat();
    }

    // -------- Humidity --------
    int hLow = payload.indexOf("\"humidity\"");
    if (hLow != -1) {
        humLowThreshold  = payload.substring(payload.indexOf("\"low\":", hLow) + 6).toFloat();
        humHighThreshold = payload.substring(payload.indexOf("\"high\":", hLow) + 7).toFloat();
    }

    Serial.println(F("[Threshold] Updated:"));
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
        return;
    }

    if (action == "enable") {
        if (target == "fan") enableFan();
        if (target == "gate") enableGate();
        if (target == "pump") startPump();
        if (target == "feeder") startFeeder();
        return;
    }

    if (action == "disable") {
        if (target == "fan") disableFan();
        if (target == "gate") disableGate();
        if (target == "pump") stopPump();
        if (target == "feeder") stopFeeder();
        return;
    }

    if (action == "toggle") {
        if (target == "fan") toggleFan();
        if (target == "gate") toggleGate();
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