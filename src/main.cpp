#include <Arduino.h>
#include <PCF8574.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>

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

#define SD_CS   10
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK  12

#define PCF8574_ADDRESS_1 0x20
#define PCF8574_ADDRESS_2 0x21

#define PUMP_ACTIVE_DUR     3500
#define FEEDER_ACTIVE_DUR   2500
#define DEBOUNCE_DELAY_MS     50              // Debounce delay for button press
#define PCF_READ_INTERVAL_MS  30

#define MAX_SCHEDULES   8

PCF8574 pcf1(PCF8574_ADDRESS_1);
PCF8574 pcf2(PCF8574_ADDRESS_2);
RTC_DS3231 rtc;
SPIClass spiSD(FSPI);

enum ScheduleType : uint8_t {
  SCHED_FEEDER,
  SCHED_WATER
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

ActuatorTimer pumpTimer   = { false, 0, PUMP_ACTIVE_DUR };
ActuatorTimer feederTimer = { false, 0, FEEDER_ACTIVE_DUR };
ScheduleSlot schedules[MAX_SCHEDULES];

bool rtcInitialized = false;
bool sdInitialized = false;

byte buttonState = 0b01111111; // Only use 7 bit
byte lastButtonState = 0b01111111; // Only use 7 bit
byte rawButtonState = 0b01111111; // Only use 7 bit
byte stableButtonState = 0b01111111; // Only use 7 bit
byte lastRawButtonState = 0b01111111; // Only use 7 bit

unsigned long lastDebounceTime = 0;
unsigned long lastPCFReadTime = 0;

byte actuatorState = 0b00000111; // Only use 3 bit for now
byte lastActuatorState = 0b00000111; // Only use 3 bit for now

void initializeButtons (void);
void initializeRelays (void);
void initializeFeeder (void); // Not yet implemented now
void initializeRTC (void);
void initializeSDCardReader (void);

void readRawButtonInput (void);
void updateButtonInput (void);
void checkButtonStateChange (void);
void checkRelayActivity (void);
void checkSerialCommand (void);
void checkScheduleExecution (void);
void resetScheduleExecutionAtMidnight (void);
void printCurrentTime (void);
void printSchedule (void);
void printSDCardInfo (void);
void loadScheduleFromSDCard (void);
ScheduleType parseScheduleType (const char* str);

void startPump (void);
void stopPump (void);
void startFeeder (void);
void stopFeeder (void); 

void setup () {
    Serial.begin(115200);
    initializeButtons();
    initializeRelays();
    initializeFeeder();
    initializeRTC();
    initializeSDCardReader();
    loadScheduleFromSDCard();
}

void loop () {
    readRawButtonInput();
    updateButtonInput();
    checkRelayActivity();
    checkScheduleExecution();
    checkSerialCommand();
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
    pinMode(FEEDER_PIN, OUTPUT);
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
            } else {
                Serial.println(F("R Button Released."));
            }
        }

        if ((buttonState & _BV(GATE_BUTTON)) != (lastButtonState & _BV(GATE_BUTTON))) {
            if (!(buttonState & _BV(GATE_BUTTON))) {
                Serial.println(F("Gate Button Pressed."));
                if (actuatorState & _BV(GATE_RELAY)) {
                    actuatorState &= ~_BV(GATE_RELAY);
                } else {
                    actuatorState |= _BV(GATE_RELAY);
                }
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
                if (actuatorState & _BV(FAN_RELAY)) {
                    actuatorState &= ~_BV(FAN_RELAY);
                } else {
                    actuatorState |= _BV(FAN_RELAY);
                }
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
                if (actuatorState & _BV(GATE_RELAY)) {
                    actuatorState &= ~_BV(GATE_RELAY);
                } else {
                    actuatorState |= _BV(GATE_RELAY);
                }
                Serial.println(F("Gate Relay Toggled."));
                break;
            case '2':
                startPump();
                Serial.println(F("Pump Relay Activated."));
                break;
            case '3':
                if (actuatorState & _BV(FAN_RELAY)) {
                    actuatorState &= ~_BV(FAN_RELAY);
                } else {
                    actuatorState |= _BV(FAN_RELAY);
                }
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
                rtc.adjust(DateTime(2025, 1, 1, 16, 58, 0));
                Serial.println(F("RTC manually set to 16:58 for testing."));
                break;
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
    pcf2.digitalWrite(PUMP_RELAY, LOW);
    Serial.println(F("Pump ON"));
}

void stopPump(void) {
    pumpTimer.active = false;
    pcf2.digitalWrite(PUMP_RELAY, HIGH);
    Serial.println(F("Pump OFF"));
}

void startFeeder(void) {
    if (feederTimer.active) return; // prevent stacking

    feederTimer.active = true;
    feederTimer.startTime = millis();

    // For now: LED / relay simulation
    analogWrite(FEEDER_PIN, 255);
    Serial.println(F("Feeder ON"));
}

void stopFeeder(void) {
    feederTimer.active = false;
    analogWrite(FEEDER_PIN, 0);
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
    }

    if (now.hour() != 0 || now.minute() != 0) {
        static bool alreadyReset = false;
        alreadyReset = false;
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
    static bool alreadyReset = false;

    if (!alreadyReset) {
        Serial.println(F("Midnight reached. Resetting schedules."));
        for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
            schedules[i].executed = false;
        }
        alreadyReset = true;
    }
}
