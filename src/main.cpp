#include <Arduino.h>
#include <PCF8574.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <myconfig.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>

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

#define PCF8574_ADDRESS_1 0x20
#define PCF8574_ADDRESS_2 0x21

#define TIMEZONE_OFFSET_SEC (7 * 3600)
#define NTP_SYNC_INTERVAL (6UL * 60UL * 60UL * 1000UL)

PCF8574 pcf1(PCF8574_ADDRESS_1);
PCF8574 pcf2(PCF8574_ADDRESS_2);

enum PendingActionType {
  NONE = 0,
  CLEAR_COMMAND,
  SEND_STATUS
};

struct PendingAction {
  PendingActionType type = NONE;
  String key;     // Firebase command key
  String body;    // Status JSON
  int retries = 0;
};

PendingAction pendingAction;

byte button_state = 0b01111111; // Only use 7 bit
byte last_button_state = 0b01111111; // Only use 7 bit

byte actuator_state = 0b00000111; // Only use 3 bit for now
byte last_actuator_state = 0b00000111; // Only use 3 bit for now

WiFiClientSecure fbClient;
HTTPClient http;
bool firebaseReady = false;
unsigned long lastFirebaseConnect = 0;

WiFiUDP ntpUDP;
NTPClient ntpClient(
  ntpUDP,
  "pool.ntp.org",
  TIMEZONE_OFFSET_SEC,
  60 * 1000
);

static unsigned long cachedEpoch = 0;
static unsigned long cachedMillis = 0;
static bool timeValid = false;

static unsigned long lastNtpSync = 0;

void initialize_buttons (void);
void initialize_relays (void);
void read_button_state (void);
void check_button_state_change (void);
void check_relay_state_change (void);
void check_serial_command (void);

void fetch_relay_command (void);
void process_relay_command (const String& key, const String& target, const String& action);
void clear_relay_command ();
void queue_status_update (const String& action);
void perform_pending_firebase_actions ();

void connect_to_wifi(void);
bool ensure_firebase_connection(const String& url);

void time_init (void);
void time_loop (void);
unsigned long time_now_epoch (void);
String time_now_hhmm (void);
String time_now_string (void);
bool time_is_valid (void);

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing peripheral");

  initialize_buttons();
  initialize_relays();

  Serial.println(F("All PCF8574s has been initialized successfully."));
  delay(500);
  connect_to_wifi();
  Serial.println(F("Try to tap the buttons."));
}

void loop() {
  read_button_state();
  fetch_relay_command();
  check_button_state_change();
  check_serial_command();
  check_relay_state_change();
  perform_pending_firebase_actions();
  delay(50);
}

void initialize_buttons (void) {
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

void initialize_relays (void) {
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

void read_button_state(void) {
  if (pcf1.digitalRead(L_BUTTON)) {
    button_state |= _BV(L_BUTTON);
  } else {
    button_state &= ~_BV(L_BUTTON);
  }

  if (pcf1.digitalRead(C_BUTTON)) {
    button_state |= _BV(C_BUTTON);
  } else {
    button_state &= ~_BV(C_BUTTON);
  }

  if (pcf1.digitalRead(R_BUTTON)) {
    button_state |= _BV(R_BUTTON);
  } else {
    button_state &= ~_BV(R_BUTTON);
  }

  if (pcf1.digitalRead(GATE_BUTTON)) {
    button_state |= _BV(GATE_BUTTON);
  } else {
    button_state &= ~_BV(GATE_BUTTON);
  }

  if (pcf1.digitalRead(PUMP_BUTTON)) {
    button_state |= _BV(PUMP_BUTTON);
  } else {
    button_state &= ~_BV(PUMP_BUTTON);
  }

  if (pcf1.digitalRead(FAN_BUTTON)) {
    button_state |= _BV(FAN_BUTTON);
  } else {
    button_state &= ~_BV(FAN_BUTTON);
  }

  if (pcf1.digitalRead(FEEDER_BUTTON)) {
    button_state |= _BV(FEEDER_BUTTON);
  } else {
    button_state &= ~_BV(FEEDER_BUTTON);
  }
}

void check_button_state_change(void) {
  if (button_state != last_button_state) {
    if ((button_state & _BV(L_BUTTON)) != (last_button_state & _BV(L_BUTTON))) {
      if (!(button_state & _BV(L_BUTTON))) {
        Serial.println(F("L Button Pressed."));
      } else {
        Serial.println(F("L Button Released."));
      }
    }

    if ((button_state & _BV(C_BUTTON)) != (last_button_state & _BV(C_BUTTON))) {
      if (!(button_state & _BV(C_BUTTON))) {
        Serial.println(F("C Button Pressed."));
      } else {
        Serial.println(F("C Button Released."));
      }
    }

    if ((button_state & _BV(R_BUTTON)) != (last_button_state & _BV(R_BUTTON))) {
      if (!(button_state & _BV(R_BUTTON))) {
        Serial.println(F("R Button Pressed."));
      } else {
        Serial.println(F("R Button Released."));
      }
    }

    if ((button_state & _BV(GATE_BUTTON)) != (last_button_state & _BV(GATE_BUTTON))) {
      if (!(button_state & _BV(GATE_BUTTON))) {
        Serial.println(F("Gate Button Pressed."));
        if (actuator_state & _BV(GATE_RELAY)) {
          actuator_state &= ~_BV(GATE_RELAY);
        } else {
          actuator_state |= _BV(GATE_RELAY);
        }
      } else {
        Serial.println(F("Gate Button Released."));
      }
    }

    if ((button_state & _BV(PUMP_BUTTON)) != (last_button_state & _BV(PUMP_BUTTON))) {
      if (!(button_state & _BV(PUMP_BUTTON))) {
        Serial.println(F("Pump Button Pressed."));
        if (actuator_state & _BV(PUMP_RELAY)) {
          actuator_state &= ~_BV(PUMP_RELAY);
        } else {
          actuator_state |= _BV(PUMP_RELAY);
        }
      } else {
        Serial.println(F("Pump Button Released."));
      }
    }

    if ((button_state & _BV(FAN_BUTTON)) != (last_button_state & _BV(FAN_BUTTON))) {
      if (!(button_state & _BV(FAN_BUTTON))) {
        Serial.println(F("Fan Button Pressed."));
        if (actuator_state & _BV(FAN_RELAY)) {
          actuator_state &= ~_BV(FAN_RELAY);
        } else {
          actuator_state |= _BV(FAN_RELAY);
        }
      } else {
        Serial.println(F("Fan Button Released."));
      }
    }

    if ((button_state & _BV(FEEDER_BUTTON)) != (last_button_state & _BV(FEEDER_BUTTON))) {
      if (!(button_state & _BV(FEEDER_BUTTON))) {
        Serial.println(F("Feeder Button Pressed."));
      } else {
        Serial.println(F("Feeder Button Released."));
      }
    }

    last_button_state = button_state;
  }
}

void check_relay_state_change (void) {
  if (actuator_state != last_actuator_state) {
    if ((actuator_state & _BV(GATE_RELAY)) != (last_actuator_state & _BV(GATE_RELAY))) {
      if (!(actuator_state & _BV(GATE_RELAY))) {
        pcf2.digitalWrite(GATE_RELAY, LOW);
        Serial.println(F("Gate Relay ON."));
      } else {
        pcf2.digitalWrite(GATE_RELAY, HIGH);
        Serial.println(F("Gate Relay OFF."));
      }
    }

    if ((actuator_state & _BV(PUMP_RELAY)) != (last_actuator_state & _BV(PUMP_RELAY))) {
      if (!(actuator_state & _BV(PUMP_RELAY))) {
        pcf2.digitalWrite(PUMP_RELAY, LOW);
        Serial.println(F("Pump Relay ON."));
      } else {
        pcf2.digitalWrite(PUMP_RELAY, HIGH);
        Serial.println(F("Pump Relay OFF."));
      }
    }

    if ((actuator_state & _BV(FAN_RELAY)) != (last_actuator_state & _BV(FAN_RELAY))) {
      if (!(actuator_state & _BV(FAN_RELAY))) {
        pcf2.digitalWrite(FAN_RELAY, LOW);
        Serial.println(F("Fan Relay ON."));
      } else {
        pcf2.digitalWrite(FAN_RELAY, HIGH);
        Serial.println(F("Fan Relay OFF."));
      }
    }

    Serial.println(actuator_state);
    last_actuator_state = actuator_state;
  }
}

void check_serial_command (void) {
  while (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case '1': 
        if (actuator_state & _BV(GATE_RELAY)) {
          actuator_state &= ~_BV(GATE_RELAY);
        } else {
          actuator_state |= _BV(GATE_RELAY);
        }
        break;
      case '2':
        if (actuator_state & _BV(PUMP_RELAY)) {
          actuator_state &= ~_BV(PUMP_RELAY);
        } else {
          actuator_state |= _BV(PUMP_RELAY);
        }
        break;
      case '3':
        if (actuator_state & _BV(FAN_RELAY)) {
          actuator_state &= ~_BV(FAN_RELAY);
        } else {
          actuator_state |= _BV(FAN_RELAY);
        }
        break;
      case 'q':
      case 'Q':
        actuator_state &= ~_BV(GATE_RELAY); break;
      case 'w':
      case 'W':
        actuator_state |= _BV(GATE_RELAY); break;
      case 'e':
      case 'E':
        actuator_state &= ~_BV(PUMP_RELAY); break;
      case 'r':
      case 'R':
        actuator_state |= _BV(PUMP_RELAY); break;
      case 't':
      case 'T':
        actuator_state &= ~_BV(FAN_RELAY); break;
      case 'y':
      case 'Y':
        actuator_state |= _BV(FAN_RELAY); break;
    }
  }
}

void connect_to_wifi (void) {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  fbClient.setInsecure();
}

bool ensure_firebase_connection(const String& url) {
  if (!http.begin(fbClient, url)) {
    Serial.println("[FB] begin() failed");
    return false;
  }
  http.useHTTP10(false);
  http.setReuse(true);
  http.setTimeout(5000);
  return true;
}

void fetch_relay_command (void) {
  if (WiFi.status() != WL_CONNECTED) return;
  if (pendingAction.type != NONE) return; // busy

  String url = FIREBASE_URL + String("/commands");
  url += ".json?orderBy=\"$key\"&limitToLast=1";

  if (!ensure_firebase_connection(url)) return;

  int code = http.GET();
  if (code != 200) {
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  if (payload == "{}") return;

  // --- extract key ---
  int k1 = payload.indexOf("\"");
  int k2 = payload.indexOf("\"", k1 + 1);
  String key = payload.substring(k1 + 1, k2);

  // --- extract target ---
  int t1 = payload.indexOf("\"target\"");
  int t2 = payload.indexOf("\"", t1 + 8);
  int t3 = payload.indexOf("\"", t2 + 1);
  String target = payload.substring(t2 + 1, t3);

  // --- extract action ---
  int a1 = payload.indexOf("\"action\"");
  int a2 = payload.indexOf("\"", a1 + 8);
  int a3 = payload.indexOf("\"", a2 + 1);
  String action = payload.substring(a2 + 1, a3);

  process_relay_command(key, target, action);
}

void process_relay_command (const String& key, const String& target, const String& action) {
  uint8_t bit;

  if (target == "gate") bit = _BV(GATE_RELAY);
  else if (target == "pump") bit = _BV(PUMP_RELAY);
  else if (target == "fan") bit = _BV(FAN_RELAY);
  else return;

  if (action == "toggle") {
    actuator_state ^= bit;
  } else if (action == "on") {
    actuator_state &= ~bit;   // active LOW
  } else if (action == "off") {
    actuator_state |= bit;
  } else {
    return;
  }

  Serial.printf("[CMD] %s %s\n", target.c_str(), action.c_str());

  pendingAction.type = CLEAR_COMMAND;
  pendingAction.key = key;
  pendingAction.retries = 0;

  queue_status_update(target + "_" + action);
}

void clear_relay_command () {
  String base = FIREBASE_URL + String("/commands");
  if (base.endsWith(".json"))
    base.remove(base.length() - 5);

  String url = base + "/" + pendingAction.key + ".json";

  if (!ensure_firebase_connection(url)) return;

  http.sendRequest("DELETE", "");
  http.end();
}

void queue_status_update (const String& action) {
  String body = "{";
  body += "\"gate\":" + String(!(actuator_state & _BV(GATE_RELAY)) ? "true":"false") + ",";
  body += "\"pump\":" + String(!(actuator_state & _BV(PUMP_RELAY)) ? "true":"false") + ",";
  body += "\"fan\":"  + String(!(actuator_state & _BV(FAN_RELAY))  ? "true":"false") + ",";
  body += "\"last_action\":\"" + action + "\",";
  body += "\"ts\":" + String(millis() / 1000);
  body += "}";

  pendingAction.body = body;
}

void perform_pending_firebase_actions () {
  if (pendingAction.type == NONE) return;

  if (pendingAction.type == CLEAR_COMMAND) {
    clear_relay_command();
    pendingAction.type = SEND_STATUS;
    return;
  }

  if (pendingAction.type == SEND_STATUS) {
    String url = FIREBASE_URL + String("/status");
    if (!url.endsWith(".json")) url += ".json";

    if (!ensure_firebase_connection(url)) return;

    http.addHeader("Content-Type", "application/json");
    http.PATCH(pendingAction.body);
    http.end();

    pendingAction = {}; // reset
  }
}

void time_init(void) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[TIME] WiFi not ready, NTP delayed");
    return;
  }

  ntpClient.begin();

  if (ntpClient.forceUpdate()) {
    cachedEpoch = ntpClient.getEpochTime();
    cachedMillis = millis();
    timeValid = true;
    lastNtpSync = millis();
    Serial.println("[TIME] NTP sync OK");
  } else {
    Serial.println("[TIME] NTP sync failed");
  }
}

void time_loop(void) {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!timeValid || millis() - lastNtpSync > NTP_SYNC_INTERVAL) {
    if (ntpClient.forceUpdate()) {
      cachedEpoch = ntpClient.getEpochTime();
      cachedMillis = millis();
      timeValid = true;
      lastNtpSync = millis();

      Serial.println("[TIME] NTP re-sync");
    }
  }
}

unsigned long time_now_epoch(void) {
  if (!timeValid) return 0;
  return cachedEpoch + ((millis() - cachedMillis) / 1000);
}

String time_now_hhmm(void) {
  if (!timeValid) return "";

  time_t t = time_now_epoch();
  struct tm tm;
  localtime_r(&t, &tm);

  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", tm.tm_hour, tm.tm_min);
  return String(buf);
}

String time_now_string(void) {
  if (!timeValid) return "invalid";

  time_t t = time_now_epoch();
  struct tm tm;
  localtime_r(&t, &tm);

  char buf[24];
  snprintf(
    buf, sizeof(buf),
    "%04d-%02d-%02d %02d:%02d:%02d",
    tm.tm_year + 1900,
    tm.tm_mon + 1,
    tm.tm_mday,
    tm.tm_hour,
    tm.tm_min,
    tm.tm_sec
  );
  return String(buf);
}

bool time_is_valid(void) {
  return timeValid;
}
