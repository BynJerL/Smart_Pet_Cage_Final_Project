#include <Arduino.h>
#include <PCF8574.h>

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

PCF8574 pcf1(PCF8574_ADDRESS_1);
PCF8574 pcf2(PCF8574_ADDRESS_2);

byte button_state = 0b01111111; // Only use 7 bit
byte last_button_state = 0b01111111; // Only use 7 bit

byte actuator_state = 0b00000111; // Only use 3 bit for now
byte last_actuator_state = 0b00000111; // Only use 3 bit for now

void read_button_state (void);
void check_button_state_change (void);
void check_relay_state_change (void);

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing peripheral");

  pcf1.pinMode(L_BUTTON, INPUT);
  pcf1.pinMode(C_BUTTON, INPUT);
  pcf1.pinMode(R_BUTTON, INPUT);
  pcf1.pinMode(GATE_BUTTON, INPUT);
  pcf1.pinMode(PUMP_BUTTON, INPUT);
  pcf1.pinMode(FAN_BUTTON, INPUT);
  pcf1.pinMode(FEEDER_BUTTON, INPUT);

  pcf2.pinMode(GATE_RELAY, OUTPUT);
  pcf2.pinMode(PUMP_RELAY, OUTPUT);
  pcf2.pinMode(FAN_RELAY, OUTPUT);
  
  if (!pcf1.begin()) {
    Serial.println(F("ERROR: Could not initialize PCF8574(A)! Check wiring, I2C address, SDA/SCL connections and power."));
    while (1) delay(100);
  }
  if (!pcf2.begin()) {
    Serial.println(F("ERROR: Could not initialize PCF8574(B)! Check wiring, I2C address, SDA/SCL connections and power."));
    while (1) delay(100);
  }

  pcf2.digitalWrite(GATE_RELAY, HIGH);
  pcf2.digitalWrite(PUMP_RELAY, HIGH);
  pcf2.digitalWrite(FAN_RELAY, HIGH);

  Serial.println(F("PCF8574s initialized successfully."));
  delay(500);
  Serial.println(F("Try to tap the buttons."));
}

void loop() {
  read_button_state();
  check_button_state_change();
  check_relay_state_change();
  delay(50);
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
          pcf2.digitalWrite(GATE_RELAY, LOW);
        } else {
          actuator_state |= _BV(GATE_RELAY);
          pcf2.digitalWrite(GATE_RELAY, HIGH);
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
          pcf2.digitalWrite(PUMP_RELAY, LOW);
        } else {
          actuator_state |= _BV(PUMP_RELAY);
          pcf2.digitalWrite(PUMP_RELAY, HIGH);
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
          pcf2.digitalWrite(FAN_RELAY, LOW);
        } else {
          actuator_state |= _BV(FAN_RELAY);
          pcf2.digitalWrite(FAN_RELAY, HIGH);
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