#include <Arduino.h>
#include <PCF8574.h>

#define L_BUTTON      P0
#define C_BUTTON      P1
#define R_BUTTON      P2
#define GATE_BUTTON   P3
#define PUMP_BUTTON   P4
#define FAN_BUTTON    P5
#define FEEDER_BUTTON P6

#define PCF8574_ADDRESS_1 0x20 

PCF8574 pcf1(PCF8574_ADDRESS_1);

byte button_state = 0b01111111; // Only use 7 bit
byte last_button_state = 0b01111111; // Only use 7 bit

void read_button_state (void);
void check_button_state_change (void);

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

  if (!pcf1.begin()) {
    Serial.println(F("ERROR: Could not initialize PCF8574! Check wiring, I2C address, SDA/SCL connections and power."));
    while (1) delay(100);
  }

  Serial.println(F("PCF8574 initialized successfully."));
  delay(500);
  Serial.println(F("Try to tap the buttons."));
}

void loop() {
  read_button_state();
  check_button_state_change();
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
      } else {
        Serial.println(F("Gate Button Released."));
      }
    }

    if ((button_state & _BV(PUMP_BUTTON)) != (last_button_state & _BV(PUMP_BUTTON))) {
      if (!(button_state & _BV(PUMP_BUTTON))) {
        Serial.println(F("Pump Button Pressed."));
      } else {
        Serial.println(F("Pump Button Released."));
      }
    }

    if ((button_state & _BV(FAN_BUTTON)) != (last_button_state & _BV(FAN_BUTTON))) {
      if (!(button_state & _BV(FAN_BUTTON))) {
        Serial.println(F("Fan Button Pressed."));
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

    Serial.println(button_state);
    last_button_state = button_state;
  }
}