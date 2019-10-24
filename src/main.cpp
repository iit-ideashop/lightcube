#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include <SparkFun_MMA8452Q.h>

#define NPINS 3           // number of output pins
#define PERIOD 20000       // time in milliseconds to fade between colors
#define RIPPLE_PERIOD 5000
#define PINSTATE_PRINT_PERIOD 100

// I2C pins are 4 SDA and 5 SCL
#define ACCEL_DOUBLE_TAP_DELAY 2000

#define CONTROL_PORT 42069

#define STATE_CONN 0
#define STATE_ERR 1
#define STATE_ON 2
#define STATE_OFF 3
#define STATE_SOLID 4
#define STATE_RIPPLE 5

#define PKT_OFF 0
#define PKT_ON 1
#define PKT_SOLID 2
#define PKT_RIPPLE 3

int pins[NPINS] = {14, 12, 15};

int to[NPINS];
int from[NPINS];
int pinState[NPINS];
unsigned long targetTime;
unsigned long tapTime;
unsigned long pinStatePrintTime;
unsigned int tapCount;
unsigned int state = STATE_CONN;
WiFiUDP udp;
const char *announcePkt = "LightCube";
MMA8452Q accel;

struct control_packet {
  char type;
  char r;
  char g;
  char b;
};

void updateTargets() {
  Serial.println("Updating target colors");
  if (state == STATE_RIPPLE) state = STATE_ON;
  for (int i = 0; i < NPINS; i++) {
    Serial.print("[");
    Serial.print(i);
    Serial.print("] ");    
    to[i] = random((int)(PWMRANGE * .75));
    Serial.print(to[i]);
    Serial.print("; ");
  }
  Serial.println();
}

int linearEasing(int t, int duration, int from, int to) {
  return (int)((to - from) * ((double)t / duration) + from);
}

int quadraticEaseUp(int t, int duration, int from, int to) {
  return (int)((to - from) * pow((double)t / duration, 2.0) + from);
}
int quadraticEaseDown(int t, int duration, int from, int to) {
  return (int)((to - from) * (1.0 / pow(((double)t / duration) + 1, 2.0)) + from);
}

void setup() {
  // get a random seed
  Serial.begin(115200);
  Serial.println("LightCube");
  Serial.flush();
  pinMode(LED_BUILTIN, OUTPUT);
  randomSeed(analogRead(A0));

  Wire.begin();
  accel.begin();

  if (WiFi.SSID() != "EventCtl")
    WiFi.begin("EventCtl", "Kaplan2019!");

  yield();

  udp.begin(CONTROL_PORT);

  Serial.println("LightCube Hello");
  Serial.flush();

  udp.beginPacket("255.255.255.255", 42069);
  udp.write(announcePkt);
  udp.endPacket();
  Serial.println("Initial announce sent");
  Serial.flush();

  yield();

  for (int i = 0; i < NPINS; i++) {
    pinMode(pins[i], OUTPUT);
    to[i] = 0;
    digitalWrite(pins[i], 0);
    Serial.print("Enable pin ");
    Serial.println(pins[i]);
    Serial.flush();
  }
  Serial.println("Pins enabled");
  Serial.flush();
  for (int i = 0; i < NPINS; i++) {
    digitalWrite(pins[i], 1);
    delay(500);
    digitalWrite(pins[i], 0);
    delay(500);
  }
  Serial.println("Pin test complete");
  Serial.flush();

  updateTargets();
  targetTime = millis() + PERIOD;
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    // update target colors if needed and send announce packet
    if (millis() > targetTime) {
      if (state == STATE_ON || state == STATE_RIPPLE) {
        updateTargets();
      }
      Serial.println("Sending announce");
      udp.beginPacket("255.255.255.255", 42069);
      udp.write(announcePkt);
      udp.endPacket();
      targetTime = millis() + PERIOD;
    }
    
    // check for available packets
    if (udp.parsePacket() == sizeof(struct control_packet)) {
      struct control_packet pkt;
      udp.read((char *) &pkt, sizeof(struct control_packet));

      if (pkt.type == PKT_OFF) {
        Serial.println("Received PKT_OFF");
        state = STATE_OFF;
      } else if (pkt.type == PKT_ON) {
        state = STATE_ON;
        Serial.println("Received PKT_ON");
      } else if (pkt.type == PKT_SOLID) {
        state = STATE_SOLID;
        Serial.println("Received PKT_SOLID");
        to[0] = pkt.r * (PWMRANGE / 255);
        to[1] = pkt.g * (PWMRANGE / 255);
        to[2] = pkt.b * (PWMRANGE / 255);
      } else if (pkt.type == PKT_RIPPLE && state == STATE_ON) {
        targetTime = millis() + RIPPLE_PERIOD;
        state = STATE_RIPPLE;
        Serial.println("Received PKT_RIPPLE");
        to[0] = pkt.r * (PWMRANGE / 255);
        to[1] = pkt.g * (PWMRANGE / 255);
        to[2] = pkt.b * (PWMRANGE / 255);
      }
    }

    // check for taps
    if ((state == STATE_ON || state == STATE_RIPPLE) && accel.available() && accel.readTap() > 0) {
      if (millis() - tapTime < ACCEL_DOUBLE_TAP_DELAY) {
        tapCount++;
      } else {
        tapCount = 1;
      }
      tapTime = millis();
      Serial.print("Tapped, count is ");
      Serial.println(tapCount);

      if (tapCount == 1) {
        // change color
        updateTargets();
        targetTime = millis() + RIPPLE_PERIOD;
        state = STATE_RIPPLE;
      } else if (tapCount > 1) {
        // send ripple
        udp.beginPacket("255.255.255.255", 42069);
        struct control_packet ripplePkt = {
          PKT_RIPPLE,
          (char)(to[0] / (PWMRANGE/255)),
          (char)(to[1] / (PWMRANGE/255)),
          (char)(to[2] / (PWMRANGE/255))
        };
        udp.write((const char*) &ripplePkt, sizeof(struct control_packet));
        udp.endPacket();
        Serial.println("Sent PKT_RIPPLE");
        targetTime = millis() + RIPPLE_PERIOD;
        state = STATE_RIPPLE;
      }
    }

    // update PWM outputs
    if (state == STATE_ON) {
      // breathe random colors
      for (int i = 0; i < NPINS; i++) {
        if (targetTime - millis() > PERIOD / 2) {
          if (pinStatePrintTime - millis() > PINSTATE_PRINT_PERIOD)
            Serial.print("EASE UP ");
          pinState[i] = to[i] - linearEasing(targetTime - millis() - (PERIOD / 2), PERIOD / 2, 0, to[i]);
          //digitalWrite(pins[i], HIGH);
        } else {
          if (pinStatePrintTime - millis() > PINSTATE_PRINT_PERIOD)
            Serial.print("EASE DOWN ");
          pinState[i] = to[i] + linearEasing(targetTime - millis() - (PERIOD / 2), PERIOD / 2, 0, to[i]);
          //digitalWrite(pins[i], LOW);
        }
      }
    } else if (state == STATE_SOLID) {
      for (int i = 0; i < NPINS; i++) {
        pinState[i] = to[i];
      }
    } else if (state == STATE_RIPPLE) {
      for (int i = 0; i < NPINS; i++) {
        if (targetTime - millis() > RIPPLE_PERIOD / 2) {
          if (pinStatePrintTime - millis() > PINSTATE_PRINT_PERIOD)
            Serial.print("EASE SOLID ");
          pinState[i] = to[i];
          //digitalWrite(pins[i], HIGH);
        } else {
          if (pinStatePrintTime - millis() > PINSTATE_PRINT_PERIOD)
            Serial.print("EASE DOWN ");
          pinState[i] = to[i] + linearEasing(targetTime - millis() - (RIPPLE_PERIOD / 2), RIPPLE_PERIOD / 2, 0, to[i]);
          //digitalWrite(pins[i], LOW);
        }
      }
    } else {
      state = STATE_OFF;
      for (int i = 0; i < NPINS; i++) {
        digitalWrite(pins[i], 0);
      }
    }
    for(int i = 0; i < NPINS; i++) {
      if (pinStatePrintTime - millis() > PINSTATE_PRINT_PERIOD) {
        Serial.print("[");
        Serial.print(i);
        Serial.print(":");
        Serial.print(pins[i]);
        Serial.print("] ");
        Serial.print(pinState[i]);
        Serial.print("/");
        Serial.print(to[i]);
        Serial.print("; ");
      }
      analogWrite(pins[i], pinState[i]);
    }
    if (pinStatePrintTime - millis() > PINSTATE_PRINT_PERIOD) {
      const char *state_msg;
      switch (state) {
        case STATE_CONN:
          state_msg = "STATE_CONN";
          break;
        case STATE_ON:
          state_msg = "STATE_ON";
          break;
        case STATE_OFF:
          state_msg = "STATE_OFF";
          break;
        case STATE_RIPPLE:
          state_msg = "STATE_RIPPLE";
          break;
        case STATE_SOLID:
          state_msg = "STATE_SOLID";
          break;
        default:
          state_msg = "STATE_ERROR";
          break;
      }
      Serial.println(state_msg);
      pinStatePrintTime = millis() + PINSTATE_PRINT_PERIOD;
    }
  } else { // wifi is not connected
    state = STATE_ERR;
    digitalWrite(pins[2], 1);
    delay(500);
    digitalWrite(pins[2], 0);
    delay(500);
  }
  delay(10);
}