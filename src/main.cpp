#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include <SparkFun_MMA8452Q.h>

#define NPINS 3           // number of output pins
#define PERIOD 20000       // time in milliseconds to fade between colors
#define RIPPLE_PERIOD 5000

// I2C pins are 4 SDA and 5 SCL
#define ACCEL_DOUBLE_TAP_DELAY 2000
#define ACCEL_TAP_THRESHOLD 1.0

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

int pins[NPINS] = {14, 6, 1};

int to[NPINS];
int from[NPINS];
unsigned long targetTime;
unsigned long tapTime;
unsigned int tapCount;
unsigned int state = STATE_CONN;
WiFiUDP udp;
char *announcePkt = "LightCube";
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
  return (int)(((to - from) * ((double)t / duration)) + from);
}

int quadraticEasing(int t, int duration, int from, int to) {
  return (int)((to - from) * pow((double)t / duration, 2.0) + from);
}

void setup() {
  // get a random seed
  Serial.begin(115200);
  Serial.println("LightCube");
  Serial.flush();
  pinMode(16, INPUT);
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

  for (int i = 0; i < NPINS; i++) {
    pinMode(pins[i], OUTPUT);
    to[i] = 0;
    digitalWrite(pins[i], 0);
  }
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
    if (millis() > targetTime) {
      if (state == STATE_ON) {
        updateTargets();
      }
      Serial.println("Sending announce");
      udp.beginPacket("255.255.255.255", 42069);
      udp.write(announcePkt);
      udp.endPacket();
      targetTime = millis() + PERIOD;
    }
    
    if (udp.parsePacket() == sizeof(struct control_packet)) {
      struct control_packet pkt;
      udp.read((char *) &pkt, sizeof(struct control_packet));

      if (pkt.type == PKT_OFF) {
        state = STATE_OFF;
      } else if (pkt.type == PKT_ON) {
        state = STATE_ON;
      } else if (pkt.type == PKT_SOLID) {
        state = STATE_SOLID;
        to[0] = pkt.r * (PWMRANGE / 255);
        to[1] = pkt.g * (PWMRANGE / 255);
        to[2] = pkt.b * (PWMRANGE / 255);
      } else if (pkt.type == PKT_RIPPLE && state == STATE_ON) {
        targetTime = millis() + RIPPLE_PERIOD;
        state = STATE_RIPPLE;
        to[0] = pkt.r * (PWMRANGE / 255);
        to[1] = pkt.g * (PWMRANGE / 255);
        to[2] = pkt.b * (PWMRANGE / 255);
      }
    }

    if (accel.available() && accel.readTap() > 0) {
      if (millis() - tapTime < ACCEL_DOUBLE_TAP_DELAY) {
        tapCount++;
      } else {
        tapCount = 1;
        tapTime = millis();
      }

      if (tapCount == 1) {
        // change color
        updateTargets();
        targetTime = millis() + PERIOD;
      } else if (tapCount > 1) {
        // send ripple
        udp.beginPacket("255.255.255.255", 42069);
        struct control_packet ripplePkt = {PKT_RIPPLE, to[0], to[1], to[2]};
        udp.write((const char*) &ripplePkt, sizeof(struct control_packet));
        udp.endPacket();
        // broadcast packets are received by their sender so no need to do anything else,
        // mode change happens on the next iteration
      }
    }

    if (state == STATE_ON) {
      // breathe random colors
      for (int i = 0; i < NPINS; i++) {
        if (targetTime - millis() < PERIOD / 2)
          analogWrite(pins[i], quadraticEasing(targetTime - millis(), PERIOD / 2, 0, to[i]));
          //digitalWrite(pins[i], HIGH);
        else
          analogWrite(pins[i], quadraticEasing(targetTime - millis() - PERIOD / 2, PERIOD / 2, to[i], 0));
          //digitalWrite(pins[i], LOW);
      }
    } else if (state == STATE_SOLID) {
      for (int i = 0; i < NPINS; i++) {
        analogWrite(pins[i], to[i]);
      }
    } else if (state == STATE_RIPPLE) {
      for (int i = 0; i < NPINS; i++) {
        if (targetTime - millis() < RIPPLE_PERIOD / 2)
          analogWrite(pins[i], to[i]);
          //digitalWrite(pins[i], HIGH);
        else
          analogWrite(pins[i], quadraticEasing(targetTime - millis() - RIPPLE_PERIOD / 2, RIPPLE_PERIOD / 2, to[i], 0));
          //digitalWrite(pins[i], LOW);
      }
    } else {
      state = STATE_OFF;
      for (int i = 0; i < NPINS; i++) {
        digitalWrite(pins[i], 0);
      }
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