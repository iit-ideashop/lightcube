#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define NPINS 3           // number of output pins
#define PERIOD 20000       // time in milliseconds to fade between colors

#define CONTROL_PORT 42069

#define STATE_CONN 0
#define STATE_ERR 1
#define STATE_ON 2
#define STATE_OFF 3
#define STATE_SOLID 4

#define PKT_OFF 0
#define PKT_ON 1
#define PKT_SOLID 2

int pins[NPINS] = {14, 6, 1};

int to[NPINS];
int from[NPINS];
unsigned long targetTime;
unsigned int state = STATE_CONN;
WiFiUDP udp;
char *announcePkt = "LightCube";

struct control_packet {
  char type;
  char r;
  char g;
  char b;
};

void updateTargets() {
  for (int i = 0; i < NPINS; i++) {
    to[i] = random((int)(PWMRANGE * .75));
  }
}

int linearEasing(int t, int duration, int from, int to) {
  return (int)(((to - from) * ((double)t / duration)) + from);
}

void setup() {
  // get a random seed
  pinMode(16, INPUT);
  randomSeed(analogRead(16));

  WiFi.begin("EventCtl", "Kaplan2019!");

  udp.begin(CONTROL_PORT);

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

  updateTargets();
  targetTime = millis() + PERIOD;
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (millis() > targetTime) {
      if (state == STATE_ON)
        updateTargets();
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
      }
    }

    if (state == STATE_ON) {
      // breathe random colors
      for (int i = 0; i < NPINS; i++) {
        if (targetTime - millis() < PERIOD / 2)
          analogWrite(pins[i], linearEasing(targetTime - millis(), PERIOD / 2, 0, to[i]));
          //digitalWrite(pins[i], HIGH);
        else
          analogWrite(pins[i], linearEasing(targetTime - millis() - PERIOD / 2, PERIOD / 2, to[i], 0));
          //digitalWrite(pins[i], LOW);
      }
    } else if (state == STATE_SOLID) {
      for (int i = 0; i < NPINS; i++) {
        analogWrite(pins[i], to[i]);
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