// sumo-wrestler: HC-SR04 ultrasonic sensor test.
//
// Python calls distance() over the Bridge and gets centimeters back,
// or -1 if no echo came back (nothing in range or bad wiring).

#include "Arduino_RouterBridge.h"

// Wiring: HC-SR04 -> UNO Q (sensor powered from 5V)
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

const unsigned long ECHO_TIMEOUT_US = 30000;  // ~5 m round trip

// Exposed to Python
float distance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
    if (us == 0) return -1;
    return us / 58.0;  // speed of sound, there and back
}

void setup() {
    Monitor.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    Bridge.begin();
    // provide_safe runs distance() in the loop thread, so pulseIn timing isn't interrupted by RPC handling
    Bridge.provide_safe("distance", distance);
    Monitor.println("sumo-wrestler: ultrasonic ready");
}

void loop() {
}
