// sumo-wrestler: drive motors, HC-SR04 ultrasonic and MPU-6050 accelerometer.
//
// Python calls over the Bridge:
//   drive(left, right)  speeds -255..255; motors stop if no call for WATCHDOG_MS
//   distance()          cm, or -1 if no echo came back
//   imu_check()         "found, whoami=0x.." or "not found at 0x68"
//   imu_read()          "ax,ay,az,gx,gy,gz" in g and deg/s, "" if not found
//
// Wiring:
//   Cytron Maker Drive  M1A D3, M1B D5, M2A D6, M2B D9 (all PWM)
//   HC-SR04 (5V)        Trig D7, Echo D8
//   MPU-6050            header SDA/SCL, address 0x68
//
// Maker Drive logic per motor: A=PWM,B=0 forward; A=0,B=PWM reverse; both 0 coast.

#include <Wire.h>
#include "Arduino_RouterBridge.h"

// ---------- motors ----------
const int M1A = 3;
const int M1B = 5;
const int M2A = 6;
const int M2B = 9;

// Flip if a motor spins the wrong way for a positive speed
const bool INVERT_M1 = false;
const bool INVERT_M2 = false;

const unsigned long WATCHDOG_MS = 200;

unsigned long last_cmd_ms = 0;
bool stopped = true;

void set_motor(int pin_a, int pin_b, int speed, bool invert) {
    speed = constrain(speed, -255, 255);
    if (invert) speed = -speed;
    // analogWrite only (no digitalWrite) so the pins stay in PWM mode
    analogWrite(pin_a, speed > 0 ? speed : 0);
    analogWrite(pin_b, speed < 0 ? -speed : 0);
}

void stop_motors() {
    set_motor(M1A, M1B, 0, false);
    set_motor(M2A, M2B, 0, false);
    stopped = true;
}

// Exposed to Python: left -> motor 1, right -> motor 2
void drive(int left, int right) {
    set_motor(M1A, M1B, left, INVERT_M1);
    set_motor(M2A, M2B, right, INVERT_M2);
    last_cmd_ms = millis();
    stopped = (left == 0 && right == 0);
}

// ---------- ultrasonic ----------
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

// ~2.5 m round trip; kept short because pulseIn blocks the loop thread
const unsigned long ECHO_TIMEOUT_US = 15000;

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

// ---------- accelerometer ----------
const uint8_t IMU_ADDR = 0x68;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_WHO_AM_I = 0x75;

const uint8_t RANGE_MAX = 0x18;     // ±16 g / ±2000 deg/s in the config registers
const float ACCEL_LSB = 2048.0;     // counts per g at ±16 g
const float GYRO_LSB = 16.4;        // counts per deg/s at ±2000 deg/s

bool imu_found = false;

void write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Point at reg, then request n bytes
void request(uint8_t reg, uint8_t n) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_ADDR, n);
}

String imu_check() {
    if (!imu_found) return "not found at 0x68";
    request(REG_WHO_AM_I, 1);
    return "found, whoami=0x" + String(Wire.read(), HEX);
}

String imu_read() {
    if (!imu_found) return "";
    request(REG_ACCEL_XOUT_H, 14);
    int16_t v[7];  // ax, ay, az, temp, gx, gy, gz
    for (int i = 0; i < 7; i++) {
        uint8_t hi = Wire.read();
        uint8_t lo = Wire.read();
        v[i] = (int16_t)((hi << 8) | lo);
    }
    String s;
    for (int i = 0; i < 3; i++) s += String(v[i] / ACCEL_LSB, 3) + ",";
    for (int i = 4; i < 7; i++) s += String(v[i] / GYRO_LSB, 1) + (i < 6 ? "," : "");
    return s;
}

// ---------- main ----------
void setup() {
    Monitor.begin(115200);
    stop_motors();

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    Wire.begin();
    Wire.beginTransmission(IMU_ADDR);
    imu_found = Wire.endTransmission() == 0;
    if (imu_found) {
        write_reg(REG_PWR_MGMT_1, 0);  // wake from sleep
        write_reg(REG_GYRO_CONFIG, RANGE_MAX);
        write_reg(REG_ACCEL_CONFIG, RANGE_MAX);
    }

    Bridge.begin();
    // provide_safe runs these in the loop thread: drive() never races the
    // watchdog, pulseIn timing isn't interrupted, and I2C calls never overlap
    Bridge.provide_safe("drive", drive);
    Bridge.provide_safe("distance", distance);
    Bridge.provide_safe("imu_check", imu_check);
    Bridge.provide_safe("imu_read", imu_read);
    Monitor.println(imu_found ? "sumo-wrestler: ready" : "sumo-wrestler: ready, IMU not found");
}

void loop() {
    if (!stopped && millis() - last_cmd_ms > WATCHDOG_MS) {
        stop_motors();
        Monitor.println("watchdog: no drive command, motors stopped");
    }
}
