// ══════════════════════════════════════════════════════════════════
//  Robot 2 Controller — Integrated IMU Edition
//  Arduino Mega 2560 · L298N · 4× Encoders · MPU6050
//
//  Streams sensor data at 50 Hz for Raspberry Pi EKF fusion.
//  Zero external library dependencies (Wire.h only).
//
//  Serial Protocol:
//    IN  → F / B / L / R / S / P<0-255>
//    OUT ← D:ms,e1,e2,e3,e4,ax,ay,az,gx,gy,gz  (50 Hz)
//         READY                                   (on boot)
//         IMU:OK / IMU:FAIL                       (on boot)
// ══════════════════════════════════════════════════════════════════

#include <Wire.h>

// ─────────────────────────────────────
// PIN MAP — Arduino Mega 2560
// ─────────────────────────────────────
//
//  ┌─────────────────────────────────────────────────┐
//  │ L298N Motor Driver                              │
//  │   ENA  → Pin 10  (Left PWM)                     │
//  │   IN1  → Pin 8   (Left direction A)             │
//  │   IN2  → Pin 9   (Left direction B)             │
//  │   ENB  → Pin 11  (Right PWM)                    │
//  │   IN3  → Pin 12  (Right direction A)            │
//  │   IN4  → Pin 13  (Right direction B)            │
//  ├─────────────────────────────────────────────────┤
//  │ Encoders (interrupt pins → Channel A)           │
//  │   M1 Front-Left:  ENCA=2   ENCB=22              │
//  │   M2 Rear-Left:   ENCA=3   ENCB=24              │
//  │   M3 Front-Right: ENCA=18  ENCB=26              │
//  │   M4 Rear-Right:  ENCA=19  ENCB=28              │
//  ├─────────────────────────────────────────────────┤
//  │ MPU6050 IMU (I2C — built-in on Mega)            │
//  │   SDA → Pin 20                                  │
//  │   SCL → Pin 21                                  │
//  │   VCC → 5V (module has onboard 3.3V regulator)  │
//  │   GND → GND                                     │
//  │   AD0 → GND (address 0x68)                      │
//  └─────────────────────────────────────────────────┘

// ═══════════════════════════════════════
// 1. MOTOR DRIVER PINS
// ═══════════════════════════════════════
#define ENA 10
#define IN1 8
#define IN2 9
#define ENB 11
#define IN3 12
#define IN4 13

// ═══════════════════════════════════════
// 2. ENCODER PINS
// ═══════════════════════════════════════
#define M1_ENCA 2      // Front Left
#define M1_ENCB 22
#define M2_ENCA 3      // Rear Left
#define M2_ENCB 24
#define M3_ENCA 18     // Front Right
#define M3_ENCB 26
#define M4_ENCA 19     // Rear Right
#define M4_ENCB 28

// ═══════════════════════════════════════
// 3. IMU CONFIG
// ═══════════════════════════════════════
#define MPU_ADDR      0x68
#define GYRO_SCALE    1     // FS_SEL=1 → ±500°/s  (65.5 LSB/°/s)
#define ACCEL_SCALE   1     // AFS_SEL=1 → ±4g     (8192 LSB/g)
#define DLPF_MODE     3     // 42 Hz bandwidth — good noise rejection

// ═══════════════════════════════════════
// 4. TIMING
// ═══════════════════════════════════════
#define STREAM_HZ     50
#define STREAM_MS     (1000 / STREAM_HZ)

// ═══════════════════════════════════════
// 5. STATE
// ═══════════════════════════════════════
volatile long enc[4] = {0, 0, 0, 0};
int16_t imu[7];           // ax,ay,az,temp,gx,gy,gz (raw)
int pwmSpeed = 150;
bool imuReady = false;
unsigned long lastStream = 0;

// ═══════════════════════════════════════
// 6. ENCODER ISRs
// ═══════════════════════════════════════
void isrM1() { enc[0] += digitalRead(M1_ENCB) ? 1 : -1; }
void isrM2() { enc[1] += digitalRead(M2_ENCB) ? 1 : -1; }
void isrM3() { enc[2] += digitalRead(M3_ENCB) ? -1 : 1; }   // Negated: right-side wires swapped
void isrM4() { enc[3] += digitalRead(M4_ENCB) ? -1 : 1; }   // Negated: right-side wires swapped

// ═══════════════════════════════════════
// 7. IMU — Raw Register Access
// ═══════════════════════════════════════
void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool mpuInit() {
    Wire.begin();
    Wire.setClock(400000);                  // Fast I2C

    // Check WHO_AM_I (register 0x75 should return 0x68)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
    if (!Wire.available()) return false;
    uint8_t whoAmI = Wire.read();
    if (whoAmI != 0x68 && whoAmI != 0x71 && whoAmI != 0x73) return false;

    mpuWrite(0x6B, 0x00);                  // Wake up (PWR_MGMT_1)
    delay(10);
    mpuWrite(0x6B, 0x01);                  // Clock source = PLL with X-axis gyro
    mpuWrite(0x1B, GYRO_SCALE << 3);       // Gyro range
    mpuWrite(0x1C, ACCEL_SCALE << 3);      // Accel range
    mpuWrite(0x1A, DLPF_MODE);             // Digital Low Pass Filter

    return true;
}

void mpuRead() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);                      // Start at ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);

    for (int i = 0; i < 7; i++) {
        imu[i] = ((int16_t)Wire.read() << 8) | Wire.read();
    }
}

// ═══════════════════════════════════════
// 8. MOTOR CONTROL
// ═══════════════════════════════════════
void drive(int left, int right) {
    // Left channel
    if (left > 0) {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    } else if (left < 0) {
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    } else {
        digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
    }
    analogWrite(ENA, abs(left));

    // Right channel
    if (right > 0) {
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else if (right < 0) {
        digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    } else {
        digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
    }
    analogWrite(ENB, abs(right));
}

// ═══════════════════════════════════════
// 9. DATA STREAMING
// ═══════════════════════════════════════
//
// Format:  D:timestamp,enc1,enc2,enc3,enc4,ax,ay,az,gx,gy,gz
//
// - timestamp: millis() for ΔT calculation on Raspberry Pi
// - enc1-4:    cumulative encoder ticks (Front-L, Rear-L, Front-R, Rear-R)
// - ax,ay,az:  raw accelerometer (convert on Pi: raw * 9.81 / 8192.0)
// - gx,gy,gz:  raw gyroscope     (convert on Pi: raw * π / (180.0 × 65.5))

void streamSensors() {
    // Snapshot encoder values atomically
    noInterrupts();
    long e0 = enc[0], e1 = enc[1], e2 = enc[2], e3 = enc[3];
    interrupts();

    // Single serial print for efficiency
    char buf[128];
    snprintf(buf, sizeof(buf), "D:%lu,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d",
        millis(), e0, e1, e2, e3,
        imu[0], imu[1], imu[2],   // ax, ay, az
        imu[4], imu[5], imu[6]);  // gx, gy, gz (skip temp at index 3)
    Serial.println(buf);
}

// ═══════════════════════════════════════
// 10. COMMAND HANDLER
// ═══════════════════════════════════════
void handleCommand() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    char cmd = toupper(input[0]);

    switch (cmd) {
        case 'F': drive( pwmSpeed,  pwmSpeed); break;
        case 'B': drive(-pwmSpeed, -pwmSpeed); break;
        case 'L': drive(-pwmSpeed,  pwmSpeed); break;
        case 'R': drive( pwmSpeed, -pwmSpeed); break;
        case 'S': drive(0, 0);                 break;
        case 'P':
            pwmSpeed = constrain(input.substring(1).toInt(), 0, 255);
            break;
    }
}

// ═══════════════════════════════════════
// 11. SETUP
// ═══════════════════════════════════════
void setup() {
    Serial.begin(115200);

    // Motors
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    drive(0, 0);

    // Encoders
    uint8_t encA[] = {M1_ENCA, M2_ENCA, M3_ENCA, M4_ENCA};
    uint8_t encB[] = {M1_ENCB, M2_ENCB, M3_ENCB, M4_ENCB};
    for (int i = 0; i < 4; i++) {
        pinMode(encA[i], INPUT_PULLUP);
        pinMode(encB[i], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(M1_ENCA), isrM1, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENCA), isrM2, RISING);
    attachInterrupt(digitalPinToInterrupt(M3_ENCA), isrM3, RISING);
    attachInterrupt(digitalPinToInterrupt(M4_ENCA), isrM4, RISING);

    // IMU
    imuReady = mpuInit();
    Serial.println(imuReady ? "IMU:OK" : "IMU:FAIL");
    Serial.println("READY");
}

// ═══════════════════════════════════════
// 12. MAIN LOOP
// ═══════════════════════════════════════
void loop() {
    handleCommand();

    if (imuReady) mpuRead();

    unsigned long now = millis();
    if (now - lastStream >= STREAM_MS) {
        lastStream = now;
        streamSensors();
    }
}
