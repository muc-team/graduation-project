// ═══════════════════════════════════════════════════════════════════════
//  ROBOT 2 CONTROLLER  —  10-DOF Edition
//  ─────────────────────────────────────────────────────────────────────
//   Platform   : Arduino Mega 2560
//   Drive      : L298N Dual H-Bridge
//   Odometry   : 4 × Quadrature Encoders (interrupt-driven)
//   IMU        : GY-87 10-DOF Module
//                  ├─ MPU6050   →  3-axis Gyro + 3-axis Accelerometer
//                  ├─ HMC5883L  →  3-axis Magnetometer  (via I2C bypass)
//                  └─ BMP180    →  Barometric Pressure + Temperature
//
//   Purpose    : Stream fused sensor data to Raspberry Pi for EKF /
//                dead-reckoning / SLAM navigation.
//
//   Dependencies: Wire.h only  (no external libraries)
//   Version     : 2.0
// ═══════════════════════════════════════════════════════════════════════

#include <Wire.h>

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                      1.  PIN ASSIGNMENTS                          ║
// ╚═══════════════════════════════════════════════════════════════════╝

// --- L298N Motor Driver ---------------------------------------------
#define ENA          10     // Left  side PWM
#define IN1          8      // Left  direction A
#define IN2          9      // Left  direction B
#define ENB          11     // Right side PWM
#define IN3          12     // Right direction A
#define IN4          13     // Right direction B

// --- Quadrature Encoders (Channel A on hardware interrupt pins) -----
#define M1_ENCA      2      // Front-Left
#define M1_ENCB      22
#define M2_ENCA      3      // Rear-Left
#define M2_ENCB      24
#define M3_ENCA      18     // Front-Right
#define M3_ENCB      26
#define M4_ENCA      19     // Rear-Right
#define M4_ENCB      28

// --- I2C Bus (built-in on Mega: SDA=20, SCL=21) ---------------------
#define I2C_CLOCK    400000UL

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                  2.  I2C DEVICE ADDRESSES                         ║
// ╚═══════════════════════════════════════════════════════════════════╝
#define ADDR_MPU6050    0x68
#define ADDR_HMC5883L   0x1E    // Honeywell original
#define ADDR_QMC5883L   0x0D    // QST clone (common in modern GY-87)
#define ADDR_BMP180     0x77

// Magnetometer chip type (auto-detected)
#define MAG_NONE      0
#define MAG_HMC5883L  1
#define MAG_QMC5883L  2

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                  3.  SENSOR CONFIGURATION                         ║
// ╚═══════════════════════════════════════════════════════════════════╝

// --- MPU6050 --------------------------------------------------------
#define MPU_GYRO_FS     1       // 0:±250  1:±500  2:±1000  3:±2000 °/s
#define MPU_ACCEL_FS    1       // 0:±2g   1:±4g   2:±8g    3:±16g
#define MPU_DLPF_MODE   3       // 3 → 42 Hz bandwidth (good noise rej.)

// Conversion factors (for documentation — Pi performs conversion)
//   gyro:  raw * (500.0/32768.0) = °/s    →  ≈ 65.5 LSB / (°/s)
//   accel: raw * (4.0 /32768.0) = g       →  ≈ 8192 LSB / g

// --- HMC5883L (Honeywell original) ----------------------------------
#define HMC_CONFIG_A    0x78    // 8-sample average, 75 Hz output rate
#define HMC_CONFIG_B    0x20    // Gain ±1.3 Gauss   (1090 LSB/Gauss)
#define HMC_MODE_REG    0x00    // Continuous measurement mode

// --- QMC5883L (QST clone) -------------------------------------------
//  Control 1 (0x09):
//    OSR=512(00) | RNG=8G(01) | ODR=200Hz(11) | MODE=Continuous(01)
//    → 0b00011101 = 0x1D
//  Control 2 (0x0A): soft-reset bit cleared, interrupt enabled = 0x01
//  SET/RESET period (0x0B): recommended value = 0x01
#define QMC_CONFIG_1    0x1D
#define QMC_CONFIG_2    0x01
#define QMC_PERIOD      0x01

// --- BMP180 ---------------------------------------------------------
#define BMP_OSS         1       // Oversampling: 0(fast) … 3(precise)

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                  4.  TIMING                                       ║
// ╚═══════════════════════════════════════════════════════════════════╝
#define STREAM_HZ            50
#define STREAM_PERIOD_MS     (1000UL / STREAM_HZ)
#define BARO_PERIOD_MS       100UL    // 10 Hz (BMP180 is slow)
#define HEARTBEAT_MS         1000UL   // verbose status print
#define WATCHDOG_MS          1000UL   // auto-stop if no command
#define GYRO_CALIB_SAMPLES   1000     // gyro bias calibration

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                  5.  GLOBAL STATE                                 ║
// ╚═══════════════════════════════════════════════════════════════════╝

// Encoder counts (modified inside ISRs)
volatile long enc[4] = {0, 0, 0, 0};

// MPU6050 raw readings
int16_t  ax = 0, ay = 0, az = 0;
int16_t  gx = 0, gy = 0, gz = 0;
int16_t  mpuTemp = 0;

// Gyro bias offsets (computed during calibration)
int16_t  gxOff = 0, gyOff = 0, gzOff = 0;

// HMC5883L raw readings
int16_t  mx = 0, my = 0, mz = 0;

// BMP180 calibration & state
int16_t  ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
int32_t  baroPressurePa = 0;     // in Pascals
int16_t  baroTempDeci   = 0;     // in 0.1 °C
int32_t  bmp_B5 = 0;             // shared term between temp & pressure
uint8_t  bmp_state = 0;          // state machine
unsigned long bmp_t0 = 0;        // wait timer
int32_t  bmp_UT = 0;             // raw temperature
int32_t  bmp_UP = 0;             // raw pressure

// System status flags
bool mpuOK    = false;
bool magOK    = false;
bool baroOK   = false;
bool baroReady = false;     // becomes true after first valid baro reading
uint8_t magType = MAG_NONE; // MAG_HMC5883L or MAG_QMC5883L
bool verbose  = false;

// Motor control state
int  pwmSpeed       = 150;
int  lastLeftCmd    = 0;
int  lastRightCmd   = 0;
unsigned long lastCmdTime = 0;
bool watchdogActive = true;

// Stream timing
unsigned long lastStream    = 0;
unsigned long lastBaro      = 0;
unsigned long lastHeartbeat = 0;
uint32_t      streamCount   = 0;

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                  6.  ENCODER INTERRUPT HANDLERS                   ║
// ╚═══════════════════════════════════════════════════════════════════╝
//  Quadrature decoding using only one edge (rising on A).
//  Direction is read from B channel state at the moment A goes high.
//  Right-side motors are wired with reversed polarity (M3, M4 negated).

void isrM1() { enc[0] += digitalRead(M1_ENCB) ?  1 : -1; }
void isrM2() { enc[1] += digitalRead(M2_ENCB) ?  1 : -1; }
void isrM3() { enc[2] += digitalRead(M3_ENCB) ? -1 :  1; }
void isrM4() { enc[3] += digitalRead(M4_ENCB) ? -1 :  1; }

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                  7.  I2C UTILITY FUNCTIONS                        ║
// ╚═══════════════════════════════════════════════════════════════════╝

void i2cWrite(uint8_t devAddr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t i2cReadByte(uint8_t devAddr, uint8_t reg) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(devAddr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

bool i2cReadBytes(uint8_t devAddr, uint8_t reg, uint8_t *buf, uint8_t n) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(devAddr, n);
    for (uint8_t i = 0; i < n; i++) {
        if (!Wire.available()) return false;
        buf[i] = Wire.read();
    }
    return true;
}

bool i2cDeviceExists(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                  8.  MPU6050 DRIVER                               ║
// ╚═══════════════════════════════════════════════════════════════════╝

bool mpuInit() {
    if (!i2cDeviceExists(ADDR_MPU6050)) return false;

    // Verify identity
    uint8_t who = i2cReadByte(ADDR_MPU6050, 0x75);
    if (who != 0x68 && who != 0x71 && who != 0x73) return false;

    i2cWrite(ADDR_MPU6050, 0x6B, 0x80);          // Reset device
    delay(100);
    i2cWrite(ADDR_MPU6050, 0x6B, 0x01);          // Wake + PLL with X-gyro
    i2cWrite(ADDR_MPU6050, 0x6C, 0x00);          // Enable all axes
    i2cWrite(ADDR_MPU6050, 0x1A, MPU_DLPF_MODE); // DLPF config
    i2cWrite(ADDR_MPU6050, 0x19, 0x00);          // Sample rate = 1 kHz
    i2cWrite(ADDR_MPU6050, 0x1B, MPU_GYRO_FS << 3);
    i2cWrite(ADDR_MPU6050, 0x1C, MPU_ACCEL_FS << 3);

    // Enable I2C bypass so HMC5883L is reachable on the main bus
    i2cWrite(ADDR_MPU6050, 0x6A, 0x00);          // USER_CTRL: disable master
    i2cWrite(ADDR_MPU6050, 0x37, 0x02);          // INT_PIN_CFG: bypass enable
    delay(10);

    return true;
}

void mpuRead() {
    uint8_t b[14];
    if (!i2cReadBytes(ADDR_MPU6050, 0x3B, b, 14)) return;

    ax      = ((int16_t)b[0]  << 8) | b[1];
    ay      = ((int16_t)b[2]  << 8) | b[3];
    az      = ((int16_t)b[4]  << 8) | b[5];
    mpuTemp = ((int16_t)b[6]  << 8) | b[7];
    gx      = (((int16_t)b[8]  << 8) | b[9])  - gxOff;
    gy      = (((int16_t)b[10] << 8) | b[11]) - gyOff;
    gz      = (((int16_t)b[12] << 8) | b[13]) - gzOff;
}

void mpuCalibrateGyro() {
    Serial.print(F("[CALIB] Sampling gyro bias ("));
    Serial.print(GYRO_CALIB_SAMPLES);
    Serial.println(F(" samples)... keep robot still!"));

    int32_t sx = 0, sy = 0, sz = 0;
    int16_t saveOff_x = gxOff, saveOff_y = gyOff, saveOff_z = gzOff;
    gxOff = gyOff = gzOff = 0;

    for (uint16_t i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        mpuRead();
        sx += gx; sy += gy; sz += gz;
        delay(2);
        if (i % 100 == 0) Serial.print(F("."));
    }
    gxOff = sx / GYRO_CALIB_SAMPLES;
    gyOff = sy / GYRO_CALIB_SAMPLES;
    gzOff = sz / GYRO_CALIB_SAMPLES;

    Serial.println();
    Serial.print(F("[CALIB] Gyro offsets  →  gx="));
    Serial.print(gxOff);
    Serial.print(F("  gy=")); Serial.print(gyOff);
    Serial.print(F("  gz=")); Serial.println(gzOff);
}

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                  9.  MAGNETOMETER DRIVER (HMC + QMC)              ║
// ╚═══════════════════════════════════════════════════════════════════╝
//   Auto-detects whether the on-board chip is a Honeywell HMC5883L
//   (address 0x1E) or a QST QMC5883L (address 0x0D). Most modern GY-87
//   clones ship with QMC5883L.

bool hmcInit() {
    // Verify ID  ("H43" → 0x48,0x34,0x33 in registers 10/11/12)
    uint8_t id[3];
    if (!i2cReadBytes(ADDR_HMC5883L, 0x0A, id, 3)) return false;
    if (id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33) return false;

    i2cWrite(ADDR_HMC5883L, 0x00, HMC_CONFIG_A);
    i2cWrite(ADDR_HMC5883L, 0x01, HMC_CONFIG_B);
    i2cWrite(ADDR_HMC5883L, 0x02, HMC_MODE_REG);
    delay(6);
    return true;
}

bool qmcInit() {
    // Soft reset
    i2cWrite(ADDR_QMC5883L, 0x0A, 0x80);
    delay(10);
    // Configure (period + control)
    i2cWrite(ADDR_QMC5883L, 0x0B, QMC_PERIOD);
    i2cWrite(ADDR_QMC5883L, 0x0A, QMC_CONFIG_2);
    i2cWrite(ADDR_QMC5883L, 0x09, QMC_CONFIG_1);
    delay(10);

    // Verify by reading status — should be reachable
    Wire.beginTransmission(ADDR_QMC5883L);
    Wire.write(0x06);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)ADDR_QMC5883L, (uint8_t)1);
    return Wire.available() > 0;
}

bool magInit() {
    // Try Honeywell first
    if (i2cDeviceExists(ADDR_HMC5883L) && hmcInit()) {
        magType = MAG_HMC5883L;
        return true;
    }
    // Fall back to QST clone
    if (i2cDeviceExists(ADDR_QMC5883L) && qmcInit()) {
        magType = MAG_QMC5883L;
        return true;
    }
    magType = MAG_NONE;
    return false;
}

void magRead() {
    uint8_t b[6];

    if (magType == MAG_HMC5883L) {
        if (!i2cReadBytes(ADDR_HMC5883L, 0x03, b, 6)) return;
        // HMC5883L register order: X, Z, Y  (big-endian)
        mx = ((int16_t)b[0] << 8) | b[1];
        mz = ((int16_t)b[2] << 8) | b[3];
        my = ((int16_t)b[4] << 8) | b[5];
    }
    else if (magType == MAG_QMC5883L) {
        if (!i2cReadBytes(ADDR_QMC5883L, 0x00, b, 6)) return;
        // QMC5883L register order: X, Y, Z  (little-endian!)
        mx = ((int16_t)b[1] << 8) | b[0];
        my = ((int16_t)b[3] << 8) | b[2];
        mz = ((int16_t)b[5] << 8) | b[4];
    }
}

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                 10.  BMP180 BAROMETER DRIVER                      ║
// ╚═══════════════════════════════════════════════════════════════════╝
//   Uses non-blocking state machine to avoid stalling the 50 Hz loop:
//     State 0: kick off temperature conversion
//     State 1: wait 5 ms → read raw temp, kick off pressure conversion
//     State 2: wait for pressure → read, compute, store
//     State 3: idle until next BARO_PERIOD_MS

bool baroInit() {
    if (!i2cDeviceExists(ADDR_BMP180)) return false;

    // Chip-ID register (0xD0) must return 0x55
    if (i2cReadByte(ADDR_BMP180, 0xD0) != 0x55) return false;

    // Read 22 bytes of factory calibration from EEPROM (0xAA … 0xBF)
    uint8_t buf[22];
    if (!i2cReadBytes(ADDR_BMP180, 0xAA, buf, 22)) return false;

    ac1 = (buf[0]  << 8) | buf[1];
    ac2 = (buf[2]  << 8) | buf[3];
    ac3 = (buf[4]  << 8) | buf[5];
    ac4 = (buf[6]  << 8) | buf[7];
    ac5 = (buf[8]  << 8) | buf[9];
    ac6 = (buf[10] << 8) | buf[11];
    b1  = (buf[12] << 8) | buf[13];
    b2  = (buf[14] << 8) | buf[15];
    mb  = (buf[16] << 8) | buf[17];
    mc  = (buf[18] << 8) | buf[19];
    md  = (buf[20] << 8) | buf[21];
    return true;
}

void baroUpdate() {
    unsigned long now = millis();

    switch (bmp_state) {
        case 0:  // Kick off temperature measurement
            i2cWrite(ADDR_BMP180, 0xF4, 0x2E);
            bmp_t0 = now;
            bmp_state = 1;
            break;

        case 1:  // Wait 5 ms, read temp, kick off pressure
            if (now - bmp_t0 < 5) return;
            {
                uint8_t b[2];
                i2cReadBytes(ADDR_BMP180, 0xF6, b, 2);
                bmp_UT = ((int32_t)b[0] << 8) | b[1];
            }
            i2cWrite(ADDR_BMP180, 0xF4, 0x34 | (BMP_OSS << 6));
            bmp_t0 = now;
            bmp_state = 2;
            break;

        case 2:  // Wait for pressure (depends on OSS), then compute
        {
            uint16_t wait_ms = 5 + (3 << BMP_OSS);   // 8,14,26 ms typical
            if (now - bmp_t0 < wait_ms) return;

            uint8_t b[3];
            i2cReadBytes(ADDR_BMP180, 0xF6, b, 3);
            bmp_UP = (((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2])
                     >> (8 - BMP_OSS);

            // --- Temperature compensation (Bosch datasheet) ---
            int32_t X1 = ((bmp_UT - (int32_t)ac6) * (int32_t)ac5) >> 15;
            int32_t X2 = ((int32_t)mc << 11) / (X1 + md);
            bmp_B5     = X1 + X2;
            baroTempDeci = (bmp_B5 + 8) >> 4;        // 0.1 °C units

            // --- Pressure compensation ---
            int32_t B6 = bmp_B5 - 4000;
            X1 = ((int32_t)b2 * ((B6 * B6) >> 12)) >> 11;
            X2 = ((int32_t)ac2 * B6) >> 11;
            int32_t X3 = X1 + X2;
            int32_t B3 = ((((int32_t)ac1 * 4 + X3) << BMP_OSS) + 2) >> 2;
            X1 = ((int32_t)ac3 * B6) >> 13;
            X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
            X3 = ((X1 + X2) + 2) >> 2;
            uint32_t B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
            uint32_t B7 = ((uint32_t)bmp_UP - B3) * (50000UL >> BMP_OSS);
            int32_t p;
            if (B7 < 0x80000000UL) p = (B7 * 2) / B4;
            else                   p = (B7 / B4) * 2;
            X1 = (p >> 8) * (p >> 8);
            X1 = (X1 * 3038) >> 16;
            X2 = (-7357 * p) >> 16;
            baroPressurePa = p + ((X1 + X2 + 3791) >> 4);
            baroReady = true;          // first valid reading now available

            bmp_state = 0;   // ready for next cycle
            break;
        }
    }
}

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                 11.  MOTOR CONTROL                                ║
// ╚═══════════════════════════════════════════════════════════════════╝

void drive(int left, int right) {
    lastLeftCmd  = left;
    lastRightCmd = right;

    // ----- Left channel -----
    if (left > 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
    else if (left < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
    else               { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }
    analogWrite(ENA, constrain(abs(left), 0, 255));

    // ----- Right channel -----
    if (right > 0)      { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
    else if (right < 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
    else                { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
    analogWrite(ENB, constrain(abs(right), 0, 255));
}

void emergencyStop() {
    drive(0, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                 12.  COMMAND PARSER                               ║
// ╚═══════════════════════════════════════════════════════════════════╝

void printHelp() {
    Serial.println(F("─── COMMAND REFERENCE ───────────────────────────"));
    Serial.println(F(" Movement:"));
    Serial.println(F("   F             Forward"));
    Serial.println(F("   B             Backward"));
    Serial.println(F("   L             Turn left"));
    Serial.println(F("   R             Turn right"));
    Serial.println(F("   S             Stop"));
    Serial.println(F("   T<l>,<r>      Tank drive (-255..255 each)"));
    Serial.println(F(" Configuration:"));
    Serial.println(F("   P<0-255>      Set PWM speed"));
    Serial.println(F("   Z             Zero encoder counts"));
    Serial.println(F("   C             Recalibrate gyro bias"));
    Serial.println(F("   W<0|1>        Watchdog disable / enable"));
    Serial.println(F(" Diagnostics:"));
    Serial.println(F("   ?             Show this help"));
    Serial.println(F("   I             Show sensor status"));
    Serial.println(F("   V             Toggle verbose mode"));
    Serial.println(F("─────────────────────────────────────────────────"));
}

void printStatus() {
    Serial.println(F("─── SYSTEM STATUS ───────────────────────────────"));
    Serial.print(F(" Uptime         : ")); Serial.print(millis()/1000.0, 1); Serial.println(F(" s"));
    Serial.print(F(" Stream count   : ")); Serial.println(streamCount);
    Serial.print(F(" PWM speed      : ")); Serial.println(pwmSpeed);
    Serial.print(F(" Last cmd       : L=")); Serial.print(lastLeftCmd);
    Serial.print(F("  R=")); Serial.println(lastRightCmd);
    Serial.print(F(" Watchdog       : ")); Serial.println(watchdogActive ? F("ON") : F("OFF"));
    Serial.println();
    Serial.print(F(" MPU6050        : ")); Serial.println(mpuOK  ? F("OK") : F("FAIL"));
    Serial.print(F(" Magnetometer   : "));
    if      (magType == MAG_HMC5883L) Serial.println(F("OK (HMC5883L)"));
    else if (magType == MAG_QMC5883L) Serial.println(F("OK (QMC5883L)"));
    else                              Serial.println(F("FAIL"));
    Serial.print(F(" BMP180         : ")); Serial.println(baroOK ? F("OK") : F("FAIL"));
    Serial.println();
    Serial.print(F(" Gyro offsets   : ")); Serial.print(gxOff);
    Serial.print(F(", ")); Serial.print(gyOff);
    Serial.print(F(", ")); Serial.println(gzOff);
    Serial.print(F(" Encoders FL,RL : ")); Serial.print(enc[0]); Serial.print(F(", ")); Serial.println(enc[1]);
    Serial.print(F(" Encoders FR,RR : ")); Serial.print(enc[2]); Serial.print(F(", ")); Serial.println(enc[3]);
    Serial.print(F(" Accel  (raw)   : ")); Serial.print(ax); Serial.print(','); Serial.print(ay); Serial.print(','); Serial.println(az);
    Serial.print(F(" Gyro   (raw)   : ")); Serial.print(gx); Serial.print(','); Serial.print(gy); Serial.print(','); Serial.println(gz);
    Serial.print(F(" Mag    (raw)   : ")); Serial.print(mx); Serial.print(','); Serial.print(my); Serial.print(','); Serial.println(mz);
    Serial.print(F(" Pressure       : ")); Serial.print(baroPressurePa); Serial.println(F(" Pa"));
    Serial.print(F(" Baro temp      : ")); Serial.print(baroTempDeci / 10.0, 1); Serial.println(F(" °C"));

    // Quick computed values
    float heading = atan2((float)my, (float)mx) * 180.0 / PI;
    if (heading < 0) heading += 360.0;
    Serial.print(F(" Heading (mag)  : ")); Serial.print(heading, 1); Serial.println(F(" °"));
    Serial.println(F("─────────────────────────────────────────────────"));
}

void handleCommand() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    char cmd = toupper(input[0]);
    lastCmdTime = millis();

    switch (cmd) {
        case 'F': drive( pwmSpeed,  pwmSpeed); Serial.println(F("OK:FORWARD"));  break;
        case 'B': drive(-pwmSpeed, -pwmSpeed); Serial.println(F("OK:BACKWARD")); break;
        case 'L': drive(-pwmSpeed,  pwmSpeed); Serial.println(F("OK:LEFT"));     break;
        case 'R': drive( pwmSpeed, -pwmSpeed); Serial.println(F("OK:RIGHT"));    break;
        case 'S': drive(0, 0);                 Serial.println(F("OK:STOP"));     break;

        case 'P': {
            int v = input.substring(1).toInt();
            pwmSpeed = constrain(v, 0, 255);
            Serial.print(F("OK:PWM=")); Serial.println(pwmSpeed);
            break;
        }

        case 'T': {
            int comma = input.indexOf(',');
            if (comma > 1) {
                int l = input.substring(1, comma).toInt();
                int r = input.substring(comma + 1).toInt();
                drive(constrain(l, -255, 255), constrain(r, -255, 255));
                Serial.print(F("OK:TANK L=")); Serial.print(l);
                Serial.print(F(" R="));        Serial.println(r);
            } else {
                Serial.println(F("ERR:TANK_SYNTAX (use T<left>,<right>)"));
            }
            break;
        }

        case 'Z':
            noInterrupts();
            enc[0] = enc[1] = enc[2] = enc[3] = 0;
            interrupts();
            Serial.println(F("OK:ENCODERS_ZEROED"));
            break;

        case 'C':
            emergencyStop();
            mpuCalibrateGyro();
            Serial.println(F("OK:CALIBRATED"));
            break;

        case 'W':
            watchdogActive = (input.substring(1).toInt() != 0);
            Serial.print(F("OK:WATCHDOG=")); Serial.println(watchdogActive ? F("ON") : F("OFF"));
            break;

        case 'V':
            verbose = !verbose;
            Serial.print(F("OK:VERBOSE=")); Serial.println(verbose ? F("ON") : F("OFF"));
            break;

        case '?':
        case 'H':
            printHelp();
            break;

        case 'I':
            printStatus();
            break;

        default:
            Serial.print(F("ERR:UNKNOWN_CMD '"));
            Serial.print(cmd);
            Serial.println(F("'  (type ? for help)"));
            break;
    }
}

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                 13.  DATA STREAMING                               ║
// ╚═══════════════════════════════════════════════════════════════════╝
//
//  HIGH-RATE PACKET  (50 Hz):
//    D:ts,e1,e2,e3,e4,ax,ay,az,gx,gy,gz,mx,my,mz
//
//  LOW-RATE PACKET   (10 Hz):
//    B:ts,pressure_pa,temp_deci_c
//
//  All values are decimal ASCII for easy debugging.
//  Pi-side conversion factors are documented in section 3.

void streamHighRate() {
    noInterrupts();
    long e0 = enc[0], e1 = enc[1], e2 = enc[2], e3 = enc[3];
    interrupts();

    char buf[160];
    snprintf(buf, sizeof(buf),
        "D:%lu,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d",
        millis(),
        e0, e1, e2, e3,
        ax, ay, az,
        gx, gy, gz,
        mx, my, mz);
    Serial.println(buf);
    streamCount++;
}

void streamBaro() {
    char buf[64];
    snprintf(buf, sizeof(buf),
        "B:%lu,%ld,%d",
        millis(), baroPressurePa, baroTempDeci);
    Serial.println(buf);
}

void printHeartbeat() {
    if (!verbose) return;

    float heading = atan2((float)my, (float)mx) * 180.0 / PI;
    if (heading < 0) heading += 360.0;

    Serial.print(F("# Hz≈")); Serial.print(streamCount);
    Serial.print(F("  ENC[")); Serial.print(enc[0]); Serial.print(',');
                               Serial.print(enc[1]); Serial.print(',');
                               Serial.print(enc[2]); Serial.print(',');
                               Serial.print(enc[3]); Serial.print(']');
    Serial.print(F("  HDG=")); Serial.print(heading, 0); Serial.print(F("°"));
    Serial.print(F("  P=")); Serial.print(baroPressurePa); Serial.print(F("Pa"));
    Serial.print(F("  T=")); Serial.print(baroTempDeci / 10.0, 1); Serial.println(F("°C"));
    streamCount = 0;
}

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                 14.  BANNER & DIAGNOSTICS                         ║
// ╚═══════════════════════════════════════════════════════════════════╝

void printBanner() {
    Serial.println();
    Serial.println(F("╔══════════════════════════════════════════════════════╗"));
    Serial.println(F("║      ROBOT 2 CONTROLLER  —  10-DOF Edition v2.0      ║"));
    Serial.println(F("║   Arduino Mega · L298N · 4× Encoders · GY-87 IMU     ║"));
    Serial.println(F("╚══════════════════════════════════════════════════════╝"));
    Serial.println();
}

void reportInit(const __FlashStringHelper *label, bool ok) {
    Serial.print(F("[INIT] "));
    Serial.print(label);
    while (strlen_P((PGM_P)label) < 24) {
        Serial.print(' ');
        // simple padding loop bounded by 30 chars
        static uint8_t pad = 0;
        if (++pad > 30) break;
    }
    Serial.print(F(" : "));
    Serial.println(ok ? F("OK") : F("FAIL"));
}

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                 15.  SETUP                                        ║
// ╚═══════════════════════════════════════════════════════════════════╝

void setup() {
    Serial.begin(115200);
    delay(50);

    printBanner();

    // --- Motor pins ---
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    drive(0, 0);
    Serial.println(F("[INIT] Motor driver         : OK"));

    // --- Encoder pins & interrupts ---
    const uint8_t encA[] = {M1_ENCA, M2_ENCA, M3_ENCA, M4_ENCA};
    const uint8_t encB[] = {M1_ENCB, M2_ENCB, M3_ENCB, M4_ENCB};
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(encA[i], INPUT_PULLUP);
        pinMode(encB[i], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(M1_ENCA), isrM1, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENCA), isrM2, RISING);
    attachInterrupt(digitalPinToInterrupt(M3_ENCA), isrM3, RISING);
    attachInterrupt(digitalPinToInterrupt(M4_ENCA), isrM4, RISING);
    Serial.println(F("[INIT] Encoders (4ch)       : OK"));

    // --- I2C bus ---
    Wire.begin();
    Wire.setClock(I2C_CLOCK);
    Serial.print(F("[INIT] I2C bus              : OK  ("));
    Serial.print(I2C_CLOCK / 1000); Serial.println(F(" kHz)"));

    // --- I2C scanner: list every responding device ---
    Serial.print(F("[SCAN] I2C devices found    :"));
    uint8_t devCount = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F(" 0x"));
            if (addr < 0x10) Serial.print('0');
            Serial.print(addr, HEX);
            devCount++;
        }
    }
    if (devCount == 0) Serial.print(F(" (none — check wiring!)"));
    Serial.println();

    // --- MPU6050 ---
    mpuOK = mpuInit();
    Serial.print(F("[INIT] MPU6050  (Gyro/Accel): "));
    Serial.println(mpuOK ? F("OK") : F("FAIL"));

    // --- Magnetometer (auto-detect HMC5883L vs QMC5883L) ---
    magOK = magInit();
    Serial.print(F("[INIT] Magnetometer         : "));
    if      (magType == MAG_HMC5883L) Serial.println(F("OK  (HMC5883L @ 0x1E)"));
    else if (magType == MAG_QMC5883L) Serial.println(F("OK  (QMC5883L @ 0x0D)"));
    else                              Serial.println(F("FAIL  (no chip at 0x1E or 0x0D)"));

    // --- BMP180 ---
    baroOK = baroInit();
    Serial.print(F("[INIT] BMP180   (Baro/Temp) : "));
    Serial.println(baroOK ? F("OK") : F("FAIL"));

    Serial.println();

    // --- Gyro calibration ---
    if (mpuOK) {
        mpuCalibrateGyro();
        Serial.println();
    }

    Serial.print(F("[READY] Streaming @ "));
    Serial.print(STREAM_HZ); Serial.println(F(" Hz."));
    Serial.println(F("[READY] Type '?' for command help."));
    Serial.println();

    lastCmdTime = millis();
}

// ╔═══════════════════════════════════════════════════════════════════╗
// ║                 16.  MAIN LOOP                                    ║
// ╚═══════════════════════════════════════════════════════════════════╝

void loop() {
    // ---- 1. Service incoming serial commands ----
    handleCommand();

    // ---- 2. Read fast sensors every loop ----
    if (mpuOK) mpuRead();
    if (magOK) magRead();

    // ---- 3. Service slow barometer state machine ----
    if (baroOK) baroUpdate();

    // ---- 4. Watchdog: stop motors if no command lately ----
    if (watchdogActive && (lastLeftCmd != 0 || lastRightCmd != 0)) {
        if (millis() - lastCmdTime > WATCHDOG_MS) {
            drive(0, 0);
            Serial.println(F("WARN:WATCHDOG_TIMEOUT — motors stopped"));
            lastCmdTime = millis();  // prevent message spam
        }
    }

    // ---- 5. Stream high-rate sensor packet ----
    unsigned long now = millis();
    if (now - lastStream >= STREAM_PERIOD_MS) {
        lastStream = now;
        streamHighRate();
    }

    // ---- 6. Stream barometer (low rate, after first valid reading) ----
    if (baroOK && baroReady && now - lastBaro >= BARO_PERIOD_MS) {
        lastBaro = now;
        streamBaro();
    }

    // ---- 7. Verbose heartbeat once per second ----
    if (now - lastHeartbeat >= HEARTBEAT_MS) {
        lastHeartbeat = now;
        printHeartbeat();
    }
}
