/*
 * RESCUE ROBOT - MOTOR CONTROLLER v2
 * Arduino Mega 2560
 * 
 * Features:
 * - Variable speed (PWM 0-255)
 * - MAX speed for turning (robot needs high torque)
 * - Watchdog timeout
 * - Emergency stop
 * 
 * Commands: F, B, L, R, S, E, X, P###, ?
 */

// Motor Pins (4WD)
const int RL_PWM = 4, RL_IN1 = 22, RL_IN2 = 23;
const int RR_PWM = 5, RR_IN1 = 24, RR_IN2 = 25;
const int FR_PWM = 6, FR_IN1 = 27, FR_IN2 = 26;
const int FL_PWM = 7, FL_IN1 = 29, FL_IN2 = 28;

// Config
const int DEFAULT_SPEED = 180;
const int MIN_SPEED = 80;
const int MAX_SPEED = 255;
const unsigned long TIMEOUT_MS = 2000;  // 2 seconds (was 500ms)

// State
int targetSpeed = DEFAULT_SPEED;
int currentSpeed = 0;
char currentCommand = 'S';
bool emergencyStop = false;
unsigned long lastCommandTime = 0;

void setup() {
    Serial.begin(115200);
    
    int pins[] = {FR_PWM, FR_IN1, FR_IN2, FL_PWM, FL_IN1, FL_IN2,
                  RR_PWM, RR_IN1, RR_IN2, RL_PWM, RL_IN1, RL_IN2};
    for (int i = 0; i < 12; i++) pinMode(pins[i], OUTPUT);
    
    stopAllMotors();
    Serial.println("RESCUE ROBOT v2.0 Ready");
    Serial.println("OK:READY");
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = toupper(Serial.read());
        processCommand(cmd);
    }
    
    // Watchdog
    if (currentCommand != 'S' && !emergencyStop) {
        if (millis() - lastCommandTime > TIMEOUT_MS) {
            currentCommand = 'S';
            Serial.println("TIMEOUT");
        }
    }
    
    updateSpeed();
    executeMovement();
    delay(10);
}

void processCommand(char cmd) {
    if (cmd == '\n' || cmd == '\r' || cmd == ' ') return;
    lastCommandTime = millis();
    
    switch (cmd) {
        case 'F': case 'B': case 'L': case 'R':
            if (!emergencyStop) {
                currentCommand = cmd;
                Serial.print("OK:"); Serial.println(cmd);
            }
            break;
        case 'S':
            currentCommand = 'S';
            Serial.println("OK:STOP");
            break;
        case 'E':
            emergencyStop = true;
            currentCommand = 'S';
            currentSpeed = 0;
            hardBrake();
            Serial.println("OK:ESTOP");
            break;
        case 'X':
            emergencyStop = false;
            Serial.println("OK:RELEASED");
            break;
        case 'P':
            readSpeed();
            break;
        case '?':
            Serial.print("STS:");
            Serial.print(emergencyStop ? "ESTOP" : (currentCommand == 'S' ? "IDLE" : "MOVING"));
            Serial.print(",SPD:"); Serial.println(currentSpeed);
            break;
    }
}

void readSpeed() {
    delay(10);
    String s = "";
    while (Serial.available()) {
        char c = Serial.read();
        if (isDigit(c)) s += c; else break;
    }
    if (s.length() > 0) {
        targetSpeed = constrain(s.toInt(), MIN_SPEED, MAX_SPEED);
        Serial.print("OK:SPD="); Serial.println(targetSpeed);
    }
}

void updateSpeed() {
    int desired = (currentCommand == 'S' || emergencyStop) ? 0 : targetSpeed;
    if (currentSpeed < desired) currentSpeed = min(currentSpeed + 15, desired);
    else if (currentSpeed > desired) currentSpeed = max(currentSpeed - 30, desired);
}

void motorSet(int pwm, int in1, int in2, int speed, bool fwd) {
    analogWrite(pwm, abs(speed));
    if (speed == 0) { digitalWrite(in1, LOW); digitalWrite(in2, LOW); }
    else if (fwd) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
    else { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); }
}

void executeMovement() {
    switch (currentCommand) {
        case 'F': moveForward(); break;
        case 'B': moveBackward(); break;
        case 'L': turnLeft(); break;
        case 'R': turnRight(); break;
        default: stopAllMotors(); break;
    }
}

void moveForward() {
    motorSet(FR_PWM, FR_IN1, FR_IN2, currentSpeed, true);
    motorSet(FL_PWM, FL_IN1, FL_IN2, currentSpeed, true);
    motorSet(RR_PWM, RR_IN1, RR_IN2, currentSpeed, true);
    motorSet(RL_PWM, RL_IN1, RL_IN2, currentSpeed, true);
}

void moveBackward() {
    motorSet(FR_PWM, FR_IN1, FR_IN2, currentSpeed, false);
    motorSet(FL_PWM, FL_IN1, FL_IN2, currentSpeed, false);
    motorSet(RR_PWM, RR_IN1, RR_IN2, currentSpeed, false);
    motorSet(RL_PWM, RL_IN1, RL_IN2, currentSpeed, false);
}

void turnLeft() {
    // MAX speed for turning!
    motorSet(FR_PWM, FR_IN1, FR_IN2, MAX_SPEED, true);
    motorSet(RR_PWM, RR_IN1, RR_IN2, MAX_SPEED, true);
    motorSet(FL_PWM, FL_IN1, FL_IN2, MAX_SPEED, false);
    motorSet(RL_PWM, RL_IN1, RL_IN2, MAX_SPEED, false);
}

void turnRight() {
    // MAX speed for turning!
    motorSet(FL_PWM, FL_IN1, FL_IN2, MAX_SPEED, true);
    motorSet(RL_PWM, RL_IN1, RL_IN2, MAX_SPEED, true);
    motorSet(FR_PWM, FR_IN1, FR_IN2, MAX_SPEED, false);
    motorSet(RR_PWM, RR_IN1, RR_IN2, MAX_SPEED, false);
}

void stopAllMotors() {
    motorSet(FR_PWM, FR_IN1, FR_IN2, 0, true);
    motorSet(FL_PWM, FL_IN1, FL_IN2, 0, true);
    motorSet(RR_PWM, RR_IN1, RR_IN2, 0, true);
    motorSet(RL_PWM, RL_IN1, RL_IN2, 0, true);
}

void hardBrake() {
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, HIGH);
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, HIGH);
    digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, HIGH);
    digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, HIGH);
    analogWrite(FR_PWM, 255); analogWrite(FL_PWM, 255);
    analogWrite(RR_PWM, 255); analogWrite(RL_PWM, 255);
    delay(100);
    stopAllMotors();
}
