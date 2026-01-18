/*
 * ╔═══════════════════════════════════════════════════════════════╗
 * ║        RESCUE ROBOT - PROFESSIONAL MOTOR CONTROLLER           ║
 * ║                     Arduino Mega 2560                          ║
 * ╚═══════════════════════════════════════════════════════════════╝
 * 
 * Features:
 * - Variable speed control (0-255 PWM)
 * - Smooth acceleration/deceleration
 * - Emergency stop
 * - Status feedback via Serial
 * - Watchdog timeout (auto-stop if no commands)
 * 
 * Protocol:
 *   Commands: Single character or with speed
 *   F       - Forward at current speed
 *   B       - Backward at current speed
 *   L       - Turn left at current speed
 *   R       - Turn right at current speed
 *   S       - Stop immediately
 *   E       - Emergency stop (hard brake)
 *   Pxxx    - Set speed (e.g., P150 = speed 150)
 *   ?       - Request status
 * 
 * Response:
 *   OK:CMD  - Command acknowledged
 *   SPD:xxx - Current speed
 *   STS:xxx - Status (IDLE/MOVING/ESTOP)
 */

// ============================================
// Motor Pin Definitions (4WD Configuration)
// ============================================

// Rear Left Motor
const int RL_PWM = 4;
const int RL_IN1 = 22;
const int RL_IN2 = 23;

// Rear Right Motor
const int RR_PWM = 5;
const int RR_IN1 = 24;
const int RR_IN2 = 25;

// Front Right Motor
const int FR_PWM = 6;
const int FR_IN1 = 27;
const int FR_IN2 = 26;

// Front Left Motor
const int FL_PWM = 7;
const int FL_IN1 = 29;
const int FL_IN2 = 28;

// ============================================
// Configuration
// ============================================
const int DEFAULT_SPEED = 180;        // Default speed (0-255)
const int MIN_SPEED = 80;             // Minimum speed to move
const int MAX_SPEED = 255;            // Maximum speed
const int ACCEL_STEP = 15;            // Acceleration step per loop
const int ACCEL_DELAY = 20;           // ms between acceleration steps
const unsigned long TIMEOUT_MS = 500; // Auto-stop if no commands for 500ms

// ============================================
// State Variables
// ============================================
int targetSpeed = DEFAULT_SPEED;
int currentSpeed = 0;
char currentCommand = 'S';
char lastCommand = 'S';
bool emergencyStop = false;
unsigned long lastCommandTime = 0;

// ============================================
// Setup
// ============================================
void setup() {
    Serial.begin(115200);  // Higher baud rate for faster response
    
    // Initialize motor pins
    int pins[] = {FR_PWM, FR_IN1, FR_IN2, FL_PWM, FL_IN1, FL_IN2,
                  RR_PWM, RR_IN1, RR_IN2, RL_PWM, RL_IN1, RL_IN2};
    for (int i = 0; i < 12; i++) {
        pinMode(pins[i], OUTPUT);
    }
    
    // Make sure motors are stopped
    stopAllMotors();
    
    // Startup message
    Serial.println("=================================");
    Serial.println("  RESCUE ROBOT MOTOR CONTROLLER");
    Serial.println("  Firmware v2.0 - Professional");
    Serial.println("=================================");
    Serial.print("Default Speed: ");
    Serial.println(DEFAULT_SPEED);
    Serial.println("Ready for commands...");
    Serial.println("OK:READY");
}

// ============================================
// Main Loop
// ============================================
void loop() {
    // Read commands from Serial
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        processCommand(cmd);
    }
    
    // Watchdog: stop if no commands for TIMEOUT_MS
    if (currentCommand != 'S' && !emergencyStop) {
        if (millis() - lastCommandTime > TIMEOUT_MS) {
            currentCommand = 'S';
            Serial.println("WARN:TIMEOUT");
        }
    }
    
    // Smooth acceleration
    updateSpeed();
    
    // Execute movement
    executeMovement();
    
    delay(10);  // Small delay for stability
}

// ============================================
// Command Processing
// ============================================
void processCommand(char cmd) {
    // Ignore whitespace
    if (cmd == '\n' || cmd == '\r' || cmd == ' ') return;
    
    cmd = toupper(cmd);
    lastCommandTime = millis();
    
    switch (cmd) {
        case 'F':  // Forward
        case 'B':  // Backward
        case 'L':  // Left
        case 'R':  // Right
            if (!emergencyStop) {
                currentCommand = cmd;
                Serial.print("OK:");
                Serial.println(cmd);
            } else {
                Serial.println("ERR:ESTOP_ACTIVE");
            }
            break;
            
        case 'S':  // Stop
            currentCommand = 'S';
            Serial.println("OK:STOP");
            break;
            
        case 'E':  // Emergency Stop
            emergencyStop = true;
            currentCommand = 'S';
            currentSpeed = 0;
            hardBrake();
            Serial.println("OK:ESTOP");
            break;
            
        case 'X':  // Release Emergency Stop
            emergencyStop = false;
            Serial.println("OK:ESTOP_RELEASED");
            break;
            
        case 'P':  // Set speed (P followed by 3 digits)
            readSpeed();
            break;
            
        case '?':  // Status request
            sendStatus();
            break;
            
        default:
            // Check if it's a digit (part of speed command)
            if (!isDigit(cmd)) {
                Serial.print("ERR:UNKNOWN_CMD:");
                Serial.println(cmd);
            }
            break;
    }
}

void readSpeed() {
    delay(10);  // Wait for digits to arrive
    String speedStr = "";
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (isDigit(c)) {
            speedStr += c;
        } else {
            break;
        }
    }
    
    if (speedStr.length() > 0) {
        int newSpeed = speedStr.toInt();
        newSpeed = constrain(newSpeed, MIN_SPEED, MAX_SPEED);
        targetSpeed = newSpeed;
        Serial.print("OK:SPD=");
        Serial.println(targetSpeed);
    }
}

void sendStatus() {
    Serial.print("STS:");
    if (emergencyStop) {
        Serial.print("ESTOP");
    } else if (currentCommand == 'S') {
        Serial.print("IDLE");
    } else {
        Serial.print("MOVING:");
        Serial.print(currentCommand);
    }
    Serial.print(",SPD:");
    Serial.print(currentSpeed);
    Serial.print(",TGT:");
    Serial.println(targetSpeed);
}

// ============================================
// Speed Control (Smooth Acceleration)
// ============================================
void updateSpeed() {
    static unsigned long lastAccelTime = 0;
    
    if (millis() - lastAccelTime < ACCEL_DELAY) return;
    lastAccelTime = millis();
    
    int desiredSpeed = (currentCommand == 'S' || emergencyStop) ? 0 : targetSpeed;
    
    if (currentSpeed < desiredSpeed) {
        currentSpeed = min(currentSpeed + ACCEL_STEP, desiredSpeed);
    } else if (currentSpeed > desiredSpeed) {
        currentSpeed = max(currentSpeed - ACCEL_STEP * 2, desiredSpeed);  // Decel faster
    }
}

// ============================================
// Motor Control Functions
// ============================================
void executeMovement() {
    switch (currentCommand) {
        case 'F':
            moveForward();
            break;
        case 'B':
            moveBackward();
            break;
        case 'L':
            turnLeft();
            break;
        case 'R':
            turnRight();
            break;
        case 'S':
        default:
            stopAllMotors();
            break;
    }
}

void motorSet(int pwmPin, int in1, int in2, int speed, bool forward) {
    analogWrite(pwmPin, abs(speed));
    if (speed == 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    } else if (forward) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
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
    // IMPORTANT: Use MAX speed for turning (robot needs high torque to turn)
    int turnSpeed = MAX_SPEED;
    // Right side forward, left side backward
    motorSet(FR_PWM, FR_IN1, FR_IN2, turnSpeed, true);
    motorSet(RR_PWM, RR_IN1, RR_IN2, turnSpeed, true);
    motorSet(FL_PWM, FL_IN1, FL_IN2, turnSpeed, false);
    motorSet(RL_PWM, RL_IN1, RL_IN2, turnSpeed, false);
}

void turnRight() {
    // IMPORTANT: Use MAX speed for turning (robot needs high torque to turn)
    int turnSpeed = MAX_SPEED;
    // Left side forward, right side backward
    motorSet(FL_PWM, FL_IN1, FL_IN2, turnSpeed, true);
    motorSet(RL_PWM, RL_IN1, RL_IN2, turnSpeed, true);
    motorSet(FR_PWM, FR_IN1, FR_IN2, turnSpeed, false);
    motorSet(RR_PWM, RR_IN1, RR_IN2, turnSpeed, false);
}

void stopAllMotors() {
    motorSet(FR_PWM, FR_IN1, FR_IN2, 0, true);
    motorSet(FL_PWM, FL_IN1, FL_IN2, 0, true);
    motorSet(RR_PWM, RR_IN1, RR_IN2, 0, true);
    motorSet(RL_PWM, RL_IN1, RL_IN2, 0, true);
}

void hardBrake() {
    // Active braking by shorting motor terminals
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, HIGH);
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, HIGH);
    digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, HIGH);
    digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, HIGH);
    
    analogWrite(FR_PWM, 255);
    analogWrite(FL_PWM, 255);
    analogWrite(RR_PWM, 255);
    analogWrite(RL_PWM, 255);
    
    delay(100);  // Brief brake
    stopAllMotors();
}
