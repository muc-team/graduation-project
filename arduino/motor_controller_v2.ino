// ==========================================
// Professional Motor Controller v2 (Arduino)
// Pairs exactly with professional_motor_controller.py
// ==========================================

// Motor Driver Pins (L298N)
#define ENA 10
#define IN1 8
#define IN2 9

#define ENB 11
#define IN3 12
#define IN4 13

// Encoders (Mega 2560)
#define M1_ENCA 2
#define M1_ENCB 22
#define M2_ENCA 3
#define M2_ENCB 24
#define M3_ENCA 18
#define M3_ENCB 26
#define M4_ENCA 19
#define M4_ENCB 28

volatile long countM1 = 0;
volatile long countM2 = 0;
volatile long countM3 = 0;
volatile long countM4 = 0;

void readEncoderM1() { if (digitalRead(M1_ENCB) > 0) countM1++; else countM1--; }
void readEncoderM2() { if (digitalRead(M2_ENCB) > 0) countM2++; else countM2--; }
void readEncoderM3() { if (digitalRead(M3_ENCB) > 0) countM3++; else countM3--; }
void readEncoderM4() { if (digitalRead(M4_ENCB) > 0) countM4++; else countM4--; }

int currentSpeed = 150;
bool eStop = false;

void setup() {
  Serial.begin(115200); // Must match baud_rate in python script!

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  pinMode(M1_ENCA, INPUT_PULLUP); pinMode(M1_ENCB, INPUT_PULLUP);
  pinMode(M2_ENCA, INPUT_PULLUP); pinMode(M2_ENCB, INPUT_PULLUP);
  pinMode(M3_ENCA, INPUT_PULLUP); pinMode(M3_ENCB, INPUT_PULLUP);
  pinMode(M4_ENCA, INPUT_PULLUP); pinMode(M4_ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENCA), readEncoderM1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENCA), readEncoderM2, RISING);
  attachInterrupt(digitalPinToInterrupt(M3_ENCA), readEncoderM3, RISING);
  attachInterrupt(digitalPinToInterrupt(M4_ENCA), readEncoderM4, RISING);

  Serial.println("Ready");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    char cmd = toupper(input[0]);

    if (cmd == 'E') {
      eStop = true;
      stopMotors();
    } 
    else if (cmd == 'X') {
      eStop = false;
    }
    else if (cmd == 'P') {
      int speedVal = input.substring(1).toInt();
      if (speedVal >= 0 && speedVal <= 255) {
        currentSpeed = speedVal;
      }
    }
    else if (cmd == '?') {
      // Send Status to Python script: "STS: speed,eStop,enc1,enc2,enc3,enc4"
      Serial.print("STS:");
      Serial.print(currentSpeed);
      Serial.print(",");
      Serial.print(eStop ? "1" : "0");
      Serial.print(",");
      Serial.print(countM1); Serial.print(",");
      Serial.print(countM2); Serial.print(",");
      Serial.print(countM3); Serial.print(",");
      Serial.println(countM4);
    }
    else if (!eStop) {
      if (cmd == 'F') moveForward(currentSpeed);
      else if (cmd == 'B') moveBackward(currentSpeed);
      else if (cmd == 'L') turnLeft(currentSpeed);
      else if (cmd == 'R') turnRight(currentSpeed);
      else if (cmd == 'S') stopMotors();
    }
  }
}

// --- Movement Functions ---
void moveForward(int speed) {
  analogWrite(ENA, speed); analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void moveBackward(int speed) {
  analogWrite(ENA, speed); analogWrite(ENB, speed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void turnLeft(int speed) {
  analogWrite(ENA, speed); analogWrite(ENB, speed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnRight(int speed) {
  analogWrite(ENA, speed); analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
