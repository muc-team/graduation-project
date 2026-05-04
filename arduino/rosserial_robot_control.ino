#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>

// ==========================================
// 1. Motor Driver Pins (L298N)
// ==========================================
#define ENA 10
#define IN1 8
#define IN2 9

#define ENB 11
#define IN3 12
#define IN4 13

// ==========================================
// 2. Encoder Pins definition (Mega 2560)
// ==========================================
#define M1_ENCA 2   // Front Left
#define M1_ENCB 22
#define M2_ENCA 3   // Rear Left
#define M2_ENCB 24
#define M3_ENCA 18  // Front Right
#define M3_ENCB 26
#define M4_ENCA 19  // Rear Right
#define M4_ENCB 28

// ==========================================
// 3. Encoder Counters 
// ==========================================
volatile long countM1 = 0;
volatile long countM2 = 0;
volatile long countM3 = 0;
volatile long countM4 = 0;

// ==========================================
// 4. Interrupt Service Routines (ISRs)
// ==========================================
void readEncoderM1() { if (digitalRead(M1_ENCB) > 0) countM1++; else countM1--; }
void readEncoderM2() { if (digitalRead(M2_ENCB) > 0) countM2++; else countM2--; }
void readEncoderM3() { if (digitalRead(M3_ENCB) > 0) countM3++; else countM3--; }
void readEncoderM4() { if (digitalRead(M4_ENCB) > 0) countM4++; else countM4--; }

// ==========================================
// 5. ROS Setup
// ==========================================
ros::NodeHandle nh;

// Array to hold encoder data for publishing
std_msgs::Int32MultiArray enc_msg;
ros::Publisher pub_enc("/encoders", &enc_msg);

// Failsafe / Timeout variables
unsigned long lastCmdTime = 0;
const unsigned long TIMEOUT_MS = 1000; // Stop motors if no command received for 1 sec

// ==========================================
// 6. Motor Control Functions
// ==========================================
void setLeftMotors(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setRightMotors(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopAllMotors() {
  setLeftMotors(0);
  setRightMotors(0);
}

// ==========================================
// 7. ROS Command Callback (Listens to dashboard)
// ==========================================
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  lastCmdTime = millis(); // Reset timeout timer
  
  float linear = msg.linear.x;   // Forward/Backward
  float angular = msg.angular.z; // Left/Right Turn

  // Mix linear and angular to get left and right speeds (Differential Drive)
  // Multiply by 255 assuming the dashboard sends values from -1.0 to 1.0
  float left_speed = (linear - angular) * 255.0;
  float right_speed = (linear + angular) * 255.0;

  // Constrain speeds to valid PWM range (-255 to 255)
  int left_pwm = constrain((int)left_speed, -255, 255);
  int right_pwm = constrain((int)right_speed, -255, 255);

  setLeftMotors(left_pwm);
  setRightMotors(right_pwm);
}

// Subscriber listening exactly to the topic your dashboard sends
ros::Subscriber<geometry_msgs::Twist> sub_cmd("/manual_cmd", cmdVelCallback);

// ==========================================
// 8. Main Setup
// ==========================================
void setup() {
  // Setup Motor Pins
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  stopAllMotors();

  // Setup Encoder Pins
  pinMode(M1_ENCA, INPUT_PULLUP); pinMode(M1_ENCB, INPUT_PULLUP);
  pinMode(M2_ENCA, INPUT_PULLUP); pinMode(M2_ENCB, INPUT_PULLUP);
  pinMode(M3_ENCA, INPUT_PULLUP); pinMode(M3_ENCB, INPUT_PULLUP);
  pinMode(M4_ENCA, INPUT_PULLUP); pinMode(M4_ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENCA), readEncoderM1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENCA), readEncoderM2, RISING);
  attachInterrupt(digitalPinToInterrupt(M3_ENCA), readEncoderM3, RISING);
  attachInterrupt(digitalPinToInterrupt(M4_ENCA), readEncoderM4, RISING);

  // Initialize ROS Node (Baud rate is 57600 by default for rosserial)
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  
  nh.subscribe(sub_cmd);
  nh.advertise(pub_enc);

  // Prepare encoder array memory
  enc_msg.data_length = 4;
  enc_msg.data = (int32_t*)malloc(sizeof(int32_t) * 4);
}

// ==========================================
// 9. Main Loop
// ==========================================
unsigned long lastPublishTime = 0;

void loop() {
  unsigned long currentTime = millis();

  // Failsafe: Stop motors if we lose connection to the Raspberry Pi for > 1 sec
  // This prevents the robot from driving into a wall if WiFi or serial drops
  if (currentTime - lastCmdTime > TIMEOUT_MS) {
    stopAllMotors();
  }

  // Publish Encoder Data every 100ms (10 Hz)
  if (currentTime - lastPublishTime >= 100) {
    enc_msg.data[0] = countM1;
    enc_msg.data[1] = countM2;
    enc_msg.data[2] = countM3;
    enc_msg.data[3] = countM4;
    pub_enc.publish(&enc_msg);
    
    lastPublishTime = currentTime;
  }

  // Process incoming ROS messages and keep connection alive
  nh.spinOnce();
  delay(10);
}
