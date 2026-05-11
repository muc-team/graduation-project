#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ==========================================
// 1. WiFi Configuration
// ==========================================
const char* ssid = "Sonnet";
const char* password = "Eng_Matouk_HelloSonnet"; 

WebServer server(80);

// ==========================================
// 2. PIN DEFINITIONS (Verified Working)
// ==========================================
// --- Motor Side A (Left) ---
#define ENA 13
#define IN1 32
#define IN2 33

// --- Motor Side B (Right) ---
#define ENB 25
#define IN3 27
#define IN4 26

// --- Sensors & Buzzer ---
#define TRIG_PIN 18
#define ECHO_PIN 5
#define GAS_PIN 34     
#define BUZZER_PIN 15  

// ==========================================
// 3. Global Variables
// ==========================================
Adafruit_MPU6050 mpu;
int driveSpeed = 230; 
float dist_cm = 0;
int gas_val = 0;
int gas_limit = 3000; // UPDATED: High threshold for serious gas detection
float ax = 0, ay = 0;

unsigned long last_update = 0;

// ==========================================
// 4. English Dashboard UI
// ==========================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Swarm Node Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: 'Segoe UI', sans-serif; background: #121212; color: #eee; text-align: center; }
    .status-card { background: #1e1e1e; padding: 20px; border-radius: 15px; margin: 15px auto; width: 85%; max-width: 350px; border: 1px solid #333; }
    .val { font-size: 28px; color: #00ffcc; font-weight: bold; }
    .alarm { color: #ff3e3e; font-weight: bold; animation: blink 1s infinite; }
    @keyframes blink { 50% { opacity: 0; } }
    .btn { background: #007bff; border: none; color: #fff; padding: 18px; font-size: 20px; border-radius: 12px; margin: 8px; width: 110px; cursor: pointer; }
    .btn:active { background: #0056b3; transform: scale(0.95); }
    .stop-btn { background: #dc3545; width: 130px; }
  </style>
</head>
<body>
  <h1>🛰️ SWARM NODE V1</h1>
  <div class="status-card">
    <p>Ultrasonic: <span id="dist" class="val">0</span> cm</p>
    <p>Gas Level: <span id="gas" class="val">0</span> <span id="warn"></span></p>
    <p>Tilt X: <span id="tx">0</span> | Y: <span id="ty">0</span></p>
  </div>
  <div class="status-card">
    <button class="btn" onmousedown="cmd('F')" onmouseup="cmd('S')">UP</button><br>
    <button class="btn" onmousedown="cmd('L')" onmouseup="cmd('S')">LEFT</button>
    <button class="btn stop-btn" onclick="cmd('S')">STOP</button>
    <button class="btn" onmousedown="cmd('R')" onmouseup="cmd('S')">RIGHT</button><br>
    <button class="btn" onmousedown="cmd('B')" onmouseup="cmd('S')">DOWN</button>
  </div>
  <script>
    setInterval(() => {
      fetch('/telemetry').then(r => r.json()).then(data => {
        document.getElementById('dist').innerText = data.d;
        document.getElementById('gas').innerText = data.g;
        document.getElementById('tx').innerText = data.x;
        document.getElementById('ty').innerText = data.y;
        document.getElementById('warn').innerText = data.a ? " (DANGER!)" : " (OK)";
        document.getElementById('warn').className = data.a ? "alarm" : "";
      });
    }, 500);
    function cmd(dir) { fetch('/control?dir=' + dir); }
  </script>
</body>
</html>
)rawliteral";

// ==========================================
// 5. Navigation Logic
// ==========================================
void stop() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void forward() {
  analogWrite(ENA, driveSpeed); analogWrite(ENB, driveSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void backward() {
  analogWrite(ENA, driveSpeed); analogWrite(ENB, driveSpeed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void left() {
  analogWrite(ENA, driveSpeed); analogWrite(ENB, driveSpeed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void right() {
  analogWrite(ENA, driveSpeed); analogWrite(ENB, driveSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

// ==========================================
// 6. Setup
// ==========================================
void setup() {
  Serial.begin(115200);
  
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  // Startup Test Beep
  digitalWrite(BUZZER_PIN, HIGH); delay(200); digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH); delay(200); digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(21, 22);
  mpu.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Ready! IP: " + WiFi.localIP().toString());

  server.on("/", []() { server.send(200, "text/html", INDEX_HTML); });
  server.on("/telemetry", []() {
    String json = "{\"d\":" + String(dist_cm) + ",\"g\":" + String(gas_val) + 
                  ",\"x\":" + String(ax) + ",\"y\":" + String(ay) + 
                  ",\"a\":" + String(gas_val > gas_limit ? 1 : 0) + "}";
    server.send(200, "application/json", json);
  });
  server.on("/control", []() {
    String d = server.arg("dir");
    if (d == "F") forward(); else if (d == "B") backward();
    else if (d == "L") left(); else if (d == "R") right();
    else stop();
    server.send(200, "text/plain", "OK");
  });
  server.begin();
}

// ==========================================
// 7. Loop
// ==========================================
void loop() {
  server.handleClient();
  
  if (millis() - last_update > 250) {
    // Distance
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    dist_cm = pulseIn(ECHO_PIN, HIGH, 30000) * 0.034 / 2;

    // Gas & Alarm Logic
    gas_val = analogRead(GAS_PIN);
    // Buzzer will only turn ON if gas level exceeds 3000
    digitalWrite(BUZZER_PIN, (gas_val > gas_limit) ? HIGH : LOW);

    // MPU6050
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    ax = a.acceleration.x; ay = a.acceleration.y;

    // Debugging Output
    Serial.printf("Distance: %.1f cm | Gas: %d | Buzzer: %s\n", dist_cm, gas_val, (gas_val > gas_limit ? "ALARM" : "OK"));
    last_update = millis();
  }
}