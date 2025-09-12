/*
 * ESP32 Battery Monitor & Differential Motor Control
 * 
 * Funzionalità:
 * - Monitoraggio corrente/tensione 3 pacchi batterie (2x6S + 1x4S)
 * - Controllo PWM differenziale per 2 motori
 * - Interfaccia autopilota (2 PWM input + 2 digital direction)
 * - Telemetria via Serial/WiFi
 * 
 * Pin ESP32 DEVKIT V1:
 * ADC Input: GPIO32,33,34,35,36,39
 * PWM Input: GPIO18,19 (Autopilota)
 * Digital Input: GPIO17,21 (Direction)
 * PWM Output: GPIO22,23 (Motori)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ============================================================================
// CONFIGURAZIONE PIN
// ============================================================================

// ADC per Sensori Corrente (ACS758)
#define CURRENT_6S1_PIN    32    // GPIO32 - ADC1_CH4
#define CURRENT_6S2_PIN    33    // GPIO33 - ADC1_CH5  
#define CURRENT_4S_PIN     34    // GPIO34 - ADC1_CH6

// ADC per Sensori Tensione (Partitori)
#define VOLTAGE_6S1_PIN    35    // GPIO35 - ADC1_CH7
#define VOLTAGE_6S2_PIN    36    // GPIO36 - ADC1_CH0
#define VOLTAGE_4S_PIN     39    // GPIO39 - ADC1_CH3

// PWM Input dall'Autopilota
#define PWM_IN_THROTTLE    18    // GPIO18 - Throttle
#define PWM_IN_STEERING    19    // GPIO19 - Steering

// Digital Input per Direzione
#define DIR_RIGHT_PIN      21    // GPIO21 - Direzione Motore Destro
#define DIR_LEFT_PIN       17    // GPIO17 - Direzione Motore Sinistro

// PWM Output per Motori
#define PWM_OUT_RIGHT      22    // GPIO22 - Motore Destro
#define PWM_OUT_LEFT       23    // GPIO23 - Motore Sinistro

// ============================================================================
// COSTANTI E PARAMETRI
// ============================================================================

// Parametri ADC
#define ADC_RESOLUTION     4095.0
#define ADC_VREF           3.3
#define ADC_SAMPLES        64

// Parametri Sensori Corrente ACS758-50A
#define ACS758_SENSITIVITY 0.04  // 40mV/A per modello 50A
#define ACS758_VREF        2.5   // Tensione di riferimento (VCC/2)

// Parametri Partitori Tensione
#define DIVIDER_6S_RATIO   8.4   // 25.2V -> 3.0V
#define DIVIDER_4S_RATIO   5.6   // 16.8V -> 3.0V

// Parametri PWM
#define PWM_FREQ           50    // 50Hz per ESC/Servo
#define PWM_RESOLUTION     16    // 16-bit resolution
#define PWM_MIN            1000  // 1000μs
#define PWM_MAX            2000  // 2000μs
#define PWM_CENTER         1500  // 1500μs

// Parametri Sistema
#define TELEMETRY_INTERVAL 100   // ms
#define PWM_READ_TIMEOUT   25000 // μs timeout per lettura PWM

// ============================================================================
// VARIABILI GLOBALI
// ============================================================================

// Strutture Dati
struct BatteryData {
  float current;      // A
  float voltage;      // V
  float power;        // W
  float capacity;     // Ah (stimata)
};

struct PWMData {
  uint16_t throttle;  // 1000-2000μs
  uint16_t steering;  // 1000-2000μs
  bool dir_right;     // true=forward, false=reverse
  bool dir_left;      // true=forward, false=reverse
};

struct MotorOutput {
  uint16_t right_pwm; // 1000-2000μs
  uint16_t left_pwm;  // 1000-2000μs
};

// Dati Sistema
BatteryData batteries[3];  // 0=6S#1, 1=6S#2, 2=4S
PWMData autopilot_input;
MotorOutput motor_output;

// Statistiche
unsigned long last_telemetry = 0;
unsigned long loop_count = 0;
float loop_frequency = 0.0;

// WiFi e Web Server
const char* ssid = "ESP32_BatteryMonitor";
const char* password = "battery123";
WebServer server(80);

// ============================================================================
// FUNZIONI UTILITY
// ============================================================================

// Lettura ADC con media mobile
float readADC(int pin, int samples = ADC_SAMPLES) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  return (float)sum / samples;
}

// Conversione ADC a Tensione
float adcToVoltage(float adc_value) {
  return (adc_value / ADC_RESOLUTION) * ADC_VREF;
}

// Lettura Corrente da ACS758
float readCurrent(int pin) {
  float adc_value = readADC(pin);
  float voltage = adcToVoltage(adc_value);
  float current = (voltage - ACS758_VREF) / ACS758_SENSITIVITY;
  return current;
}

// Lettura Tensione da Partitore
float readVoltage(int pin, float divider_ratio) {
  float adc_value = readADC(pin);
  float voltage = adcToVoltage(adc_value);
  return voltage * divider_ratio;
}

// Lettura PWM Input
uint16_t readPWM(int pin) {
  unsigned long pulse_width = pulseIn(pin, HIGH, PWM_READ_TIMEOUT);
  if (pulse_width == 0) return PWM_CENTER; // Default se nessun segnale
  return constrain(pulse_width, PWM_MIN, PWM_MAX);
}

// Scrittura PWM Output
void writePWM(int pin, uint16_t pulse_width) {
  uint32_t duty = map(pulse_width, PWM_MIN, PWM_MAX, 0, 65535);
  ledcWrite(pin, duty);
}

// ============================================================================
// FUNZIONI PRINCIPALI
// ============================================================================

void readBatteryData() {
  // Lettura Correnti
  batteries[0].current = readCurrent(CURRENT_6S1_PIN);
  batteries[1].current = readCurrent(CURRENT_6S2_PIN);
  batteries[2].current = readCurrent(CURRENT_4S_PIN);
  
  // Lettura Tensioni
  batteries[0].voltage = readVoltage(VOLTAGE_6S1_PIN, DIVIDER_6S_RATIO);
  batteries[1].voltage = readVoltage(VOLTAGE_6S2_PIN, DIVIDER_6S_RATIO);
  batteries[2].voltage = readVoltage(VOLTAGE_4S_PIN, DIVIDER_4S_RATIO);
  
  // Calcolo Potenze
  for (int i = 0; i < 3; i++) {
    batteries[i].power = batteries[i].voltage * batteries[i].current;
  }
}

void readAutopilotInput() {
  autopilot_input.throttle = readPWM(PWM_IN_THROTTLE);
  autopilot_input.steering = readPWM(PWM_IN_STEERING);
  autopilot_input.dir_right = digitalRead(DIR_RIGHT_PIN);
  autopilot_input.dir_left = digitalRead(DIR_LEFT_PIN);
}

void calculateMotorOutput() {
  // Calcolo velocità base dal throttle
  float throttle_factor = (float)(autopilot_input.throttle - PWM_CENTER) / (PWM_CENTER - PWM_MIN);
  
  // Calcolo differenziale dal steering
  float steering_factor = (float)(autopilot_input.steering - PWM_CENTER) / (PWM_CENTER - PWM_MIN);
  
  // Mixing differenziale
  float right_speed = throttle_factor + (steering_factor * 0.5);
  float left_speed = throttle_factor - (steering_factor * 0.5);
  
  // Limitazione
  right_speed = constrain(right_speed, -1.0, 1.0);
  left_speed = constrain(left_speed, -1.0, 1.0);
  
  // Conversione a PWM
  uint16_t right_pwm = PWM_CENTER + (right_speed * (PWM_CENTER - PWM_MIN));
  uint16_t left_pwm = PWM_CENTER + (left_speed * (PWM_CENTER - PWM_MIN));
  
  // Applicazione direzione
  if (!autopilot_input.dir_right) {
    right_pwm = PWM_CENTER + (PWM_CENTER - right_pwm);
  }
  if (!autopilot_input.dir_left) {
    left_pwm = PWM_CENTER + (PWM_CENTER - left_pwm);
  }
  
  // Limitazione finale
  motor_output.right_pwm = constrain(right_pwm, PWM_MIN, PWM_MAX);
  motor_output.left_pwm = constrain(left_pwm, PWM_MIN, PWM_MAX);
}

void updateMotorOutput() {
  writePWM(PWM_OUT_RIGHT, motor_output.right_pwm);
  writePWM(PWM_OUT_LEFT, motor_output.left_pwm);
}

// ============================================================================
// TELEMETRIA E WEB SERVER
// ============================================================================

void sendTelemetry() {
  if (millis() - last_telemetry < TELEMETRY_INTERVAL) return;
  
  // Calcolo frequenza loop
  loop_frequency = 1000.0 / (millis() - last_telemetry);
  last_telemetry = millis();
  
  // Output Serial
  Serial.println("=== TELEMETRIA ===");
  Serial.printf("Loop: %lu, Freq: %.1f Hz\n", loop_count++, loop_frequency);
  
  for (int i = 0; i < 3; i++) {
    const char* names[] = {"6S#1", "6S#2", "4S"};
    Serial.printf("Batteria %s: %.2fV, %.2fA, %.1fW\n", 
                  names[i], batteries[i].voltage, batteries[i].current, batteries[i].power);
  }
  
  Serial.printf("Autopilota: Throttle=%d, Steering=%d, DirR=%d, DirL=%d\n",
                autopilot_input.throttle, autopilot_input.steering, 
                autopilot_input.dir_right, autopilot_input.dir_left);
                
  Serial.printf("Motori: Right=%d, Left=%d\n", 
                motor_output.right_pwm, motor_output.left_pwm);
  Serial.println();
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>ESP32 Battery Monitor</title>";
  html += "<meta http-equiv='refresh' content='1'>";
  html += "<style>body{font-family:Arial;margin:20px;}";
  html += ".battery{background:#f0f0f0;padding:10px;margin:10px;border-radius:5px;}";
  html += ".status{background:#e6f3ff;padding:10px;margin:10px;border-radius:5px;}</style></head><body>";
  
  html += "<h1>🔋 ESP32 Battery Monitor</h1>";
  
  // Batterie
  const char* names[] = {"6S Battery #1", "6S Battery #2", "4S Battery"};
  for (int i = 0; i < 3; i++) {
    html += "<div class='battery'>";
    html += "<h3>" + String(names[i]) + "</h3>";
    html += "<p><strong>Tensione:</strong> " + String(batteries[i].voltage, 2) + " V</p>";
    html += "<p><strong>Corrente:</strong> " + String(batteries[i].current, 2) + " A</p>";
    html += "<p><strong>Potenza:</strong> " + String(batteries[i].power, 1) + " W</p>";
    html += "</div>";
  }
  
  // Status Sistema
  html += "<div class='status'>";
  html += "<h3>📊 Status Sistema</h3>";
  html += "<p><strong>Frequenza Loop:</strong> " + String(loop_frequency, 1) + " Hz</p>";
  html += "<p><strong>Throttle:</strong> " + String(autopilot_input.throttle) + " μs</p>";
  html += "<p><strong>Steering:</strong> " + String(autopilot_input.steering) + " μs</p>";
  html += "<p><strong>Dir Right:</strong> " + String(autopilot_input.dir_right ? "Forward" : "Reverse") + "</p>";
  html += "<p><strong>Dir Left:</strong> " + String(autopilot_input.dir_left ? "Forward" : "Reverse") + "</p>";
  html += "<p><strong>Motor Right:</strong> " + String(motor_output.right_pwm) + " μs</p>";
  html += "<p><strong>Motor Left:</strong> " + String(motor_output.left_pwm) + " μs</p>";
  html += "</div>";
  
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleAPI() {
  DynamicJsonDocument doc(1024);
  
  // Batterie
  JsonArray batteryArray = doc.createNestedArray("batteries");
  for (int i = 0; i < 3; i++) {
    JsonObject battery = batteryArray.createNestedObject();
    battery["voltage"] = batteries[i].voltage;
    battery["current"] = batteries[i].current;
    battery["power"] = batteries[i].power;
  }
  
  // Autopilota
  JsonObject autopilot = doc.createNestedObject("autopilot");
  autopilot["throttle"] = autopilot_input.throttle;
  autopilot["steering"] = autopilot_input.steering;
  autopilot["dir_right"] = autopilot_input.dir_right;
  autopilot["dir_left"] = autopilot_input.dir_left;
  
  // Motori
  JsonObject motors = doc.createNestedObject("motors");
  motors["right_pwm"] = motor_output.right_pwm;
  motors["left_pwm"] = motor_output.left_pwm;
  
  // Sistema
  doc["loop_frequency"] = loop_frequency;
  doc["uptime"] = millis();
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// ============================================================================
// SETUP E LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("🚀 ESP32 Battery Monitor & Motor Control");
  Serial.println("========================================");
  
  // Configurazione Pin
  pinMode(DIR_RIGHT_PIN, INPUT_PULLUP);
  pinMode(DIR_LEFT_PIN, INPUT_PULLUP);
  
  // Configurazione ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // 0-3.3V range
  
  // Configurazione PWM Output
  ledcSetup(PWM_OUT_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_OUT_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_OUT_RIGHT, PWM_OUT_RIGHT);
  ledcAttachPin(PWM_OUT_LEFT, PWM_OUT_LEFT);
  
  // Inizializzazione PWM Output
  writePWM(PWM_OUT_RIGHT, PWM_CENTER);
  writePWM(PWM_OUT_LEFT, PWM_CENTER);
  
  // WiFi Access Point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.printf("📡 WiFi AP: %s\n", ssid);
  Serial.printf("🌐 IP: %s\n", IP.toString().c_str());
  
  // Web Server
  server.on("/", handleRoot);
  server.on("/api", handleAPI);
  server.begin();
  Serial.println("🌍 Web Server avviato");
  
  Serial.println("✅ Sistema inizializzato!");
  Serial.println("📊 Telemetria ogni " + String(TELEMETRY_INTERVAL) + "ms");
  Serial.println("🔗 Web Interface: http://" + IP.toString());
  Serial.println();
}

void loop() {
  // Lettura dati
  readBatteryData();
  readAutopilotInput();
  
  // Calcolo output motori
  calculateMotorOutput();
  updateMotorOutput();
  
  // Telemetria
  sendTelemetry();
  
  // Web Server
  server.handleClient();
  
  // Piccola pausa per stabilità
  delay(1);
}
