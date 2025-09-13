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
#define PWM_IN_RIGHT       18    // GPIO18 - Motore Destro
#define PWM_IN_LEFT        19    // GPIO19 - Motore Sinistro

// Digital Input per Direzione
#define DIR_RIGHT_PIN      21    // GPIO21 - Direzione Motore Destro
#define DIR_LEFT_PIN       17    // GPIO17 - Direzione Motore Sinistro

// PWM Output per Motori
#define PWM_OUT_RIGHT      26    // GPIO25 - Motore Destro
#define PWM_OUT_LEFT       27    // GPIO26     - Motore Sinistro

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
// CONFIGURAZIONE LEDC CHANNELS
// ============================================================================
#define PWM_OUT_RIGHT_CHANNEL  0    // LEDC Channel 0
#define PWM_OUT_LEFT_CHANNEL   1    // LEDC Channel 1

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
  uint16_t motor_right;  // 1000-2000μs
  uint16_t motor_left;   // 1000-2000μs
  bool dir_right;        // true=forward, false=reverse
  bool dir_left;         // true=forward, false=reverse
};

struct MotorOutput {
  uint16_t right_pwm; // 1000-2000μs
  uint16_t left_pwm;  // 1000-2000μs
};

// Strutture per Taratura
struct CalibrationData {
  float voltage_offset;     // Offset tensione
  float voltage_scale;      // Scala tensione
  float current_offset;     // Offset corrente
  float current_scale;      // Scala corrente
  float divider_ratio;      // Rapporto partitore
};

// Strutture per Grafici (dati storici)
struct ChartData {
  float values[600];        // 600 punti (10 minuti a 1Hz)
  int index;                // Indice corrente
  bool filled;              // Buffer riempito
  unsigned long last_update; // Ultimo aggiornamento
};

// Dati Sistema
BatteryData batteries[3];  // 0=6S#1, 1=6S#2, 2=4S
PWMData autopilot_input;
MotorOutput motor_output;

// Dati Taratura
CalibrationData calibration[3];  // Taratura per ogni batteria

// Dati Grafici
ChartData voltage_charts[3];     // Grafici tensione
ChartData current_charts[3];     // Grafici corrente

// Statistiche
unsigned long last_telemetry = 0;
unsigned long last_chart_update = 0;
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


// Lettura Tensione da Partitore (con taratura)
float readVoltage(int pin, int battery_index) {
  float adc_value = readADC(pin);
  float voltage = adcToVoltage(adc_value);
  float raw_voltage = voltage * calibration[battery_index].divider_ratio;
  // Applica taratura: (valore_raw + offset) * scala
  return (raw_voltage + calibration[battery_index].voltage_offset) * calibration[battery_index].voltage_scale;
}

// Lettura Corrente da ACS758 (con taratura)
float readCurrent(int pin, int battery_index) {
  float adc_value = readADC(pin);
  float voltage = adcToVoltage(adc_value);
  float raw_current = (voltage - ACS758_VREF) / ACS758_SENSITIVITY;
  // Applica taratura: (valore_raw + offset) * scala
  return (raw_current + calibration[battery_index].current_offset) * calibration[battery_index].current_scale;
}

// Lettura PWM Input
uint16_t readPWM(int pin) {
  unsigned long pulse_width = pulseIn(pin, HIGH, PWM_READ_TIMEOUT);
  if (pulse_width == 0) return PWM_CENTER; // Default se nessun segnale
  return constrain(pulse_width, PWM_MIN, PWM_MAX);
}

// Scrittura PWM Output - CORRETTA PER 1000-2000μs
void writePWM(int pin, uint16_t pulse_width) {
  // Calcola duty cycle per impulsi da 1000-2000μs a 50Hz
  // Periodo = 20ms = 20,000μs
  // Duty cycle = (pulse_width / 20000) * 4095 (12-bit resolution)
  uint16_t duty = (pulse_width * 4095) / 20000;
  
  if (pin == PWM_OUT_RIGHT) {
    ledcWrite(PWM_OUT_RIGHT_CHANNEL, duty);
  } else if (pin == PWM_OUT_LEFT) {
    ledcWrite(PWM_OUT_LEFT_CHANNEL, duty);
  }
}

// ============================================================================
// FUNZIONI GRAFICI E TARATURA
// ============================================================================

// Inizializza grafico
void initChart(ChartData* chart) {
  chart->index = 0;
  chart->filled = false;
  chart->last_update = 0;
  for (int i = 0; i < 600; i++) {
    chart->values[i] = 0.0;
  }
}

// Aggiungi valore al grafico
void addToChart(ChartData* chart, float value) {
  chart->values[chart->index] = value;
  chart->index++;
  if (chart->index >= 600) {
    chart->index = 0;
    chart->filled = true;
  }
}

// Inizializza taratura con valori di default
void initCalibration() {
  for (int i = 0; i < 3; i++) {
    calibration[i].voltage_offset = 0.0;
    calibration[i].voltage_scale = 1.0;
    calibration[i].current_offset = 0.0;
    calibration[i].current_scale = 1.0;
    
    // Valori di default per partitori
    if (i < 2) { // 6S batteries
      calibration[i].divider_ratio = DIVIDER_6S_RATIO;
    } else { // 4S battery
      calibration[i].divider_ratio = DIVIDER_4S_RATIO;
    }
  }
}

// Aggiorna grafici
void updateCharts() {
  if (millis() - last_chart_update < 10000) return; // Aggiorna ogni 10 secondi
  
  for (int i = 0; i < 3; i++) {
    // Aggiungi dati reali o di test
    float test_voltage = batteries[i].voltage;
    float test_current = batteries[i].current;
    
    // Se i dati sono zero, aggiungi dati di test per verificare i grafici
    if (test_voltage < 0.1) {
      test_voltage = 20.0 + i * 2.0 + sin(millis() / 1000.0 + i) * 2.0;
    }
    if (test_current < 0.1) {
      test_current = 5.0 + i * 1.0 + cos(millis() / 2000.0 + i) * 1.5;
    }
    
    addToChart(&voltage_charts[i], test_voltage);
    addToChart(&current_charts[i], test_current);
  }
  
  last_chart_update = millis();
}

// ============================================================================
// FUNZIONI PRINCIPALI
// ============================================================================

void readBatteryData() {
  // Lettura Correnti (con taratura)
  batteries[0].current = readCurrent(CURRENT_6S1_PIN, 0);
  batteries[1].current = readCurrent(CURRENT_6S2_PIN, 1);
  batteries[2].current = readCurrent(CURRENT_4S_PIN, 2);
  
  // Lettura Tensioni (con taratura)
  batteries[0].voltage = readVoltage(VOLTAGE_6S1_PIN, 0);
  batteries[1].voltage = readVoltage(VOLTAGE_6S2_PIN, 1);
  batteries[2].voltage = readVoltage(VOLTAGE_4S_PIN, 2);
  
  // Calcolo Potenze
  for (int i = 0; i < 3; i++) {
    batteries[i].power = batteries[i].voltage * batteries[i].current;
  }
}

void readAutopilotInput() {
  autopilot_input.motor_right = readPWM(PWM_IN_RIGHT);
  autopilot_input.motor_left = readPWM(PWM_IN_LEFT);
  autopilot_input.dir_right = digitalRead(DIR_RIGHT_PIN);
  autopilot_input.dir_left = digitalRead(DIR_LEFT_PIN);
}

void calculateMotorOutput() {
  // Input diretti per motore destro e sinistro
  uint16_t right_input = autopilot_input.motor_right;
  uint16_t left_input = autopilot_input.motor_left;
  
  // Mappatura corretta: 1500 = neutro (1000 output)
  // 1000-1500: mappa a 2000-1000 (inverso), direzione BACKWARD
  // 1500-2000: mappa a 1000-2000 (normale), direzione FORWARD
  // Per motore destro
  if (right_input <= PWM_CENTER) {
    // 1000-1500: mappa a 2000-1000 (inverso), direzione BACKWARD
    motor_output.right_pwm = map(right_input, PWM_MIN, PWM_CENTER, PWM_MAX, PWM_MIN);
    digitalWrite(DIR_RIGHT_PIN, LOW);  // BACKWARD
  } else {
    // 1500-2000: mappa a 1000-2000 (normale), direzione FORWARD
    motor_output.right_pwm = map(right_input, PWM_CENTER, PWM_MAX, PWM_MIN, PWM_MAX);
    digitalWrite(DIR_RIGHT_PIN, HIGH); // FORWARD
  }
  
  // Per motore sinistro
  if (left_input <= PWM_CENTER) {
    // 1000-1500: mappa a 2000-1000 (inverso), direzione BACKWARD
    motor_output.left_pwm = map(left_input, PWM_MIN, PWM_CENTER, PWM_MAX, PWM_MIN);
    digitalWrite(DIR_LEFT_PIN, LOW);   // BACKWARD
  } else {
    // 1500-2000: mappa a 1000-2000 (normale), direzione FORWARD
    motor_output.left_pwm = map(left_input, PWM_CENTER, PWM_MAX, PWM_MIN, PWM_MAX);
    digitalWrite(DIR_LEFT_PIN, HIGH);  // FORWARD
  }
  
  // Limitazione finale
  motor_output.right_pwm = constrain(motor_output.right_pwm, PWM_MIN, PWM_MAX);
  motor_output.left_pwm = constrain(motor_output.left_pwm, PWM_MIN, PWM_MAX);
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
  
  Serial.printf("Autopilota: MotorRight=%d, MotorLeft=%d, DirR=%d, DirL=%d\n",
                autopilot_input.motor_right, autopilot_input.motor_left, 
                autopilot_input.dir_right, autopilot_input.dir_left);
                
  Serial.printf("Motori: Right=%d, Left=%d\n", 
                motor_output.right_pwm, motor_output.left_pwm);
  Serial.println();
}

// ============================================================================
// FUNZIONI WEB SERVER - TARATURA
// ============================================================================

void handleCalibration() {
  if (server.method() == HTTP_POST) {
    // Ricevi dati di taratura
    String body = server.arg("plain");
    DynamicJsonDocument doc(512);
    deserializeJson(doc, body);
    
    int battery_index = doc["battery"];
    String type = doc["type"];
    
    if (type == "voltage") {
      calibration[battery_index].voltage_offset = doc["offset"];
      calibration[battery_index].voltage_scale = doc["scale"];
      calibration[battery_index].divider_ratio = doc["divider_ratio"];
    } else if (type == "current") {
      calibration[battery_index].current_offset = doc["offset"];
      calibration[battery_index].current_scale = doc["scale"];
    }
    
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  } else {
    // Restituisci dati di taratura attuali
    DynamicJsonDocument doc(1024);
    for (int i = 0; i < 3; i++) {
      JsonObject cal = doc.createNestedObject("battery_" + String(i));
      cal["voltage_offset"] = calibration[i].voltage_offset;
      cal["voltage_scale"] = calibration[i].voltage_scale;
      cal["current_offset"] = calibration[i].current_offset;
      cal["current_scale"] = calibration[i].current_scale;
      cal["divider_ratio"] = calibration[i].divider_ratio;
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  }
}

void handleCharts() {
  DynamicJsonDocument doc(4096);
  
  // Ottieni parametri query
  String scale = server.arg("scale");
  int points = 60; // Default 10 minuti
  
  if (scale == "10s") points = 1;      // 10 secondi
  else if (scale == "30s") points = 3; // 30 secondi  
  else if (scale == "1m") points = 6;  // 1 minuto
  else if (scale == "5m") points = 30; // 5 minuti
  else if (scale == "10m") points = 60; // 10 minuti
  
  // Dati grafici tensione
  JsonArray voltageArray = doc.createNestedArray("voltage");
  for (int i = 0; i < 3; i++) {
    JsonArray batteryArray = voltageArray.createNestedArray();
    int start = voltage_charts[i].filled ? voltage_charts[i].index : 0;
    int total_points = voltage_charts[i].filled ? 600 : voltage_charts[i].index;
    int step = max(1, total_points / points); // Campionamento per ridurre punti
    
    for (int j = 0; j < points && j * step < total_points; j++) {
      int idx = (start + j * step) % 600;
      batteryArray.add(voltage_charts[i].values[idx]);
    }
  }
  
  // Dati grafici corrente
  JsonArray currentArray = doc.createNestedArray("current");
  for (int i = 0; i < 3; i++) {
    JsonArray batteryArray = currentArray.createNestedArray();
    int start = current_charts[i].filled ? current_charts[i].index : 0;
    int total_points = current_charts[i].filled ? 600 : current_charts[i].index;
    int step = max(1, total_points / points);
    
    for (int j = 0; j < points && j * step < total_points; j++) {
      int idx = (start + j * step) % 600;
      batteryArray.add(current_charts[i].values[idx]);
    }
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleCSV() {
  String csv = "Timestamp,6S1_Voltage,6S1_Current,6S2_Voltage,6S2_Current,4S_Voltage,4S_Current\n";
  
  // Genera timestamp e dati CSV
  for (int i = 0; i < 600; i++) {
    int idx = (voltage_charts[0].filled ? voltage_charts[0].index : 0 + i) % 600;
    
    // Timestamp (secondi dall'inizio)
    csv += String(i * 10) + ",";
    
    // Dati batterie
    csv += String(voltage_charts[0].values[idx], 2) + ",";
    csv += String(current_charts[0].values[idx], 2) + ",";
    csv += String(voltage_charts[1].values[idx], 2) + ",";
    csv += String(current_charts[1].values[idx], 2) + ",";
    csv += String(voltage_charts[2].values[idx], 2) + ",";
    csv += String(current_charts[2].values[idx], 2) + "\n";
  }
  
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=battery_data.csv");
  server.send(200, "text/csv", csv);
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>ESP32 Battery Monitor</title>";
  html += "<style>";
  html += "*{box-sizing:border-box}body{margin:0;padding:16px;font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;background:#0b1220;color:#e6edf3}";
  html += ".container{max-width:1000px;margin:0 auto}";
  html += "h1{font-size:22px;margin:0 0 16px;color:#c9d1d9}";
  html += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px}";
  html += ".card{background:#111827;border:1px solid #1f2937;border-radius:10px;padding:12px;box-shadow:0 2px 8px rgba(0,0,0,.25)}";
  html += ".card h3{margin:0 0 8px;font-size:16px;color:#e5e7eb}";
  html += ".row{display:flex;justify-content:space-between;margin:6px 0;font-size:14px;color:#cbd5e1}";
  html += ".muted{color:#94a3b8}";
  html += ".status-badges{display:flex;flex-wrap:wrap;gap:8px;margin-top:8px}";
  html += ".badge{display:inline-flex;align-items:center;gap:6px;padding:4px 8px;border-radius:999px;background:#0f172a;border:1px solid #1f2937;font-size:12px;color:#cbd5e1}";
  html += ".ok{color:#10b981}.warn{color:#f59e0b}.err{color:#ef4444}";
  html += ".footer{margin-top:14px;font-size:12px;color:#94a3b8}";
  html += "a{color:#60a5fa;text-decoration:none}";
  html += "</style></head><body><div class='container'>";
  html += "<h1>🔋 ESP32 Battery Monitor - TEST</h1>";
  html += "<div class='grid'>";
  html += "<div class='card' id='b0'><h3>6S Battery #1</h3><div class='row'><span class='muted'>Tensione</span><strong><span id='b0v'>-</span> V</strong></div><div class='row'><span class='muted'>Corrente</span><strong><span id='b0c'>-</span> A</strong></div><div class='row'><span class='muted'>Potenza</span><strong><span id='b0p'>-</span> W</strong></div></div>";
  html += "<div class='card' id='b1'><h3>6S Battery #2</h3><div class='row'><span class='muted'>Tensione</span><strong><span id='b1v'>-</span> V</strong></div><div class='row'><span class='muted'>Corrente</span><strong><span id='b1c'>-</span> A</strong></div><div class='row'><span class='muted'>Potenza</span><strong><span id='b1p'>-</span> W</strong></div></div>";
  html += "<div class='card' id='b2'><h3>4S Battery</h3><div class='row'><span class='muted'>Tensione</span><strong><span id='b2v'>-</span> V</strong></div><div class='row'><span class='muted'>Corrente</span><strong><span id='b2c'>-</span> A</strong></div><div class='row'><span class='muted'>Potenza</span><strong><span id='b2p'>-</span> W</strong></div></div>";
  html += "<div class='card'><h3>Status Sistema</h3><div class='row'><span class='muted'>Frequenza Loop</span><strong><span id='lf'>-</span> Hz</strong></div><div class='row'><span class='muted'>Input Right</span><strong><span id='ir'>-</span> μs</strong></div><div class='row'><span class='muted'>Input Left</span><strong><span id='il'>-</span> μs</strong></div><div class='status-badges'><span class='badge' id='dr'><span>Dir Right</span><strong>-</strong></span><span class='badge' id='dl'><span>Dir Left</span><strong>-</strong></span><span class='badge'><span>Output Right</span><strong id='or'>- μs</strong></span><span class='badge'><span>Output Left</span><strong id='ol'>- μs</strong></span></div></div>";
  html += "</div>";
  html += "<div class='footer'>Aggiornamento ogni 1s via <a href='/api'>/api</a> | <a href='/calibration'>Taratura</a> | <a href='/charts'>Grafici</a></div>";
  html += "</div><script>(function(){function q(id){return document.getElementById(id)};function setText(id,val,dec){q(id).textContent=(typeof dec==='number'?Number(val).toFixed(dec):val)};function upd(d){for(var i=0;i<3;i++){setText('b'+i+'v',d.batteries[i].voltage,2);setText('b'+i+'c',d.batteries[i].current,2);setText('b'+i+'p',d.batteries[i].power,1)}setText('lf',d.loop_frequency,1);setText('ir',d.autopilot.motor_right);setText('il',d.autopilot.motor_left);var dr=d.autopilot.dir_right,dl=d.autopilot.dir_left;q('dr').className='badge '+(dr?'ok':'err');q('dr').lastElementChild.textContent=dr?'Forward':'Reverse';q('dl').className='badge '+(dl?'ok':'err');q('dl').lastElementChild.textContent=dl?'Forward':'Reverse';setText('or',d.motors.right_pwm+' μs');setText('ol',d.motors.left_pwm+' μs')}function tick(){fetch('/api',{cache:'no-store'}).then(function(r){return r.json()}).then(upd).catch(function(){}).finally(function(){setTimeout(tick,1000)})}tick()})();</script></body></html>";
  server.send(200, "text/html", html);
}

void handleCalibrationPage() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Taratura</title>";
  html += "<style>body{font-family:Arial;background:#0b1220;color:#e6edf3;padding:20px}";
  html += ".card{background:#111827;border:1px solid #1f2937;border-radius:10px;padding:16px;margin:10px 0}";
  html += "input,button{padding:8px;margin:4px;border:1px solid #1f2937;border-radius:4px;background:#0f172a;color:#e6edf3}";
  html += "button{background:#3b82f6;cursor:pointer}</style></head><body>";
  html += "<h1>⚙️ Taratura Sensori</h1>";
  for(int i = 0; i < 3; i++) {
    String batteryName = (i < 2) ? "6S Battery #" + String(i+1) : "4S Battery";
    html += "<div class='card'><h3>" + batteryName + "</h3>";
    html += "<label>Tensione Offset:</label><input type='number' id='voff" + String(i) + "' step='0.01'><br>";
    html += "<label>Tensione Scala:</label><input type='number' id='vscale" + String(i) + "' step='0.01' value='1.00'><br>";
    html += "<label>Corrente Offset:</label><input type='number' id='coff" + String(i) + "' step='0.01'><br>";
    html += "<label>Corrente Scala:</label><input type='number' id='cscale" + String(i) + "' step='0.01' value='1.00'><br>";
    html += "<label>Rapporto Partitore:</label><input type='number' id='divider" + String(i) + "' step='0.1' value='" + String((i<2)?8.4:5.6) + "'><br>";
    html += "<button onclick='saveCal(" + String(i) + ")'>💾 Salva</button></div>";
  }
  html += "<a href='/'>← Torna al Monitor</a>";
  html += "<script>function saveCal(b){";
  html += "fetch('/calibration',{method:'POST',headers:{'Content-Type':'application/json'},";
  html += "body:JSON.stringify({battery:b,type:'voltage',offset:parseFloat(document.getElementById('voff'+b).value),";
  html += "scale:parseFloat(document.getElementById('vscale'+b).value),divider_ratio:parseFloat(document.getElementById('divider'+b).value)})})";
  html += ".then(()=>alert('Taratura tensione salvata!'));";
  html += "fetch('/calibration',{method:'POST',headers:{'Content-Type':'application/json'},";
  html += "body:JSON.stringify({battery:b,type:'current',offset:parseFloat(document.getElementById('coff'+b).value),";
  html += "scale:parseFloat(document.getElementById('cscale'+b).value)})})";
  html += ".then(()=>alert('Taratura corrente salvata!'));}</script></body></html>";
  server.send(200, "text/html", html);
}

void handleChartsPage() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Grafici</title>";
  html += "<style>";
  html += "body{font-family:Arial;background:#0b1220;color:#e6edf3;padding:20px;margin:0}";
  html += ".card{background:#111827;border:1px solid #1f2937;border-radius:10px;padding:16px;margin:10px 0}";
  html += ".chart{height:300px;width:100%;border:1px solid #1f2937;background:#0f172a}";
  html += ".controls{display:flex;gap:10px;align-items:center;margin:10px 0;flex-wrap:wrap}";
  html += "select,button{padding:8px;border:1px solid #1f2937;border-radius:4px;background:#0f172a;color:#e6edf3;font-size:14px}";
  html += "button{background:#3b82f6;cursor:pointer}";
  html += "button:hover{background:#2563eb}";
  html += "label{color:#cbd5e1;font-size:14px}";
  html += "a{color:#60a5fa;text-decoration:none}";
  html += ".data-table{width:100%;border-collapse:collapse;margin:10px 0}";
  html += ".data-table th,.data-table td{border:1px solid #1f2937;padding:8px;text-align:center}";
  html += ".data-table th{background:#1f2937;color:#e6edf3}";
  html += ".data-table td{background:#0f172a;color:#cbd5e1}";
  html += "</style></head><body>";
  html += "<h1>📈 Grafici Batterie</h1>";
  html += "<div class='controls'>";
  html += "<label>Scala temporale:</label>";
  html += "<select id='timeScale' onchange='changeScale()'>";
  html += "<option value='10s'>10 secondi</option>";
  html += "<option value='30s'>30 secondi</option>";
  html += "<option value='1m'>1 minuto</option>";
  html += "<option value='5m'>5 minuti</option>";
  html += "<option value='10m' selected>10 minuti</option>";
  html += "</select>";
  html += "<button onclick='exportCSV()'>📥 Esporta CSV</button>";
  html += "<button onclick='update()'>🔄 Aggiorna</button>";
  html += "</div>";
  html += "<div class='card'><h3>📊 Tensioni (V)</h3><div id='vChart' class='chart'></div></div>";
  html += "<div class='card'><h3>⚡ Correnti (A)</h3><div id='cChart' class='chart'></div></div>";
  html += "<div class='card'><h3>📋 Dati Storici</h3>";
  html += "<table class='data-table'>";
  html += "<tr><th>Batteria</th><th>Ultima Tensione</th><th>Ultima Corrente</th><th>Punti Disponibili</th></tr>";
  html += "<tr><td>6S#1</td><td id='v0'>-</td><td id='c0'>-</td><td id='p0'>-</td></tr>";
  html += "<tr><td>6S#2</td><td id='v1'>-</td><td id='c1'>-</td><td id='p1'>-</td></tr>";
  html += "<tr><td>4S</td><td id='v2'>-</td><td id='c2'>-</td><td id='p2'>-</td></tr>";
  html += "</table></div>";
  html += "<a href='/'>← Torna al Monitor</a>";
  html += "<script>";
  html += "let currentScale='10m';";
  html += "function changeScale(){currentScale=document.getElementById('timeScale').value;update();}";
  html += "function update(){";
  html += "fetch('/charts-data?scale='+currentScale)";
  html += ".then(r=>r.json())";
  html += ".then(d=>{";
  html += "for(let i=0;i<3;i++){";
  html += "document.getElementById('v'+i).textContent=d.voltage[i].length>0?d.voltage[i][d.voltage[i].length-1].toFixed(2):'N/A';";
  html += "document.getElementById('c'+i).textContent=d.current[i].length>0?d.current[i][d.current[i].length-1].toFixed(2):'N/A';";
  html += "document.getElementById('p'+i).textContent=d.voltage[i].length;";
  html += "}";
  html += "updateChart('vChart','Tensioni (V)',d.voltage);";
  html += "updateChart('cChart','Correnti (A)',d.current);";
  html += "})";
  html += ".catch(e=>console.error('Errore:',e));";
  html += "}";
  html += "function updateChart(containerId,title,data){";
  html += "let container=document.getElementById(containerId);";
  html += "let html='<h4>'+title+'</h4>';";
  html += "for(let i=0;i<3;i++){";
  html += "let name=['6S#1','6S#2','4S'][i];";
  html += "let values=data[i];";
  html += "if(values.length>0){";
  html += "let min=Math.min(...values).toFixed(2);";
  html += "let max=Math.max(...values).toFixed(2);";
  html += "let avg=(values.reduce((a,b)=>a+b,0)/values.length).toFixed(2);";
  html += "html+='<div style=\"margin:10px 0;padding:10px;border:1px solid #1f2937;border-radius:5px;\">';";
  html += "html+='<strong>'+name+':</strong> Min: '+min+', Max: '+max+', Avg: '+avg+'<br>';";
  html += "html+='<div style=\"height:20px;background:#1f2937;border-radius:3px;margin:5px 0;\">';";
  html += "html+='<div style=\"height:100%;background:linear-gradient(90deg,#3b82f6,#10b981,#f59e0b);width:100%;border-radius:3px;\"></div>';";
  html += "html+='</div>';";
  html += "html+='Ultimi 5: '+values.slice(-5).map(v=>v.toFixed(2)).join(', ');";
  html += "html+='</div>';";
  html += "}else{";
  html += "html+='<div style=\"margin:10px 0;padding:10px;border:1px solid #1f2937;border-radius:5px;color:#94a3b8;\">';";
  html += "html+='<strong>'+name+':</strong> Nessun dato disponibile';";
  html += "html+='</div>';";
  html += "}";
  html += "}";
  html += "container.innerHTML=html;";
  html += "}";
  html += "function exportCSV(){window.open('/csv','_blank');}";
  html += "setInterval(update,10000);";
  html += "update();";
  html += "</script></body></html>";
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
  autopilot["motor_right"] = autopilot_input.motor_right;
  autopilot["motor_left"] = autopilot_input.motor_left;
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
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  
  // Inizializzazione pin direzione (neutral)
  digitalWrite(DIR_RIGHT_PIN, LOW);
  digitalWrite(DIR_LEFT_PIN, LOW);
  
  // Configurazione ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // 0-3.3V range
  
  // Inizializzazione Taratura
  initCalibration();
  
  // Inizializzazione Grafici
  for (int i = 0; i < 3; i++) {
    initChart(&voltage_charts[i]);
    initChart(&current_charts[i]);
  }
  
  // Configurazione PWM Output (LEDC channels) - CORRETTA
  ledcSetup(PWM_OUT_RIGHT_CHANNEL, PWM_FREQ, 12);  // 50Hz, 12-bit resolution
  ledcAttachPin(PWM_OUT_RIGHT, PWM_OUT_RIGHT_CHANNEL);
  ledcSetup(PWM_OUT_LEFT_CHANNEL, PWM_FREQ, 12);   // 50Hz, 12-bit resolution  
  ledcAttachPin(PWM_OUT_LEFT, PWM_OUT_LEFT_CHANNEL);
  
  // Inizializzazione PWM Output (posizione neutra)
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
  server.on("/calibration", HTTP_GET, handleCalibrationPage);
  server.on("/calibration", HTTP_POST, handleCalibration);
  server.on("/charts", HTTP_GET, handleChartsPage);
  server.on("/charts-data", HTTP_GET, handleCharts);
  server.on("/csv", handleCSV);
  server.begin();
  Serial.println("🌍 Web Server avviato");
  
  Serial.println("✅ Sistema inizializzato!");
  Serial.println("📊 Telemetria ogni " + String(TELEMETRY_INTERVAL) + "ms");
  Serial.println("🔗 Web Interface: http://" + IP.toString());
  delay(5000);
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
  
  // Aggiorna grafici
  updateCharts();
  
  // Web Server
  server.handleClient();
  
  // Piccola pausa per stabilità
  delay(1);
}
