/*
 * AlixBlimp Battery Monitor & Differential Motor Control
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
#include <Preferences.h>
#include <SPIFFS.h>
#include <FS.h>

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
#define PWM_IN_UNDER       5     // GPIO5 - Motori Sottostanti

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
  // Dati raw per calibrazione
  float raw_current_adc;  // Valore ADC raw corrente
  float raw_voltage_adc;  // Valore ADC raw tensione
  float raw_current_voltage; // Tensione raw dal sensore corrente
  float raw_voltage_voltage; // Tensione raw dal partitore
};

struct PWMData {
  uint16_t motor_right;  // 1000-2000μs
  uint16_t motor_left;   // 1000-2000μs
  uint16_t motor_under;  // 1000-2000μs - Motori sottostanti
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

// Strutture per Grafici (dati storici ottimizzati)
struct ChartData {
  float values[300];        // 300 punti (5 minuti a 1Hz) - ridotto per memoria
  int index;                // Indice corrente
  bool filled;              // Buffer riempito
  unsigned long last_update; // Ultimo aggiornamento
  float min_value;          // Valore minimo nel buffer
  float max_value;          // Valore massimo nel buffer
  float avg_value;          // Valore medio nel buffer
  unsigned long total_samples; // Totale campioni raccolti
};

// Struttura per dati persistenti su Flash (storage lungo termine)
struct LongTermDataPoint {
  uint32_t timestamp;       // 4 bytes - millisecondi dall'avvio
  // Batterie (dati convertiti)
  float v1, c1, v2, c2, v3, c3;  // 24 bytes - tensioni e correnti
  // Dati raw
  float rv1, rc1, rv2, rc2, rv3, rc3;  // 24 bytes
  // PWM motori
  uint16_t m1, m2, m3;      // 6 bytes
  // Totale: 58 bytes per campione
  // 4 ore @ 0.1Hz = 1440 campioni = 83,520 bytes (~82KB)
} __attribute__((packed));

// Gestione storage lungo termine
struct LongTermStorage {
  int write_index;          // Indice scrittura (circolare)
  int total_points;         // Punti totali salvati
  unsigned long last_save;  // Ultimo salvataggio
  bool initialized;         // Flag inizializzazione
  File dataFile;            // File handle
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
ChartData raw_voltage_charts[3]; // Grafici tensione raw
ChartData raw_current_charts[3]; // Grafici corrente raw
ChartData motor_charts[3];       // Grafici PWM motori (right, left, under)

// Statistiche
unsigned long last_telemetry = 0;
unsigned long last_chart_update = 0;
unsigned long loop_count = 0;
float loop_frequency = 0.0;

// WiFi e Web Server
const char* ssid = "ESP32_BatteryMonitor";
const char* password = "battery123";
WebServer server(80);
Preferences preferences;

// Storage lungo termine
LongTermStorage longTermStorage;
#define LONG_TERM_FILE "/data.bin"
#define LONG_TERM_MAX_POINTS 1440  // 4 ore @ 0.1Hz (1 campione ogni 10 secondi)
#define LONG_TERM_SAVE_INTERVAL 10000  // Salva ogni 10 secondi

// ============================================================================
// GESTIONE MEMORIA FLASH - IMPOSTAZIONI PERMANENTI
// ============================================================================

// Salva le impostazioni di calibrazione nella memoria flash
void saveCalibrationToFlash() {
  preferences.begin("calibration", false);
  
  for (int i = 0; i < 3; i++) {
    String prefix = "bat" + String(i) + "_";
    
    preferences.putFloat((prefix + "v_offset").c_str(), calibration[i].voltage_offset);
    preferences.putFloat((prefix + "v_scale").c_str(), calibration[i].voltage_scale);
    preferences.putFloat((prefix + "c_offset").c_str(), calibration[i].current_offset);
    preferences.putFloat((prefix + "c_scale").c_str(), calibration[i].current_scale);
    preferences.putFloat((prefix + "divider").c_str(), calibration[i].divider_ratio);
  }
  
  preferences.end();
  Serial.println("💾 Impostazioni di calibrazione salvate nella memoria flash");
}

// Carica le impostazioni di calibrazione dalla memoria flash
void loadCalibrationFromFlash() {
  preferences.begin("calibration", false);
  
  for (int i = 0; i < 3; i++) {
    String prefix = "bat" + String(i) + "_";
    
    // Carica valori salvati, usa valori di default se non trovati
    calibration[i].voltage_offset = preferences.getFloat((prefix + "v_offset").c_str(), 0.0);
    calibration[i].voltage_scale = preferences.getFloat((prefix + "v_scale").c_str(), 1.0);
    calibration[i].current_offset = preferences.getFloat((prefix + "c_offset").c_str(), 0.0);
    calibration[i].current_scale = preferences.getFloat((prefix + "c_scale").c_str(), 1.0);
    calibration[i].divider_ratio = preferences.getFloat((prefix + "divider").c_str(), DIVIDER_6S_RATIO);
  }
  
  preferences.end();
  Serial.println("📂 Impostazioni di calibrazione caricate dalla memoria flash");
}

// Reset delle impostazioni di calibrazione (torna ai valori di default)
void resetCalibrationToDefault() {
  preferences.begin("calibration", false);
  preferences.clear();
  preferences.end();
  
  // Reimposta ai valori di default
  for (int i = 0; i < 3; i++) {
    calibration[i].voltage_offset = 0.0;
    calibration[i].voltage_scale = 1.0;
    calibration[i].current_offset = 0.0;
    calibration[i].current_scale = 1.0;
    calibration[i].divider_ratio = DIVIDER_6S_RATIO;
  }
  
  Serial.println("🔄 Impostazioni di calibrazione ripristinate ai valori di default");
}

// ============================================================================
// GESTIONE STORAGE LUNGO TERMINE - SPIFFS
// ============================================================================

// Inizializza SPIFFS e file dati
bool initLongTermStorage() {
  Serial.println("💾 Inizializzazione SPIFFS...");
  
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ Errore montaggio SPIFFS!");
    longTermStorage.initialized = false;
    return false;
  }
  
  // Informazioni filesystem
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  Serial.printf("📊 SPIFFS: %d KB totali, %d KB usati, %d KB liberi\n", 
                totalBytes/1024, usedBytes/1024, (totalBytes-usedBytes)/1024);
  
  // Controlla se esiste file dati
  if (SPIFFS.exists(LONG_TERM_FILE)) {
    File file = SPIFFS.open(LONG_TERM_FILE, FILE_READ);
    if (file) {
      size_t fileSize = file.size();
      int points = fileSize / sizeof(LongTermDataPoint);
      Serial.printf("📂 File dati esistente: %d bytes, %d campioni\n", fileSize, points);
      
      longTermStorage.total_points = min(points, LONG_TERM_MAX_POINTS);
      longTermStorage.write_index = longTermStorage.total_points % LONG_TERM_MAX_POINTS;
      file.close();
    }
  } else {
    Serial.println("📝 Creazione nuovo file dati...");
    File file = SPIFFS.open(LONG_TERM_FILE, FILE_WRITE);
    if (file) {
      file.close();
      longTermStorage.total_points = 0;
      longTermStorage.write_index = 0;
    }
  }
  
  longTermStorage.last_save = 0;
  longTermStorage.initialized = true;
  Serial.println("✅ Storage lungo termine inizializzato!");
  
  return true;
}

// Salva un campione su SPIFFS
void saveLongTermDataPoint() {
  if (!longTermStorage.initialized) return;
  
  if (millis() - longTermStorage.last_save < LONG_TERM_SAVE_INTERVAL) return;
  
  // Prepara struttura dati
  LongTermDataPoint dataPoint;
  dataPoint.timestamp = millis();
  
  // Dati batterie convertiti
  dataPoint.v1 = batteries[0].voltage;
  dataPoint.c1 = batteries[0].current;
  dataPoint.v2 = batteries[1].voltage;
  dataPoint.c2 = batteries[1].current;
  dataPoint.v3 = batteries[2].voltage;
  dataPoint.c3 = batteries[2].current;
  
  // Dati raw
  dataPoint.rv1 = batteries[0].raw_voltage_voltage;
  dataPoint.rc1 = batteries[0].raw_current_voltage;
  dataPoint.rv2 = batteries[1].raw_voltage_voltage;
  dataPoint.rc2 = batteries[1].raw_current_voltage;
  dataPoint.rv3 = batteries[2].raw_voltage_voltage;
  dataPoint.rc3 = batteries[2].raw_current_voltage;
  
  // PWM motori
  dataPoint.m1 = autopilot_input.motor_right;
  dataPoint.m2 = autopilot_input.motor_left;
  dataPoint.m3 = autopilot_input.motor_under;
  
  // Apri file in modalità lettura/scrittura
  File file = SPIFFS.open(LONG_TERM_FILE, FILE_WRITE);
  if (!file) {
    Serial.println("❌ Errore apertura file per scrittura!");
    return;
  }
  
  // Posiziona al punto di scrittura (circular buffer)
  size_t seekPos = (longTermStorage.write_index * sizeof(LongTermDataPoint));
  file.seek(seekPos);
  
  // Scrivi dati
  size_t written = file.write((uint8_t*)&dataPoint, sizeof(LongTermDataPoint));
  file.close();
  
  if (written == sizeof(LongTermDataPoint)) {
    // Aggiorna indici
    longTermStorage.write_index = (longTermStorage.write_index + 1) % LONG_TERM_MAX_POINTS;
    if (longTermStorage.total_points < LONG_TERM_MAX_POINTS) {
      longTermStorage.total_points++;
    }
    longTermStorage.last_save = millis();
    
    // Debug ogni 10 salvataggi
    if (longTermStorage.total_points % 10 == 0) {
      Serial.printf("💾 Salvati %d/%d campioni long-term (%.1f%% buffer)\n", 
                    longTermStorage.total_points, LONG_TERM_MAX_POINTS,
                    (longTermStorage.total_points * 100.0) / LONG_TERM_MAX_POINTS);
    }
  } else {
    Serial.println("❌ Errore scrittura dati!");
  }
}

// Leggi campioni dal file long-term
int readLongTermData(LongTermDataPoint* buffer, int maxPoints, int startIndex = 0) {
  if (!longTermStorage.initialized) return 0;
  
  File file = SPIFFS.open(LONG_TERM_FILE, FILE_READ);
  if (!file) return 0;
  
  int pointsToRead = min(maxPoints, longTermStorage.total_points - startIndex);
  if (pointsToRead <= 0) {
    file.close();
    return 0;
  }
  
  // Posiziona al punto di lettura
  file.seek(startIndex * sizeof(LongTermDataPoint));
  
  // Leggi dati
  int pointsRead = 0;
  for (int i = 0; i < pointsToRead; i++) {
    size_t read = file.read((uint8_t*)&buffer[i], sizeof(LongTermDataPoint));
    if (read == sizeof(LongTermDataPoint)) {
      pointsRead++;
    } else {
      break;
    }
  }
  
  file.close();
  return pointsRead;
}

// Azzera storage lungo termine
void clearLongTermStorage() {
  if (!longTermStorage.initialized) return;
  
  Serial.println("🗑️ Cancellazione storage lungo termine...");
  
  SPIFFS.remove(LONG_TERM_FILE);
  
  File file = SPIFFS.open(LONG_TERM_FILE, FILE_WRITE);
  if (file) {
    file.close();
  }
  
  longTermStorage.total_points = 0;
  longTermStorage.write_index = 0;
  longTermStorage.last_save = 0;
  
  Serial.println("✅ Storage lungo termine azzerato!");
}

// Ottieni statistiche storage
void getLongTermStorageInfo(JsonObject& info) {
  info["initialized"] = longTermStorage.initialized;
  info["total_points"] = longTermStorage.total_points;
  info["max_points"] = LONG_TERM_MAX_POINTS;
  info["write_index"] = longTermStorage.write_index;
  
  if (longTermStorage.initialized && longTermStorage.total_points > 0) {
    // Calcola durata copertura
    float hours = (longTermStorage.total_points * LONG_TERM_SAVE_INTERVAL) / (1000.0 * 3600.0);
    info["coverage_hours"] = hours;
    info["percent_full"] = (longTermStorage.total_points * 100.0) / LONG_TERM_MAX_POINTS;
  }
  
  // Info filesystem
  info["spiffs_total_kb"] = SPIFFS.totalBytes() / 1024;
  info["spiffs_used_kb"] = SPIFFS.usedBytes() / 1024;
  info["spiffs_free_kb"] = (SPIFFS.totalBytes() - SPIFFS.usedBytes()) / 1024;
}

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


// Lettura Tensione da Partitore (con taratura e dati raw)
float readVoltage(int pin, int battery_index, float* raw_adc, float* raw_voltage_out) {
  float adc_value = readADC(pin);
  float voltage = adcToVoltage(adc_value);
  float raw_voltage = voltage * calibration[battery_index].divider_ratio;
  
  // Salva dati raw
  *raw_adc = adc_value;
  *raw_voltage_out = voltage;
  
  // Applica taratura: (valore_raw + offset) * scala
  return (raw_voltage + calibration[battery_index].voltage_offset) * calibration[battery_index].voltage_scale;
}

// Lettura Corrente da ACS758 (con taratura e dati raw)
float readCurrent(int pin, int battery_index, float* raw_adc, float* raw_voltage_out) {
  float adc_value = readADC(pin);
  float voltage = adcToVoltage(adc_value);
  float raw_current = (voltage - ACS758_VREF) / ACS758_SENSITIVITY;
  
  // Salva dati raw
  *raw_adc = adc_value;
  *raw_voltage_out = voltage;
  
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
  // Duty cycle = (pulse_width / 20000) * (2^PWM_RESOLUTION - 1)
  uint32_t max_duty = (1 << PWM_RESOLUTION) - 1;  // 2^16 - 1 = 65535 per 16-bit
  uint32_t duty = (pulse_width * max_duty) / 20000;
  
  if (pin == PWM_OUT_RIGHT) {
    ledcWrite(PWM_OUT_RIGHT_CHANNEL, duty);
  } else if (pin == PWM_OUT_LEFT) {
    ledcWrite(PWM_OUT_LEFT_CHANNEL, duty);
  }
}

// ============================================================================
// FUNZIONI GRAFICI E TARATURA
// ============================================================================

// Calibrazione semplice con conversione lineare
void simpleCalibration(int battery_index, float measured_voltage, float measured_current) {
  // Per tensione: V_measured = (V_raw * divider_ratio + offset) * scale
  // Assumendo offset = 0 e scale = 1, calcoliamo il divider_ratio corretto
  float raw_voltage = batteries[battery_index].raw_voltage_voltage;
  if (raw_voltage > 0.01) { // Evita divisione per zero
    calibration[battery_index].divider_ratio = measured_voltage / raw_voltage;
    calibration[battery_index].voltage_offset = 0.0;
    calibration[battery_index].voltage_scale = 1.0;
  }
  
  // Per corrente: I_measured = ((V_raw - V_ref) / sensitivity + offset) * scale
  // Assumendo offset = 0 e scale = 1, calcoliamo la sensitivity corretta
  float raw_current_voltage = batteries[battery_index].raw_current_voltage;
  float voltage_diff = raw_current_voltage - ACS758_VREF;
  if (abs(voltage_diff) > 0.01) { // Evita divisione per zero
    float calculated_sensitivity = voltage_diff / measured_current;
    // Aggiorna la sensitivity globale (nota: questo influenzerà tutti i sensori)
    // Per ora salviamo come scale factor
    calibration[battery_index].current_offset = 0.0;
    calibration[battery_index].current_scale = ACS758_SENSITIVITY / calculated_sensitivity;
  }
  
  Serial.printf("🔧 Calibrazione semplice batteria %d:\n", battery_index);
  Serial.printf("   Tensione: %.2fV -> Divider ratio: %.2f\n", measured_voltage, calibration[battery_index].divider_ratio);
  Serial.printf("   Corrente: %.2fA -> Scale factor: %.3f\n", measured_current, calibration[battery_index].current_scale);
  
  // Salva le impostazioni nella memoria flash
  saveCalibrationToFlash();
}

// Calibrazione a due punti per maggiore precisione
void twoPointCalibration(int battery_index, float v1_measured, float v1_raw, float v2_measured, float v2_raw,
                        float i1_measured, float i1_raw, float i2_measured, float i2_raw) {
  
  // Calibrazione tensione: V = (raw * ratio + offset) * scale
  // Sistema: v1 = (r1 * ratio + offset) * scale
  //          v2 = (r2 * ratio + offset) * scale
  float raw_diff = v2_raw - v1_raw;
  float meas_diff = v2_measured - v1_measured;
  
  if (abs(raw_diff) > 0.01) {
    calibration[battery_index].voltage_scale = meas_diff / raw_diff;
    calibration[battery_index].voltage_offset = (v1_measured / calibration[battery_index].voltage_scale) - v1_raw;
    calibration[battery_index].divider_ratio = 1.0; // Non usato in modalità avanzata
  }
  
  // Calibrazione corrente: I = ((raw - V_ref) / sensitivity + offset) * scale
  float i_raw_diff = i2_raw - i1_raw;
  float i_meas_diff = i2_measured - i1_measured;
  
  if (abs(i_raw_diff) > 0.01) {
    calibration[battery_index].current_scale = i_meas_diff / i_raw_diff;
    calibration[battery_index].current_offset = (i1_measured / calibration[battery_index].current_scale) - i1_raw;
  }
  
  Serial.printf("🔧 Calibrazione a due punti batteria %d:\n", battery_index);
  Serial.printf("   Tensione: Scale=%.3f, Offset=%.3f\n", calibration[battery_index].voltage_scale, calibration[battery_index].voltage_offset);
  Serial.printf("   Corrente: Scale=%.3f, Offset=%.3f\n", calibration[battery_index].current_scale, calibration[battery_index].current_offset);
  
  // Salva le impostazioni nella memoria flash
  saveCalibrationToFlash();
}

// Inizializza grafico ottimizzato
void initChart(ChartData* chart) {
  chart->index = 0;
  chart->filled = false;
  chart->last_update = 0;
  chart->min_value = 9999.0;
  chart->max_value = -9999.0;
  chart->avg_value = 0.0;
  chart->total_samples = 0;
  for (int i = 0; i < 300; i++) {
    chart->values[i] = 0.0;
  }
}

// Aggiungi valore al grafico con statistiche
void addToChart(ChartData* chart, float value) {
  chart->values[chart->index] = value;
  chart->total_samples++;
  
  // Aggiorna statistiche
  if (value < chart->min_value) chart->min_value = value;
  if (value > chart->max_value) chart->max_value = value;
  
  // Calcola media mobile
  if (chart->filled) {
    chart->avg_value = (chart->avg_value * 299 + value) / 300;
  } else {
    chart->avg_value = (chart->avg_value * chart->index + value) / (chart->index + 1);
  }
  
  // CORREZIONE: Incrementa l'indice e gestisci il wrap-around
  chart->index++;
  if (chart->index >= 300) {
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

// Aggiorna grafici con frequenza ottimizzata
void updateCharts() {
  if (millis() - last_chart_update < 1000) return; // Aggiorna ogni 1 secondo
  
  for (int i = 0; i < 3; i++) {
    // Usa SOLO i dati reali delle batterie
    float real_voltage = batteries[i].voltage;
    float real_current = batteries[i].current;
    float real_raw_voltage = batteries[i].raw_voltage_voltage;
    float real_raw_current = batteries[i].raw_current_voltage;
    
    // Aggiungi SOLO dati reali ai grafici
    addToChart(&voltage_charts[i], real_voltage);
    addToChart(&current_charts[i], real_current);
    addToChart(&raw_voltage_charts[i], real_raw_voltage);
    addToChart(&raw_current_charts[i], real_raw_current);
  }
  
  // Aggiungi dati motori PWM
  addToChart(&motor_charts[0], (float)autopilot_input.motor_right); // Motore destro
  addToChart(&motor_charts[1], (float)autopilot_input.motor_left);  // Motore sinistro
  addToChart(&motor_charts[2], (float)autopilot_input.motor_under); // Motori sottostanti
  
  last_chart_update = millis();
}

// Ottieni dati grafico per scala temporale specifica
void getChartData(ChartData* chart, int points, float* output_data, int* actual_points) {
  int total_points = chart->filled ? 300 : chart->index;
  int step = max(1, total_points / points);
  *actual_points = min(points, total_points / step);
  
  int start = chart->filled ? chart->index : 0;
  for (int i = 0; i < *actual_points; i++) {
    int idx = (start + i * step) % 300;
    output_data[i] = chart->values[idx];
  }
}

// Ottieni statistiche grafico
void getChartStats(ChartData* chart, float* min_val, float* max_val, float* avg_val) {
  *min_val = chart->min_value;
  *max_val = chart->max_value;
  *avg_val = chart->avg_value;
}

// Azzera tutti i grafici
void clearAllCharts() {
  for (int i = 0; i < 3; i++) {
    initChart(&voltage_charts[i]);
    initChart(&current_charts[i]);
    initChart(&raw_voltage_charts[i]);
    initChart(&raw_current_charts[i]);
    initChart(&motor_charts[i]);
  }
  Serial.println("🗑️ Tutti i grafici sono stati azzerati");
}

// Calibrazione automatica basata sui dati raw attuali
void autoCalibration(int battery_index, float raw_voltage, float raw_current, float measured_voltage, float measured_current) {
  // Calibrazione tensione: V_measured = (V_raw * divider_ratio + offset) * scale
  // Assumendo offset = 0, calcoliamo divider_ratio e scale
  if (raw_voltage > 0.01) {
    calibration[battery_index].divider_ratio = measured_voltage / raw_voltage;
    calibration[battery_index].voltage_offset = 0.0;
    calibration[battery_index].voltage_scale = 1.0;
  }
  
  // Calibrazione corrente: I_measured = ((V_raw - V_ref) / sensitivity + offset) * scale
  // Assumendo offset = 0, calcoliamo la sensitivity corretta
  float voltage_diff = raw_current - ACS758_VREF;
  if (abs(voltage_diff) > 0.01) {
    float calculated_sensitivity = voltage_diff / measured_current;
    calibration[battery_index].current_offset = 0.0;
    calibration[battery_index].current_scale = ACS758_SENSITIVITY / calculated_sensitivity;
  }
  
  Serial.printf("🔄 Calibrazione automatica batteria %d completata:\n", battery_index);
  Serial.printf("   Tensione: %.3fV raw -> %.2fV misurato (ratio: %.3f)\n", 
                raw_voltage, measured_voltage, calibration[battery_index].divider_ratio);
  Serial.printf("   Corrente: %.3fV raw -> %.2fA misurato (scale: %.3f)\n", 
                raw_current, measured_current, calibration[battery_index].current_scale);
  
  // Salva le impostazioni nella memoria flash
  saveCalibrationToFlash();
}

// ============================================================================
// FUNZIONI PRINCIPALI
// ============================================================================

void readBatteryData() {
  // Lettura Correnti (con taratura e dati raw)
  batteries[0].current = readCurrent(CURRENT_6S1_PIN, 0, &batteries[0].raw_current_adc, &batteries[0].raw_current_voltage);
  batteries[1].current = readCurrent(CURRENT_6S2_PIN, 1, &batteries[1].raw_current_adc, &batteries[1].raw_current_voltage);
  batteries[2].current = readCurrent(CURRENT_4S_PIN, 2, &batteries[2].raw_current_adc, &batteries[2].raw_current_voltage);
  
  // Lettura Tensioni (con taratura e dati raw)
  batteries[0].voltage = readVoltage(VOLTAGE_6S1_PIN, 0, &batteries[0].raw_voltage_adc, &batteries[0].raw_voltage_voltage);
  batteries[1].voltage = readVoltage(VOLTAGE_6S2_PIN, 1, &batteries[1].raw_voltage_adc, &batteries[1].raw_voltage_voltage);
  batteries[2].voltage = readVoltage(VOLTAGE_4S_PIN, 2, &batteries[2].raw_voltage_adc, &batteries[2].raw_voltage_voltage);
  
  // Calcolo Potenze
  for (int i = 0; i < 3; i++) {
    batteries[i].power = batteries[i].voltage * batteries[i].current;
  }
}

void readAutopilotInput() {
  autopilot_input.motor_right = readPWM(PWM_IN_RIGHT);
  autopilot_input.motor_left = readPWM(PWM_IN_LEFT);
  autopilot_input.motor_under = readPWM(PWM_IN_UNDER);
  // RIMOSSO: I pin di direzione sono OUTPUT, non INPUT!
  // autopilot_input.dir_right = digitalRead(DIR_RIGHT_PIN);  // ❌ ERRORE
  // autopilot_input.dir_left = digitalRead(DIR_LEFT_PIN);    // ❌ ERRORE
}

void calculateMotorOutput() {
  // Input diretti per motore destro e sinistro
  uint16_t right_input = autopilot_input.motor_right;
  uint16_t left_input = autopilot_input.motor_left;
  
  // Logica corretta PWM:
  // AVANTI: 1500→1000, 2000→2000 (HIGH)
  // INDIETRO: 1500→1000, 1000→2000 (LOW)
  // Per motore destro
  if (right_input <= PWM_CENTER) {
    // 1000-1500: BACKWARD - mappa 1000→2000, 1500→1000
    motor_output.right_pwm = map(right_input, PWM_MIN, PWM_CENTER, PWM_MAX, PWM_MIN);
    digitalWrite(DIR_RIGHT_PIN, LOW);  // BACKWARD - DISATTIVO
    // DEBUG: Stampa quando va indietro
    if (right_input < 1500) {
      Serial.printf("🔙 MOTORE DESTRO INDIETRO: Input=%d, Output=%d, DIR=LOW\n", right_input, motor_output.right_pwm);
    }
  } else {
    // 1500-2000: FORWARD - mappa 1500→1000, 2000→2000
    motor_output.right_pwm = map(right_input, PWM_CENTER, PWM_MAX, PWM_MIN, PWM_MAX);
    digitalWrite(DIR_RIGHT_PIN, HIGH); // FORWARD - ATTIVO
    // DEBUG: Stampa quando va avanti
    if (right_input > 1500) {
      Serial.printf("🔜 MOTORE DESTRO AVANTI: Input=%d, Output=%d, DIR=HIGH\n", right_input, motor_output.right_pwm);
    }
  }
  
  // Per motore sinistro
  if (left_input <= PWM_CENTER) {
    // 1000-1500: BACKWARD - mappa 1000→2000, 1500→1000
    motor_output.left_pwm = map(left_input, PWM_MIN, PWM_CENTER, PWM_MAX, PWM_MIN);
    digitalWrite(DIR_LEFT_PIN, LOW);   // BACKWARD - DISATTIVO
    // DEBUG: Stampa quando va indietro
    if (left_input < 1500) {
      Serial.printf("🔙 MOTORE SINISTRO INDIETRO: Input=%d, Output=%d, DIR=LOW\n", left_input, motor_output.left_pwm);
    }
  } else {
    // 1500-2000: FORWARD - mappa 1500→1000, 2000→2000
    motor_output.left_pwm = map(left_input, PWM_CENTER, PWM_MAX, PWM_MIN, PWM_MAX);
    digitalWrite(DIR_LEFT_PIN, HIGH);  // FORWARD - ATTIVO
    // DEBUG: Stampa quando va avanti
    if (left_input > 1500) {
      Serial.printf("🔜 MOTORE SINISTRO AVANTI: Input=%d, Output=%d, DIR=HIGH\n", left_input, motor_output.left_pwm);
    }
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
  
  Serial.printf("Autopilota: MotorRight=%d, MotorLeft=%d, MotorUnder=%d, DirR=%d, DirL=%d\n",
                autopilot_input.motor_right, autopilot_input.motor_left, autopilot_input.motor_under,
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
    String mode = doc["mode"] | "advanced";
    
    if (mode == "simple") {
      // Modalità semplice: solo valori letti
      float measured_voltage = doc["measured_voltage"];
      float measured_current = doc["measured_current"];
      simpleCalibration(battery_index, measured_voltage, measured_current);
    } else if (mode == "auto") {
      // Modalità automatica: calibrazione basata sui dati raw attuali
      float raw_voltage = doc["raw_voltage"];
      float raw_current = doc["raw_current"];
      float measured_voltage = doc["measured_voltage"];
      float measured_current = doc["measured_current"];
      autoCalibration(battery_index, raw_voltage, raw_current, measured_voltage, measured_current);
    } else if (mode == "two_point") {
      // Modalità a due punti
      float v1_measured = doc["v1_measured"];
      float v1_raw = doc["v1_raw"];
      float v2_measured = doc["v2_measured"];
      float v2_raw = doc["v2_raw"];
      float i1_measured = doc["i1_measured"];
      float i1_raw = doc["i1_raw"];
      float i2_measured = doc["i2_measured"];
      float i2_raw = doc["i2_raw"];
      twoPointCalibration(battery_index, v1_measured, v1_raw, v2_measured, v2_raw,
                         i1_measured, i1_raw, i2_measured, i2_raw);
    } else {
      // Modalità avanzata: parametri manuali
    if (type == "voltage") {
      calibration[battery_index].voltage_offset = doc["offset"];
      calibration[battery_index].voltage_scale = doc["scale"];
      calibration[battery_index].divider_ratio = doc["divider_ratio"];
    } else if (type == "current") {
      calibration[battery_index].current_offset = doc["offset"];
      calibration[battery_index].current_scale = doc["scale"];
      }
      
      // Salva le impostazioni nella memoria flash per modalità avanzata
      saveCalibrationToFlash();
    }
    
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Calibrazione salvata\"}");
  } else {
    // Restituisci dati di taratura attuali + dati raw correnti
    DynamicJsonDocument doc(2048);
    for (int i = 0; i < 3; i++) {
      JsonObject cal = doc.createNestedObject("battery_" + String(i));
      cal["voltage_offset"] = calibration[i].voltage_offset;
      cal["voltage_scale"] = calibration[i].voltage_scale;
      cal["current_offset"] = calibration[i].current_offset;
      cal["current_scale"] = calibration[i].current_scale;
      cal["divider_ratio"] = calibration[i].divider_ratio;
      
      // Aggiungi dati raw correnti per calibrazione
      cal["current_raw_voltage"] = batteries[i].raw_voltage_voltage;
      cal["current_raw_current_voltage"] = batteries[i].raw_current_voltage;
      cal["current_measured_voltage"] = batteries[i].voltage;
      cal["current_measured_current"] = batteries[i].current;
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  }
}

void handleCharts() {
  DynamicJsonDocument doc(8192);  // Aumentato per dati long-term
  
  // Ottieni parametri query
  String scale = server.arg("scale");
  int points = 60;
  bool useLongTerm = false;
  
  // Scale temporali - RAM per scale brevi, Flash per scale lunghe
  if (scale == "10s") {
    points = 10;      // 10 secondi @ 1Hz (RAM)
    useLongTerm = false;
  } else if (scale == "1m") {
    points = 60;      // 1 minuto @ 1Hz (RAM)
    useLongTerm = false;
  } else if (scale == "5m") {
    points = 300;     // 5 minuti @ 1Hz (RAM - tutto il buffer)
    useLongTerm = false;
  } else if (scale == "10m") {
    points = 60;      // 10 minuti @ 0.1Hz (Flash)
    useLongTerm = true;
  } else if (scale == "30m") {
    points = 180;     // 30 minuti @ 0.1Hz (Flash)
    useLongTerm = true;
  } else if (scale == "1h") {
    points = 360;     // 1 ora @ 0.1Hz (Flash)
    useLongTerm = true;
  } else if (scale == "4h") {
    points = 1440;    // 4 ore @ 0.1Hz (Flash - tutto il buffer)
    useLongTerm = true;
  } else {
    points = 60;      // Default 1 minuto
    useLongTerm = false;
  }
  
  // Buffer temporaneo per i dati
  float temp_data[1500];  // Aumentato per supportare 4 ore
  int actual_points;
  
  // Prepara array per dati
  JsonArray voltageArray = doc.createNestedArray("voltage");
  JsonArray currentArray = doc.createNestedArray("current");
  JsonArray rawVoltageArray = doc.createNestedArray("raw_voltage");
  JsonArray rawCurrentArray = doc.createNestedArray("raw_current");
  JsonArray motorArray = doc.createNestedArray("motors");
  JsonArray voltageStats = doc.createNestedArray("voltage_stats");
  JsonArray currentStats = doc.createNestedArray("current_stats");
  
  if (useLongTerm && longTermStorage.initialized) {
    // ===== DATI DA FLASH (Scale lunghe: 10m, 30m, 1h, 4h) =====
    
    // Alloca buffer per lettura long-term
    LongTermDataPoint* ltBuffer = new LongTermDataPoint[points];
    if (!ltBuffer) {
      Serial.println("❌ Errore allocazione memoria per long-term data!");
      doc["error"] = "Memoria insufficiente";
      String response;
      serializeJson(doc, response);
      server.send(500, "application/json", response);
      return;
    }
    
    // Leggi dati long-term
    int ltPoints = readLongTermData(ltBuffer, points, 0);
    actual_points = ltPoints;
    
    // Estrai dati per ogni batteria
    for (int i = 0; i < 3; i++) {
      JsonArray vArray = voltageArray.createNestedArray();
      JsonArray cArray = currentArray.createNestedArray();
      JsonArray rvArray = rawVoltageArray.createNestedArray();
      JsonArray rcArray = rawCurrentArray.createNestedArray();
      
      JsonObject vStats = voltageStats.createNestedObject();
      JsonObject cStats = currentStats.createNestedObject();
      
      float vMin = 9999.0, vMax = -9999.0, vSum = 0.0;
      float cMin = 9999.0, cMax = -9999.0, cSum = 0.0;
      
      for (int j = 0; j < ltPoints; j++) {
        float v, c, rv, rc;
        
        if (i == 0) {
          v = ltBuffer[j].v1; c = ltBuffer[j].c1;
          rv = ltBuffer[j].rv1; rc = ltBuffer[j].rc1;
        } else if (i == 1) {
          v = ltBuffer[j].v2; c = ltBuffer[j].c2;
          rv = ltBuffer[j].rv2; rc = ltBuffer[j].rc2;
        } else {
          v = ltBuffer[j].v3; c = ltBuffer[j].c3;
          rv = ltBuffer[j].rv3; rc = ltBuffer[j].rc3;
        }
        
        vArray.add(v);
        cArray.add(c);
        rvArray.add(rv);
        rcArray.add(rc);
        
        // Statistiche
        if (v < vMin) vMin = v;
        if (v > vMax) vMax = v;
        vSum += v;
        if (c < cMin) cMin = c;
        if (c > cMax) cMax = c;
        cSum += c;
      }
      
      vStats["min"] = vMin;
      vStats["max"] = vMax;
      vStats["avg"] = ltPoints > 0 ? vSum / ltPoints : 0.0;
      vStats["samples"] = ltPoints;
      
      cStats["min"] = cMin;
      cStats["max"] = cMax;
      cStats["avg"] = ltPoints > 0 ? cSum / ltPoints : 0.0;
      cStats["samples"] = ltPoints;
    }
    
    // Dati motori PWM
    for (int i = 0; i < 3; i++) {
      JsonArray mArray = motorArray.createNestedArray();
      for (int j = 0; j < ltPoints; j++) {
        if (i == 0) mArray.add(ltBuffer[j].m1);
        else if (i == 1) mArray.add(ltBuffer[j].m2);
        else mArray.add(ltBuffer[j].m3);
      }
    }
    
    delete[] ltBuffer;
    
  } else {
    // ===== DATI DA RAM (Scale brevi: 10s, 1m, 5m) =====
    
    for (int i = 0; i < 3; i++) {
      JsonArray vArray = voltageArray.createNestedArray();
      JsonArray cArray = currentArray.createNestedArray();
      JsonArray rvArray = rawVoltageArray.createNestedArray();
      JsonArray rcArray = rawCurrentArray.createNestedArray();
      
      JsonObject vStats = voltageStats.createNestedObject();
      JsonObject cStats = currentStats.createNestedObject();
      
      // Tensioni
      getChartData(&voltage_charts[i], points, temp_data, &actual_points);
      for (int j = 0; j < actual_points; j++) {
        vArray.add(temp_data[j]);
      }
      
      // Correnti
      getChartData(&current_charts[i], points, temp_data, &actual_points);
      for (int j = 0; j < actual_points; j++) {
        cArray.add(temp_data[j]);
      }
      
      // Raw tensioni
      getChartData(&raw_voltage_charts[i], points, temp_data, &actual_points);
      for (int j = 0; j < actual_points; j++) {
        rvArray.add(temp_data[j]);
      }
      
      // Raw correnti
      getChartData(&raw_current_charts[i], points, temp_data, &actual_points);
      for (int j = 0; j < actual_points; j++) {
        rcArray.add(temp_data[j]);
      }
      
      // Statistiche
      float min_val, max_val, avg_val;
      getChartStats(&voltage_charts[i], &min_val, &max_val, &avg_val);
      vStats["min"] = min_val;
      vStats["max"] = max_val;
      vStats["avg"] = avg_val;
      vStats["samples"] = voltage_charts[i].total_samples;
      
      getChartStats(&current_charts[i], &min_val, &max_val, &avg_val);
      cStats["min"] = min_val;
      cStats["max"] = max_val;
      cStats["avg"] = avg_val;
      cStats["samples"] = current_charts[i].total_samples;
    }
    
    // Dati motori PWM
    for (int i = 0; i < 3; i++) {
      JsonArray mArray = motorArray.createNestedArray();
      getChartData(&motor_charts[i], points, temp_data, &actual_points);
      for (int j = 0; j < actual_points; j++) {
        mArray.add(temp_data[j]);
      }
    }
  }
  
  // Dati reali attuali delle batterie
  JsonArray currentBatteries = doc.createNestedArray("current_batteries");
  for (int i = 0; i < 3; i++) {
    JsonObject battery = currentBatteries.createNestedObject();
    battery["voltage"] = batteries[i].voltage;
    battery["current"] = batteries[i].current;
    battery["power"] = batteries[i].power;
    battery["raw_voltage"] = batteries[i].raw_voltage_voltage;
    battery["raw_current"] = batteries[i].raw_current_voltage;
  }
  
  // Metadati
  doc["scale"] = scale;
  doc["points"] = actual_points;
  doc["timestamp"] = millis();
  doc["source"] = useLongTerm ? "flash" : "ram";
  
  // Info storage
  JsonObject storageInfo = doc.createNestedObject("storage_info");
  getLongTermStorageInfo(storageInfo);
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleCSV() {
  // Controlla quale tipo di export richiede l'utente
  String exportType = server.arg("type");  // "ram", "flash", "all" (default)
  
  String csv = "Timestamp_ms,6S1_Voltage,6S1_Current,6S1_RawVoltage,6S1_RawCurrent,";
  csv += "6S2_Voltage,6S2_Current,6S2_RawVoltage,6S2_RawCurrent,";
  csv += "4S_Voltage,4S_Current,4S_RawVoltage,4S_RawCurrent,";
  csv += "MotorRight,MotorLeft,MotorUnder,Source\n";
  
  bool includeRam = (exportType == "ram" || exportType == "" || exportType == "all");
  bool includeFlash = (exportType == "flash" || exportType == "" || exportType == "all");
  
  // ===== Export dati Flash (Long-term) - PIÙ VECCHI =====
  if (includeFlash && longTermStorage.initialized && longTermStorage.total_points > 0) {
    Serial.println("📤 Generazione CSV da Flash...");
    
    // Leggi tutti i dati long-term
    LongTermDataPoint* ltBuffer = new LongTermDataPoint[longTermStorage.total_points];
    if (ltBuffer) {
      int ltPoints = readLongTermData(ltBuffer, longTermStorage.total_points, 0);
      
      for (int i = 0; i < ltPoints; i++) {
        csv += String(ltBuffer[i].timestamp) + ",";
        
        // Batteria 1
        csv += String(ltBuffer[i].v1, 2) + ",";
        csv += String(ltBuffer[i].c1, 2) + ",";
        csv += String(ltBuffer[i].rv1, 3) + ",";
        csv += String(ltBuffer[i].rc1, 3) + ",";
        
        // Batteria 2
        csv += String(ltBuffer[i].v2, 2) + ",";
        csv += String(ltBuffer[i].c2, 2) + ",";
        csv += String(ltBuffer[i].rv2, 3) + ",";
        csv += String(ltBuffer[i].rc2, 3) + ",";
        
        // Batteria 3
        csv += String(ltBuffer[i].v3, 2) + ",";
        csv += String(ltBuffer[i].c3, 2) + ",";
        csv += String(ltBuffer[i].rv3, 3) + ",";
        csv += String(ltBuffer[i].rc3, 3) + ",";
        
        // Motori
        csv += String(ltBuffer[i].m1) + ",";
        csv += String(ltBuffer[i].m2) + ",";
        csv += String(ltBuffer[i].m3) + ",";
        csv += "Flash\n";
      }
      
      delete[] ltBuffer;
      Serial.printf("✅ Esportati %d campioni da Flash\n", ltPoints);
    }
  }
  
  // ===== Export dati RAM (Short-term) - PIÙ RECENTI =====
  if (includeRam) {
    Serial.println("📤 Generazione CSV da RAM...");
    
    int max_points = 0;
    for (int i = 0; i < 3; i++) {
      int points = voltage_charts[i].filled ? 300 : voltage_charts[i].index;
      if (points > max_points) max_points = points;
    }
    
    // Calcola timestamp base (ultimi 5 minuti da ora)
    unsigned long baseTimestamp = millis() - (max_points * 1000);
    
    for (int i = 0; i < max_points; i++) {
      // Timestamp stimato
      csv += String(baseTimestamp + (i * 1000)) + ",";
      
      // Dati batterie (convertiti e raw)
      for (int j = 0; j < 3; j++) {
        int idx = ((voltage_charts[j].filled ? voltage_charts[j].index : 0) + i) % 300;
        csv += String(voltage_charts[j].values[idx], 2) + ",";  // Tensione convertita
        csv += String(current_charts[j].values[idx], 2) + ",";  // Corrente convertita
        csv += String(raw_voltage_charts[j].values[idx], 3) + ","; // Tensione raw
        csv += String(raw_current_charts[j].values[idx], 3) + ",";    // Corrente raw
      }
      
      // Dati motori PWM
      for (int j = 0; j < 3; j++) {
        int idx = ((motor_charts[j].filled ? motor_charts[j].index : 0) + i) % 300;
        csv += String((int)motor_charts[j].values[idx]);
        if (j < 2) csv += ",";
      }
      csv += ",RAM\n";
    }
    
    Serial.printf("✅ Esportati %d campioni da RAM\n", max_points);
  }
  
  // Genera nome file con timestamp
  String filename = "battery_data_" + String(millis()/1000) + ".csv";
  
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
  server.send(200, "text/csv", csv);
  
  Serial.println("📥 CSV inviato al client");
}

void handleClearCharts() {
  if (server.method() == HTTP_POST) {
    // Parametro opzionale per cancellare anche Flash
    String clearType = server.arg("type");  // "ram", "flash", "all" (default)
    
    bool clearRam = (clearType == "ram" || clearType == "" || clearType == "all");
    bool clearFlash = (clearType == "flash" || clearType == "" || clearType == "all");
    
    if (clearRam) {
      clearAllCharts();
    }
    
    if (clearFlash) {
      clearLongTermStorage();
    }
    
    String message = clearRam && clearFlash ? "Tutti i dati azzerati (RAM + Flash)" :
                     clearRam ? "Dati RAM azzerati" :
                     clearFlash ? "Dati Flash azzerati" : "Nessun dato azzerato";
    
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"" + message + "\"}");
  } else {
    server.send(405, "application/json", "{\"status\":\"error\",\"message\":\"Metodo non consentito\"}");
  }
}

void handleResetCalibration() {
  if (server.method() == HTTP_POST) {
    resetCalibrationToDefault();
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Calibrazione ripristinata ai valori di default\"}");
  } else {
    server.send(405, "application/json", "{\"status\":\"error\",\"message\":\"Metodo non consentito\"}");
  }
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Alix Blimp Battery Monitor</title>";
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
  html += "<h1>🔋 Alix Blimp Battery Monitor - TEST</h1>";
  html += "<div class='grid'>";
  html += "<div class='card' id='b0'><h3>6S Battery #1</h3><div class='row'><span class='muted'>Tensione</span><strong><span id='b0v'>-</span> V</strong></div><div class='row'><span class='muted'>Corrente</span><strong><span id='b0c'>-</span> A</strong></div><div class='row'><span class='muted'>Potenza</span><strong><span id='b0p'>-</span> W</strong></div></div>";
  html += "<div class='card' id='b1'><h3>6S Battery #2</h3><div class='row'><span class='muted'>Tensione</span><strong><span id='b1v'>-</span> V</strong></div><div class='row'><span class='muted'>Corrente</span><strong><span id='b1c'>-</span> A</strong></div><div class='row'><span class='muted'>Potenza</span><strong><span id='b1p'>-</span> W</strong></div></div>";
  html += "<div class='card' id='b2'><h3>4S Battery</h3><div class='row'><span class='muted'>Tensione</span><strong><span id='b2v'>-</span> V</strong></div><div class='row'><span class='muted'>Corrente</span><strong><span id='b2c'>-</span> A</strong></div><div class='row'><span class='muted'>Potenza</span><strong><span id='b2p'>-</span> W</strong></div></div>";
  html += "<div class='card'><h3>Status Sistema</h3><div class='row'><span class='muted'>Frequenza Loop</span><strong><span id='lf'>-</span> Hz</strong></div><div class='row'><span class='muted'>Input Right</span><strong><span id='ir'>-</span> μs</strong></div><div class='row'><span class='muted'>Input Left</span><strong><span id='il'>-</span> μs</strong></div><div class='row'><span class='muted'>Input Under</span><strong><span id='iu'>-</span> μs</strong></div><div class='status-badges'><span class='badge' id='dr'><span>Dir Right</span><strong>-</strong></span><span class='badge' id='dl'><span>Dir Left</span><strong>-</strong></span><span class='badge'><span>Output Right</span><strong id='or'>- μs</strong></span><span class='badge'><span>Output Left</span><strong id='ol'>- μs</strong></span></div></div>";
  html += "</div>";
  html += "<div class='footer'>Aggiornamento ogni 1s via <a href='/api'>/api</a> | <a href='/calibration'>Taratura</a> | <a href='/charts'>Grafici</a></div>";
  html += "</div><script>(function(){function q(id){return document.getElementById(id)};function setText(id,val,dec){q(id).textContent=(typeof dec==='number'?Number(val).toFixed(dec):val)};function upd(d){for(var i=0;i<3;i++){setText('b'+i+'v',d.batteries[i].voltage,2);setText('b'+i+'c',d.batteries[i].current,2);setText('b'+i+'p',d.batteries[i].power,1)}setText('lf',d.loop_frequency,1);setText('ir',d.autopilot.motor_right);setText('il',d.autopilot.motor_left);setText('iu',d.autopilot.motor_under);var dr=d.autopilot.dir_right,dl=d.autopilot.dir_left;q('dr').className='badge '+(dr?'ok':'err');q('dr').lastElementChild.textContent=dr?'Forward':'Reverse';q('dl').className='badge '+(dl?'ok':'err');q('dl').lastElementChild.textContent=dl?'Forward':'Reverse';setText('or',d.motors.right_pwm+' μs');setText('ol',d.motors.left_pwm+' μs')}function tick(){fetch('/api',{cache:'no-store'}).then(function(r){return r.json()}).then(upd).catch(function(){}).finally(function(){setTimeout(tick,1000)})}tick()})();</script></body></html>";
  server.send(200, "text/html", html);
}

void handleCalibrationPage() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Taratura Sensori</title>";
  html += "<style>";
  html += "body{font-family:Arial;background:#0b1220;color:#e6edf3;padding:20px;margin:0}";
  html += ".card{background:#111827;border:1px solid #1f2937;border-radius:10px;padding:16px;margin:10px 0}";
  html += ".mode-selector{display:flex;gap:10px;margin:10px 0}";
  html += ".mode-btn{padding:8px 16px;border:1px solid #1f2937;border-radius:4px;background:#0f172a;color:#e6edf3;cursor:pointer}";
  html += ".mode-btn.active{background:#3b82f6;border-color:#3b82f6}";
  html += ".mode-btn:hover{background:#1f2937}";
  html += "input,button,select{padding:8px;margin:4px;border:1px solid #1f2937;border-radius:4px;background:#0f172a;color:#e6edf3;font-size:14px}";
  html += "button{background:#3b82f6;cursor:pointer}";
  html += "button:hover{background:#2563eb}";
  html += "button.secondary{background:#6b7280}";
  html += "button.secondary:hover{background:#4b5563}";
  html += "label{display:block;margin:8px 0 4px;color:#cbd5e1;font-size:14px}";
  html += ".form-group{margin:10px 0}";
  html += ".form-row{display:flex;gap:10px;align-items:end}";
  html += ".form-row input{flex:1}";
  html += ".current-values{background:#1f2937;padding:10px;border-radius:5px;margin:10px 0}";
  html += ".current-values h4{margin:0 0 10px;color:#10b981}";
  html += ".value-row{display:flex;justify-content:space-between;margin:5px 0}";
  html += "a{color:#60a5fa;text-decoration:none}";
  html += ".hidden{display:none}";
  html += "</style></head><body>";
  
  html += "<h1>⚙️ Taratura Sensori</h1>";
  html += "<div class='mode-selector'>";
  html += "<button class='mode-btn active' onclick='setMode(\"simple\")'>🔧 Semplice</button>";
  html += "<button class='mode-btn' onclick='setMode(\"two_point\")'>📊 Due Punti</button>";
  html += "<button class='mode-btn' onclick='setMode(\"advanced\")'>⚙️ Avanzata</button>";
  html += "</div>";
  
  for(int i = 0; i < 3; i++) {
    String batteryName = (i < 2) ? "6S Battery #" + String(i+1) : "4S Battery";
    html += "<div class='card'><h3>" + batteryName + "</h3>";
    
    // Valori correnti
    html += "<div class='current-values'>";
    html += "<h4>📊 Valori Correnti</h4>";
    html += "<div class='value-row'><span>Tensione Raw:</span><span id='rawV" + String(i) + "'>-</span></div>";
    html += "<div class='value-row'><span>Corrente Raw:</span><span id='rawC" + String(i) + "'>-</span></div>";
    html += "<div class='value-row'><span>Tensione Convertita:</span><span id='convV" + String(i) + "'>-</span></div>";
    html += "<div class='value-row'><span>Corrente Convertita:</span><span id='convC" + String(i) + "'>-</span></div>";
    html += "</div>";
    
    // Modalità semplice
    html += "<div id='simple" + String(i) + "' class='calibration-mode'>";
    html += "<h4>🔧 Calibrazione Semplice</h4>";
    html += "<p style='color:#94a3b8;font-size:12px;margin:5px 0'>Inserisci i valori misurati con multimetro. Il sistema calcolerà automaticamente i parametri di conversione.</p>";
    html += "<div class='form-group'>";
    html += "<label>Tensione Misurata (V):</label>";
    html += "<input type='number' id='measV" + String(i) + "' step='0.01' placeholder='es. 24.5'>";
    html += "</div>";
    html += "<div class='form-group'>";
    html += "<label>Corrente Misurata (A):</label>";
    html += "<input type='number' id='measC" + String(i) + "' step='0.01' placeholder='es. 5.2'>";
    html += "</div>";
    html += "<button onclick='saveSimpleCal(" + String(i) + ")'>💾 Calibra Semplice</button>";
    html += "<button onclick='autoCalibrate(" + String(i) + ")' style='background:#10b981;color:white;margin-left:10px'>🔄 Calibra Auto</button>";
    html += "<button onclick='resetCalibration()' style='background:#ef4444;color:white;margin-left:10px'>🔄 Reset Calibrazione</button>";
    html += "</div>";
    
    // Modalità due punti
    html += "<div id='two_point" + String(i) + "' class='calibration-mode hidden'>";
    html += "<h4>📊 Calibrazione a Due Punti</h4>";
    html += "<p style='color:#94a3b8;font-size:12px;margin:5px 0'>Misura due punti diversi per maggiore precisione.</p>";
    html += "<div class='form-row'>";
    html += "<div><label>Punto 1 - Tensione (V):</label><input type='number' id='v1m" + String(i) + "' step='0.01'></div>";
    html += "<div><label>Punto 1 - Corrente (A):</label><input type='number' id='i1m" + String(i) + "' step='0.01'></div>";
    html += "</div>";
    html += "<div class='form-row'>";
    html += "<div><label>Punto 2 - Tensione (V):</label><input type='number' id='v2m" + String(i) + "' step='0.01'></div>";
    html += "<div><label>Punto 2 - Corrente (A):</label><input type='number' id='i2m" + String(i) + "' step='0.01'></div>";
    html += "</div>";
    html += "<button onclick='saveTwoPointCal(" + String(i) + ")'>💾 Calibra Due Punti</button>";
    html += "</div>";
    
    // Modalità avanzata
    html += "<div id='advanced" + String(i) + "' class='calibration-mode hidden'>";
    html += "<h4>⚙️ Calibrazione Avanzata</h4>";
    html += "<p style='color:#94a3b8;font-size:12px;margin:5px 0'>Imposta manualmente tutti i parametri di calibrazione.</p>";
    html += "<div class='form-group'>";
    html += "<label>Tensione Offset:</label>";
    html += "<input type='number' id='voff" + String(i) + "' step='0.01'>";
    html += "</div>";
    html += "<div class='form-group'>";
    html += "<label>Tensione Scala:</label>";
    html += "<input type='number' id='vscale" + String(i) + "' step='0.01' value='1.00'>";
    html += "</div>";
    html += "<div class='form-group'>";
    html += "<label>Corrente Offset:</label>";
    html += "<input type='number' id='coff" + String(i) + "' step='0.01'>";
    html += "</div>";
    html += "<div class='form-group'>";
    html += "<label>Corrente Scala:</label>";
    html += "<input type='number' id='cscale" + String(i) + "' step='0.01' value='1.00'>";
    html += "</div>";
    html += "<div class='form-group'>";
    html += "<label>Rapporto Partitore:</label>";
    html += "<input type='number' id='divider" + String(i) + "' step='0.1' value='" + String((i<2)?8.4:5.6) + "'>";
    html += "</div>";
    html += "<button onclick='saveAdvancedCal(" + String(i) + ")'>💾 Salva Avanzata</button>";
    html += "</div>";
    
    html += "</div>";
  }
  
  html += "<a href='/'>← Torna al Monitor</a>";
  
  html += "<script>";
  html += "let currentMode='simple';";
  html += "let calibrationData={};";
  html += "";
  html += "function setMode(mode){";
  html += "currentMode=mode;";
  html += "document.querySelectorAll('.mode-btn').forEach(btn=>btn.classList.remove('active'));";
  html += "event.target.classList.add('active');";
  html += "document.querySelectorAll('.calibration-mode').forEach(el=>el.classList.add('hidden'));";
  html += "for(let i=0;i<3;i++){";
  html += "document.getElementById(mode+i).classList.remove('hidden');";
  html += "}";
  html += "}";
  html += "";
  html += "function loadCalibrationData(){";
  html += "fetch('/calibration')";
  html += ".then(r=>r.json())";
  html += ".then(data=>{";
  html += "calibrationData=data;";
  html += "for(let i=0;i<3;i++){";
  html += "let bat='battery_'+i;";
  html += "if(data[bat]){";
  html += "document.getElementById('rawV'+i).textContent=data[bat].current_raw_voltage.toFixed(3)+'V';";
  html += "document.getElementById('rawC'+i).textContent=data[bat].current_raw_current_voltage.toFixed(3)+'V';";
  html += "document.getElementById('convV'+i).textContent=data[bat].current_measured_voltage.toFixed(2)+'V';";
  html += "document.getElementById('convC'+i).textContent=data[bat].current_measured_current.toFixed(2)+'A';";
  html += "}";
  html += "}";
  html += "})";
  html += ".catch(e=>console.error('Errore caricamento:',e));";
  html += "}";
  html += "";
  html += "function saveSimpleCal(b){";
  html += "let measV=parseFloat(document.getElementById('measV'+b).value);";
  html += "let measC=parseFloat(document.getElementById('measC'+b).value);";
  html += "if(isNaN(measV)||isNaN(measC)){alert('Inserisci valori validi');return;}";
  html += "fetch('/calibration',{method:'POST',headers:{'Content-Type':'application/json'},";
  html += "body:JSON.stringify({battery:b,mode:'simple',measured_voltage:measV,measured_current:measC})})";
  html += ".then(()=>{alert('Calibrazione semplice salvata!');loadCalibrationData();})";
  html += ".catch(e=>alert('Errore: '+e));";
  html += "}";
  html += "";
  html += "function saveTwoPointCal(b){";
  html += "let v1m=parseFloat(document.getElementById('v1m'+b).value);";
  html += "let i1m=parseFloat(document.getElementById('i1m'+b).value);";
  html += "let v2m=parseFloat(document.getElementById('v2m'+b).value);";
  html += "let i2m=parseFloat(document.getElementById('i2m'+b).value);";
  html += "if(isNaN(v1m)||isNaN(i1m)||isNaN(v2m)||isNaN(i2m)){alert('Inserisci tutti i valori');return;}";
  html += "let bat='battery_'+b;";
  html += "if(!calibrationData[bat]){alert('Dati non disponibili');return;}";
  html += "fetch('/calibration',{method:'POST',headers:{'Content-Type':'application/json'},";
  html += "body:JSON.stringify({battery:b,mode:'two_point',";
  html += "v1_measured:v1m,v1_raw:calibrationData[bat].current_raw_voltage,";
  html += "v2_measured:v2m,v2_raw:calibrationData[bat].current_raw_voltage,";
  html += "i1_measured:i1m,i1_raw:calibrationData[bat].current_raw_current_voltage,";
  html += "i2_measured:i2m,i2_raw:calibrationData[bat].current_raw_current_voltage})})";
  html += ".then(()=>{alert('Calibrazione a due punti salvata!');loadCalibrationData();})";
  html += ".catch(e=>alert('Errore: '+e));";
  html += "}";
  html += "";
  html += "function saveAdvancedCal(b){";
  html += "fetch('/calibration',{method:'POST',headers:{'Content-Type':'application/json'},";
  html += "body:JSON.stringify({battery:b,type:'voltage',offset:parseFloat(document.getElementById('voff'+b).value),";
  html += "scale:parseFloat(document.getElementById('vscale'+b).value),divider_ratio:parseFloat(document.getElementById('divider'+b).value)})})";
  html += ".then(()=>alert('Taratura tensione salvata!'));";
  html += "fetch('/calibration',{method:'POST',headers:{'Content-Type':'application/json'},";
  html += "body:JSON.stringify({battery:b,type:'current',offset:parseFloat(document.getElementById('coff'+b).value),";
  html += "scale:parseFloat(document.getElementById('cscale'+b).value)})})";
  html += ".then(()=>{alert('Taratura corrente salvata!');loadCalibrationData();})";
  html += ".catch(e=>alert('Errore: '+e));";
  html += "}";
  html += "";
  html += "function autoCalibrate(b){";
  html += "if(!confirm('Calibrazione automatica: il sistema calcolerà i coefficienti basandosi sui dati raw attuali. Continuare?'))return;";
  html += "var bat='battery_'+b;";
  html += "if(!calibrationData[bat]){alert('Dati non disponibili');return;}";
  html += "var rawV=calibrationData[bat].current_raw_voltage;";
  html += "var rawC=calibrationData[bat].current_raw_current_voltage;";
  html += "var measV=calibrationData[bat].current_measured_voltage;";
  html += "var measC=calibrationData[bat].current_measured_current;";
  html += "if(rawV<0.1||rawC<0.1){alert('Dati raw insufficienti per calibrazione');return;}";
  html += "fetch('/calibration',{method:'POST',headers:{'Content-Type':'application/json'},";
  html += "body:JSON.stringify({battery:b,mode:'auto',raw_voltage:rawV,raw_current:rawC,measured_voltage:measV,measured_current:measC})})";
  html += ".then(function(r){return r.json();})";
  html += ".then(function(d){";
  html += "if(d.status==='ok'){";
  html += "alert('Calibrazione automatica completata!\\nNuovi coefficienti calcolati e salvati.');";
  html += "loadCalibrationData();";
  html += "}else{";
  html += "alert('Errore durante la calibrazione: '+d.message);";
  html += "}";
  html += "})";
  html += ".catch(function(e){alert('Errore: '+e);});";
  html += "}";
  html += "";
  html += "function resetCalibration(){";
  html += "if(!confirm('ATTENZIONE: Questo ripristinerà TUTTE le impostazioni di calibrazione ai valori di default. Continuare?'))return;";
  html += "fetch('/reset-calibration',{method:'POST'})";
  html += ".then(function(r){return r.json();})";
  html += ".then(function(d){";
  html += "if(d.status==='ok'){";
  html += "alert('Calibrazione ripristinata ai valori di default!\\nLe impostazioni sono state salvate permanentemente.');";
  html += "loadCalibrationData();";
  html += "}else{";
  html += "alert('Errore durante il reset: '+d.message);";
  html += "}";
  html += "})";
  html += ".catch(function(e){alert('Errore: '+e);});";
  html += "}";
  html += "";
  html += "// Aggiorna dati ogni 500ms per calibrazione in tempo reale";
  html += "setInterval(loadCalibrationData,500);";
  html += "loadCalibrationData();";
  html += "</script></body></html>";
  server.send(200, "text/html", html);
}

void handleChartsPage() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Grafici Storici</title>";
  html += "<style>";
  html += "body{font-family:Arial;background:#0b1220;color:#e6edf3;padding:20px;margin:0}";
  html += ".card{background:#111827;border:1px solid #1f2937;border-radius:10px;padding:16px;margin:10px 0}";
  html += ".chart{height:400px;width:100%;border:1px solid #1f2937;background:#0f172a;position:relative;overflow:hidden}";
  html += ".chart-canvas{width:100%;height:100%;background:#0f172a}";
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
  html += ".legend{display:flex;gap:15px;margin:10px 0;flex-wrap:wrap}";
  html += ".legend-item{display:flex;align-items:center;gap:5px;font-size:12px}";
  html += ".legend-color{width:12px;height:12px;border-radius:2px}";
  html += ".stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:10px;margin:10px 0}";
  html += ".stat-card{background:#1f2937;padding:10px;border-radius:5px;text-align:center}";
  html += ".stat-value{font-size:18px;font-weight:bold;color:#10b981}";
  html += ".stat-label{font-size:12px;color:#94a3b8}";
  html += "</style></head><body>";
  html += "<h1>📈 Grafici Storici Batterie</h1>";
  html += "<div class='controls'>";
  html += "<label>Scala temporale:</label>";
  html += "<select id='timeScale' onchange='changeScale()'>";
  html += "<optgroup label='📊 RAM (Veloce - 1Hz)'>";
  html += "<option value='10s'>10 secondi</option>";
  html += "<option value='1m' selected>1 minuto</option>";
  html += "<option value='5m'>5 minuti</option>";
  html += "</optgroup>";
  html += "<optgroup label='💾 Flash (Lungo - 0.1Hz)'>";
  html += "<option value='10m'>10 minuti</option>";
  html += "<option value='30m'>30 minuti</option>";
  html += "<option value='1h'>1 ora</option>";
  html += "<option value='4h'>4 ore</option>";
  html += "</optgroup>";
  html += "</select>";
  html += "<button onclick='exportCSV()'>📥 Esporta CSV</button>";
  html += "<button onclick='update()'>🔄 Aggiorna</button>";
  html += "<button onclick='toggleAutoUpdate()'>⏸️ Auto</button>";
  html += "<button onclick='resetZoom()'>🔍 Reset Zoom</button>";
  html += "<button onclick='toggleGrid()'>📐 Griglia</button>";
  html += "<button onclick='clearCharts(\"ram\")' style='background:#f59e0b'>🗑️ Azzera RAM</button>";
  html += "<button onclick='clearCharts(\"all\")' style='background:#ef4444;color:white'>🗑️ Azzera Tutto</button>";
  html += "<span id='storageInfo' style='margin-left:10px;color:#94a3b8;font-size:12px'></span>";
  html += "</div>";
  
  html += "<div class='card'><h3>📊 Tensioni (V)</h3>";
  html += "<div class='legend'>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#3b82f6'></div>6S#1</div>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#10b981'></div>6S#2</div>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#f59e0b'></div>4S</div>";
  html += "</div>";
  html += "<canvas id='vChart' class='chart-canvas'></canvas></div>";
  
  html += "<div class='card'><h3>⚡ Correnti (A)</h3>";
  html += "<div class='legend'>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#3b82f6'></div>6S#1</div>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#10b981'></div>6S#2</div>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#f59e0b'></div>4S</div>";
  html += "</div>";
  html += "<canvas id='cChart' class='chart-canvas'></canvas></div>";
  
  html += "<div class='card'><h3>🔧 Dati Raw Tensione (V)</h3>";
  html += "<div class='legend'>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#3b82f6'></div>6S#1</div>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#10b981'></div>6S#2</div>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#f59e0b'></div>4S</div>";
  html += "</div>";
  html += "<canvas id='rvChart' class='chart-canvas'></canvas></div>";
  
  html += "<div class='card'><h3>🔧 Dati Raw Corrente (V)</h3>";
  html += "<div class='legend'>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#3b82f6'></div>6S#1</div>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#10b981'></div>6S#2</div>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#f59e0b'></div>4S</div>";
  html += "</div>";
  html += "<canvas id='rcChart' class='chart-canvas'></canvas></div>";
  
  html += "<div class='card'><h3>🚁 Input Motori PWM (μs)</h3>";
  html += "<div class='legend'>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#3b82f6'></div>Right</div>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#10b981'></div>Left</div>";
  html += "<div class='legend-item'><div class='legend-color' style='background:#f59e0b'></div>Under</div>";
  html += "</div>";
  html += "<canvas id='mChart' class='chart-canvas'></canvas></div>";
  
  html += "<div class='card'><h3>📋 Statistiche</h3>";
  html += "<div class='stats' id='stats'></div></div>";
  
  html += "<div class='card'><h3>📋 Dati Storici</h3>";
  html += "<table class='data-table'>";
  html += "<tr><th>Batteria</th><th>Ultima Tensione</th><th>Ultima Corrente</th><th>Campioni</th><th>Min</th><th>Max</th><th>Media</th></tr>";
  html += "<tr><td>6S#1</td><td id='v0'>-</td><td id='c0'>-</td><td id='p0'>-</td><td id='min0'>-</td><td id='max0'>-</td><td id='avg0'>-</td></tr>";
  html += "<tr><td>6S#2</td><td id='v1'>-</td><td id='c1'>-</td><td id='p1'>-</td><td id='min1'>-</td><td id='max1'>-</td><td id='avg1'>-</td></tr>";
  html += "<tr><td>4S</td><td id='v2'>-</td><td id='c2'>-</td><td id='p2'>-</td><td id='min2'>-</td><td id='max2'>-</td><td id='avg2'>-</td></tr>";
  html += "</table></div>";
  html += "<a href='/'>← Torna al Monitor</a>";
  
  html += "<script>";
  html += "let currentScale='1m';";
  html += "let autoUpdate=true;";
  html += "let vCtx,cCtx,rvCtx,rcCtx,mCtx;";
  html += "let colors=['#3b82f6','#10b981','#f59e0b'];";
  html += "let names=['6S#1','6S#2','4S'];";
  html += "let showGrid=true;";
  html += "";
  html += "function initCharts(){";
  html += "vCtx=document.getElementById('vChart').getContext('2d');";
  html += "cCtx=document.getElementById('cChart').getContext('2d');";
  html += "rvCtx=document.getElementById('rvChart').getContext('2d');";
  html += "rcCtx=document.getElementById('rcChart').getContext('2d');";
  html += "mCtx=document.getElementById('mChart').getContext('2d');";
  html += "}";
  html += "";
  html += "function updateStorageInfo(storageInfo){";
  html += "if(!storageInfo||!storageInfo.initialized)return;";
  html += "let info='💾 Flash: '+storageInfo.total_points+'/'+storageInfo.max_points+' campioni';";
  html += "if(storageInfo.coverage_hours){";
  html += "info+=' ('+storageInfo.coverage_hours.toFixed(1)+'h)';";
  html += "}";
  html += "info+=' | '+storageInfo.spiffs_used_kb+'/'+storageInfo.spiffs_total_kb+'KB';";
  html += "document.getElementById('storageInfo').textContent=info;";
  html += "}";
  html += "";
  html += "function changeScale(){";
  html += "currentScale=document.getElementById('timeScale').value;";
  html += "update();";
  html += "}";
  html += "";
  html += "function toggleAutoUpdate(){";
  html += "autoUpdate=!autoUpdate;";
  html += "var btn=document.querySelector('button[onclick=\"toggleAutoUpdate()\"]');";
  html += "btn.textContent=autoUpdate?'⏸️ Auto':'▶️ Auto';";
  html += "}";
  html += "";
  html += "function resetZoom(){";
  html += "update();";
  html += "}";
  html += "";
  html += "function toggleGrid(){";
  html += "showGrid=!showGrid;";
  html += "update();";
  html += "}";
  html += "";
  html += "function update(){";
  html += "fetch('/charts-data?scale='+currentScale)";
  html += ".then(function(r){return r.json();})";
  html += ".then(function(d){";
  html += "if(d){";
  html += "updateTable(d);";
  html += "updateStats(d);";
  html += "drawChart(vCtx,d.voltage || [],'voltage');";
  html += "drawChart(cCtx,d.current || [],'current');";
  html += "drawChart(rvCtx,d.raw_voltage || [],'raw');";
  html += "drawChart(rcCtx,d.raw_current || [],'raw');";
  html += "drawChart(mCtx,d.motors || [],'motor');";
  html += "if(d.storage_info){updateStorageInfo(d.storage_info);}";
  html += "}";
  html += "})";
  html += ".catch(function(e){console.error('Errore:',e);});";
  html += "}";
  html += "";
  html += "function updateTable(d){";
  html += "for(var i=0;i<3;i++){";
  html += "if(d.current_batteries && d.current_batteries[i]){";
  html += "var bat=d.current_batteries[i];";
  html += "document.getElementById('v'+i).textContent=bat.voltage.toFixed(2);";
  html += "document.getElementById('c'+i).textContent=bat.current.toFixed(2);";
  html += "if(d.voltage_stats && d.voltage_stats[i]){";
  html += "document.getElementById('p'+i).textContent=d.voltage_stats[i].samples;";
  html += "}else{";
  html += "document.getElementById('p'+i).textContent='0';";
  html += "}";
  html += "}else{";
  html += "document.getElementById('v'+i).textContent='N/A';";
  html += "document.getElementById('c'+i).textContent='N/A';";
  html += "document.getElementById('p'+i).textContent='0';";
  html += "}";
  html += "if(d.voltage_stats && d.voltage_stats[i] && d.voltage_stats[i].samples>0){";
  html += "document.getElementById('min'+i).textContent=d.voltage_stats[i].min.toFixed(2);";
  html += "document.getElementById('max'+i).textContent=d.voltage_stats[i].max.toFixed(2);";
  html += "document.getElementById('avg'+i).textContent=d.voltage_stats[i].avg.toFixed(2);";
  html += "}else{";
  html += "document.getElementById('min'+i).textContent='N/A';";
  html += "document.getElementById('max'+i).textContent='N/A';";
  html += "document.getElementById('avg'+i).textContent='N/A';";
  html += "}";
  html += "}";
  html += "}";
  html += "";
  html += "function updateStats(d){";
  html += "var html='';";
  html += "for(var i=0;i<3;i++){";
  html += "html+='<div class=\"stat-card\">';";
  html += "html+='<div class=\"stat-value\">'+names[i]+'</div>';";
  html += "if(d.current_batteries && d.current_batteries[i]){";
  html += "var bat=d.current_batteries[i];";
  html += "html+='<div class=\"stat-label\">Tensione: '+bat.voltage.toFixed(2)+'V</div>';";
  html += "html+='<div class=\"stat-label\">Corrente: '+bat.current.toFixed(2)+'A</div>';";
  html += "html+='<div class=\"stat-label\">Potenza: '+bat.power.toFixed(1)+'W</div>';";
  html += "}else{";
  html += "html+='<div class=\"stat-label\">Tensione: N/A</div>';";
  html += "html+='<div class=\"stat-label\">Corrente: N/A</div>';";
  html += "html+='<div class=\"stat-label\">Potenza: N/A</div>';";
  html += "}";
  html += "html+='</div>';";
  html += "}";
  html += "var statsEl=document.getElementById('stats');";
  html += "if(statsEl){";
  html += "statsEl.innerHTML=html;";
  html += "}";
  html += "}";
  html += "";
  html += "function drawChart(ctx,data,chartType){";
  html += "var canvas=ctx.canvas;";
  html += "var width=canvas.width=canvas.offsetWidth;";
  html += "var height=canvas.height=canvas.offsetHeight;";
  html += "ctx.clearRect(0,0,width,height);";
  html += "";
  html += "var min,max,range;";
  html += "if(chartType==='voltage'){";
  html += "min=0;max=40;range=40;";
  html += "}else if(chartType==='current'){";
  html += "min=-30;max=30;range=60;";
  html += "}else{";
  html += "var allValues=[];";
  html += "for(var i=0;i<data.length;i++){";
  html += "for(var j=0;j<data[i].length;j++){";
  html += "allValues.push(data[i][j]);";
  html += "}";
  html += "}";
  html += "if(allValues.length===0){";
  html += "ctx.fillStyle='#94a3b8';";
  html += "ctx.font='16px Arial';";
  html += "ctx.textAlign='center';";
  html += "ctx.fillText('Nessun dato disponibile',width/2,height/2);";
  html += "return;";
  html += "}";
  html += "min=Math.min.apply(Math,allValues);";
  html += "max=Math.max.apply(Math,allValues);";
  html += "range=max-min;";
  html += "if(range===0)range=1;";
  html += "}";
  html += "";
  html += "if(showGrid){";
  html += "ctx.strokeStyle='#1f2937';";
  html += "ctx.lineWidth=1;";
  html += "ctx.fillStyle='#94a3b8';";
  html += "ctx.font='10px Arial';";
  html += "for(var i=0;i<=10;i++){";
  html += "var y=height-(i*height/10);";
  html += "ctx.beginPath();";
  html += "ctx.moveTo(0,y);";
  html += "ctx.lineTo(width,y);";
  html += "ctx.stroke();";
  html += "var value=min+(i*(max-min)/10);";
  html += "ctx.fillText(value.toFixed(1),5,y-2);";
  html += "}";
  html += "for(var i=0;i<=20;i++){";
  html += "var x=(i*width/20);";
  html += "ctx.beginPath();";
  html += "ctx.moveTo(x,0);";
  html += "ctx.lineTo(x,height);";
  html += "ctx.stroke();";
  html += "}";
  html += "}";
  html += "";
  html += "if(chartType==='current'){";
  html += "ctx.strokeStyle='#ef4444';";
  html += "ctx.lineWidth=1;";
  html += "ctx.setLineDash([5,5]);";
  html += "var zeroY=height-((0-min)/range)*height;";
  html += "ctx.beginPath();";
  html += "ctx.moveTo(0,zeroY);";
  html += "ctx.lineTo(width,zeroY);";
  html += "ctx.stroke();";
  html += "ctx.setLineDash([]);";
  html += "}";
  html += "";
  html += "if(chartType==='voltage'){";
  html += "ctx.strokeStyle='#10b981';";
  html += "ctx.lineWidth=1;";
  html += "ctx.setLineDash([5,5]);";
  html += "var nominalY=height-((24-min)/range)*height;";
  html += "ctx.beginPath();";
  html += "ctx.moveTo(0,nominalY);";
  html += "ctx.lineTo(width,nominalY);";
  html += "ctx.stroke();";
  html += "ctx.setLineDash([]);";
  html += "}";
  html += "";
  html += "if(data && data.length>0){";
  html += "for(var i=0;i<data.length;i++){";
  html += "if(!data[i] || data[i].length<2)continue;";
  html += "var values=data[i];";
  html += "ctx.strokeStyle=colors[i] || '#3b82f6';";
  html += "ctx.lineWidth=2;";
  html += "ctx.beginPath();";
  html += "for(var j=0;j<values.length;j++){";
  html += "var x=(j/(values.length-1))*width;";
  html += "var y=height-((values[j]-min)/range)*height;";
  html += "if(j===0)ctx.moveTo(x,y);";
  html += "else ctx.lineTo(x,y);";
  html += "}";
  html += "ctx.stroke();";
  html += "}";
  html += "}";
  html += "}";
  html += "";
  html += "function exportCSV(){";
  html += "window.open('/csv','_blank');";
  html += "}";
  html += "";
  html += "function clearCharts(type){";
  html += "let msg=type==='ram'?'RAM (5 minuti)':type==='flash'?'Flash (4 ore)':'TUTTI i dati (RAM + Flash)';";
  html += "if(confirm('Sei sicuro di voler azzerare '+msg+'?')){";
  html += "fetch('/clear-charts?type='+(type||'all'),{method:'POST'})";
  html += ".then(function(r){return r.json();})";
  html += ".then(function(d){";
  html += "if(d.status==='ok'){";
  html += "alert(d.message);";
  html += "update();";
  html += "}else{";
  html += "alert('Errore: '+d.message);";
  html += "}";
  html += "})";
  html += ".catch(function(e){alert('Errore: '+e);});";
  html += "}";
  html += "}";
  html += "";
  html += "window.onload=function(){";
  html += "initCharts();";
  html += "update();";
  html += "setInterval(function(){if(autoUpdate)update();},5000);";
  html += "};";
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
  autopilot["motor_under"] = autopilot_input.motor_under;
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
  Serial.println("🚀 AlixBlimp Battery Monitor & Motor Control v2.0");
  Serial.println("========================================");
  Serial.println("📦 Storage Multi-Rate: RAM (5min @ 1Hz) + Flash (4h @ 0.1Hz)");
  Serial.println();
  
  // Configurazione Pin
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  
  // Inizializzazione pin direzione (neutral)
  digitalWrite(DIR_RIGHT_PIN, LOW);
  digitalWrite(DIR_LEFT_PIN, LOW);
  
  // Configurazione ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // 0-3.3V range
  
  // Inizializzazione SPIFFS per storage lungo termine
  if (!initLongTermStorage()) {
    Serial.println("⚠️ Storage lungo termine non disponibile - continuo solo con RAM");
  }
  
  // Inizializzazione Taratura
  initCalibration();
  
  // Carica le impostazioni di calibrazione salvate dalla memoria flash
  loadCalibrationFromFlash();
  
  // Inizializzazione Grafici
  for (int i = 0; i < 3; i++) {
    initChart(&voltage_charts[i]);
    initChart(&current_charts[i]);
    initChart(&raw_voltage_charts[i]);
    initChart(&raw_current_charts[i]);
    initChart(&motor_charts[i]);
  }
  
  // Configurazione PWM Output (LEDC channels) - CORRETTA per Arduino ESP32
  ledcSetup(PWM_OUT_RIGHT_CHANNEL, PWM_FREQ, PWM_RESOLUTION);  // 50Hz, 16-bit resolution
  ledcAttachPin(PWM_OUT_RIGHT, PWM_OUT_RIGHT_CHANNEL);
  ledcSetup(PWM_OUT_LEFT_CHANNEL, PWM_FREQ, PWM_RESOLUTION);   // 50Hz, 16-bit resolution  
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
  server.on("/clear-charts", HTTP_POST, handleClearCharts);
  server.on("/reset-calibration", HTTP_POST, handleResetCalibration);
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
  
  // Aggiorna grafici RAM (ogni 1s)
  updateCharts();
  
  // Salva dati su Flash per storage lungo termine (ogni 10s)
  saveLongTermDataPoint();
  
  // Web Server
  server.handleClient();
  
  // Piccola pausa per stabilità
  delay(1);
}
