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
#define PWM_IN_UNDER       22    // GPIO22 - Motori Sottostanti (era GPIO5 - problematico al boot)

// Digital Input per Direzione
#define DIR_RIGHT_PIN      21    // GPIO21 - Direzione Motore Destro
#define DIR_LEFT_PIN       23    // GPIO23 - Direzione Motore Sinistro

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

// Parametri Sensori Corrente ACS758 con Partitore (valori di default)
#define ACS758_SENSITIVITY_DEFAULT 0.01  // 10mV/A (dopo partitore 5V→3V)
#define ACS758_VREF_DEFAULT        1.46  // Tensione di riferimento @ 0A (dopo partitore)

// Variabili globali per parametri calibrati
float ACS758_SENSITIVITY = ACS758_SENSITIVITY_DEFAULT;
float ACS758_VREF = ACS758_VREF_DEFAULT;

// Parametri Partitori Tensione
#define DIVIDER_6S_RATIO   11   // 33.6V -> 3.05V
#define DIVIDER_4S_RATIO   11   // 12.6V -> 1.15V

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
#define PWM_OUT_RIGHT_CHANNEL  0    // LEDC Channel 0 - PWM Motore Destro
#define PWM_OUT_LEFT_CHANNEL   1    // LEDC Channel 1 - PWM Motore Sinistro
#define DIR_RIGHT_CHANNEL      2    // LEDC Channel 2 - Direzione Motore Destro
#define DIR_LEFT_CHANNEL       3    // LEDC Channel 3 - Direzione Motore Sinistro

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
  // Parametri sensore corrente calibrati
  float acs758_vref;        // VREF calibrato per questo sensore
  float acs758_sensitivity; // Sensibilità calibrata per questo sensore
};

// Strutture per Grafici (dati storici ottimizzati)
struct ChartData {
  float values[120];        // 120 punti (2 minuti a 1Hz) - ultimi 2 minuti
  int index;                // Indice corrente
  bool filled;              // Buffer riempito
  unsigned long last_update; // Ultimo aggiornamento
  float min_value;          // Valore minimo nel buffer
  float max_value;          // Valore massimo nel buffer
  float avg_value;          // Valore medio nel buffer
  unsigned long total_samples; // Totale campioni raccolti
};

// Struttura per dati in Flash (salvato ogni 2 minuti, fino a 4 ore)
struct LogEntry {
  unsigned long timestamp;  // Timestamp in millisecondi
  float voltage;            // Tensione
  float current;            // Corrente
  float power;              // Potenza
};

// Buffer RAM (2 minuti @ 1Hz = 120 campioni) - condiviso con grafici
// Usa direttamente ChartData esistente

// Snapshot temporaneo per salvataggio (1 snapshot = 2 minuti)
struct FlashLogSnapshot {
  LogEntry entries[120];    // 120 campioni (2 minuti @ 1Hz)
  unsigned long timestamp;  // Timestamp dello snapshot
};

// Metadati log in SPIFFS (solo indici, non i dati)
struct FlashLog {
  int snapshot_count;        // Numero di snapshot salvati (max 120)
  int current_index;         // Indice corrente nel buffer circolare (0-119)
  unsigned long last_save;   // Ultimo salvataggio in flash
  bool filled;               // Buffer riempito almeno una volta (≥120 snapshot)
  bool save_pending;         // Flag: snapshot in coda per salvataggio asincrono
};

// Buffer per salvataggio asincrono (evita blocchi durante controllo PWM)
struct AsyncSaveBuffer {
  FlashLogSnapshot snapshot; // Snapshot da salvare
  int battery_index;         // Indice batteria (0-2)
  int file_index;            // Indice file (0-119)
  bool ready;                // Buffer pronto per salvataggio
  bool saving;               // Salvataggio in corso
};

// File SPIFFS: /logs/bat0_XXX.bin dove XXX = indice snapshot (000-119)

// Dati Sistema
BatteryData batteries[3];  // 0=6S#1, 1=6S#2, 2=4S
PWMData autopilot_input;
MotorOutput motor_output;

// Dati Taratura
CalibrationData calibration[3];  // Taratura per ogni batteria

// Dati Grafici (ultimi 2 minuti, aggiornati ogni secondo)
ChartData voltage_charts[3];     // Grafici tensione
ChartData current_charts[3];     // Grafici corrente
ChartData raw_voltage_charts[3]; // Grafici tensione raw
ChartData raw_current_charts[3]; // Grafici corrente raw
ChartData motor_charts[3];       // Grafici PWM motori (right, left, under)

// Metadati Log in SPIFFS (solo indici, dati su file system)
FlashLog flash_logs[3];          // Metadati per ogni batteria (~12 bytes × 3)

// Buffer asincrono per salvataggio SPIFFS (evita blocchi PWM)
AsyncSaveBuffer async_save_queue[3];  // Una coda per batteria
int async_save_count = 0;              // Numero di snapshot in coda

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
    preferences.putFloat((prefix + "acs_vref").c_str(), calibration[i].acs758_vref);
    preferences.putFloat((prefix + "acs_sens").c_str(), calibration[i].acs758_sensitivity);
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
    calibration[i].acs758_vref = preferences.getFloat((prefix + "acs_vref").c_str(), ACS758_VREF_DEFAULT);
    calibration[i].acs758_sensitivity = preferences.getFloat((prefix + "acs_sens").c_str(), ACS758_SENSITIVITY_DEFAULT);
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
    
    // Valori di default per partitori
    if (i < 2) { // 6S batteries
      calibration[i].divider_ratio = DIVIDER_6S_RATIO;
    } else { // 4S battery
      calibration[i].divider_ratio = DIVIDER_4S_RATIO;
    }
    
    calibration[i].acs758_vref = ACS758_VREF_DEFAULT;
    calibration[i].acs758_sensitivity = ACS758_SENSITIVITY_DEFAULT;
  }
  
  // Ripristina anche i parametri globali
  ACS758_VREF = ACS758_VREF_DEFAULT;
  ACS758_SENSITIVITY = ACS758_SENSITIVITY_DEFAULT;
  
  Serial.println("🔄 Impostazioni di calibrazione ripristinate ai valori di default");
}

// Inizializza la calibrazione con valori di default
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
    
    calibration[i].acs758_vref = ACS758_VREF_DEFAULT;
    calibration[i].acs758_sensitivity = ACS758_SENSITIVITY_DEFAULT;
  }
  
  // Inizializza parametri globali
  ACS758_VREF = ACS758_VREF_DEFAULT;
  ACS758_SENSITIVITY = ACS758_SENSITIVITY_DEFAULT;
  
  Serial.println("🔧 Calibrazione inizializzata con valori di default");
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
  
  // Usa parametri calibrati per questo sensore specifico
  float vref = calibration[battery_index].acs758_vref;
  float sensitivity = calibration[battery_index].acs758_sensitivity;
  
  float raw_current = (voltage - vref) / sensitivity;
  
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
  // Duty cycle = (pulse_width / 20000) * 4095 (12-bit resolution)
  uint16_t duty = (pulse_width * 4095) / 20000;
  
  if (pin == PWM_OUT_RIGHT) {
    ledcWrite(PWM_OUT_RIGHT_CHANNEL, duty);
  } else if (pin == PWM_OUT_LEFT) {
    ledcWrite(PWM_OUT_LEFT_CHANNEL, duty);
  }
}

// Scrittura PWM per pin di direzione
void writeDirPWM(int pin, bool active) {
  // Se attivo: PWM a 1500μs, se disattivo: PWM a 1000μs (o 0)
  uint16_t pulse_width = active ? 2000 : 1000;
  uint16_t duty = (pulse_width * 4095) / 20000;
  
  if (pin == DIR_RIGHT_PIN) {
    ledcWrite(DIR_RIGHT_CHANNEL, duty);
  } else if (pin == DIR_LEFT_PIN) {
    ledcWrite(DIR_LEFT_CHANNEL, duty);
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
  // Calibrazione a due punti: 0A e corrente nota
  float raw_current_voltage = batteries[battery_index].raw_current_voltage;
  
  if (abs(measured_current) > 0.1) { // Solo se c'è corrente significativa
    // Calcola la sensibilità: (V_measured - V_0A) / I_measured
    // Assumendo che V_0A sia il VREF attuale
    float voltage_diff = raw_current_voltage - calibration[battery_index].acs758_vref;
    float calculated_sensitivity = voltage_diff / measured_current;
    
    // Aggiorna i parametri del sensore
    calibration[battery_index].acs758_sensitivity = calculated_sensitivity;
    calibration[battery_index].current_offset = 0.0;
    calibration[battery_index].current_scale = 1.0;
    
    Serial.printf("   Corrente: %.2fA -> Sensibilità: %.4f V/A\n", measured_current, calculated_sensitivity);
  } else {
    // Se corrente = 0, aggiorna solo il VREF
    calibration[battery_index].acs758_vref = raw_current_voltage;
    calibration[battery_index].current_offset = 0.0;
    calibration[battery_index].current_scale = 1.0;
    
    Serial.printf("   Corrente: 0A -> VREF aggiornato: %.3fV\n", raw_current_voltage);
  }
  
  Serial.printf("🔧 Calibrazione semplice batteria %d:\n", battery_index);
  Serial.printf("   Tensione: %.2fV -> Divider ratio: %.2f\n", measured_voltage, calibration[battery_index].divider_ratio);
  Serial.printf("   Corrente: %.2fA -> VREF: %.3fV, Sensibilità: %.4f V/A\n", 
                measured_current, calibration[battery_index].acs758_vref, calibration[battery_index].acs758_sensitivity);
  
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
  for (int i = 0; i < 120; i++) {
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
    chart->avg_value = (chart->avg_value * 119 + value) / 120;
  } else {
    chart->avg_value = (chart->avg_value * chart->index + value) / (chart->index + 1);
  }
  
  chart->index++;
  if (chart->index >= 120) {
    chart->index = 0;
    chart->filled = true;
  }
}

// Inizializza taratura con valori di default

// Aggiorna grafici con frequenza ottimizzata
void updateCharts() {
  if (millis() - last_chart_update < 1000) return; // Aggiorna ogni 1 secondo (ultimi 2 minuti)
  
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
  int total_points = chart->filled ? 120 : chart->index;
  int step = max(1, total_points / points);
  *actual_points = min(points, total_points / step);
  
  int start = chart->filled ? chart->index : 0;
  for (int i = 0; i < *actual_points; i++) {
    int idx = (start + i * step) % 120;
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
  Serial.println("🗑️ Tutti i grafici sono stati azzerati (ultimi 2 minuti)");
}

// ============================================================================
// GESTIONE LOG IN SPIFFS (4 ORE @ 1 CAMPIONE/SECONDO, SNAPSHOT OGNI 2 MIN)
// ============================================================================

// Inizializza SPIFFS e crea directory logs
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ Errore montaggio SPIFFS!");
    return;
  }
  
  // Crea directory /logs se non esiste
  if (!SPIFFS.exists("/logs")) {
    // SPIFFS non ha mkdir, i file con path creano automaticamente le "directory"
    Serial.println("📁 Directory /logs pronta");
  }
  
  // Info SPIFFS
  size_t total = SPIFFS.totalBytes();
  size_t used = SPIFFS.usedBytes();
  Serial.printf("💾 SPIFFS: %d KB totali, %d KB usati, %d KB liberi\n", 
                total/1024, used/1024, (total-used)/1024);
}

// Inizializza metadati log
void initFlashLog(FlashLog* log) {
  log->snapshot_count = 0;
  log->current_index = 0;
  log->filled = false;
  log->last_save = 0;
  log->save_pending = false;
}

// Inizializza buffer asincrono
void initAsyncSaveBuffer() {
  for (int i = 0; i < 3; i++) {
    async_save_queue[i].ready = false;
    async_save_queue[i].saving = false;
    async_save_queue[i].battery_index = i;
  }
  async_save_count = 0;
}

// Prepara snapshot per salvataggio asincrono (NON blocca il loop!)
void prepareSnapshotForAsync(int battery_index, ChartData* voltage_chart, ChartData* current_chart) {
  FlashLog* log = &flash_logs[battery_index];
  AsyncSaveBuffer* buffer = &async_save_queue[battery_index];
  
  // Se c'è già un salvataggio in corso, salta (protegge PWM)
  if (buffer->ready || buffer->saving) {
    Serial.printf("⚠️ Salvataggio in corso, skip snapshot bat%d\n", battery_index);
    return;
  }
  
  // Prepara snapshot nel buffer (operazione veloce in RAM)
  buffer->snapshot.timestamp = millis();
  buffer->battery_index = battery_index;
  buffer->file_index = log->current_index;
  
  // Copia i 120 campioni dai grafici RAM (< 1ms)
  int start_idx = voltage_chart->filled ? voltage_chart->index : 0;
  for (int i = 0; i < 120; i++) {
    int idx = (start_idx + i) % 120;
    
    buffer->snapshot.entries[i].timestamp = millis() - ((120 - i) * 1000);
    buffer->snapshot.entries[i].voltage = voltage_chart->values[idx];
    buffer->snapshot.entries[i].current = current_chart->values[idx];
    buffer->snapshot.entries[i].power = voltage_chart->values[idx] * current_chart->values[idx];
  }
  
  // Marca buffer come pronto per salvataggio asincrono
  buffer->ready = true;
  log->save_pending = true;
  async_save_count++;
  
  // Aggiorna indice (la scrittura vera avverrà in processAsyncSaves())
  log->current_index++;
  if (log->current_index >= 120) {
    log->current_index = 0;
    log->filled = true;
  }
  if (log->snapshot_count < 120) {
    log->snapshot_count++;
  }
  log->last_save = millis();
}

// Processa salvataggi asincroni (chiamata in momenti sicuri, fuori dal loop critico)
void processAsyncSaves() {
  // Processa UN solo snapshot per chiamata (limita il tempo di blocco)
  for (int i = 0; i < 3; i++) {
    AsyncSaveBuffer* buffer = &async_save_queue[i];
    
    if (buffer->ready && !buffer->saving) {
      buffer->saving = true;
      
      // Nome file: /logs/bat0_042.bin
      char filename[32];
      sprintf(filename, "/logs/bat%d_%03d.bin", buffer->battery_index, buffer->file_index);
      
      // Scrittura SPIFFS (operazione bloccante ~10-50ms)
      File file = SPIFFS.open(filename, FILE_WRITE);
      if (file) {
        size_t written = file.write((uint8_t*)&buffer->snapshot, sizeof(FlashLogSnapshot));
        file.close();
        
        if (written == sizeof(FlashLogSnapshot)) {
          // Successo: salva metadati su NVS
          flash_logs[buffer->battery_index].save_pending = false;
          saveLogsMetadata();
          
          Serial.printf("✅ Salvato %s (%d KB)\n", filename, sizeof(FlashLogSnapshot)/1024);
        } else {
          Serial.printf("❌ Errore scrittura %s\n", filename);
        }
      } else {
        Serial.printf("❌ Errore apertura %s\n", filename);
      }
      
      // Libera buffer
      buffer->ready = false;
      buffer->saving = false;
      async_save_count--;
      
      return; // Processa solo uno per volta
    }
  }
}

// Prepara snapshot ogni 2 minuti (chiamata dal loop - NON blocca!)
void saveLogsToFlash() {
  static unsigned long last_flash_save = 0;
  
  // Prepara snapshot ogni 2 minuti (120 secondi)
  if (millis() - last_flash_save < 120000) return;
  
  // Prepara i 3 snapshot (veloce, solo copia RAM→RAM)
  for (int i = 0; i < 3; i++) {
    prepareSnapshotForAsync(i, &voltage_charts[i], &current_charts[i]);
  }
  
  last_flash_save = millis();
  
  Serial.printf("📦 Snapshot preparati per salvataggio asincrono (Coda: %d, Idx: %d/120)\n", 
                async_save_count,
                flash_logs[0].current_index);
}

// Carica metadati log da NVS (per ripristino dopo reboot)
void loadLogsFromFlash() {
  preferences.begin("logs", true);
  
  for (int i = 0; i < 3; i++) {
    String prefix = "log" + String(i) + "_";
    
    // Carica solo i metadati (i dati veri sono su SPIFFS)
    flash_logs[i].snapshot_count = preferences.getInt((prefix + "snap_cnt").c_str(), 0);
    flash_logs[i].current_index = preferences.getInt((prefix + "curr_idx").c_str(), 0);
    flash_logs[i].filled = preferences.getBool((prefix + "filled").c_str(), false);
    flash_logs[i].last_save = millis(); // Reset timestamp al boot
    flash_logs[i].save_pending = false; // Nessun salvataggio pendente al boot
    
    Serial.printf("📂 Bat%d: %d snapshot, indice %d, %s\n", 
                  i, flash_logs[i].snapshot_count, flash_logs[i].current_index,
                  flash_logs[i].filled ? "pieno" : "parziale");
  }
  
  preferences.end();
}

// Salva metadati log su NVS (chiamato dopo ogni scrittura SPIFFS)
void saveLogsMetadata() {
  preferences.begin("logs", false);
  
  for (int i = 0; i < 3; i++) {
    String prefix = "log" + String(i) + "_";
    
    preferences.putInt((prefix + "snap_cnt").c_str(), flash_logs[i].snapshot_count);
    preferences.putInt((prefix + "curr_idx").c_str(), flash_logs[i].current_index);
    preferences.putBool((prefix + "filled").c_str(), flash_logs[i].filled);
  }
  
  preferences.end();
}

// Esporta log da SPIFFS come JSON per API
void getFlashLogJSON(int battery_index, String* output, int max_entries) {
  FlashLog* log = &flash_logs[battery_index];
  int total_snapshots = log->snapshot_count;
  int total_entries = total_snapshots * 120; // Ogni snapshot ha 120 campioni
  
  // Limita il numero di entries da esportare
  int entries_to_export = min(max_entries, total_entries);
  int snapshots_to_export = (entries_to_export + 119) / 120; // Arrotonda per eccesso
  
  *output = "[";
  
  int start_snapshot = log->filled ? log->current_index : 0;
  int entry_count = 0;
  
  FlashLogSnapshot snapshot;
  
  for (int s = 0; s < snapshots_to_export && entry_count < entries_to_export; s++) {
    int snap_idx = (start_snapshot + s) % 120;
    
    // Leggi snapshot da SPIFFS
    char filename[32];
    sprintf(filename, "/logs/bat%d_%03d.bin", battery_index, snap_idx);
    
    File file = SPIFFS.open(filename, FILE_READ);
    if (!file) {
      Serial.printf("⚠️ Snapshot %s non trovato\n", filename);
      continue;
    }
    
    file.read((uint8_t*)&snapshot, sizeof(FlashLogSnapshot));
    file.close();
    
    // Aggiungi campioni al JSON
    for (int e = 0; e < 120 && entry_count < entries_to_export; e++) {
      if (entry_count > 0) *output += ",";
      *output += "{";
      *output += "\"ts\":" + String(snapshot.entries[e].timestamp) + ",";
      *output += "\"v\":" + String(snapshot.entries[e].voltage, 2) + ",";
      *output += "\"c\":" + String(snapshot.entries[e].current, 2) + ",";
      *output += "\"p\":" + String(snapshot.entries[e].power, 1);
      *output += "}";
      entry_count++;
    }
  }
  
  *output += "]";
}

// Azzera tutti i log da SPIFFS
void clearAllLogs() {
  // Cancella tutti i file snapshot
  for (int bat = 0; bat < 3; bat++) {
    for (int snap = 0; snap < 120; snap++) {
      char filename[32];
      sprintf(filename, "/logs/bat%d_%03d.bin", bat, snap);
      if (SPIFFS.exists(filename)) {
        SPIFFS.remove(filename);
      }
    }
    // Reset metadati
    initFlashLog(&flash_logs[bat]);
  }
  
  Serial.println("🗑️ Tutti i log su SPIFFS sono stati azzerati");
}

// ============================================================================
// FUNZIONI CALIBRAZIONE
// ============================================================================

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
  // AVANTI: 1500→1000, 2000→2000 (PWM ATTIVO)
  // INDIETRO: 1500→1000, 1000→2000 (PWM DISATTIVO)
  // Per motore destro
  if (right_input <= PWM_CENTER) {
    // 1000-1500: BACKWARD - mappa 1000→2000, 1500→1000
    motor_output.right_pwm = map(right_input, PWM_MIN, PWM_CENTER, PWM_MAX, PWM_MIN);
    writeDirPWM(DIR_RIGHT_PIN, false);  // BACKWARD - PWM DISATTIVO
    // DEBUG: Stampa quando va indietro
    if (right_input < 1500) {
      Serial.printf("🔙 MOTORE DESTRO INDIETRO: Input=%d, Output=%d, DIR=PWM_OFF\n", right_input, motor_output.right_pwm);
    }
  } else {
    // 1500-2000: FORWARD - mappa 1500→1000, 2000→2000
    motor_output.right_pwm = map(right_input, PWM_CENTER, PWM_MAX, PWM_MIN, PWM_MAX);
    writeDirPWM(DIR_RIGHT_PIN, true);   // FORWARD - PWM ATTIVO
    // DEBUG: Stampa quando va avanti
    if (right_input > 1500) {
      Serial.printf("🔜 MOTORE DESTRO AVANTI: Input=%d, Output=%d, DIR=PWM_ON\n", right_input, motor_output.right_pwm);
    }
  }
  
  // Per motore sinistro
  if (left_input <= PWM_CENTER) {
    // 1000-1500: BACKWARD - mappa 1000→2000, 1500→1000
    motor_output.left_pwm = map(left_input, PWM_MIN, PWM_CENTER, PWM_MAX, PWM_MIN);
    writeDirPWM(DIR_LEFT_PIN, false);   // BACKWARD - PWM DISATTIVO
    // DEBUG: Stampa quando va indietro
    if (left_input < 1500) {
      Serial.printf("🔙 MOTORE SINISTRO INDIETRO: Input=%d, Output=%d, DIR=PWM_OFF\n", left_input, motor_output.left_pwm);
    }
  } else {
    // 1500-2000: FORWARD - mappa 1500→1000, 2000→2000
    motor_output.left_pwm = map(left_input, PWM_CENTER, PWM_MAX, PWM_MIN, PWM_MAX);
    writeDirPWM(DIR_LEFT_PIN, true);    // FORWARD - PWM ATTIVO
    // DEBUG: Stampa quando va avanti
    if (left_input > 1500) {
      Serial.printf("🔜 MOTORE SINISTRO AVANTI: Input=%d, Output=%d, DIR=PWM_ON\n", left_input, motor_output.left_pwm);
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
    // Usa parametri calibrati per questo sensore
    float vref = calibration[i].acs758_vref;
    float sensitivity = calibration[i].acs758_sensitivity;
    float raw_calc = (batteries[i].raw_current_voltage - vref) / sensitivity;
    
    Serial.printf("  -> RAW: ADC=%.0f, Voltage=%.3fV, Raw_I=%.2fA\n",
                  batteries[i].raw_current_adc, 
                  batteries[i].raw_current_voltage,
                  raw_calc);
    Serial.printf("     VREF=%.3fV, Sens=%.4fV/A, Calibr: (%.2f + %.3f) × %.3f = %.2fA\n",
                  vref, sensitivity, raw_calc,
                  calibration[i].current_offset,
                  calibration[i].current_scale,
                  batteries[i].current);
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
  DynamicJsonDocument doc(4096);
  
  // Ottieni parametri query
  String scale = server.arg("scale");
  int points = 60; // Default 5 minuti
  
  if (scale == "10s") points = 10;      // 10 secondi
  else if (scale == "30s") points = 30; // 30 secondi  
  else if (scale == "1m") points = 60;  // 1 minuto
  else if (scale == "2m") points = 120; // 2 minuti
  else if (scale == "5m") points = 300; // 5 minuti (tutto il buffer)
  
  // Buffer temporaneo per i dati
  float temp_data[300];
  int actual_points;
  
  // Dati grafici tensione
  JsonArray voltageArray = doc.createNestedArray("voltage");
  JsonArray voltageStats = doc.createNestedArray("voltage_stats");
  for (int i = 0; i < 3; i++) {
    JsonArray batteryArray = voltageArray.createNestedArray();
    JsonObject stats = voltageStats.createNestedObject();
    
    getChartData(&voltage_charts[i], points, temp_data, &actual_points);
    for (int j = 0; j < actual_points; j++) {
      batteryArray.add(temp_data[j]);
    }
    
    // Aggiungi statistiche
    float min_val, max_val, avg_val;
    getChartStats(&voltage_charts[i], &min_val, &max_val, &avg_val);
    stats["min"] = min_val;
    stats["max"] = max_val;
    stats["avg"] = avg_val;
    stats["samples"] = voltage_charts[i].total_samples;
  }
  
  // Dati grafici corrente
  JsonArray currentArray = doc.createNestedArray("current");
  JsonArray currentStats = doc.createNestedArray("current_stats");
  for (int i = 0; i < 3; i++) {
    JsonArray batteryArray = currentArray.createNestedArray();
    JsonObject stats = currentStats.createNestedObject();
    
    getChartData(&current_charts[i], points, temp_data, &actual_points);
    for (int j = 0; j < actual_points; j++) {
      batteryArray.add(temp_data[j]);
    }
    
    // Aggiungi statistiche
    float min_val, max_val, avg_val;
    getChartStats(&current_charts[i], &min_val, &max_val, &avg_val);
    stats["min"] = min_val;
    stats["max"] = max_val;
    stats["avg"] = avg_val;
    stats["samples"] = current_charts[i].total_samples;
  }
  
  // Dati grafici raw tensione
  JsonArray rawVoltageArray = doc.createNestedArray("raw_voltage");
  for (int i = 0; i < 3; i++) {
    JsonArray batteryArray = rawVoltageArray.createNestedArray();
    getChartData(&raw_voltage_charts[i], points, temp_data, &actual_points);
    for (int j = 0; j < actual_points; j++) {
      batteryArray.add(temp_data[j]);
    }
  }
  
  // Dati grafici raw corrente
  JsonArray rawCurrentArray = doc.createNestedArray("raw_current");
  for (int i = 0; i < 3; i++) {
    JsonArray batteryArray = rawCurrentArray.createNestedArray();
    getChartData(&raw_current_charts[i], points, temp_data, &actual_points);
    for (int j = 0; j < actual_points; j++) {
      batteryArray.add(temp_data[j]);
    }
  }
  
  // Dati grafici motori PWM
  JsonArray motorArray = doc.createNestedArray("motors");
  for (int i = 0; i < 3; i++) {
    JsonArray motorData = motorArray.createNestedArray();
    getChartData(&motor_charts[i], points, temp_data, &actual_points);
    for (int j = 0; j < actual_points; j++) {
      motorData.add(temp_data[j]);
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
  doc["points"] = points;
  doc["timestamp"] = millis();
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleCSV() {
  String csv = "Timestamp,6S1_Voltage,6S1_Current,6S1_RawVoltage,6S1_RawCurrent,6S2_Voltage,6S2_Current,6S2_RawVoltage,6S2_RawCurrent,4S_Voltage,4S_Current,4S_RawVoltage,4S_RawCurrent,MotorRight,MotorLeft,MotorUnder\n";
  
  // Genera timestamp e dati CSV
  int max_points = 0;
  for (int i = 0; i < 3; i++) {
    int points = voltage_charts[i].filled ? 300 : voltage_charts[i].index;
    if (points > max_points) max_points = points;
  }
  
  for (int i = 0; i < max_points; i++) {
    // Timestamp (secondi dall'inizio)
    csv += String(i) + ",";
    
    // Dati batterie (convertiti e raw)
    for (int j = 0; j < 3; j++) {
      int idx = (voltage_charts[j].filled ? voltage_charts[j].index : 0 + i) % 300;
      csv += String(voltage_charts[j].values[idx], 2) + ",";  // Tensione convertita
      csv += String(current_charts[j].values[idx], 2) + ",";  // Corrente convertita
      csv += String(raw_voltage_charts[j].values[idx], 3) + ","; // Tensione raw
      csv += String(raw_current_charts[j].values[idx], 3);    // Corrente raw
      if (j < 2) csv += ",";
    }
    
    // Dati motori PWM
    csv += ",";
    for (int j = 0; j < 3; j++) {
      int idx = (motor_charts[j].filled ? motor_charts[j].index : 0 + i) % 300;
      csv += String(motor_charts[j].values[idx], 0);
      if (j < 2) csv += ",";
    }
    csv += "\n";
  }
  
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=battery_data_complete.csv");
  server.send(200, "text/csv", csv);
}

void handleClearCharts() {
  if (server.method() == HTTP_POST) {
    clearAllCharts();
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Grafici azzerati\"}");
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

void handleFlashLogs() {
  // Restituisce i log storici dalla flash (4 ore @ 1Hz, snapshot ogni 2 min)
  int battery = server.arg("battery").toInt();
  int max_entries = server.arg("limit").toInt();
  if (max_entries == 0) max_entries = 14400; // Default: tutti (120 snapshot × 120 campioni)
  
  if (battery < 0 || battery > 2) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Batteria non valida\"}");
    return;
  }
  
  String json_output;
  getFlashLogJSON(battery, &json_output, max_entries);
  
  int total_snapshots = flash_logs[battery].snapshot_count;
  int total_entries = total_snapshots * 120;
  
  String response = "{\"status\":\"ok\",\"battery\":" + String(battery) + 
                    ",\"snapshots\":" + String(total_snapshots) +
                    ",\"entries\":" + String(total_entries) +
                    ",\"data\":" + json_output + "}";
  
  server.send(200, "application/json", response);
}

void handleClearLogs() {
  if (server.method() == HTTP_POST) {
    clearAllLogs();
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Log azzerati\"}");
  } else {
    server.send(405, "application/json", "{\"status\":\"error\",\"message\":\"Metodo non consentito\"}");
  }
}

void handleStorageInfo() {
  // Informazioni sulla memoria SPIFFS
  size_t total = SPIFFS.totalBytes();
  size_t used = SPIFFS.usedBytes();
  size_t free = total - used;
  
  // Conta i file di log per ogni batteria
  int file_counts[3] = {0, 0, 0};
  
  for (int bat = 0; bat < 3; bat++) {
    for (int snap = 0; snap < 120; snap++) {
      char filename[32];
      sprintf(filename, "/logs/bat%d_%03d.bin", bat, snap);
      if (SPIFFS.exists(filename)) {
        file_counts[bat]++;
      }
    }
  }
  
  // Calcola ore registrate (ogni file = 2 minuti di dati)
  float hours_bat0 = (file_counts[0] * 2.0) / 60.0;
  float hours_bat1 = (file_counts[1] * 2.0) / 60.0;
  float hours_bat2 = (file_counts[2] * 2.0) / 60.0;
  
  // Percentuale riempimento (max 120 file per batteria)
  float percent_bat0 = (file_counts[0] / 120.0) * 100.0;
  float percent_bat1 = (file_counts[1] / 120.0) * 100.0;
  float percent_bat2 = (file_counts[2] / 120.0) * 100.0;
  
  // Memoria disponibile per nuovi snapshot (approssimativo)
  int snapshot_size = sizeof(FlashLogSnapshot);
  int available_snapshots = free / snapshot_size;
  
  String json = "{";
  json += "\"total_bytes\":" + String(total) + ",";
  json += "\"used_bytes\":" + String(used) + ",";
  json += "\"free_bytes\":" + String(free) + ",";
  json += "\"percent_used\":" + String((used * 100) / total) + ",";
  json += "\"snapshot_size\":" + String(snapshot_size) + ",";
  json += "\"available_snapshots\":" + String(available_snapshots) + ",";
  json += "\"batteries\":[";
  
  for (int i = 0; i < 3; i++) {
    if (i > 0) json += ",";
    float hours = (file_counts[i] * 2.0) / 60.0;
    float percent = (file_counts[i] / 120.0) * 100.0;
    json += "{";
    json += "\"id\":" + String(i) + ",";
    json += "\"files\":" + String(file_counts[i]) + ",";
    json += "\"hours\":" + String(hours, 1) + ",";
    json += "\"percent\":" + String(percent, 1) + ",";
    json += "\"filled\":" + String(flash_logs[i].filled ? "true" : "false") + ",";
    json += "\"current_index\":" + String(flash_logs[i].current_index);
    json += "}";
  }
  
  json += "]";
  json += "}";
  
  server.send(200, "application/json", json);
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
  html += ".row-raw{display:flex;justify-content:space-between;margin:4px 0;font-size:11px;color:#6b7280;padding-left:8px}";
  html += ".muted{color:#94a3b8}";
  html += ".status-badges{display:flex;flex-wrap:wrap;gap:8px;margin-top:8px}";
  html += ".badge{display:inline-flex;align-items:center;gap:6px;padding:4px 8px;border-radius:999px;background:#0f172a;border:1px solid #1f2937;font-size:12px;color:#cbd5e1}";
  html += ".ok{color:#10b981}.warn{color:#f59e0b}.err{color:#ef4444}";
  html += ".footer{margin-top:14px;font-size:12px;color:#94a3b8}";
  html += "a{color:#60a5fa;text-decoration:none}";
  html += "</style></head><body><div class='container'>";
  html += "<h1>🔋 Alix Blimp Battery Monitor - TEST</h1>";
  html += "<div class='grid'>";
  html += "<div class='card' id='b0'><h3>6S Battery #1</h3><div class='row'><span class='muted'>Tensione</span><strong><span id='b0v'>-</span> V</strong></div><div class='row-raw'><span class='muted'>RAW V:</span><span><span id='b0rv'>-</span> V (ADC: <span id='b0rva'>-</span>)</span></div><div class='row'><span class='muted'>Corrente</span><strong><span id='b0c'>-</span> A</strong></div><div class='row-raw'><span class='muted'>RAW I:</span><span><span id='b0rc'>-</span> V (ADC: <span id='b0rca'>-</span>)</span></div><div class='row'><span class='muted'>Potenza</span><strong><span id='b0p'>-</span> W</strong></div></div>";
  html += "<div class='card' id='b1'><h3>6S Battery #2</h3><div class='row'><span class='muted'>Tensione</span><strong><span id='b1v'>-</span> V</strong></div><div class='row-raw'><span class='muted'>RAW V:</span><span><span id='b1rv'>-</span> V (ADC: <span id='b1rva'>-</span>)</span></div><div class='row'><span class='muted'>Corrente</span><strong><span id='b1c'>-</span> A</strong></div><div class='row-raw'><span class='muted'>RAW I:</span><span><span id='b1rc'>-</span> V (ADC: <span id='b1rca'>-</span>)</span></div><div class='row'><span class='muted'>Potenza</span><strong><span id='b1p'>-</span> W</strong></div></div>";
  html += "<div class='card' id='b2'><h3>4S Battery</h3><div class='row'><span class='muted'>Tensione</span><strong><span id='b2v'>-</span> V</strong></div><div class='row-raw'><span class='muted'>RAW V:</span><span><span id='b2rv'>-</span> V (ADC: <span id='b2rva'>-</span>)</span></div><div class='row'><span class='muted'>Corrente</span><strong><span id='b2c'>-</span> A</strong></div><div class='row-raw'><span class='muted'>RAW I:</span><span><span id='b2rc'>-</span> V (ADC: <span id='b2rca'>-</span>)</span></div><div class='row'><span class='muted'>Potenza</span><strong><span id='b2p'>-</span> W</strong></div></div>";
  html += "<div class='card'><h3>Status Sistema</h3><div class='row'><span class='muted'>Frequenza Loop</span><strong><span id='lf'>-</span> Hz</strong></div><div class='row'><span class='muted'>Input Right</span><strong><span id='ir'>-</span> μs</strong></div><div class='row'><span class='muted'>Input Left</span><strong><span id='il'>-</span> μs</strong></div><div class='row'><span class='muted'>Input Under</span><strong><span id='iu'>-</span> μs</strong></div><div class='status-badges'><span class='badge' id='dr'><span>Dir Right</span><strong>-</strong></span><span class='badge' id='dl'><span>Dir Left</span><strong>-</strong></span><span class='badge'><span>Output Right</span><strong id='or'>- μs</strong></span><span class='badge'><span>Output Left</span><strong id='ol'>- μs</strong></span></div></div>";
  html += "</div>";
  html += "<div class='footer'>Aggiornamento ogni 1s via <a href='/api'>/api</a> | <a href='/calibration'>Taratura</a> | <a href='/charts'>Grafici</a> | <a href='/storage'>Storage</a></div>";
  html += "</div><script>(function(){function q(id){return document.getElementById(id)};function setText(id,val,dec){q(id).textContent=(typeof dec==='number'?Number(val).toFixed(dec):val)};function upd(d){for(var i=0;i<3;i++){setText('b'+i+'v',d.batteries[i].voltage,2);setText('b'+i+'c',d.batteries[i].current,2);setText('b'+i+'p',d.batteries[i].power,1);setText('b'+i+'rv',d.batteries[i].raw_voltage,3);setText('b'+i+'rva',d.batteries[i].raw_voltage_adc,0);setText('b'+i+'rc',d.batteries[i].raw_current,3);setText('b'+i+'rca',d.batteries[i].raw_current_adc,0);}setText('lf',d.loop_frequency,1);setText('ir',d.autopilot.motor_right);setText('il',d.autopilot.motor_left);setText('iu',d.autopilot.motor_under);var dr=d.autopilot.dir_right,dl=d.autopilot.dir_left;q('dr').className='badge '+(dr?'ok':'err');q('dr').lastElementChild.textContent=dr?'Forward':'Reverse';q('dl').className='badge '+(dl?'ok':'err');q('dl').lastElementChild.textContent=dl?'Forward':'Reverse';setText('or',d.motors.right_pwm+' μs');setText('ol',d.motors.left_pwm+' μs')}function tick(){fetch('/api',{cache:'no-store'}).then(function(r){return r.json()}).then(upd).catch(function(){}).finally(function(){setTimeout(tick,1000)})}tick()})();</script></body></html>";
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
  html += "<h1>📈 Grafici Storici Batterie (Ultimi 2 Minuti)</h1>";
  html += "<div class='controls'>";
  html += "<label>Scala temporale:</label>";
  html += "<select id='timeScale' onchange='changeScale()'>";
  html += "<option value='10s'>10 secondi</option>";
  html += "<option value='30s'>30 secondi</option>";
  html += "<option value='1m'>1 minuto</option>";
  html += "<option value='2m' selected>2 minuti</option>";
  html += "<option value='5m'>5 minuti</option>";
  html += "</select>";
  html += "<button onclick='exportCSV()'>📥 Esporta CSV</button>";
  html += "<button onclick='update()'>🔄 Aggiorna</button>";
  html += "<button onclick='toggleAutoUpdate()'>⏸️ Auto</button>";
  html += "<button onclick='resetZoom()'>🔍 Reset Zoom</button>";
  html += "<button onclick='toggleGrid()'>📐 Griglia</button>";
  html += "<button onclick='clearCharts()' style='background:#ef4444;color:white'>🗑️ Azzera Dati</button>";
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
  html += "let currentScale='5m';";
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
  html += "function clearCharts(){";
  html += "if(confirm('Sei sicuro di voler azzerare tutti i dati storici?')){";
  html += "fetch('/clear-charts',{method:'POST'})";
  html += ".then(function(r){return r.json();})";
  html += ".then(function(d){";
  html += "if(d.status==='ok'){";
  html += "alert('Dati azzerati con successo!');";
  html += "update();";
  html += "}else{";
  html += "alert('Errore durante l\\'azzeramento');";
  html += "}";
  html += "})";
  html += ".catch(function(e){alert('Errore: '+e);});";
  html += "}";
  html += "}";
  html += "";
  html += "window.onload=function(){";
  html += "initCharts();";
  html += "update();";
  html += "setInterval(function(){if(autoUpdate)update();},1000);";
  html += "};";
  html += "</script></body></html>";
  server.send(200, "text/html", html);
}

void handleStoragePage() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Storage - SPIFFS</title>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;background:#0b1220;color:#e6edf3;padding:20px;margin:0}";
  html += ".container{max-width:900px;margin:0 auto}";
  html += "h1{font-size:24px;margin-bottom:16px;color:#c9d1d9}";
  html += ".card{background:#111827;border:1px solid #1f2937;border-radius:10px;padding:16px;margin:12px 0;box-shadow:0 2px 8px rgba(0,0,0,.25)}";
  html += ".card h3{margin:0 0 12px;font-size:18px;color:#e5e7eb}";
  html += ".info-row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #1f2937}";
  html += ".info-row:last-child{border-bottom:none}";
  html += ".label{color:#94a3b8;font-size:14px}";
  html += ".value{color:#e6edf3;font-size:14px;font-weight:600}";
  html += ".progress-bar{width:100%;height:24px;background:#0f172a;border-radius:12px;overflow:hidden;margin:8px 0;position:relative}";
  html += ".progress-fill{height:100%;background:linear-gradient(90deg,#10b981,#3b82f6);transition:width 0.3s}";
  html += ".progress-text{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);font-size:12px;font-weight:600;color:#fff}";
  html += ".battery-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px;margin:12px 0}";
  html += ".stat{text-align:center;padding:8px}";
  html += ".stat-value{font-size:28px;font-weight:700;color:#3b82f6;margin-bottom:4px}";
  html += ".stat-label{font-size:12px;color:#94a3b8}";
  html += ".btn{display:inline-block;padding:10px 20px;border-radius:6px;border:none;cursor:pointer;font-size:14px;margin:4px}";
  html += ".btn-primary{background:#3b82f6;color:#fff}";
  html += ".btn-danger{background:#ef4444;color:#fff}";
  html += ".btn:hover{opacity:0.85}";
  html += "a{color:#60a5fa;text-decoration:none}";
  html += ".footer{margin-top:20px;font-size:12px;color:#94a3b8;text-align:center}";
  html += ".warning{background:#422006;border:1px solid #f59e0b;color:#fbbf24;padding:12px;border-radius:6px;margin:12px 0;font-size:14px}";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<h1>💾 Storage - SPIFFS</h1>";
  
  html += "<div class='card'>";
  html += "<h3>Memoria Flash</h3>";
  html += "<div class='info-row'><span class='label'>Totale:</span><span class='value' id='totalMem'>-</span></div>";
  html += "<div class='info-row'><span class='label'>Usata:</span><span class='value' id='usedMem'>-</span></div>";
  html += "<div class='info-row'><span class='label'>Libera:</span><span class='value' id='freeMem'>-</span></div>";
  html += "<div class='progress-bar'><div class='progress-fill' id='progressBar' style='width:0%'></div><div class='progress-text' id='progressText'>0%</div></div>";
  html += "<div class='info-row'><span class='label'>Snapshot disponibili:</span><span class='value' id='availSnap'>-</span></div>";
  html += "<div class='info-row'><span class='label'>Dimensione snapshot:</span><span class='value' id='snapSize'>-</span></div>";
  html += "</div>";
  
  html += "<div class='card'>";
  html += "<h3>Log per Batteria</h3>";
  html += "<div class='battery-grid'>";
  html += "<div class='stat'><div class='stat-value' id='files0'>-</div><div class='stat-label'>6S#1 - File</div></div>";
  html += "<div class='stat'><div class='stat-value' id='hours0'>-</div><div class='stat-label'>Ore registrate</div></div>";
  html += "<div class='stat'><div class='stat-value' id='percent0'>-</div><div class='stat-label'>% Capacità</div></div>";
  html += "</div>";
  html += "<div class='battery-grid'>";
  html += "<div class='stat'><div class='stat-value' id='files1'>-</div><div class='stat-label'>6S#2 - File</div></div>";
  html += "<div class='stat'><div class='stat-value' id='hours1'>-</div><div class='stat-label'>Ore registrate</div></div>";
  html += "<div class='stat'><div class='stat-value' id='percent1'>-</div><div class='stat-label'>% Capacità</div></div>";
  html += "</div>";
  html += "<div class='battery-grid'>";
  html += "<div class='stat'><div class='stat-value' id='files2'>-</div><div class='stat-label'>4S - File</div></div>";
  html += "<div class='stat'><div class='stat-value' id='hours2'>-</div><div class='stat-label'>Ore registrate</div></div>";
  html += "<div class='stat'><div class='stat-value' id='percent2'>-</div><div class='stat-label'>% Capacità</div></div>";
  html += "</div>";
  html += "</div>";
  
  html += "<div id='warningBox' style='display:none' class='warning'>⚠️ La memoria è quasi piena! Quando raggiungerà il 100%, i nuovi dati non verranno più salvati.</div>";
  
  html += "<div class='card'>";
  html += "<h3>Azioni</h3>";
  html += "<button class='btn btn-primary' onclick='refreshData()'>🔄 Aggiorna</button>";
  html += "<button class='btn btn-danger' onclick='clearLogs()'>🗑️ Cancella Tutti i Log</button>";
  html += "</div>";
  
  html += "<div class='footer'><a href='/'>← Torna al Monitor</a> | <a href='/charts'>Grafici</a></div>";
  html += "</div>";
  
  html += "<script>";
  html += "function formatBytes(bytes){";
  html += "if(bytes<1024)return bytes+' B';";
  html += "if(bytes<1048576)return(bytes/1024).toFixed(1)+' KB';";
  html += "return(bytes/1048576).toFixed(2)+' MB';";
  html += "}";
  html += "";
  html += "function refreshData(){";
  html += "fetch('/storage-info')";
  html += ".then(r=>r.json())";
  html += ".then(data=>{";
  html += "document.getElementById('totalMem').textContent=formatBytes(data.total_bytes);";
  html += "document.getElementById('usedMem').textContent=formatBytes(data.used_bytes);";
  html += "document.getElementById('freeMem').textContent=formatBytes(data.free_bytes);";
  html += "document.getElementById('availSnap').textContent=data.available_snapshots;";
  html += "document.getElementById('snapSize').textContent=formatBytes(data.snapshot_size);";
  html += "var percent=data.percent_used;";
  html += "document.getElementById('progressBar').style.width=percent+'%';";
  html += "document.getElementById('progressText').textContent=percent+'%';";
  html += "if(percent>85){";
  html += "document.getElementById('warningBox').style.display='block';";
  html += "}else{";
  html += "document.getElementById('warningBox').style.display='none';";
  html += "}";
  html += "for(var i=0;i<3;i++){";
  html += "document.getElementById('files'+i).textContent=data.batteries[i].files;";
  html += "document.getElementById('hours'+i).textContent=data.batteries[i].hours+'h';";
  html += "document.getElementById('percent'+i).textContent=data.batteries[i].percent.toFixed(1)+'%';";
  html += "}";
  html += "})";
  html += ".catch(e=>alert('Errore caricamento dati: '+e));";
  html += "}";
  html += "";
  html += "function clearLogs(){";
  html += "if(confirm('Sei sicuro di voler cancellare TUTTI i log? Questa operazione è irreversibile!')){";
  html += "fetch('/clear-logs',{method:'POST'})";
  html += ".then(r=>r.json())";
  html += ".then(data=>{";
  html += "if(data.status==='ok'){";
  html += "alert('Log cancellati con successo!');";
  html += "refreshData();";
  html += "}else{";
  html += "alert('Errore durante la cancellazione');";
  html += "}";
  html += "})";
  html += ".catch(e=>alert('Errore: '+e));";
  html += "}";
  html += "}";
  html += "";
  html += "window.onload=function(){";
  html += "refreshData();";
  html += "setInterval(refreshData,5000);";
  html += "};";
  html += "</script></body></html>";
  
  server.send(200, "text/html", html);
}

void handleAPI() {
  DynamicJsonDocument doc(1024);
  
  // Batterie con dati RAW
  JsonArray batteryArray = doc.createNestedArray("batteries");
  for (int i = 0; i < 3; i++) {
    JsonObject battery = batteryArray.createNestedObject();
    battery["voltage"] = batteries[i].voltage;
    battery["current"] = batteries[i].current;
    battery["power"] = batteries[i].power;
    // Dati RAW
    battery["raw_voltage_adc"] = batteries[i].raw_voltage_adc;
    battery["raw_voltage"] = batteries[i].raw_voltage_voltage;
    battery["raw_current_adc"] = batteries[i].raw_current_adc;
    battery["raw_current"] = batteries[i].raw_current_voltage;
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
  Serial.println("🚀 AlixBlimp Battery Monitor & Motor Control");
  Serial.println("========================================");
  
  // Configurazione Pin PWM Input (dall'autopilota)
  pinMode(PWM_IN_RIGHT, INPUT);
  pinMode(PWM_IN_LEFT, INPUT);
  pinMode(PWM_IN_UNDER, INPUT);
  Serial.println("✅ Pin PWM Input configurati (GPIO18, GPIO19, GPIO22)");
  
  // Configurazione ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // 0-3.3V range
  
  // Test lettura ADC sensori corrente
  Serial.println("\n🔍 Test Sensori Corrente:");
  delay(100);
  float test_adc1 = analogRead(CURRENT_6S1_PIN);
  float test_v1 = (test_adc1 / 4095.0) * 3.3;
  Serial.printf("  GPIO32 (6S#1): ADC=%.0f, Voltage=%.3fV\n", test_adc1, test_v1);
  
  float test_adc2 = analogRead(CURRENT_6S2_PIN);
  float test_v2 = (test_adc2 / 4095.0) * 3.3;
  Serial.printf("  GPIO33 (6S#2): ADC=%.0f, Voltage=%.3fV\n", test_adc2, test_v2);
  
  float test_adc3 = analogRead(CURRENT_4S_PIN);
  float test_v3 = (test_adc3 / 4095.0) * 3.3;
  Serial.printf("  GPIO34 (4S):   ADC=%.0f, Voltage=%.3fV\n", test_adc3, test_v3);
  Serial.printf("  Vref atteso: %.2fV (0A)\n", ACS758_VREF_DEFAULT);
  Serial.printf("  Sensibilità attesa: %.4f V/A\n\n", ACS758_SENSITIVITY_DEFAULT);
  
  // Inizializzazione Taratura
  initCalibration();
  
  // Carica le impostazioni di calibrazione salvate dalla memoria flash
  loadCalibrationFromFlash();
  
  // Verifica calibrazione: se current_scale = 0, resetta
  Serial.println("\n🔍 Verifica Calibrazione:");
  bool need_reset = false;
  for (int i = 0; i < 3; i++) {
    Serial.printf("  Batteria %d: VREF=%.3fV, Sens=%.4fV/A, offset=%.3f, scale=%.3f\n", 
                  i, calibration[i].acs758_vref, calibration[i].acs758_sensitivity,
                  calibration[i].current_offset, calibration[i].current_scale);
    if (calibration[i].current_scale == 0.0 || isnan(calibration[i].current_scale) ||
        calibration[i].acs758_sensitivity == 0.0 || isnan(calibration[i].acs758_sensitivity)) {
      Serial.printf("    ⚠️ Parametri errati! Necessario reset.\n");
      need_reset = true;
    }
  }
  
  if (need_reset) {
    Serial.println("⚠️ Calibrazione corrotta! Ripristino valori default...");
    resetCalibrationToDefault();
    loadCalibrationFromFlash();
    Serial.println("✅ Calibrazione ripristinata!");
  }
  
  // Inizializzazione Grafici (ultimi 2 minuti, 1 secondo)
  for (int i = 0; i < 3; i++) {
    initChart(&voltage_charts[i]);
    initChart(&current_charts[i]);
    initChart(&raw_voltage_charts[i]);
    initChart(&raw_current_charts[i]);
    initChart(&motor_charts[i]);
  }
  Serial.println("📊 Grafici inizializzati (120 punti, 2 minuti @ 1Hz)");
  
  // Inizializzazione SPIFFS
  initSPIFFS();
  
  // Inizializzazione Log (metadati in RAM, snapshot su SPIFFS)
  for (int i = 0; i < 3; i++) {
    initFlashLog(&flash_logs[i]);
  }
  
  // Inizializzazione buffer asincrono
  initAsyncSaveBuffer();
  
  Serial.println("💾 Log SPIFFS pronti (120 snapshot × 120 campioni, 4 ore, salv asincrono)");
  
  // Configurazione PWM Output Motori (LEDC channels)
  ledcSetup(PWM_OUT_RIGHT_CHANNEL, PWM_FREQ, 12);  // 50Hz, 12-bit resolution
  ledcAttachPin(PWM_OUT_RIGHT, PWM_OUT_RIGHT_CHANNEL);
  ledcSetup(PWM_OUT_LEFT_CHANNEL, PWM_FREQ, 12);   // 50Hz, 12-bit resolution  
  ledcAttachPin(PWM_OUT_LEFT, PWM_OUT_LEFT_CHANNEL);
  
  // Configurazione PWM Output Direzione (LEDC channels)
  ledcSetup(DIR_RIGHT_CHANNEL, PWM_FREQ, 12);      // 50Hz, 12-bit resolution
  ledcAttachPin(DIR_RIGHT_PIN, DIR_RIGHT_CHANNEL);
  ledcSetup(DIR_LEFT_CHANNEL, PWM_FREQ, 12);       // 50Hz, 12-bit resolution
  ledcAttachPin(DIR_LEFT_PIN, DIR_LEFT_CHANNEL);
  
  // Inizializzazione PWM Output (posizione neutra)
  writePWM(PWM_OUT_RIGHT, PWM_CENTER);
  writePWM(PWM_OUT_LEFT, PWM_CENTER);
  
  // Inizializzazione PWM Direzione (disattivi)
  writeDirPWM(DIR_RIGHT_PIN, false);
  writeDirPWM(DIR_LEFT_PIN, false);
  
  Serial.println("✅ PWM Motori e Direzione configurati");
  
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
  server.on("/flash-logs", HTTP_GET, handleFlashLogs);
  server.on("/clear-charts", HTTP_POST, handleClearCharts);
  server.on("/clear-logs", HTTP_POST, handleClearLogs);
  server.on("/reset-calibration", HTTP_POST, handleResetCalibration);
  server.on("/csv", handleCSV);
  server.on("/storage", HTTP_GET, handleStoragePage);
  server.on("/storage-info", HTTP_GET, handleStorageInfo);
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
  
  // Aggiorna grafici (ogni secondo)
  updateCharts();
  
  // Salva snapshot in Flash (ogni 2 minuti, 120 snapshot = 4 ore)
  saveLogsToFlash();
  
  // Web Server
  server.handleClient();
  
  // Processa salvataggi asincroni (1 per loop, fuori dal path critico PWM)
  processAsyncSaves();
  
  // Piccola pausa per stabilità
  delay(1);
}
