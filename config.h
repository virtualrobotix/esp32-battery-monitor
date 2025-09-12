/*
 * Configurazione ESP32 Battery Monitor
 * 
 * File di configurazione per personalizzare i parametri del sistema
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// CONFIGURAZIONE HARDWARE
// ============================================================================

// Tipo di sensore corrente ACS758
#define ACS758_MODEL_50A    // Commentare se si usa 100A
// #define ACS758_MODEL_100A   // Decommentare se si usa 100A

#ifdef ACS758_MODEL_50A
  #define ACS758_SENSITIVITY 0.04  // 40mV/A
#else
  #define ACS758_SENSITIVITY 0.02  // 20mV/A
#endif

// Configurazione partitori tensione (modificare se diversi)
#define DIVIDER_6S_R1      22000  // Ohm
#define DIVIDER_6S_R2      3900   // Ohm
#define DIVIDER_4S_R1      15000  // Ohm  
#define DIVIDER_4S_R2      3900   // Ohm

// Calcolo automatico ratio
#define DIVIDER_6S_RATIO   ((float)(DIVIDER_6S_R1 + DIVIDER_6S_R2) / DIVIDER_6S_R2)
#define DIVIDER_4S_RATIO   ((float)(DIVIDER_4S_R1 + DIVIDER_4S_R2) / DIVIDER_4S_R2)

// ============================================================================
// CONFIGURAZIONE SISTEMA
// ============================================================================

// Parametri ADC
#define ADC_SAMPLES        64     // Campioni per media mobile
#define ADC_FILTER_ALPHA   0.1    // Fattore filtro esponenziale (0.0-1.0)

// Parametri PWM
#define PWM_FREQ           50     // Hz (50Hz per ESC/Servo)
#define PWM_RESOLUTION     16     // 16-bit resolution
#define PWM_MIN            1000   // μs
#define PWM_MAX            2000   // μs
#define PWM_CENTER         1500   // μs
#define PWM_DEADBAND       20     // μs zona morta intorno al centro

// Parametri Controllo
#define STEERING_GAIN      0.5    // Guadagno sterzo differenziale (0.0-1.0)
#define THROTTLE_LIMIT     0.8    // Limite massimo throttle (0.0-1.0)
#define RAMP_RATE          0.1    // Velocità variazione PWM (0.0-1.0)

// ============================================================================
// CONFIGURAZIONE TELEMETRIA
// ============================================================================

// Intervalli
#define TELEMETRY_INTERVAL 100    // ms
#define LOG_INTERVAL       1000   // ms
#define WEB_UPDATE_INTERVAL 100   // ms

// Parametri Serial
#define SERIAL_BAUD        115200
#define SERIAL_VERBOSE     true   // Output dettagliato

// ============================================================================
// CONFIGURAZIONE WIFI
// ============================================================================

// Access Point
#define AP_SSID            "ESP32_BatteryMonitor"
#define AP_PASSWORD        "battery123"
#define AP_CHANNEL         1
#define AP_MAX_CONNECTIONS 4

// Web Server
#define WEB_PORT           80
#define WEB_TIMEOUT        5000   // ms

// ============================================================================
// CONFIGURAZIONE SICUREZZA
// ============================================================================

// Limiti di Sicurezza
#define MAX_VOLTAGE_6S     26.0   // V (soglia allarme)
#define MAX_VOLTAGE_4S     17.0   // V (soglia allarme)
#define MIN_VOLTAGE_6S     18.0   // V (soglia scarica)
#define MIN_VOLTAGE_4S     12.0   // V (soglia scarica)
#define MAX_CURRENT        45.0   // A (soglia protezione)
#define MAX_TEMPERATURE    60.0   // °C (se sensore presente)

// Protezioni
#define ENABLE_OVERVOLTAGE_PROTECTION  true
#define ENABLE_UNDERVOLTAGE_PROTECTION true
#define ENABLE_OVERCURRENT_PROTECTION  true
#define ENABLE_EMERGENCY_STOP         true

// ============================================================================
// CONFIGURAZIONE DEBUG
// ============================================================================

// Debug Flags
#define DEBUG_ADC          false
#define DEBUG_PWM          false
#define DEBUG_MOTORS       false
#define DEBUG_BATTERIES    true
#define DEBUG_AUTOPILOT    true
#define DEBUG_WIFI         false

// Test Mode
#define TEST_MODE          false  // Abilita modalità test
#define TEST_PWM_OUTPUT    false  // Test output PWM
#define TEST_ADC_READING   false  // Test lettura ADC

// ============================================================================
// CONFIGURAZIONE AVANZATA
// ============================================================================

// Filtri
#define ENABLE_KALMAN_FILTER    false  // Filtro Kalman per ADC
#define ENABLE_MOVING_AVERAGE   true   // Media mobile
#define ENABLE_EXPONENTIAL_FILTER true // Filtro esponenziale

// Calibrazione
#define ENABLE_AUTO_CALIBRATION false  // Calibrazione automatica
#define CALIBRATION_SAMPLES     100    // Campioni per calibrazione

// Logging
#define ENABLE_DATA_LOGGING     true   // Salvataggio dati su SD
#define LOG_BUFFER_SIZE         1000   // Dimensione buffer log
#define LOG_FILE_MAX_SIZE       1024*1024  // 1MB max per file

// ============================================================================
// MACRO UTILITY
// ============================================================================

// Debug Print
#ifdef DEBUG_ADC
  #define DEBUG_ADC_PRINT(x) Serial.print(x)
  #define DEBUG_ADC_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_ADC_PRINT(x)
  #define DEBUG_ADC_PRINTLN(x)
#endif

#ifdef DEBUG_PWM
  #define DEBUG_PWM_PRINT(x) Serial.print(x)
  #define DEBUG_PWM_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PWM_PRINT(x)
  #define DEBUG_PWM_PRINTLN(x)
#endif

#ifdef DEBUG_MOTORS
  #define DEBUG_MOTORS_PRINT(x) Serial.print(x)
  #define DEBUG_MOTORS_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_MOTORS_PRINT(x)
  #define DEBUG_MOTORS_PRINTLN(x)
#endif

// Utility
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MAP_FLOAT(x, in_min, in_max, out_min, out_max) \
  ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

// ============================================================================
// VERSIONE
// ============================================================================

#define FIRMWARE_VERSION   "1.0.0"
#define BUILD_DATE         __DATE__
#define BUILD_TIME         __TIME__

#endif // CONFIG_H
