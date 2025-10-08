# ESP32 Battery Monitor & Differential Motor Control

Sistema completo per monitoraggio batterie e controllo differenziale motori con ESP32 DEVKIT V1.

## 🎯 Funzionalità

- **Monitoraggio Batterie**: 3 pacchi (2x6S + 1x4S) con sensori corrente ACS758
- **Controllo Differenziale**: 2 motori con direzione indipendente
- **Interfaccia Autopilota**: 2 PWM input + 2 digital direction
- **Telemetria**: Serial + Web Interface + API JSON
- **WiFi**: Access Point per monitoraggio remoto

## 🔌 Pinout ESP32 DEVKIT V1

### ADC Input (Sensori)
- **GPIO32** - Corrente 6S#1 (ACS758)
- **GPIO33** - Corrente 6S#2 (ACS758)
- **GPIO34** - Corrente 4S (ACS758)
- **GPIO35** - Tensione 6S#1 (Partitore)
- **GPIO36** - Tensione 6S#2 (Partitore)
- **GPIO39** - Tensione 4S (Partitore)

### PWM Input (Autopilota)
- **GPIO18** - Motore Destro (1000-2000μs)
- **GPIO19** - Motore Sinistro (1000-2000μs)
- **GPIO5** - Motori Sottostanti (1000-2000μs)

### Digital Output (Direzione Motori)
- **GPIO17** - Direzione Motore Sinistro (OUTPUT)
- **GPIO21** - Direzione Motore Destro (OUTPUT)

### PWM Output (Motori)
- **GPIO26** - Motore Destro (ESC)
- **GPIO27** - Motore Sinistro (ESC)

## 🔧 Circuito Elettronico

### Sensori Corrente ACS758-50A
- **Sensibilità**: 40mV/A
- **Range**: ±50A
- **Alimentazione**: 3.3V-5V
- **Output**: Analogico (VCC/2 = 0A)

### Partitori Tensione
- **6S Batterie**: R1=22kΩ, R2=3.9kΩ (Ratio 1:8.4)
- **4S Batterie**: R1=15kΩ, R2=3.9kΩ (Ratio 1:5.6)
- **Range ADC**: 0-3.3V (25.2V max per 6S, 16.8V max per 4S)

### Filtri e Protezioni
- **Condensatori**: 100nF su ogni input ADC
- **Resistenze**: 1kΩ di protezione
- **Pull-up**: Su input digitali (se necessari)

## 📊 Algoritmo Controllo Motori

### Logica PWM e Direzione
```cpp
// Input dall'autopilota: 1000-2000μs
// 1500μs = neutro

// Direzione AVANTI (1500-2000μs)
if (input > 1500) {
    pwm_output = map(input, 1500, 2000, 1000, 2000);
    direction_pin = HIGH;  // ATTIVO
}

// Direzione INDIETRO (1000-1500μs)  
if (input <= 1500) {
    pwm_output = map(input, 1000, 1500, 2000, 1000);
    direction_pin = LOW;   // DISATTIVO
}
```

### Mappatura PWM
| Input Autopilota | Range | DIR_PIN | PWM Output | Comportamento |
|------------------|-------|---------|------------|---------------|
| 2000μs | 1500-2000 | **HIGH** | 2000μs | Avanti massimo |
| 1750μs | 1500-2000 | **HIGH** | 1500μs | Avanti medio |
| 1500μs | 1500-2000 | **HIGH** | 1000μs | Neutro (avanti) |
| 1500μs | 1000-1500 | **LOW** | 1000μs | Neutro (indietro) |
| 1250μs | 1000-1500 | **LOW** | 1500μs | Indietro medio |
| 1000μs | 1000-1500 | **LOW** | 2000μs | Indietro massimo |

## 🌐 Interfaccia Web

### Access Point WiFi
- **SSID**: ESP32_BatteryMonitor
- **Password**: battery123
- **IP**: 192.168.4.1

### Endpoints
- **/** - Dashboard principale
- **/api** - API JSON per telemetria
- **/calibration** - Pagina taratura sensori
- **/charts** - Grafici storici
- **/charts-data** - Dati grafici JSON
- **/csv** - Esportazione dati CSV

### Esempio API Response
```json
{
  "batteries": [
    {"voltage": 25.1, "current": 2.3, "power": 57.7},
    {"voltage": 24.8, "current": 1.9, "power": 47.1},
    {"voltage": 16.5, "current": 0.8, "power": 13.2}
  ],
  "autopilot": {
    "motor_right": 1650,
    "motor_left": 1500,
    "motor_under": 1500,
    "dir_right": true,
    "dir_left": true
  },
  "motors": {
    "right_pwm": 1650,
    "left_pwm": 1500
  },
  "loop_frequency": 125.5,
  "uptime": 45000
}
```

## 🚀 Installazione

### 1. Hardware
- Collegare sensori ACS758 ai pin ADC (GPIO32,33,34)
- Collegare partitori tensione ai pin ADC (GPIO35,36,39)
- Collegare autopilota ai pin PWM input (GPIO18,19,5)
- Collegare ESC motori ai pin PWM output (GPIO26,27)
- Collegare pin direzione motori (GPIO17,21) - OUTPUT

### 2. Software
1. Installare ESP32 Board Package in Arduino IDE
2. Selezionare "ESP32 Dev Module"
3. Caricare il codice `esp32_battery_monitor.ino`
4. Configurare parametri se necessario

### 3. Configurazione
```cpp
// Modificare questi parametri se necessario
#define ACS758_SENSITIVITY 0.04  // 40mV/A per 50A, 20mV/A per 100A
#define DIVIDER_6S_RATIO   8.4   // Aggiustare se partitori diversi
#define DIVIDER_4S_RATIO   5.6   // Aggiustare se partitori diversi
```

## 📈 Monitoraggio

### Serial Monitor
- Telemetria ogni 100ms
- Dati batterie, autopilota, motori
- Frequenza loop e statistiche

### Web Dashboard
- Interfaccia grafica real-time
- Dati batterie con colori
- Status sistema completo
- Auto-refresh ogni secondo
- Pagina taratura sensori
- Grafici storici con esportazione CSV

### API JSON
- Endpoint `/api` per integrazione
- Dati strutturati per applicazioni esterne
- Formato JSON standard

## ⚠️ Note Importanti

### Sicurezza
- **Tensioni Alte**: 6S batterie = 25.2V max
- **Correnti Alte**: Fino a 50A per sensore
- **Isolamento**: Usare partitori resistivi
- **Fusibili**: Proteggere circuiti

### Calibrazione
- Verificare partitori con multimetro
- Calibrare sensori corrente con carico noto
- Testare range PWM con oscilloscopio
- Validare direzioni motori

### Ottimizzazione
- Aggiustare `ADC_SAMPLES` per stabilità
- Modificare `TELEMETRY_INTERVAL` per frequenza
- Regolare mixing factor per steering
- Personalizzare WiFi credentials

## 🔧 Troubleshooting

### Problemi Comuni
1. **ADC instabile**: Aumentare `ADC_SAMPLES`
2. **PWM non funziona**: Verificare frequenza ESC
3. **WiFi non si connette**: Controllare SSID/password
4. **Correnti errate**: Calibrare sensori ACS758
5. **Motori non vanno indietro**: Verificare pin direzione (GPIO17,21)
6. **Pin direzione non funzionano**: Controllare che siano configurati come OUTPUT

### Debug
- Abilitare Serial Monitor a 115200 baud
- Verificare connessioni hardware
- Testare singoli componenti
- Usare multimetro per validazione

## 📝 Changelog

### v1.1.0 (Corrente)
- ✅ **CORRETTO**: Pin direzione motori (GPIO17,21) ora funzionano correttamente
- ✅ **CORRETTO**: Logica PWM per direzione avanti/indietro
- ✅ **AGGIUNTO**: Debug seriale per monitoraggio motori
- ✅ **AGGIUNTO**: Pagina taratura sensori avanzata
- ✅ **AGGIUNTO**: Grafici storici con esportazione CSV
- ✅ **AGGIUNTO**: Motore sottostanti (GPIO5)
- ✅ **MIGLIORATO**: Algoritmo controllo motori semplificato
- ✅ **MIGLIORATO**: Pinout aggiornato (GPIO26,27 per PWM output)

### v1.0.0
- Implementazione base sistema
- Monitoraggio 3 batterie
- Controllo differenziale motori
- Interfaccia web e API
- Telemetria completa

## 📄 Licenza

Progetto di supporto al Drone Dirigibile AlixBlimp

## 🤝 Contributi

Benvenuti contributi per:
- Miglioramenti algoritmi
- Nuove funzionalità
- Ottimizzazioni performance
- Documentazione
