# 🔋 ESP32 Battery Monitor v2.1 OPTIMIZED

Sistema professionale di monitoraggio batterie con storage 4 ore e visualizzazione ottimizzata.

---

## 🚀 QUICK START

### 1. Carica Firmware
```
Arduino IDE:
- Apri: esp32_battery_monitor/esp32_battery_monitor.ino
- Tools → Board → ESP32 Dev Module
- Tools → Partition Scheme → "Default 4MB with spiffs"
- Upload
```

### 2. Connetti e Usa
```
WiFi: "ESP32_BatteryMonitor" / "battery123"
Browser: http://192.168.4.1
Grafici: http://192.168.4.1/charts
```

### 3. Leggi Documentazione
👉 **[`docs/START_HERE.md`](docs/START_HERE.md)** - Inizia qui!

---

## 📚 DOCUMENTAZIONE

### 📖 Tutta la documentazione è in [`docs/`](docs/)

#### File Principali
- **[START_HERE.md](docs/START_HERE.md)** ⭐ - Punto di partenza
- **[RIEPILOGO_VELOCE_v2.1.md](docs/RIEPILOGO_VELOCE_v2.1.md)** ⭐ - Sintesi 3 minuti
- **[GUIDA_UTENTE_v2.1.md](docs/GUIDA_UTENTE_v2.1.md)** ⭐⭐ - Manuale completo

#### File HTML → PDF
- **[DOCUMENTAZIONE_COMPLETA_v2.1.html](docs/DOCUMENTAZIONE_COMPLETA_v2.1.html)** ⭐⭐⭐ - Tutto in uno (genera PDF!)

#### Guide Complete
Vedi: **[docs/INDICE_DOCUMENTAZIONE_v2.1.md](docs/INDICE_DOCUMENTAZIONE_v2.1.md)**

---

## ⚡ CARATTERISTICHE v2.1

- 📊 **Storage**: 4 ore @ 1Hz (14,400 campioni)
- 💾 **Persistenza**: Flash SPIFFS (sopravvive riavvio)
- 📈 **Scale**: 10 temporali (10s → 4h)
- ⚡ **Grafici**: < 6s caricamento (decimazione auto)
- 🔔 **Alert**: Memoria 90%, 100% + tempo rimanente
- 📥 **Export**: CSV completo 16 parametri @ 1Hz
- 🔄 **Rolling**: Continuo infinito

---

## 🎯 SPECIFICA TECNICA

| Parametro | Valore |
|-----------|--------|
| **RAM Buffer** | 120 campioni (2 min @ 1Hz) |
| **Flash Buffer** | 14,400 campioni (4h @ 1Hz) |
| **Risoluzione** | 1 campione/secondo |
| **Cattura Transitori** | ✅ Eventi > 500ms |
| **RAM Usata** | 246 KB (53% libera) |
| **Flash Usata** | 815 KB (80% libera) |
| **Scale Disponibili** | 10 (10s, 30s, 1m, 2m, 5m, 10m, 30m, 1h, 2h, 4h) |

---

## 📊 DATI REGISTRATI

### Per Ogni Batteria (×3)
- Tensione calibrata (V)
- Corrente calibrata (A)
- Tensione raw (V)
- Corrente raw (V)

### PWM Motori (×3)
- Motor Right (μs)
- Motor Left (μs)
- Motor Under (μs)

### Metadata
- Timestamp (ms)

**Totale**: 16 parametri/campione

---

## 🌐 INTERFACCIA WEB

- **Dashboard**: http://192.168.4.1
- **Grafici**: http://192.168.4.1/charts
- **Calibrazione**: http://192.168.4.1/calibration
- **API JSON**: http://192.168.4.1/api
- **Export CSV**: http://192.168.4.1/csv

---

## 🔌 HARDWARE

### Pinout ESP32 DEVKIT V1

**ADC Input (Sensori)**:
- GPIO32, 33, 34 - Correnti (ACS758)
- GPIO35, 36, 39 - Tensioni (Partitori)

**PWM Input (Autopilota)**:
- GPIO18, 19, 5 - Motori input

**PWM Output (Motori)**:
- GPIO26, 27 - Motori output

**Digital Output (Direzioni)**:
- GPIO17, 21 - Direzioni motori

---

## 📥 EXPORT CSV

```
http://192.168.4.1/csv           → Tutti i dati
http://192.168.4.1/csv?type=ram  → Solo RAM (2 min)
http://192.168.4.1/csv?type=flash → Solo Flash (4h)
```

**Formato**: 17 colonne (16 dati + Source)  
**Risoluzione**: 1Hz completo (nessuna decimazione)

---

## 🆘 SUPPORTO

### Documentazione
📚 **[docs/](docs/)** - Tutta la documentazione

### Quick Reference
⚡ **[docs/RIEPILOGO_VELOCE_v2.1.md](docs/RIEPILOGO_VELOCE_v2.1.md)** - 3 minuti

### Manuale Completo
📖 **[docs/GUIDA_UTENTE_v2.1.md](docs/GUIDA_UTENTE_v2.1.md)** - 15 minuti

### Formato PDF
📄 **[docs/GENERA_PDF_ISTRUZIONI.md](docs/GENERA_PDF_ISTRUZIONI.md)** - Come generare PDF

---

## 📦 STRUTTURA PROGETTO

```
esp32-battery-monitor/
├── config.h                      - Configurazione sistema
├── esp32_battery_monitor/
│   └── esp32_battery_monitor.ino - Firmware v2.1 OPTIMIZED
└── docs/                         - 📚 TUTTA LA DOCUMENTAZIONE
    ├── START_HERE.md             ⭐ Inizia qui
    ├── GUIDA_UTENTE_v2.1.md      ⭐ Manuale completo
    ├── DOCUMENTAZIONE_COMPLETA_v2.1.html  ⭐ PDF tutto-in-uno
    └── ... altri 24 file
```

---

## 🎉 VERSIONE

**v2.1 OPTIMIZED** - Ottobre 2025

### Novità v2.1
- ⚡ Flash @ 1Hz (era 0.1Hz) - **+900% risoluzione**
- ⚡ Decimazione grafici - **-70% tempo caricamento**
- ⚡ Alert memoria - Proattivi 90%/100%
- ⚡ RAM ottimizzata - 2 min (era 5)
- ⚡ 10 scale temporali (era 7)

### Changelog Completo
📝 **[docs/CHANGELOG_v2.1.md](docs/CHANGELOG_v2.1.md)**

---

## 📄 LICENZA

Progetto AlixBlimp BMS  
© 2025

---

**Inizia con**: [`docs/START_HERE.md`](docs/START_HERE.md) 🚀
