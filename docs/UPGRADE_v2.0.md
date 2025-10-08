# 🚀 UPGRADE v2.0 - Sistema Multi-Rate Storage

## ✅ IMPLEMENTAZIONE COMPLETATA

Il sistema è stato aggiornato per supportare **storage multi-rate** con dati persistenti fino a **4 ore**.

---

## 📊 NUOVE FUNZIONALITÀ

### 1. **Storage Dual-Layer**

#### 🏃‍♂️ RAM Buffer (Veloce)
- **Frequenza**: 1 campione/secondo (1 Hz)
- **Durata**: 5 minuti (300 campioni)
- **Memoria**: ~10 KB RAM
- **Uso**: Visualizzazione real-time ad alta risoluzione

#### 💾 Flash/SPIFFS Buffer (Lungo Termine)
- **Frequenza**: 1 campione/10 secondi (0.1 Hz)
- **Durata**: 4 ore (1,440 campioni)
- **Memoria**: ~82 KB Flash
- **Uso**: Storico lungo termine + Persistenza dopo riavvio
- **Persistenza**: ✅ Dati salvati anche se l'ESP32 si spegne!

---

## 🎯 SCALE TEMPORALI DISPONIBILI

### Scale Brevi (da RAM)
| Scala | Campioni | Frequenza | Durata Reale | Risoluzione |
|-------|----------|-----------|--------------|-------------|
| 10s   | 10       | 1 Hz      | 10 secondi   | Alta        |
| 1min  | 60       | 1 Hz      | 1 minuto     | Alta        |
| 5min  | 300      | 1 Hz      | 5 minuti     | Alta        |

### Scale Lunghe (da Flash)
| Scala | Campioni | Frequenza | Durata Reale | Risoluzione |
|-------|----------|-----------|--------------|-------------|
| 10min | 60       | 0.1 Hz    | 10 minuti    | Media       |
| 30min | 180      | 0.1 Hz    | 30 minuti    | Media       |
| 1h    | 360      | 0.1 Hz    | 1 ora        | Media       |
| 4h    | 1440     | 0.1 Hz    | 4 ore        | Media       |

---

## 💾 EXPORT CSV POTENZIATO

### Nuove Opzioni di Export

1. **Export Completo** (Default)
   ```
   http://192.168.4.1/csv
   ```
   Esporta **RAM + Flash** = fino a 4 ore e 5 minuti di dati!

2. **Export Solo RAM**
   ```
   http://192.168.4.1/csv?type=ram
   ```
   Esporta solo gli ultimi 5 minuti ad alta risoluzione

3. **Export Solo Flash**
   ```
   http://192.168.4.1/csv?type=flash
   ```
   Esporta solo i dati storici (4 ore)

### Formato CSV
```csv
Timestamp_ms,6S1_Voltage,6S1_Current,6S1_RawVoltage,6S1_RawCurrent,...,Source
12345000,25.10,2.30,2.987,2.512,...,Flash
12355000,25.08,2.35,2.985,2.518,...,Flash
23456000,25.05,2.40,2.983,2.525,...,RAM
```

**Colonna "Source"**: indica se il dato proviene da Flash (più vecchio) o RAM (più recente)

---

## 🗑️ GESTIONE DATI

### Nuovi Pulsanti nella Pagina Grafici

1. **🗑️ Azzera RAM**
   - Cancella solo i dati RAM (ultimi 5 minuti)
   - Mantiene i dati Flash intatti

2. **🗑️ Azzera Tutto**
   - Cancella RAM + Flash
   - Reset completo di tutti i dati storici

---

## 📈 INTERFACCIA WEB AGGIORNATA

### Indicatore Storage
Nella pagina grafici, ora vedrai:
```
💾 Flash: 144/1440 campioni (0.4h) | 45/1500KB
```

**Legenda**:
- `144/1440`: campioni salvati / massimo (10% pieno)
- `0.4h`: ore di dati attualmente salvati
- `45/1500KB`: spazio utilizzato / totale SPIFFS

---

## 🔧 MODIFICHE TECNICHE

### File Modificati
- ✅ `esp32_battery_monitor.ino` - Sistema completo aggiornato

### Nuove Librerie Richieste
```cpp
#include <SPIFFS.h>  // ✅ Già presente in ESP32 Arduino Core
#include <FS.h>      // ✅ Già presente in ESP32 Arduino Core
```

### Nuove Strutture Dati
```cpp
struct LongTermDataPoint {
  uint32_t timestamp;       // 4 bytes
  float v1, c1, v2, c2, v3, c3;  // 24 bytes (tensioni/correnti)
  float rv1, rc1, rv2, rc2, rv3, rc3;  // 24 bytes (raw)
  uint16_t m1, m2, m3;      // 6 bytes (PWM motori)
  // Totale: 58 bytes/campione
} __attribute__((packed));
```

---

## 📊 USO MEMORIA

### Prima (v1.x)
- **RAM**: 9.6 KB
- **Flash**: 0 KB (dati non persistenti)
- **Durata**: 5 minuti
- **Persistenza**: ❌ Persi al riavvio

### Dopo (v2.0)
- **RAM**: 9.6 KB (invariato)
- **Flash**: ~82 KB (storage lungo termine)
- **Durata**: 4 ore e 5 minuti
- **Persistenza**: ✅ Dati salvati dopo riavvio

### Memoria Disponibile ESP32
- **RAM Totale**: 520 KB → Usata: ~250 KB → **Libera: ~270 KB** ✅
- **Flash SPIFFS**: 4 MB → Usata: ~82 KB → **Libera: ~3.9 MB** ✅

**Margine eccellente!** 🎉

---

## 🚀 COME USARE

### 1. Carica il Firmware Aggiornato
```bash
# Arduino IDE
1. Apri esp32_battery_monitor.ino
2. Seleziona "ESP32 Dev Module"
3. Tools → Partition Scheme → "Default 4MB with spiffs"
4. Upload
```

### 2. Primo Avvio
Il sistema:
- Inizializza SPIFFS automaticamente
- Crea file `/data.bin` per storage lungo termine
- Inizia a salvare dati ogni 10 secondi

### 3. Visualizza Grafici
```
http://192.168.4.1/charts
```

1. Seleziona scala temporale dal menu
2. Le scale **10s, 1min, 5min** mostrano dati ad alta risoluzione (RAM)
3. Le scale **10min, 30min, 1h, 4h** mostrano storico lungo (Flash)

### 4. Export Dati
```
http://192.168.4.1/csv
```

Download automatico CSV con tutti i dati disponibili!

---

## 🔍 VERIFICA FUNZIONAMENTO

### Serial Monitor (115200 baud)
All'avvio vedrai:
```
🚀 AlixBlimp Battery Monitor & Motor Control v2.0
========================================
📦 Storage Multi-Rate: RAM (5min @ 1Hz) + Flash (4h @ 0.1Hz)

💾 Inizializzazione SPIFFS...
📊 SPIFFS: 1465 KB totali, 45 KB usati, 1420 KB liberi
📂 File dati esistente: 8352 bytes, 144 campioni
✅ Storage lungo termine inizializzato!
```

Durante il funzionamento:
```
💾 Salvati 10/1440 campioni long-term (0.7% buffer)
💾 Salvati 20/1440 campioni long-term (1.4% buffer)
...
```

### Pagina Grafici
Verifica che appaia l'indicatore:
```
💾 Flash: 144/1440 campioni (0.4h) | 45/1500KB
```

---

## ⚠️ NOTE IMPORTANTI

### 1. Partizionamento Flash
**IMPORTANTE**: Seleziona lo schema di partizionamento corretto:
```
Tools → Partition Scheme → "Default 4MB with spiffs"
```

### 2. Primo Upload
Al primo upload, SPIFFS viene formattato automaticamente (richiede ~10 secondi).

### 3. Persistenza Dati
- ✅ Dati Flash persistono dopo riavvio
- ✅ Dati Flash persistono dopo reset
- ❌ Dati Flash vengono cancellati se ri-carichi firmware con formattazione SPIFFS

### 4. Wear Leveling
SPIFFS ha wear leveling automatico:
- Scrittura ogni 10s = 360 scritture/ora
- 100,000 cicli di vita / 360 = **278 ore** di scrittura continua per settore
- Con wear leveling distribuito = **anni di utilizzo** 🎉

---

## 🐛 TROUBLESHOOTING

### Problema: "❌ Errore montaggio SPIFFS!"
**Soluzione**: 
1. Verifica partizionamento: `Tools → Partition Scheme → Default 4MB with spiffs`
2. Carica sketch con `Tools → Erase Flash → All Flash Contents`

### Problema: Storage non si riempie
**Verifica**:
1. Serial Monitor: controlla messaggi "💾 Salvati X/1440 campioni"
2. Attendi almeno 20 secondi per vedere primi salvataggi

### Problema: CSV vuoto
**Verifica**:
1. Attendi almeno 10 secondi (primo salvataggio Flash)
2. Controlla che ci siano dati in RAM (attendi 5 secondi)
3. Verifica su Serial Monitor: "📤 Generazione CSV..."

---

## 📈 PROSSIMI MIGLIORAMENTI POSSIBILI

### Opzionale - Non Implementato
1. **RTC Real-Time Clock**: Timestamp assoluti invece di millisecondi dall'avvio
2. **SD Card Support**: Storage illimitato (richiede hardware aggiuntivo)
3. **Compressione Dati**: Aumentare durata storage da 4h a 12h+
4. **Upload Automatico**: Invio dati a server remoto via WiFi

---

## 📄 COMPATIBILITÀ

### v1.x → v2.0
- ✅ **100% Retrocompatibile**
- ✅ Tutte le funzioni precedenti mantengono comportamento
- ✅ API `/api`, `/calibration` invariate
- ✅ Nessuna modifica richiesta al circuito hardware

---

## 🎉 RIEPILOGO VANTAGGI

| Caratteristica | v1.x | v2.0 | Miglioramento |
|----------------|------|------|---------------|
| Durata Storage | 5 min | 4h 5min | **+4900%** |
| Persistenza Dati | ❌ No | ✅ Sì | Dati salvati dopo spegnimento |
| Export CSV | Solo RAM | RAM + Flash | Fino a 4 ore dati |
| Uso RAM | 9.6 KB | 9.6 KB | Invariato |
| Uso Flash | 0 KB | 82 KB | +82 KB (2% SPIFFS) |
| Scale Visualizzazione | 3 | 7 | +133% |
| Gestione Memoria | RAM | RAM + Flash | Ottimizzato |

---

## ✅ CHECKLIST POST-UPGRADE

- [ ] Firmware v2.0 caricato correttamente
- [ ] Serial Monitor mostra "✅ Storage lungo termine inizializzato!"
- [ ] Pagina grafici mostra indicatore Flash
- [ ] Scale lunghe (10min, 1h, 4h) funzionano
- [ ] Export CSV include dati Flash
- [ ] Dati persistono dopo riavvio ESP32

---

**Versione**: 2.0  
**Data**: Ottobre 2025  
**Autore**: Sistema AlixBlimp BMS  
**Licenza**: Progetto AlixBlimp
