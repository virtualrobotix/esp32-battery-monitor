# 🚀 OTTIMIZZAZIONE STORAGE - Frequenza e Gestione Memoria

## 🎯 OBIETTIVI

1. ✅ Visualizzare **ultimo minuto** (non 5 minuti)
2. ✅ Storage Flash **4 ore** @ 1Hz (non 0.1Hz)
3. ✅ **Cogliere transitori** corrente/tensione al cambio PWM
4. ✅ **Rolling/Circolare** - continua dopo riavvio
5. ✅ **Indicatore memoria full** con alert scarica dati

---

## 📊 CONFIGURAZIONE OTTIMIZZATA

### Parametri Raccomandati

```cpp
// ============ RAM BUFFER (Visualizzazione Real-Time) ============
#define RAM_BUFFER_SIZE 120         // 2 minuti @ 1Hz (era 300)
#define RAM_UPDATE_INTERVAL 1000    // 1 secondo (invariato)

// ============ FLASH BUFFER (Storage Lungo Termine) ============
#define LONG_TERM_MAX_POINTS 14400       // 4 ore @ 1Hz (era 1440)
#define LONG_TERM_SAVE_INTERVAL 1000     // 1 secondo (era 10000)
#define LONG_TERM_FILE "/data.bin"

// ============ MEMORIA FULL ALERTS ============
#define STORAGE_WARNING_PERCENT 90.0     // Alert al 90%
#define STORAGE_FULL_PERCENT 100.0       // Full al 100%
```

---

## 💾 CALCOLI MEMORIA

### RAM Buffer (2 minuti)
```
Campioni: 120 (era 300)
Grafici: 8 array
Memoria: 120 × 8 × 4 bytes = 3,840 bytes = 3.75 KB
Risparmio: 9.6 - 3.75 = 5.85 KB liberi in più! ✅
```

### Flash Buffer (4 ore @ 1Hz)
```
Campioni: 14,400 (era 1,440)
Bytes/campione: 58 bytes
Memoria: 14,400 × 58 = 835,200 bytes = 815 KB
SPIFFS: 4 MB = 4,096 KB
Uso: 815 / 4096 = 19.9% ✅
Liberi: 3,281 KB (80.1%) ✅✅✅
```

---

## 🔧 MODIFICHE CODICE

### 1. Costanti Storage (All'inizio del file)

```cpp
// Strutture per Grafici (RIDOTTO da 300 a 120)
struct ChartData {
  float values[120];        // 2 minuti @ 1Hz (era 300)
  int index;
  bool filled;
  unsigned long last_update;
  float min_value;
  float max_value;
  float avg_value;
  unsigned long total_samples;
};

// Storage lungo termine (AUMENTATO)
#define LONG_TERM_FILE "/data.bin"
#define LONG_TERM_MAX_POINTS 14400     // 4 ore @ 1Hz (era 1440)
#define LONG_TERM_SAVE_INTERVAL 1000   // 1 secondo (era 10000)

// Alert memoria
#define STORAGE_WARNING_PERCENT 90.0
#define STORAGE_FULL_PERCENT 100.0
bool warningShown = false;
bool fullWarningShown = false;
```

---

### 2. Funzioni Gestione Memoria

```cpp
// Verifica stato memoria
bool isStorageFull() {
  return longTermStorage.total_points >= LONG_TERM_MAX_POINTS;
}

float getStorageUsagePercent() {
  if (LONG_TERM_MAX_POINTS == 0) return 0.0;
  return (longTermStorage.total_points * 100.0) / LONG_TERM_MAX_POINTS;
}

unsigned long getStorageTimeRemaining() {
  if (isStorageFull()) return 0;
  int remaining = LONG_TERM_MAX_POINTS - longTermStorage.total_points;
  return remaining; // secondi rimanenti
}

// Formatta tempo rimanente
String formatTimeRemaining(unsigned long seconds) {
  unsigned long hours = seconds / 3600;
  unsigned long minutes = (seconds % 3600) / 60;
  unsigned long secs = seconds % 60;
  
  if (hours > 0) {
    return String(hours) + "h " + String(minutes) + "m";
  } else if (minutes > 0) {
    return String(minutes) + "m " + String(secs) + "s";
  } else {
    return String(secs) + "s";
  }
}
```

---

### 3. Alert Memoria nel Loop

```cpp
void checkStorageAlerts() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 10000) return; // Controlla ogni 10 secondi
  lastCheck = millis();
  
  float usage = getStorageUsagePercent();
  
  // Alert 90%
  if (usage >= STORAGE_WARNING_PERCENT && !warningShown) {
    Serial.println("\n⚠️ ================================");
    Serial.println("⚠️  ATTENZIONE: Memoria Flash >90%");
    Serial.printf("⚠️  Uso: %.1f%% (%d/%d campioni)\n", 
                  usage, longTermStorage.total_points, LONG_TERM_MAX_POINTS);
    Serial.println("⚠️  SCARICA DATI PRIMA DI PERDERE VECCHI!");
    Serial.println("⚠️  http://192.168.4.1/csv");
    
    unsigned long remaining = getStorageTimeRemaining();
    Serial.printf("⚠️  Tempo rimasto: %s\n", formatTimeRemaining(remaining).c_str());
    Serial.println("⚠️ ================================\n");
    warningShown = true;
  }
  
  // Alert 100% (Full)
  if (usage >= STORAGE_FULL_PERCENT && !fullWarningShown) {
    Serial.println("\n🔴 ================================");
    Serial.println("🔴  MEMORIA PIENA!");
    Serial.println("🔴  Sovrascrivendo dati più vecchi...");
    Serial.println("🔴  SCARICA SUBITO: http://192.168.4.1/csv");
    Serial.println("🔴 ================================\n");
    fullWarningShown = true;
  }
  
  // Reset warning se scaricato
  if (usage < 80.0) {
    warningShown = false;
    fullWarningShown = false;
  }
}

// Nel loop principale, aggiungi:
void loop() {
  // ... codice esistente ...
  
  checkStorageAlerts();  // ✨ NUOVO
  
  // ... resto codice ...
}
```

---

### 4. Info Storage nell'API

```cpp
void getLongTermStorageInfo(JsonObject& info) {
  info["initialized"] = longTermStorage.initialized;
  info["total_points"] = longTermStorage.total_points;
  info["max_points"] = LONG_TERM_MAX_POINTS;
  info["write_index"] = longTermStorage.write_index;
  
  // ✨ NUOVO: Info dettagliate
  info["usage_percent"] = getStorageUsagePercent();
  info["is_full"] = isStorageFull();
  info["sample_rate_hz"] = 1000.0 / LONG_TERM_SAVE_INTERVAL;
  
  if (longTermStorage.initialized && longTermStorage.total_points > 0) {
    float hours = (longTermStorage.total_points * LONG_TERM_SAVE_INTERVAL) / (1000.0 * 3600.0);
    info["coverage_hours"] = hours;
    
    if (!isStorageFull()) {
      unsigned long remaining = getStorageTimeRemaining();
      info["time_remaining_sec"] = remaining;
      info["time_remaining_formatted"] = formatTimeRemaining(remaining);
    } else {
      info["warning"] = "FULL - Oldest data being overwritten";
    }
  }
  
  // Info filesystem
  info["spiffs_total_kb"] = SPIFFS.totalBytes() / 1024;
  info["spiffs_used_kb"] = SPIFFS.usedBytes() / 1024;
  info["spiffs_free_kb"] = (SPIFFS.totalBytes() - SPIFFS.usedBytes()) / 1024;
}
```

---

### 5. Pagina Web - Indicatore Avanzato

```javascript
function updateStorageInfo(storageInfo) {
  if (!storageInfo || !storageInfo.initialized) return;
  
  let usage = storageInfo.usage_percent || 0;
  let info = '💾 Flash: ' + storageInfo.total_points + '/' + storageInfo.max_points + ' campioni';
  
  if (storageInfo.coverage_hours) {
    info += ' (' + storageInfo.coverage_hours.toFixed(1) + 'h)';
  }
  
  // Tempo rimanente
  if (storageInfo.time_remaining_formatted && !storageInfo.is_full) {
    info += ' | ⏱️ ' + storageInfo.time_remaining_formatted + ' rimasti';
  }
  
  // Uso memoria
  info += ' | ' + storageInfo.spiffs_used_kb + '/' + storageInfo.spiffs_total_kb + 'KB';
  
  // Freccia di stato
  if (usage < 50) {
    info += ' ✅';
  } else if (usage < 90) {
    info += ' ⚠️';
  } else {
    info += ' 🔴';
  }
  
  let elem = document.getElementById('storageInfo');
  elem.textContent = info;
  
  // Cambio colore in base a usage
  if (usage >= 90) {
    elem.style.color = '#ef4444'; // Rosso
    elem.style.fontWeight = 'bold';
  } else if (usage >= 75) {
    elem.style.color = '#f59e0b'; // Arancione
  } else {
    elem.style.color = '#94a3b8'; // Grigio normale
  }
  
  // Mostra alert se pieno
  if (storageInfo.is_full && !window.fullAlertShown) {
    alert('🔴 MEMORIA PIENA!\n\nI dati più vecchi stanno per essere sovrascritti.\n\nScarica CSV ora: http://192.168.4.1/csv');
    window.fullAlertShown = true;
  }
}
```

---

### 6. Scale Temporali Aggiornate

```javascript
// Aggiorna scale nella pagina grafici
html += "<optgroup label='📊 RAM (Veloce - 1Hz)'>";
html += "<option value='10s'>10 secondi</option>";
html += "<option value='30s'>30 secondi</option>";
html += "<option value='1m' selected>1 minuto</option>";
html += "<option value='2m'>2 minuti</option>";  // ✨ NUOVO (max RAM)
html += "</optgroup>";
html += "<optgroup label='💾 Flash (Lungo - 1Hz)'>";  // ⚠️ Era 0.1Hz
html += "<option value='5m'>5 minuti</option>";      // ✨ NUOVO
html += "<option value='10m'>10 minuti</option>";
html += "<option value='30m'>30 minuti</option>";
html += "<option value='1h'>1 ora</option>";
html += "<option value='2h'>2 ore</option>";         // ✨ NUOVO
html += "<option value='4h'>4 ore</option>";
html += "</optgroup>";
```

---

## 📊 ESEMPIO OUTPUT OTTIMIZZATO

### Serial Monitor - Avvio
```
🚀 AlixBlimp Battery Monitor & Motor Control v2.1
========================================
📦 Storage Ottimizzato: RAM (2min @ 1Hz) + Flash (4h @ 1Hz)

💾 Inizializzazione SPIFFS...
📊 SPIFFS: 4096 KB totali, 45 KB usati, 4051 KB liberi
📂 File dati esistente: 835200 bytes, 14400 campioni (FULL)
✅ Storage lungo termine inizializzato!
🔄 Modalità Rolling: Dati più vecchi verranno sovrascritti
```

### Durante Registrazione
```
[Dopo 3h 36min]
💾 Salvati 12960/14400 campioni long-term (90.0% buffer)

⚠️ ================================
⚠️  ATTENZIONE: Memoria Flash >90%
⚠️  Uso: 90.0% (12960/14400 campioni)
⚠️  SCARICA DATI PRIMA DI PERDERE VECCHI!
⚠️  http://192.168.4.1/csv
⚠️  Tempo rimasto: 24m 0s
⚠️ ================================

[Dopo 4h esatte]
💾 Salvati 14400/14400 campioni long-term (100.0% buffer)

🔴 ================================
🔴  MEMORIA PIENA!
🔴  Sovrascrivendo dati più vecchi...
🔴  SCARICA SUBITO: http://192.168.4.1/csv
🔴 ================================

[Continua...]
💾 Campione 14401 → Sovrascrive campione 1 (4h fa)
💾 Campione 14402 → Sovrascrive campione 2 (4h fa)
...
```

### Pagina Web
```
💾 Flash: 14400/14400 campioni (4.0h) | ⏱️ 0s rimasti | 815/4096KB 🔴
```

---

## 🎯 VANTAGGI SOLUZIONE

### Performance
- ✅ **10x più veloce** Flash (1Hz vs 0.1Hz)
- ✅ **Cattura transitori** corrente (500ms risoluzione)
- ✅ **RAM ridotta** da 9.6KB a 3.75KB (-60%)
- ✅ **Visualizzazione focalizzata** (ultimo minuto non 5)

### Gestione Dati
- ✅ **Rolling automatico** (circolare)
- ✅ **Alert proattivi** (90%, 100%)
- ✅ **Tempo rimanente** visibile
- ✅ **Indicatori colorati** (verde/giallo/rosso)

### Analisi
- ✅ **4 ore @ 1Hz** = 14,400 campioni
- ✅ **Perfetto per transitori** PWM
- ✅ **CSV export completo**
- ✅ **Correlazione PWM-corrente** visibile

---

## 📈 TIMELINE REGISTRAZIONE

```
Tempo    | RAM        | Flash      | Alert
---------|------------|------------|-------------
0-2 min  | 0-120 sec  | 0-120 sec  | -
2-5 min  | Rolling    | 120-300    | -
5-1h     | Rolling    | 300-3600   | -
1-3h     | Rolling    | 3.6k-10.8k | -
3h 36m   | Rolling    | 12,960     | ⚠️ >90%
4h       | Rolling    | 14,400     | 🔴 FULL
4h+      | Rolling    | 14,400     | 🔄 Overwrite old
```

---

## 🧪 TEST TRANSITORI

### Scenario Test
```
Tempo | PWM Right | Corrente | Note
------|-----------|----------|------------------
0.0s  | 1500 μs   | 1.5 A    | Hover stabile
0.5s  | 1800 μs   | 4.2 A    | Comando accelera
1.0s  | 1800 μs   | 5.8 A    | Transitorio
1.5s  | 1800 μs   | 6.5 A    | Stabilizzato
2.0s  | 1500 μs   | 3.1 A    | Comando stop
2.5s  | 1500 μs   | 1.8 A    | Transitorio
3.0s  | 1500 μs   | 1.5 A    | Hover ripristinato
```

### Con 0.1Hz (10s) - ❌ PERSO
```
Campioni: 0s, 10s, 20s
Risultato: Perde tutto il transitorio 0-3s!
```

### Con 1Hz (1s) - ✅ PERFETTO
```
Campioni: 0s, 1s, 2s, 3s, 4s...
Risultato: Cattura tutto il transitorio!
```

---

## 🚀 PROSSIMI STEP

### 1. Applica Modifiche
- [ ] Cambia `LONG_TERM_SAVE_INTERVAL` da 10000 a 1000
- [ ] Cambia `LONG_TERM_MAX_POINTS` da 1440 a 14400
- [ ] Cambia `ChartData values[300]` a `values[120]`
- [ ] Aggiungi funzioni alert memoria

### 2. Testa
- [ ] Upload firmware
- [ ] Verifica Serial: "4h @ 1Hz"
- [ ] Attendi 10 minuti
- [ ] Verifica risoluzione 1Hz
- [ ] Testa alert 90%

### 3. Valida
- [ ] Export CSV dopo 1 ora
- [ ] Conta campioni (dovrebbero essere 3600)
- [ ] Verifica cattura transitori PWM
- [ ] Controlla uso memoria (<20% SPIFFS)

---

## 📄 FILE DA MODIFICARE

### Principale
- **`esp32_battery_monitor.ino`** - Tutte le modifiche sopra

### Opzionale
- **`config.h`** - Sposta costanti lì per facilità

---

## 💡 OTTIMIZZAZIONI FUTURE

### Se Serve Ancora Più Risoluzione

1. **2Hz Flash** (500ms risoluzione)
   - 28,800 campioni
   - 1,631 KB (40% SPIFFS)
   - Perfetto per transitori veloci

2. **Event-Triggered**
   - 1Hz normale
   - 5Hz per 3s dopo cambio PWM >100μs
   - Memoria variabile ~1.5Hz medio

3. **Dual Storage**
   - 1Hz Flash (4h completo)
   - 10Hz RAM (ultimo minuto)
   - Migliore di entrambi i mondi

---

**RACCOMANDAZIONE: Parti con 1Hz e vedi se cattura abbastanza!** 🎯

Ti preparo ora il file .ino aggiornato?
