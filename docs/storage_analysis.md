# 💾 Analisi Storage ESP32 - Sistema Multi-Rate

## 🎯 Requisiti
- **Visualizzazione rapida**: 10s, 1min, 5min (da RAM)
- **Storico lungo**: fino a 4 ore (da Flash)
- **Persistenza**: dati salvati anche dopo riavvio

## 📊 Strategia Multi-Rate

### Buffer RAM (Veloce)
- **Frequenza**: 1 campione/secondo
- **Durata**: 5 minuti (300 campioni)
- **Memoria**: 300 × 8 grafici × 4 bytes = 9.6 KB ✅
- **Uso**: Visualizzazione real-time (10s, 1min, 5min)

### Buffer Flash/SPIFFS (Lento)
- **Frequenza**: 1 campione/10 secondi
- **Durata**: 4 ore (1,440 campioni)
- **Memoria**: 1,440 × 21 float × 4 bytes = 120 KB ✅
- **Uso**: Storico lungo termine + CSV export

## 🔧 Implementazione

### Struttura Dati
```cpp
// RAM - Buffer veloce (5 minuti @ 1Hz)
ChartData fast_buffer[8];  // 300 punti × 8 grafici

// Flash - File binario (4 ore @ 0.1Hz)
struct SlowDataPoint {
  uint32_t timestamp;     // 4 bytes
  float v1, c1, v2, c2, v3, c3;  // 24 bytes (tensioni/correnti)
  float rv1, rc1, rv2, rc2, rv3, rc3;  // 24 bytes (raw)
  uint16_t m1, m2, m3;    // 6 bytes (PWM motori)
  // Totale: 58 bytes/campione
};

// 4 ore: 1440 campioni × 58 bytes = 83,520 bytes (~82 KB) ✅
```

### Scale Temporali
| Scala | Campioni | Frequenza | Sorgente | RAM/Flash |
|-------|----------|-----------|----------|-----------|
| 10s   | 10       | 1Hz       | RAM      | 9.6 KB    |
| 1min  | 60       | 1Hz       | RAM      | 9.6 KB    |
| 5min  | 300      | 1Hz       | RAM      | 9.6 KB    |
| 10min | 60       | 0.1Hz     | Flash    | 3.5 KB    |
| 30min | 180      | 0.1Hz     | Flash    | 10.4 KB   |
| 1h    | 360      | 0.1Hz     | Flash    | 20.9 KB   |
| 4h    | 1440     | 0.1Hz     | Flash    | 83.5 KB   |

## 💾 SPIFFS vs Preferences vs SD Card

### SPIFFS (SCELTA MIGLIORE)
✅ 4MB disponibili
✅ Wear leveling automatico
✅ File system completo
✅ Nessun hardware aggiuntivo
⚠️ Scritture limitate (~100k cicli)

### Preferences (EEPROM emulation)
✅ Semplice da usare
❌ Solo 15KB disponibili (troppo poco!)
❌ Non adatto per storage sequenziale

### SD Card
✅ Storage illimitato
✅ Scritture illimitate
❌ Richiede hardware esterno
❌ Più complesso

## 🚀 Strategia Finale

1. **RAM Buffer**: 5 minuti @ 1Hz per visualizzazione real-time
2. **Flash Circular Buffer**: 4 ore @ 0.1Hz per storico lungo
3. **CSV Export**: Genera da RAM + Flash combinati
4. **Auto-save**: Ogni 10 secondi scrivi 1 campione su Flash
5. **Persistenza**: Carica da Flash all'avvio

## 📈 Vantaggi
- ✅ Grafici fluidi (1Hz) per scale brevi
- ✅ Storico lungo (4h) senza saturare RAM
- ✅ Dati persistenti dopo riavvio
- ✅ Export CSV completo fino a 4 ore
- ✅ Nessun hardware aggiuntivo necessario

