# ⚡ GUIDA RAPIDA v2.0 - Storage 4 Ore

## 🎯 COSA È CAMBIATO

**PRIMA**: Dati salvati solo in RAM → 5 minuti → Persi al riavvio  
**ADESSO**: Dati salvati in RAM + Flash → 4 ore → Persistenti dopo riavvio ✅

---

## 📊 SCALE TEMPORALI

### Grafici Veloci (da RAM - 1Hz)
- **10s** → 10 secondi ad alta risoluzione
- **1min** → 1 minuto ad alta risoluzione ⭐ DEFAULT
- **5min** → 5 minuti ad alta risoluzione

### Grafici Storici (da Flash - 0.1Hz)
- **10min** → 10 minuti
- **30min** → 30 minuti
- **1h** → 1 ora
- **4h** → 4 ore complete ⭐ MASSIMO

---

## 🚀 UPLOAD FIRMWARE

### 1. Configurazione Arduino IDE
```
Board: ESP32 Dev Module
Upload Speed: 921600
Flash Frequency: 80MHz
Partition Scheme: Default 4MB with spiffs ⚠️ IMPORTANTE!
```

### 2. Carica Sketch
- Apri: `esp32_battery_monitor.ino`
- Click: Upload (Ctrl+U)
- Attendi: ~30 secondi

### 3. Verifica
Serial Monitor (115200 baud):
```
✅ Storage lungo termine inizializzato!
```

---

## 📥 EXPORT CSV

### Export Tutto (4 ore + 5 minuti)
```
http://192.168.4.1/csv
```

### Export Solo RAM (5 minuti)
```
http://192.168.4.1/csv?type=ram
```

### Export Solo Flash (4 ore)
```
http://192.168.4.1/csv?type=flash
```

---

## 🗑️ CANCELLARE DATI

### Dalla Pagina Web
1. Vai su: `http://192.168.4.1/charts`
2. **Azzera RAM** → Cancella ultimi 5 minuti
3. **Azzera Tutto** → Cancella RAM + Flash (tutto)

### Via API
```bash
# Cancella tutto
curl -X POST http://192.168.4.1/clear-charts?type=all

# Cancella solo RAM
curl -X POST http://192.168.4.1/clear-charts?type=ram

# Cancella solo Flash
curl -X POST http://192.168.4.1/clear-charts?type=flash
```

---

## 💾 MEMORIA UTILIZZATA

| Tipo | Utilizzo | Disponibile | % Usata |
|------|----------|-------------|---------|
| RAM  | ~250 KB  | 520 KB      | 48%     |
| Flash SPIFFS | ~82 KB | 4 MB | 2%      |

**Margine Eccellente!** 🎉

---

## 🔍 MONITORAGGIO

### Indicatore Storage Web
Nella pagina grafici vedrai:
```
💾 Flash: 144/1440 campioni (0.4h) | 45/1500KB
```

### Serial Monitor
```
💾 Salvati 10/1440 campioni long-term (0.7% buffer)
💾 Salvati 20/1440 campioni long-term (1.4% buffer)
```

Ogni riga = +10 secondi di dati salvati!

---

## ⏱️ TIMELINE REGISTRAZIONE

```
Tempo     | RAM  | Flash | Totale Dati
----------|------|-------|-------------
0-5 min   | ✅   | ❌    | 5 min
5-10 min  | ✅   | ✅    | 10 min
10-30 min | ✅   | ✅    | 30 min
30-60 min | ✅   | ✅    | 1 ora
1-4 ore   | ✅   | ✅    | 4 ore
4+ ore    | ✅   | ✅    | 4h + 5min (circolare)
```

**Dopo 4 ore**: Flash inizia a sovrascrivere dati più vecchi (buffer circolare)

---

## 📊 FREQUENZE CAMPIONAMENTO

### RAM (Veloce)
- **Lettura sensori**: ~100 Hz (ogni 10ms)
- **Salvataggio grafico**: 1 Hz (ogni 1s)
- **Durata**: 300 campioni = 5 minuti

### Flash (Lento)
- **Salvataggio**: 0.1 Hz (ogni 10s)
- **Durata**: 1440 campioni = 4 ore

---

## 🎯 CASI D'USO

### Test Breve (< 5 minuti)
- Usa scale **10s, 1min, 5min**
- Visualizzazione fluida ad alta risoluzione
- Export CSV leggero

### Volo Lungo (1-4 ore)
- Usa scale **1h, 4h**
- Visualizza trend lungo termine
- Export CSV completo per analisi post-volo

### Analisi Completa
- Export CSV con `type=all`
- Importa in Excel/Python/MATLAB
- Analizza fino a 4 ore di dati!

---

## 🔧 TROUBLESHOOTING VELOCE

### Problema: SPIFFS non inizializza
```
Soluzione:
Tools → Erase Flash → All Flash Contents
→ Upload sketch
```

### Problema: Dati non persistono
```
Verifica:
1. Partition Scheme = "Default 4MB with spiffs"
2. Attendi 10 secondi prima di spegnere
3. Controlla Serial Monitor: "💾 Salvati..."
```

### Problema: CSV vuoto
```
Attendi:
- Minimo 5 secondi per dati RAM
- Minimo 10 secondi per dati Flash
```

---

## 📞 SUPPORTO

### Serial Monitor Debug
```
1. Baud: 115200
2. Cerca: "❌" o "Errore"
3. Copia output e segnala
```

### Info Sistema
```
http://192.168.4.1/charts-data?scale=1m
```
Controlla campo `storage_info`:
```json
{
  "storage_info": {
    "initialized": true,
    "total_points": 144,
    "max_points": 1440,
    "coverage_hours": 0.4,
    "spiffs_total_kb": 1465,
    "spiffs_used_kb": 45
  }
}
```

---

## ✅ CHECKLIST RAPIDA

Prima del volo:
- [ ] Firmware v2.0 caricato
- [ ] SPIFFS inizializzato
- [ ] Dati salvati correttamente (controlla Serial)
- [ ] Grafici visualizzabili
- [ ] CSV export funzionante

Dopo il volo:
- [ ] Export CSV immediatamente
- [ ] Verifica durata registrazione
- [ ] Backup CSV localmente
- [ ] (Opzionale) Cancella dati per prossimo volo

---

**Pronto per 4 ore di registrazione! 🚀**
