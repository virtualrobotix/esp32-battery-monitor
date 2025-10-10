# ⚡ RIEPILOGO VELOCE v2.1 - Sistema Storage e Grafici

## 🎯 IN BREVE

**ESP32 Battery Monitor v2.1 OPTIMIZED** registra **4 ore** di dati @ **1Hz** con visualizzazione veloce e persistenza completa.

---

## 📊 STORAGE E VISUALIZZAZIONE

### 💾 Due Livelli di Memoria

#### RAM (Ultimi 2 Minuti)
- **Capacità**: 120 campioni
- **Frequenza**: 1 campione/secondo
- **Uso**: Visualizzazione real-time veloce
- **Persistenza**: ❌ Volatile (perso al riavvio)

#### Flash (Ultimi 4 Ore)
- **Capacità**: 14,400 campioni  
- **Frequenza**: 1 campione/secondo
- **Uso**: Storico lungo termine
- **Persistenza**: ✅ Sopravvive a riavvio! 🎉

---

## 📈 SCALE TEMPORALI (10 totali)

### Veloci (RAM - < 1s caricamento)
- **10 secondi** → 10 campioni @ 1Hz
- **30 secondi** → 30 campioni @ 1Hz
- **1 minuto** → 60 campioni @ 1Hz ⭐ **DEFAULT**
- **2 minuti** → 120 campioni @ 1Hz

### Medie (Flash - 2-3s caricamento)
- **5 minuti** → 300 campioni @ 1Hz
- **10 minuti** → 600 campioni @ 1Hz

### Lunghe (Flash - 3-6s caricamento)
- **30 minuti** → 600 punti (decimato 3x)
- **1 ora** → 360 punti (decimato 10x)
- **2 ore** → 720 punti (decimato 10x)
- **4 ore** → 1,440 punti (decimato 10x)

---

## ⚡ DECIMAZIONE INTELLIGENTE

### Cos'è?
Per grafici lunghi, mostra solo punti necessari per velocizzare senza perdere qualità.

### Quando Si Attiva?
- **≤10 minuti**: Nessuna decimazione (tutti i punti @ 1Hz)
- **30 minuti**: Decimazione 3x (1 punto ogni 3s)
- **≥1 ora**: Decimazione 10x (1 punto ogni 10s)

### Velocità
- **Scala 4h senza**: ~21 secondi 🔴
- **Scala 4h con**: **~6 secondi** ✅ **(-71%)**

### Qualità
**Identica!** La decimazione elimina solo ridondanza invisibile.

---

## 🔔 ALERT MEMORIA

### ⚠️ Alert 90% (dopo 3h 36min)
```
Serial + Web: "Scarica dati! 24 minuti rimasti"
Colore: Arancione
```

### 🔴 Alert 100% (dopo 4h)
```
Serial + Web: "MEMORIA PIENA! Sovrascrivendo vecchi dati"
Colore: Rosso + Popup
```

### ♾️ Rolling Mode
Dopo 4 ore continua infinitamente sovrescrivendo dati più vecchi.

---

## 📥 EXPORT CSV

### Come Scaricare
```
http://192.168.4.1/csv
```

### Contenuto
- **Tutti i dati** @ 1Hz (nessuna decimazione)
- **14,400 campioni** max (4 ore)
- **16 parametri** per campione
- **Dimensione**: ~1.2 MB

### Cosa Include
- ✅ Tensioni 3 batterie (calibrate + raw)
- ✅ Correnti 3 batterie (calibrate + raw)
- ✅ PWM 3 motori
- ✅ Timestamp assoluto
- ✅ Etichetta sorgente (Flash/RAM)

---

## 🎯 COME USARE

### Monitoring Volo
```
1. Vai su: http://192.168.4.1/charts
2. Scala: "1m" (default)
3. Vedi ultimo minuto in tempo reale
4. Auto-refresh: ogni 5s
```

### Analisi Dopo Volo
```
1. Atterra
2. Scala: "4h" (overview completo)
3. Identifica anomalie
4. Zoom: Scala "5m" su zona interessante
5. Export CSV per analisi dettagliata
```

### Prima Nuova Missione
```
1. Se memoria >90%: Scarica CSV
2. Click: 🗑️ Azzera Tutto
3. Conferma reset
4. Memoria pronta! ✅
```

---

## 📊 DATI REGISTRATI

**16 parametri ogni secondo**:

### Batterie (×3)
- Tensione calibrata (V)
- Corrente calibrata (A)
- Tensione raw (V)
- Corrente raw (V)

### Motori (×3)
- PWM Right (μs)
- PWM Left (μs)
- PWM Under (μs)

### Timestamp
- Millisecondi dall'avvio

---

## ⚡ PERFORMANCE

| Operazione | Tempo |
|------------|-------|
| Caricamento scala 10s-2m | < 1s ⚡⚡⚡ |
| Caricamento scala 5m-10m | 2-3s ⚡⚡ |
| Caricamento scala 30m-4h | 4-6s ⚡ |
| Export CSV (4h) | 10-15s |
| Salvataggio Flash | Automatico ogni 1s |
| Aggiornamento grafico | Ogni 5s (auto) |

---

## 💾 MEMORIA

| Risorsa | Usata | Totale | Libera |
|---------|-------|--------|--------|
| RAM | 246 KB | 520 KB | **274 KB (53%)** ✅ |
| Flash | 815 KB | 4 MB | **3,281 KB (80%)** ✅ |

**Margine eccellente per future espansioni!** 🎉

---

## 📚 DOCUMENTI DISPONIBILI

### Per Iniziare
👉 **[GUIDA_UTENTE_v2.1.md](GUIDA_UTENTE_v2.1.md)** - Manuale completo

### Per Dettagli Tecnici
👉 **[RIEPILOGO_IMPLEMENTAZIONE.md](RIEPILOGO_IMPLEMENTAZIONE.md)** - Implementazione

### Per Ottimizzazioni
👉 **[DECIMAZIONE_GRAFICI.md](DECIMAZIONE_GRAFICI.md)** - Performance
👉 **[OTTIMIZZAZIONE_STORAGE.md](OTTIMIZZAZIONE_STORAGE.md)** - Storage

### Per Modifiche
👉 **[CHANGELOG_v2.1.md](CHANGELOG_v2.1.md)** - Cosa è cambiato

---

## ✅ CHECKLIST RAPIDA

Prima del volo:
- [ ] Firmware v2.1 caricato
- [ ] Serial: "v2.1 OPTIMIZED" visibile
- [ ] Memoria <50% (o azzerata)
- [ ] Grafici funzionanti
- [ ] WiFi raggiungibile

Durante volo:
- [ ] Sistema registra automaticamente
- [ ] (Opzionale) Monitoring real-time

Dopo volo:
- [ ] Scarica CSV immediatamente
- [ ] Backup file localmente
- [ ] Analisi con Excel/Python
- [ ] (Opzionale) Azzera dati

---

## 🚀 CARATTERISTICHE CHIAVE

### ✅ Cosa Fa Bene
- ⚡ **Cattura transitori** corrente/PWM (1s risoluzione)
- 💾 **4 ore storage** persistente
- 🔄 **Rolling infinito** automatico
- ⚠️ **Alert proattivi** memoria
- 📊 **10 scale** temporali ottimizzate
- 📥 **Export completo** CSV @ 1Hz
- ⚡ **Grafici veloci** (< 6s tutte scale)

### ⚠️ Limitazioni
- **Risoluzione minima**: 1 secondo (eventi <500ms persi)
- **Durata massima**: 4 ore (poi rolling)
- **After 4h**: Dati vecchi sovrascritti (scarica prima!)

---

## 💡 TIP DELL'ESPERTO

### Massimizza Uso Sistema

1. **Scarica CSV ogni missione** - Archivio completo
2. **Usa scala appropriata** - 1m per real-time, 4h per overview
3. **Monitora alert** - Scarica prima del 100%
4. **Analizza offline** - CSV in Python/MATLAB
5. **Backup regolare** - Non perdere dati importanti

---

## 🎉 RISULTATO FINALE

**Sistema professionale** con:
- ✅ 4 ore storage @ 1Hz
- ✅ Visualizzazione veloce (< 6s)
- ✅ Persistenza dati
- ✅ Alert intelligenti
- ✅ Decimazione ottimizzata
- ✅ Export CSV completo

**Pronto per missioni professionali!** 🚀📊💾

---

**Leggi la guida completa**: [`GUIDA_UTENTE_v2.1.md`](GUIDA_UTENTE_v2.1.md)
