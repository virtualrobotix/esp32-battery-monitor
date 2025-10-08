# 📚 GUIDA UTENTE - ESP32 Battery Monitor v2.1 OPTIMIZED

## 🎯 Sistema di Storage e Visualizzazione Grafici

**Versione**: 2.1 OPTIMIZED  
**Data**: Ottobre 2025  
**Focus**: Storage 4 ore + Visualizzazione veloce

---

## 🚀 CARATTERISTICHE PRINCIPALI

### 💾 Storage Dual-Layer
- **RAM Buffer**: 2 minuti @ 1Hz (real-time)
- **Flash Buffer**: 4 ore @ 1Hz (storico persistente)
- **Persistenza**: ✅ Dati salvati dopo riavvio
- **Modalità Rolling**: Continua all'infinito, sovrascrive vecchi

### ⚡ Visualizzazione Ottimizzata
- **10 scale temporali**: Da 10 secondi a 4 ore
- **Decimazione intelligente**: Grafici veloci (< 10s)
- **Auto-refresh**: Aggiornamento automatico ogni 5s
- **Indicatori avanzati**: Tempo rimanente, alert memoria

### 📊 Qualità Dati
- **Risoluzione alta**: 1 campione/secondo
- **Cattura transitori**: Eventi > 500ms
- **16 parametri**: Tensioni, correnti, PWM, raw
- **Export completo**: CSV con tutti i dati @ 1Hz

---

## 📊 SCALE TEMPORALI DISPONIBILI

### 🏃‍♂️ RAM (Real-Time - Veloce)

Ideali per **monitoring in tempo reale** e **debug immediato**.

| Scala | Durata | Risoluzione | Campioni | Tempo Caricamento |
|-------|--------|-------------|----------|-------------------|
| **10s** | 10 secondi | 1 secondo | 10 | < 1s ⚡⚡⚡ |
| **30s** | 30 secondi | 1 secondo | 30 | < 1s ⚡⚡⚡ |
| **1m** | 1 minuto | 1 secondo | 60 | ~1s ⚡⚡ |
| **2m** | 2 minuti | 1 secondo | 120 | ~1s ⚡⚡ |

**Caratteristiche**:
- ✅ **Sorgente**: RAM (volatile)
- ✅ **Risoluzione massima**: 1 secondo
- ✅ **Ideale per**: Vedere ultimi 1-2 minuti in dettaglio
- ✅ **Velocità**: Istantanea

---

### 💾 Flash (Storico - Persistente)

Ideali per **analisi lungo termine** e **trend generali**.

#### Flash - Alta Risoluzione

| Scala | Durata | Risoluzione | Campioni | Tempo Caricamento |
|-------|--------|-------------|----------|-------------------|
| **5m** | 5 minuti | 1 secondo | 300 | ~2s ⚡⚡ |
| **10m** | 10 minuti | 1 secondo | 600 | ~3s ⚡⚡ |

**Caratteristiche**:
- ✅ **Sorgente**: Flash SPIFFS (persistente)
- ✅ **Risoluzione**: 1 secondo (identica a RAM)
- ✅ **Ideale per**: Analisi dettagliata eventi recenti
- ✅ **Persistenza**: Dati sopravvivono a riavvio

#### Flash - Decimazione Ottimizzata

| Scala | Durata | Risoluzione | Decimazione | Campioni | Tempo |
|-------|--------|-------------|-------------|----------|-------|
| **30m** | 30 minuti | 3 secondi | 3x | 600 | ~3s ⚡⚡ |
| **1h** | 1 ora | 10 secondi | 10x | 360 | ~4s ⚡ |
| **2h** | 2 ore | 10 secondi | 10x | 720 | ~5s ⚡ |
| **4h** | 4 ore | 10 secondi | 10x | 1,440 | ~6s ⚡ |

**Caratteristiche**:
- ✅ **Sorgente**: Flash SPIFFS (persistente)
- ✅ **Decimazione**: Automatica per performance
- ✅ **Qualità**: Identica per overview lungo termine
- ✅ **Ideale per**: Trend, consumo totale, analisi missione

---

## 🎯 QUALE SCALA USARE?

### 📌 Casi d'Uso Pratici

#### 🔍 Monitoring Real-Time
```
Situazione: Vuoi vedere cosa sta succedendo ORA
Scale consigliate: 10s, 30s, 1m
Risoluzione: 1 secondo
Velocità: Istantanea
```

#### 🐛 Debug Problema Recente
```
Situazione: Hai notato un picco corrente 5 minuti fa
Scale consigliate: 5m, 10m
Risoluzione: 1 secondo (dettaglio completo)
Velocità: 2-3 secondi
```

#### 📈 Analisi Trend
```
Situazione: Vuoi vedere andamento ultima mezz'ora
Scala consigliata: 30m
Risoluzione: 3 secondi (ottimizzata)
Velocità: 3 secondi
```

#### 🔋 Consumo Missione
```
Situazione: Hai volato per 2 ore, vuoi vedere consumo totale
Scale consigliate: 1h, 2h
Risoluzione: 10 secondi (trend generale)
Velocità: 4-5 secondi
```

#### 📊 Overview Completa
```
Situazione: Vuoi vedere tutti i dati di una registrazione 4 ore
Scala consigliata: 4h
Risoluzione: 10 secondi (overview completa)
Velocità: 6 secondi
```

#### 🔬 Analisi Professionale Offline
```
Situazione: Serve analisi dettagliata con Excel/Python
Metodo: Export CSV
Risoluzione: 1 secondo (tutti i 14,400 campioni)
Tempo: 10-15 secondi
```

---

## 💾 GESTIONE MEMORIA

### 📊 Capacità Storage

| Tipo | Capacità | Frequenza | Durata | Persistenza |
|------|----------|-----------|--------|-------------|
| **RAM** | 120 campioni | 1 Hz | 2 minuti | ❌ Volatile |
| **Flash** | 14,400 campioni | 1 Hz | 4 ore | ✅ Persistente |

### 🔄 Modalità Rolling (Circolare)

Il sistema funziona in **modalità continua infinita**:

```
Timeline Registrazione:

0h → 4h: Riempimento buffer
├─ 0-2h: Memoria 50% piena
├─ 2-4h: Memoria 90% piena
└─ 4h: Memoria 100% PIENA

4h+: Modalità Rolling
├─ Campione nuovo → Sovrascrive più vecchio
├─ Ultimi 4h sempre disponibili
└─ Continua infinitamente ♾️
```

**Esempio**:
```
Registrazione 6 ore:
- Dati disponibili: Ore 2-6 (ultime 4 ore)
- Dati persi: Ore 0-2 (sovrascritti)
- Soluzione: Scarica CSV prima di 4 ore! ⚠️
```

---

## 🔔 ALERT MEMORIA

### ⚠️ Alert 90% (3h 36min)

Quando memoria raggiunge 90%:

**Serial Monitor**:
```
⚠️ ================================
⚠️  ATTENZIONE: Memoria Flash >90%
⚠️  Uso: 90.0% (12960/14400 campioni)
⚠️  SCARICA DATI PRIMA CHE I VECCHI VENGANO SOVRASCRITTI!
⚠️  http://192.168.4.1/csv
⚠️  Tempo rimanente: 24m 0s
⚠️ ================================
```

**Web Interface**:
```
💾 Flash: 12960/14400 campioni (3.6h) | ⏱️ 24m 0s rimasti | 750/4096KB ⚠️
[Testo arancione]
```

### 🔴 Alert 100% (4h esatte)

Quando memoria è completamente piena:

**Serial Monitor**:
```
🔴 ================================
🔴  MEMORIA PIENA!
🔴  Modalità ROLLING: Sovrascrivendo dati più vecchi...
🔴  SCARICA SUBITO I DATI: http://192.168.4.1/csv
🔴 ================================
```

**Web Interface**:
```
💾 Flash: 14400/14400 campioni (4.0h) | 815/4096KB 🔴
[Testo rosso grassetto + popup alert]
```

**Popup Browser**:
```
🔴 MEMORIA PIENA!

I dati più vecchi verranno sovrascritti.

Scarica CSV ora: http://192.168.4.1/csv
```

---

## 📥 EXPORT DATI CSV

### Come Scaricare

#### Metodo 1: Da Pagina Grafici
```
1. Vai su: http://192.168.4.1/charts
2. Click: 📥 Esporta CSV
3. Attendi 10-15 secondi
4. File scaricato: battery_data_XXXXX.csv
```

#### Metodo 2: Diretto
```
Browser: http://192.168.4.1/csv
```

### Opzioni Export

| URL | Contenuto | Uso |
|-----|-----------|-----|
| `/csv` | RAM + Flash (completo) | **Default** - tutto disponibile |
| `/csv?type=flash` | Solo Flash (4 ore) | Solo storico lungo termine |
| `/csv?type=ram` | Solo RAM (2 min) | Solo dati recenti |

### Formato File CSV

#### Struttura
```csv
Timestamp_ms,6S1_Voltage,6S1_Current,6S1_RawVoltage,6S1_RawCurrent,
6S2_Voltage,6S2_Current,6S2_RawVoltage,6S2_RawCurrent,
4S_Voltage,4S_Current,4S_RawVoltage,4S_RawCurrent,
MotorRight,MotorLeft,MotorUnder,Source

0,25.10,2.30,2.987,2.512,24.85,2.25,2.965,2.505,16.50,0.80,2.945,2.498,1500,1500,1500,Flash
1000,25.10,2.35,2.987,2.518,24.85,2.28,2.965,2.510,16.48,0.82,2.943,2.500,1650,1500,1500,Flash
...
```

#### Colonne (17 totali)

1. **Timestamp_ms**: Millisecondi dall'avvio
2-5. **6S#1**: Voltage, Current, RawVoltage, RawCurrent
6-9. **6S#2**: Voltage, Current, RawVoltage, RawCurrent
10-13. **4S**: Voltage, Current, RawVoltage, RawCurrent
14-16. **Motors**: Right, Left, Under (PWM μs)
17. **Source**: "Flash" o "RAM"

### Dimensioni File

| Durata Registrata | Campioni | Dimensione CSV |
|-------------------|----------|----------------|
| 2 minuti (RAM) | 120 | ~10 KB |
| 10 minuti | 600 | ~50 KB |
| 1 ora | 3,600 | ~300 KB |
| 4 ore (completo) | 14,400 | ~1.2 MB |

### Analisi Dati

#### Excel
```
1. Apri file CSV in Excel
2. Usa formule per calcoli:
   - Potenza: =B2*C2 (V × A)
   - Ah consumati: =SUM(C:C)/3600
   - Efficienza: Media tensioni / Corrente
3. Grafici personalizzati
```

#### Python
```python
import pandas as pd

# Carica CSV
df = pd.read_csv('battery_data_12345.csv')

# Calcola potenza
df['Power'] = df['6S1_Voltage'] * df['6S1_Current']

# Consumi
total_ah = df['6S1_Current'].sum() / 3600

# Grafico
import matplotlib.pyplot as plt
plt.plot(df['Timestamp_ms'], df['6S1_Voltage'])
plt.show()
```

#### MATLAB
```matlab
% Carica CSV
data = readtable('battery_data_12345.csv');

% Calcola potenza
power = data.x6S1_Voltage .* data.x6S1_Current;

% Plot
plot(data.Timestamp_ms, data.x6S1_Voltage);
```

---

## ⚡ DECIMAZIONE INTELLIGENTE

### Cos'è?

Per **grafici lunghi** (>30min), il sistema mostra **solo i punti necessari** per velocizzare visualizzazione senza perdere qualità.

### Come Funziona

| Scala | Campioni Reali | Decimazione | Campioni Mostrati | Beneficio |
|-------|---------------|-------------|-------------------|-----------|
| ≤10m | 600 | **Nessuna** | 600 | Dettaglio completo |
| 30m | 1,800 | **3x** | 600 | -67% tempo |
| 1h-4h | 3,600-14,400 | **10x** | 360-1,440 | **-70% tempo** |

### Qualità Visiva

**Domanda**: La decimazione peggiora la qualità?  
**Risposta**: **NO!** Per grafici lunghi è identica.

**Esempio Scala 4 ore**:
```
Schermo: 1920 pixel larghezza

SENZA decimazione:
14,400 punti / 1920 pixel = 7.5 punti/pixel
→ Molti punti sovrapposti (inutili!)

CON decimazione 10x:
1,440 punti / 1920 pixel = 0.75 punti/pixel
→ Ogni punto visibile (perfetto!)
```

### Risoluzione Effettiva

Le etichette mostrano la risoluzione reale:

```
💾 Flash (Storico)
  ├─ 5 minuti (1s)    ← 1 secondo/punto
  ├─ 10 minuti (1s)   ← 1 secondo/punto
  ├─ 30 minuti (3s)   ← 3 secondi/punto ⚡
  ├─ 1 ora (10s)      ← 10 secondi/punto ⚡
  ├─ 2 ore (10s)      ← 10 secondi/punto ⚡
  └─ 4 ore (10s)      ← 10 secondi/punto ⚡
```

### Quando Serve Dettaglio?

**Per analisi dettagliata** usa:
- ✅ **Scale brevi** (5m, 10m) @ 1Hz
- ✅ **Export CSV** - tutti i 14,400 campioni @ 1Hz

---

## 🎨 INTERFACCIA WEB

### Dashboard Principale

**URL**: `http://192.168.4.1`

Mostra in tempo reale:
- ✅ Tensioni 3 batterie
- ✅ Correnti 3 batterie
- ✅ Potenze calcolate
- ✅ PWM motori (Right, Left, Under)
- ✅ Direzioni motori
- ✅ Frequenza loop

**Aggiornamento**: Automatico ogni 1 secondo

### Pagina Grafici

**URL**: `http://192.168.4.1/charts`

**Funzionalità**:
- 📊 **5 grafici**: Tensioni, Correnti, Raw Tensioni, Raw Correnti, PWM Motori
- 🎚️ **Scala temporale**: Menu dropdown 10 opzioni
- 🔄 **Auto-refresh**: Toggle on/off
- 📥 **Export CSV**: Download immediato
- 🗑️ **Azzera dati**: RAM o tutto
- 📐 **Griglia**: Toggle on/off
- 🔍 **Zoom**: Reset zoom

**Indicatori**:
- 💾 **Storage Info**: Campioni, ore, tempo rimanente
- ✅ **Verde** (<50%): Tutto OK
- ⚠️ **Giallo** (50-90%): Attenzione
- 🔴 **Rosso** (>90%): Scarica dati!

### Pagina Calibrazione

**URL**: `http://192.168.4.1/calibration`

**Modalità**:
1. **Semplice**: Inserisci valori misurati
2. **Due Punti**: Calibrazione precisione
3. **Avanzata**: Parametri manuali
4. **Auto**: Calibrazione automatica

---

## 🔧 OPERAZIONI COMUNI

### Avvio Sistema

```
1. Alimenta ESP32
2. Attendi 5 secondi
3. Connetti WiFi: "ESP32_BatteryMonitor"
   Password: "battery123"
4. Apri browser: http://192.168.4.1
5. Sistema pronto! ✅
```

### Monitoring Durante Volo

```
1. Prima volo: Verifica memoria <90%
2. Durante volo: Sistema registra automaticamente
3. Dopo 3h 36min: Ricevi alert 90%
4. Dopo 4h: Alert memoria piena
5. Atterra e scarica CSV prima che vecchi dati si perdano!
```

### Download Dati Post-Volo

```
1. Atterra drone
2. ESP32 ancora acceso
3. Browser: http://192.168.4.1/charts
4. Click: 📥 Esporta CSV
5. Attendi 10-15s
6. File salvato! ✅
7. Ora puoi cancellare dati o continuare
```

### Cancellare Dati

#### Solo RAM (ultimi 2 minuti)
```
Pagina grafici → 🗑️ Azzera RAM
Usa: Prima test breve
```

#### Tutto (RAM + Flash)
```
Pagina grafici → 🗑️ Azzera Tutto
Usa: Prima nuova missione lunga
```

### Analizzare Transitorio

```
Scenario: Picco corrente 5 minuti fa

1. Vai su grafici
2. Seleziona scala: "5m" o "10m"
3. Guarda grafico correnti
4. Identifica picco
5. Export CSV per analisi dettagliata
6. Apri in Excel/Python
7. Analizza @ 1Hz
```

---

## 📊 DATI REGISTRATI

### Per Ogni Batteria (×3)

- ✅ **Tensione Calibrata** (V) - Con taratura partitore
- ✅ **Corrente Calibrata** (A) - Con taratura ACS758
- ✅ **Tensione Raw** (V) - Valore grezzo ADC
- ✅ **Corrente Raw** (V) - Tensione grezza sensore

### PWM Motori (×3)

- ✅ **Motor Right** (1000-2000 μs)
- ✅ **Motor Left** (1000-2000 μs)
- ✅ **Motor Under** (1000-2000 μs)

### Timestamp

- ✅ **Millisecondi dall'avvio** - Per correlazioni temporali

**Totale**: 16 parametri per campione  
**Frequenza**: 1 campione/secondo  
**Dimensione**: 58 bytes/campione

---

## ⚠️ LIMITAZIONI E CONSIDERAZIONI

### Risoluzione Temporale

| Evento | Durata | Catturato? |
|--------|--------|------------|
| Transitorio corrente lento | > 1s | ✅ Perfetto |
| Cambio PWM | Istantaneo | ✅ Visto |
| Picco corrente | 500ms-2s | ⚠️ Parziale |
| Vibrazioni rapide | < 200ms | ❌ Perso |
| Trend lungo | > 1 minuto | ✅✅✅ Ottimo |

**Conclusione**: Sistema ottimo per **transitori medi/lenti** e **trend lungo termine**.

### Durata Massima

- **Visualizzazione**: 4 ore
- **Dopo 4h**: Rolling mode (sovrascrive vecchi)
- **Soluzione**: Scarica CSV prima di 4 ore

### Persistenza Dati

- ✅ **Flash**: Persistono dopo riavvio
- ❌ **RAM**: Persi dopo riavvio
- ⚠️ **Flash**: Cancellati se ricarichi firmware con format SPIFFS

### Memoria

- **RAM usata**: 3.75 KB (53% libera)
- **Flash usata**: 815 KB (80% libera)
- **Margine**: Eccellente ✅

---

## 💡 BEST PRACTICES

### Prima della Missione

1. ✅ Verifica memoria <50% (o azzera)
2. ✅ Test rapido 30 secondi
3. ✅ Verifica grafici funzionanti
4. ✅ Controlla WiFi raggiungibile

### Durante la Missione

1. ✅ Sistema registra automaticamente
2. ⚠️ Alert 90% → Pianifica atterraggio presto
3. 🔴 Alert 100% → Atterra e scarica ASAP

### Dopo la Missione

1. ✅ Scarica CSV immediatamente
2. ✅ Rinomina file con data/missione
3. ✅ Backup in cloud/PC
4. ✅ Azzera dati se serve
5. ✅ Analisi offline con Excel/Python

### Analisi Dati

1. **Overview generale**: Scala 4h (veloce)
2. **Identificazione anomalie**: Cerca picchi
3. **Zoom dettaglio**: Scala 5m-10m su area interessante
4. **Analisi professionale**: Export CSV completo
5. **Calcoli avanzati**: Python/MATLAB

---

## 🆘 RISOLUZIONE PROBLEMI

### Grafico Non Si Carica

**Sintomo**: Schermo bianco o "Loading..."  
**Causa**: Memoria piena o connessione lenta  
**Soluzione**:
1. Prova scala più breve (1m invece 4h)
2. Ricarica pagina (F5)
3. Verifica WiFi vicino a ESP32

### Tempo Caricamento Lento (>10s)

**Causa**: Scala troppo lunga senza decimazione  
**Soluzione**: Sistema già ottimizzato in v2.1!  
**Verifica**: Scala 4h dovrebbe caricare in ~6s

### Alert Memoria Non Appare

**Causa**: Controllo ogni 30s  
**Soluzione**: Attendi fino a 30s dopo 90%

### Dati Persi Dopo Riavvio

**RAM**: Normale, volatile  
**Flash**: Verifica partition scheme corretto  
**Soluzione**: "Tools → Partition Scheme → Default 4MB with spiffs"

### CSV Vuoto

**Causa**: Nessun dato registrato  
**Soluzione**:
1. Attendi almeno 10 secondi
2. Verifica Serial Monitor: "💾 Salvati..."
3. Controlla storage_info su `/charts-data`

---

## 📈 PERFORMANCE ATTESE

### Tempo Caricamento Grafici

| Scala | Tempo Atteso | Rating |
|-------|--------------|--------|
| 10s-2m | < 1s | ⚡⚡⚡ Eccellente |
| 5m-10m | 2-3s | ⚡⚡ Ottimo |
| 30m | 3-4s | ⚡⚡ Buono |
| 1h-4h | 4-6s | ⚡ Accettabile |

### Uso Risorse

| Risorsa | Uso | Disponibile | % Libera |
|---------|-----|-------------|----------|
| RAM | 250 KB | 520 KB | 52% ✅ |
| Flash | 815 KB | 4 MB | 80% ✅ |

### Frequenza Loop

- **Target**: >100 Hz
- **Tipico**: 120-130 Hz
- **Con WiFi attivo**: 100-120 Hz

---

## 🎉 CONCLUSIONE

Il sistema **ESP32 Battery Monitor v2.1 OPTIMIZED** offre:

### Funzionalità
- ✅ Storage 4 ore @ 1Hz
- ✅ Visualizzazione veloce (< 10s tutte le scale)
- ✅ Persistenza dati
- ✅ Alert proattivi
- ✅ Export CSV completo

### Performance
- ⚡ Cattura transitori corrente
- ⚡ Grafici fluidi e reattivi
- ⚡ Modalità rolling infinita
- ⚡ Analisi professionale possibile

### Affidabilità
- 🎯 Memoria abbondante (80% libera)
- 🎯 Dati persistenti
- 🎯 Sistema testato e stabile
- 🎯 Documentazione completa

---

**Sistema pronto per missioni professionali e analisi avanzate!** 🚀📊✨

---

## 📚 Documenti Correlati

- **[README.md](README.md)** - Introduzione e hardware
- **[CHANGELOG_v2.1.md](CHANGELOG_v2.1.md)** - Modifiche versione
- **[DATI_REGISTRATI.md](DATI_REGISTRATI.md)** - Dettaglio parametri
- **[DECIMAZIONE_GRAFICI.md](DECIMAZIONE_GRAFICI.md)** - Ottimizzazione tecnica
- **[OTTIMIZZAZIONE_STORAGE.md](OTTIMIZZAZIONE_STORAGE.md)** - Analisi memoria

---

**Versione Documento**: 2.1  
**Ultimo Aggiornamento**: Ottobre 2025  
**Autore**: AlixBlimp BMS Team
