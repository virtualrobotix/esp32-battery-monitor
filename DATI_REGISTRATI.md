# 📊 COSA REGISTRA IL SISTEMA DI LOGGING

## 🎯 Panoramica

Il sistema registra **21 parametri** in totale, suddivisi tra **RAM** (alta frequenza) e **Flash** (bassa frequenza).

---

## 📦 DATI REGISTRATI NEL DETTAGLIO

### 🔋 **Batterie (3 pacchi)**

#### Per Ogni Batteria (×3):
1. **Tensione Convertita** (V) - Calibrata con partitore
2. **Corrente Convertita** (A) - Calibrata da sensore ACS758
3. **Tensione Raw** (V) - Valore grezzo dal partitore
4. **Corrente Raw** (V) - Tensione grezza dal sensore ACS758

**Totale per batterie**: 4 parametri × 3 batterie = **12 parametri**

#### Batterie Specifiche:
- **Batteria 6S#1** (batteria 0)
  - `v1` - Tensione calibrata
  - `c1` - Corrente calibrata
  - `rv1` - Tensione raw
  - `rc1` - Corrente raw

- **Batteria 6S#2** (batteria 1)
  - `v2` - Tensione calibrata
  - `c2` - Corrente calibrata
  - `rv2` - Tensione raw
  - `rc2` - Corrente raw

- **Batteria 4S** (batteria 2)
  - `v3` - Tensione calibrata
  - `c3` - Corrente calibrata
  - `rv3` - Tensione raw
  - `rc3` - Corrente raw

---

### 🚁 **Motori PWM (3 canali)**

1. **Motor Right** (`m1`) - PWM motore destro (1000-2000 μs)
2. **Motor Left** (`m2`) - PWM motore sinistro (1000-2000 μs)
3. **Motor Under** (`m3`) - PWM motori sottostanti (1000-2000 μs)

**Totale motori**: **3 parametri**

---

### ⏱️ **Timestamp**

1. **Timestamp** (`timestamp`) - Millisecondi dall'avvio ESP32

**Totale timestamp**: **1 parametro**

---

## 📊 RIEPILOGO TOTALE

| Categoria | Parametri | Dettaglio |
|-----------|-----------|-----------|
| **Batterie (convertite)** | 6 | Tensioni + Correnti × 3 |
| **Batterie (raw)** | 6 | Tensioni raw + Correnti raw × 3 |
| **Motori PWM** | 3 | Right, Left, Under |
| **Timestamp** | 1 | Millisecondi dall'avvio |
| **TOTALE** | **16** | **Parametri per campione** |

---

## 💾 STORAGE DUAL-LAYER

### 🏃‍♂️ RAM Buffer (Alta Frequenza)

**Frequenza**: 1 campione/secondo (1 Hz)  
**Durata**: 5 minuti (300 campioni)  
**Chiamata**: `updateCharts()` nel loop principale

#### Dati Salvati (8 grafici separati):

1. **`voltage_charts[0]`** - Tensione Batteria 6S#1
2. **`voltage_charts[1]`** - Tensione Batteria 6S#2
3. **`voltage_charts[2]`** - Tensione Batteria 4S
4. **`current_charts[0]`** - Corrente Batteria 6S#1
5. **`current_charts[1]`** - Corrente Batteria 6S#2
6. **`current_charts[2]`** - Corrente Batteria 4S
7. **`raw_voltage_charts[0-2]`** - Tensioni raw × 3
8. **`raw_current_charts[0-2]`** - Correnti raw × 3
9. **`motor_charts[0]`** - PWM Motor Right
10. **`motor_charts[1]`** - PWM Motor Left
11. **`motor_charts[2]`** - PWM Motor Under

**Totale**: 8 array da 300 float ciascuno

```cpp
// Codice effettivo (linea 630-650)
void updateCharts() {
  if (millis() - last_chart_update < 1000) return; // Ogni 1 secondo
  
  for (int i = 0; i < 3; i++) {
    addToChart(&voltage_charts[i], batteries[i].voltage);
    addToChart(&current_charts[i], batteries[i].current);
    addToChart(&raw_voltage_charts[i], batteries[i].raw_voltage_voltage);
    addToChart(&raw_current_charts[i], batteries[i].raw_current_voltage);
  }
  
  // PWM motori
  addToChart(&motor_charts[0], autopilot_input.motor_right);
  addToChart(&motor_charts[1], autopilot_input.motor_left);
  addToChart(&motor_charts[2], autopilot_input.motor_under);
}
```

---

### 💾 Flash Buffer (Bassa Frequenza)

**Frequenza**: 1 campione/10 secondi (0.1 Hz)  
**Durata**: 4 ore (1,440 campioni)  
**Chiamata**: `saveLongTermDataPoint()` nel loop principale  
**File**: `/data.bin` su SPIFFS

#### Struttura Dati (58 bytes per campione):

```cpp
struct LongTermDataPoint {
  // Timestamp (4 bytes)
  uint32_t timestamp;       // Millisecondi dall'avvio
  
  // Tensioni calibrate (12 bytes = 3 × 4 bytes)
  float v1;                 // Batteria 6S#1
  float v2;                 // Batteria 6S#2
  float v3;                 // Batteria 4S
  
  // Correnti calibrate (12 bytes = 3 × 4 bytes)
  float c1;                 // Batteria 6S#1
  float c2;                 // Batteria 6S#2
  float c3;                 // Batteria 4S
  
  // Tensioni raw (12 bytes = 3 × 4 bytes)
  float rv1;                // Raw voltage Batteria 6S#1
  float rv2;                // Raw voltage Batteria 6S#2
  float rv3;                // Raw voltage Batteria 4S
  
  // Correnti raw (12 bytes = 3 × 4 bytes)
  float rc1;                // Raw current voltage Batteria 6S#1
  float rc2;                // Raw current voltage Batteria 6S#2
  float rc3;                // Raw current voltage Batteria 4S
  
  // PWM Motori (6 bytes = 3 × 2 bytes)
  uint16_t m1;              // Motor Right
  uint16_t m2;              // Motor Left
  uint16_t m3;              // Motor Under
  
  // TOTALE: 58 bytes
} __attribute__((packed));
```

#### Codice Salvataggio (linea 300-328):

```cpp
void saveLongTermDataPoint() {
  if (millis() - last_save < 10000) return;  // Ogni 10 secondi
  
  LongTermDataPoint dataPoint;
  dataPoint.timestamp = millis();
  
  // Batterie convertite
  dataPoint.v1 = batteries[0].voltage;
  dataPoint.c1 = batteries[0].current;
  dataPoint.v2 = batteries[1].voltage;
  dataPoint.c2 = batteries[1].current;
  dataPoint.v3 = batteries[2].voltage;
  dataPoint.c3 = batteries[2].current;
  
  // Batterie raw
  dataPoint.rv1 = batteries[0].raw_voltage_voltage;
  dataPoint.rc1 = batteries[0].raw_current_voltage;
  dataPoint.rv2 = batteries[1].raw_voltage_voltage;
  dataPoint.rc2 = batteries[1].raw_current_voltage;
  dataPoint.rv3 = batteries[2].raw_voltage_voltage;
  dataPoint.rc3 = batteries[2].raw_current_voltage;
  
  // PWM motori
  dataPoint.m1 = autopilot_input.motor_right;
  dataPoint.m2 = autopilot_input.motor_left;
  dataPoint.m3 = autopilot_input.motor_under;
  
  // Scrivi su SPIFFS...
}
```

---

## 📥 DATI EXPORT CSV

### Formato File CSV

```csv
Timestamp_ms,6S1_Voltage,6S1_Current,6S1_RawVoltage,6S1_RawCurrent,
6S2_Voltage,6S2_Current,6S2_RawVoltage,6S2_RawCurrent,
4S_Voltage,4S_Current,4S_RawVoltage,4S_RawCurrent,
MotorRight,MotorLeft,MotorUnder,Source
```

### Esempio Riga CSV

```csv
12345000,25.10,2.30,2.987,2.512,24.85,2.25,2.965,2.505,16.50,0.80,2.945,2.498,1500,1500,1500,Flash
```

### Colonne (16 totali):

| # | Nome | Tipo | Unità | Descrizione |
|---|------|------|-------|-------------|
| 1 | Timestamp_ms | uint32 | ms | Millisecondi dall'avvio |
| 2 | 6S1_Voltage | float | V | Tensione calibrata 6S#1 |
| 3 | 6S1_Current | float | A | Corrente calibrata 6S#1 |
| 4 | 6S1_RawVoltage | float | V | Tensione raw 6S#1 |
| 5 | 6S1_RawCurrent | float | V | Tensione sensore ACS758 6S#1 |
| 6 | 6S2_Voltage | float | V | Tensione calibrata 6S#2 |
| 7 | 6S2_Current | float | A | Corrente calibrata 6S#2 |
| 8 | 6S2_RawVoltage | float | V | Tensione raw 6S#2 |
| 9 | 6S2_RawCurrent | float | V | Tensione sensore ACS758 6S#2 |
| 10 | 4S_Voltage | float | V | Tensione calibrata 4S |
| 11 | 4S_Current | float | A | Corrente calibrata 4S |
| 12 | 4S_RawVoltage | float | V | Tensione raw 4S |
| 13 | 4S_RawCurrent | float | V | Tensione sensore ACS758 4S |
| 14 | MotorRight | uint16 | μs | PWM motore destro |
| 15 | MotorLeft | uint16 | μs | PWM motore sinistro |
| 16 | MotorUnder | uint16 | μs | PWM motori sottostanti |
| 17 | Source | string | - | "RAM" o "Flash" |

---

## 🔍 DETTAGLIO PARAMETRI

### Tensioni Calibrate (V)

**Range**: 0-30V (tipico 18-26V per 6S, 12-17V per 4S)  
**Risoluzione**: 0.01V  
**Formula**: `(ADC_raw × 3.3 / 4095 × divider_ratio + offset) × scale`

**Esempio**:
- 6S completamente carica: ~25.2V
- 6S nominale: ~22.2V
- 6S scarica: ~18.0V

---

### Correnti Calibrate (A)

**Range**: -50A a +50A (sensore ACS758-50A)  
**Risoluzione**: 0.01A  
**Formula**: `((ADC_voltage - 2.5V) / 0.04 + offset) × scale`

**Esempio**:
- Corrente motori in volo: 2-10A
- Picchi accelerazione: 15-30A
- Riposo/hover: 1-3A

---

### Tensioni Raw (V)

**Range**: 0-3.3V (input ADC)  
**Risoluzione**: 0.001V  
**Uso**: Verifica partitori, calibrazione, debug

**Esempio**:
- Batteria 25.2V → Partitore 8.4:1 → ~3.0V ADC

---

### Correnti Raw (V)

**Range**: 0-3.3V (output ACS758)  
**Risoluzione**: 0.001V  
**Uso**: Verifica sensori, calibrazione, debug

**Esempio**:
- 0A → 2.5V (zero centrale)
- +10A → 2.9V (2.5 + 10×0.04)
- -10A → 2.1V (2.5 - 10×0.04)

---

### PWM Motori (μs)

**Range**: 1000-2000 μs  
**Centro**: 1500 μs (neutro)  
**Risoluzione**: 1 μs

**Mapping**:
- 1000 μs → Massimo indietro
- 1500 μs → Neutro/stop
- 2000 μs → Massimo avanti

---

### Timestamp (ms)

**Range**: 0 a 4,294,967,295 ms (~49 giorni)  
**Risoluzione**: 1 ms  
**Reset**: Ad ogni riavvio ESP32

**Conversione**:
- Secondi: `timestamp / 1000`
- Minuti: `timestamp / 60000`
- Ore: `timestamp / 3600000`

---

## 📊 STATISTICHE STORAGE

### RAM Buffer

| Parametro | Valore |
|-----------|--------|
| **Frequenza** | 1 Hz (1 campione/sec) |
| **Campioni** | 300 |
| **Durata** | 5 minuti |
| **Bytes/campione** | 4 bytes (float) |
| **Grafici** | 8 array separati |
| **Memoria totale** | 300 × 8 × 4 = 9,600 bytes (~9.6 KB) |

### Flash Buffer

| Parametro | Valore |
|-----------|--------|
| **Frequenza** | 0.1 Hz (1 campione/10 sec) |
| **Campioni** | 1,440 |
| **Durata** | 4 ore |
| **Bytes/campione** | 58 bytes |
| **Memoria totale** | 1,440 × 58 = 83,520 bytes (~82 KB) |
| **File SPIFFS** | `/data.bin` |

---

## 🎯 COSA NON VIENE REGISTRATO

### Dati Calcolati (ma non salvati):

1. **Potenza (W)** - Calcolata come `V × A` ma non salvata
2. **Capacità (Ah)** - Non implementata
3. **Direzione Motori** - Pin GPIO non letti (sono OUTPUT)
4. **Temperatura** - Sensori non presenti
5. **GPS** - Non presente
6. **IMU** - Non presente

### Perché Non Salvare Potenza?

La potenza può essere calcolata **dopo** l'export:
```python
# Python/Excel
power = voltage × current
```

Risparmia spazio e può essere ricalcolata quando serve!

---

## 🔬 ANALISI POSSIBILI CON I DATI

### Con Tensioni + Correnti:

1. ✅ **Consumo energetico** (integrale corrente)
2. ✅ **Potenza istantanea** (V × A)
3. ✅ **Efficienza batterie** (confronto 3 pacchi)
4. ✅ **Rate of discharge** (derivata tensione)
5. ✅ **State of Charge** (SOC stimato)
6. ✅ **Capacità residua** (Ah rimanenti)

### Con PWM Motori:

7. ✅ **Duty cycle medio/picco**
8. ✅ **Bilanciamento motori** (destra vs sinistra)
9. ✅ **Vibrazioni/oscillazioni** (FFT PWM)
10. ✅ **Correlazione consumo-throttle**

### Con Dati Raw:

11. ✅ **Validazione calibrazione**
12. ✅ **Drift sensori nel tempo**
13. ✅ **Diagnostica hardware**
14. ✅ **Re-calibrazione post-missione**

---

## 📈 ESEMPIO OUTPUT SERIAL

```
💾 Salvati 10/1440 campioni long-term (0.7% buffer)

Campione esempio:
  timestamp: 120000 ms (2 minuti dall'avvio)
  
  Batteria 6S#1:
    v1:  25.10 V
    c1:   2.30 A
    rv1:  2.987 V (ADC)
    rc1:  2.512 V (ACS758)
    
  Batteria 6S#2:
    v2:  24.85 V
    c2:   2.25 A
    rv2:  2.965 V (ADC)
    rc2:  2.505 V (ACS758)
    
  Batteria 4S:
    v3:  16.50 V
    c3:   0.80 A
    rv3:  2.945 V (ADC)
    rc3:  2.498 V (ACS758)
    
  PWM Motori:
    m1:  1650 μs (Right - Avanti)
    m2:  1500 μs (Left - Neutro)
    m3:  1500 μs (Under - Neutro)
```

---

## 🎓 CONCLUSIONE

### Riepilogo Registrazione

| Tipo Dato | RAM (1Hz) | Flash (0.1Hz) | CSV Export |
|-----------|-----------|---------------|------------|
| Tensioni calibrate (×3) | ✅ | ✅ | ✅ |
| Correnti calibrate (×3) | ✅ | ✅ | ✅ |
| Tensioni raw (×3) | ✅ | ✅ | ✅ |
| Correnti raw (×3) | ✅ | ✅ | ✅ |
| PWM motori (×3) | ✅ | ✅ | ✅ |
| Timestamp | ✅ | ✅ | ✅ |
| **TOTALE PARAMETRI** | **16** | **16** | **17** (+ Source) |

### Punti di Forza

1. ✅ **Completo**: Tutti i dati essenziali registrati
2. ✅ **Ridondante**: Dati raw + calibrati per validazione
3. ✅ **Compatto**: Solo 58 bytes/campione
4. ✅ **Persistente**: Flash SPIFFS (4 ore)
5. ✅ **Analizzabile**: CSV compatibile Excel/Python/MATLAB

---

**Il sistema registra TUTTO quello che serve per analisi professionali! 📊✨**
