# 📄 Come Generare PDF dalla Documentazione

## ✅ FILE HTML GENERATI

Ho creato **7 file HTML** pronti per essere convertiti in PDF:

1. ✅ **RIEPILOGO_VELOCE_v2.1.html** (10 KB)
2. ✅ **GUIDA_UTENTE_v2.1.html** (32 KB)
3. ✅ **RIEPILOGO_IMPLEMENTAZIONE.html** (37 KB)
4. ✅ **CHANGELOG_v2.1.html** (27 KB)
5. ✅ **DECIMAZIONE_GRAFICI.html** (29 KB)
6. ✅ **OTTIMIZZAZIONE_STORAGE.html** (53 KB)
7. ✅ **DATI_REGISTRATI.html** (34 KB)

---

## 🖨️ METODO 1: Safari/Chrome (RACCOMANDATO) ⭐

### Procedura Semplice

#### Su macOS (Safari o Chrome)

1. **Apri il file HTML**
   ```
   - Doppio click su: GUIDA_UTENTE_v2.1.html
   - Si apre in Safari/Chrome
   ```

2. **Stampa in PDF**
   ```
   - Premi: Cmd+P (o Menu → File → Stampa)
   - Nel dialogo stampa:
     ✅ Seleziona "Salva come PDF" (in basso a sinistra)
     ✅ Click su "PDF" → "Salva come PDF"
   - Scegli nome e posizione
   - Salva!
   ```

3. **Ripeti per altri documenti**

### Screenshot Guida

```
Cmd+P → Dialogo Stampa
         ↓
    [Seleziona "PDF"]
         ↓
    ["Salva come PDF"]
         ↓
    Scegli nome e cartella
         ↓
         Salva!
```

---

## 🖨️ METODO 2: Firefox

1. Apri file HTML in Firefox
2. `Cmd+P` (Stampa)
3. Destinazione: "Salva in PDF"
4. Click "Salva"

---

## 🖨️ METODO 3: Chrome Headless (Automatico)

Se vuoi automatizzare, usa questo comando:

```bash
# Naviga nella cartella
cd /Users/robertonavoni/Desktop/Lavoro/Progetti-2025/Progetti-Software/AlixBlimpBMS/esp32-battery-monitor

# Genera PDF con Chrome
/Applications/Google\ Chrome.app/Contents/MacOS/Google\ Chrome \
  --headless \
  --disable-gpu \
  --print-to-pdf=GUIDA_UTENTE_v2.1.pdf \
  file://$(pwd)/GUIDA_UTENTE_v2.1.html

# Ripeti per altri file
```

---

## 📚 QUALI PDF GENERARE?

### 🎯 Essenziali (Genera Questi)

1. **RIEPILOGO_VELOCE_v2.1.pdf** ⭐
   - Sintesi 3 minuti
   - Perfetto per referenza rapida

2. **GUIDA_UTENTE_v2.1.pdf** ⭐⭐⭐
   - Manuale completo
   - Da tenere sempre a portata

3. **RIEPILOGO_IMPLEMENTAZIONE.pdf** ⭐⭐
   - Riferimento tecnico completo
   - Per sviluppatori

### 📖 Opzionali (Se Ti Servono)

4. **CHANGELOG_v2.1.pdf**
   - Modifiche versione

5. **DECIMAZIONE_GRAFICI.pdf**
   - Dettaglio ottimizzazioni

6. **OTTIMIZZAZIONE_STORAGE.pdf**
   - Analisi memoria

7. **DATI_REGISTRATI.pdf**
   - Parametri salvati

---

## 💡 TIP PER PDF OTTIMALI

### Impostazioni Stampa Consigliate

```
Layout: Verticale (Portrait)
Margini: Standard (20mm)
Colori: Sì (per sintassi codice)
Sfondi: Sì (per tabelle)
Header/Footer: Opzionale
Scala: 100% (default)
```

### Risultato Atteso

- ✅ **Formattazione perfetta**: Tabelle, codice, emoji
- ✅ **TOC navigabile**: Link interni funzionanti
- ✅ **Stampabile**: Ottimizzato per carta A4
- ✅ **Leggibile**: Font e spaziature corrette

---

## 🚀 SCRIPT AUTOMATICO (Opzionale)

Se hai Chrome, salva questo script:

```bash
#!/bin/bash
# genera_pdf.sh

cd /Users/robertonavoni/Desktop/Lavoro/Progetti-2025/Progetti-Software/AlixBlimpBMS/esp32-battery-monitor

CHROME="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"

# Lista documenti da convertire
docs=(
  "RIEPILOGO_VELOCE_v2.1"
  "GUIDA_UTENTE_v2.1"
  "RIEPILOGO_IMPLEMENTAZIONE"
  "CHANGELOG_v2.1"
  "DECIMAZIONE_GRAFICI"
  "OTTIMIZZAZIONE_STORAGE"
  "DATI_REGISTRATI"
)

echo "🚀 Generazione PDF in corso..."

for doc in "${docs[@]}"; do
  if [ -f "${doc}.html" ]; then
    echo "📄 Generando ${doc}.pdf..."
    "$CHROME" --headless --disable-gpu \
      --print-to-pdf="${doc}.pdf" \
      "file://$(pwd)/${doc}.html" 2>/dev/null
    echo "✅ ${doc}.pdf creato!"
  fi
done

echo "🎉 Tutti i PDF generati!"
ls -lh *.pdf
```

### Uso Script

```bash
# Rendi eseguibile
chmod +x genera_pdf.sh

# Esegui
./genera_pdf.sh
```

---

## 📋 CHECKLIST GENERAZIONE PDF

### Documenti Essenziali

- [ ] **RIEPILOGO_VELOCE_v2.1.pdf** - Sintesi rapida
- [ ] **GUIDA_UTENTE_v2.1.pdf** - Manuale completo
- [ ] **RIEPILOGO_IMPLEMENTAZIONE.pdf** - Riferimento tecnico

### Documenti Opzionali

- [ ] CHANGELOG_v2.1.pdf
- [ ] DECIMAZIONE_GRAFICI.pdf
- [ ] OTTIMIZZAZIONE_STORAGE.pdf
- [ ] DATI_REGISTRATI.pdf

---

## 🎯 RACCOMANDAZIONE

### Metodo Più Semplice

1. **Apri** `GUIDA_UTENTE_v2.1.html` in Safari
2. **Cmd+P** → Stampa
3. **PDF** → Salva come PDF
4. **Nome**: `Guida_ESP32_BatteryMonitor_v2.1.pdf`
5. **Salva** sul Desktop

**Tempo**: 30 secondi per documento! ⚡

### Per Tutti i PDF

Usa lo script automatico sopra se hai Chrome installato.

---

## 📄 QUALITÀ PDF

I PDF generati avranno:
- ✅ **Indice navigabile** (link interni)
- ✅ **Formattazione perfetta** (tabelle, codice)
- ✅ **Emoji** (se supportati dal font)
- ✅ **Sintassi evidenziata** (blocchi codice)
- ✅ **Stampabile** (margini corretti)
- ✅ **Cercabile** (testo selezionabile)

---

## 🎉 RISULTATO FINALE

Dopo conversione avrai:
- 📄 **7 PDF professionali** pronti
- 📚 **Documentazione portatile** (offline)
- 🖨️ **Stampabile** (se serve copia fisica)
- 📱 **Leggibile** su tablet/e-reader

**Totale**: ~200 pagine documentazione completa! 📚✨

---

## 💡 ALTERNATIVE

### Se Non Hai Browser Grafico

```bash
# Usa pandoc con engine alternativo (richiede installazione)
brew install --cask basictex  # Richiede password
eval "$(/usr/libexec/path_helper)"
pandoc GUIDA_UTENTE_v2.1.md -o GUIDA_UTENTE_v2.1.pdf
```

### Servizi Online

1. **CloudConvert**: https://cloudconvert.com/md-to-pdf
2. **Markdown to PDF**: https://md2pdf.netlify.app
3. Upload file .md → Download PDF

---

**File HTML pronti per conversione PDF!** 📄✨

Usa il **Metodo 1** (Safari/Chrome) per risultati perfetti! 🎯
