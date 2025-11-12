# Charger EVO11KA GUI

**Versione:** 1.0.0  
**Autore:** Alessandro Zingaretti | Polimarche Racing Team

---

## 📋 Descrizione

Interfaccia grafica Python per il monitoraggio e controllo del **Charger EVO11KA**. 

Sviluppata dal **Polimarche Racing Team** per test, diagnostica e tuning del sistema di ricarica del veicolo elettrico da competizione.

**Funzionalità:**
- Comunicazione seriale con BMS (STM32)
- Decodifica messaggi CAN del charger
- Visualizzazione parametri in tempo reale
- Invio comandi di controllo

---

## 🔌 Architettura del Sistema

```
┌──────────────────────────────────────────────────────────────────────────┐
│                              PC / Laptop                                 │
│  ┌─────────────────────────────────────────────────────────────────────┐ │
│  │                     Charger EVO11KA GUI                             │ │
│  │                        (Python App)                                 │ │
│  │                                                                     │ │
│  │  • Interfaccia grafica (PyQt6)                                      │ │
│  └──────────────────┬──────────────────────────────────────────────────┘ │
└─────────────────────┼────────────────────────────────────────────────────┘
                      │
                      │  USB/Serial (115200 baud)
                      │  
                      │  Formato messaggi seriali:
                      │  • Regex pattern per frame CAN
                      │  • Es: "CanBus Rx 0x610 12 34 56 78 9A BC DE F0"
                      |        "CanBus Tx 0x618 AA BB CC DD"
                      │
                      ▼
       ┌──────────────────────────────────────┐
       │         BMS (STM32)                  │
       │    Gateway CAN ↔ Seriale             │
       │                                      │
       │  • Riceve dati seriale da PC         │
       │  • Inoltra frame sul bus CAN         │
       │  • Riceve frame CAN dal charger      │
       │  • Formatta e invia via seriale      │
       └────────────┬─────────────────────────┘
                    │
                    │  CANbus (500 kbps)
                    │  CAN High / CAN Low
                    │  Standard frames (11-bit ID)
                    │
                    ▼
         ┌────────────────────────────┐
         │    Charger EVO11KA         │
         │                            │
         │  • Riceve comandi CAN      │
         │  • Trasmette status CAN    │
         │  • Gestisce ricarica       │
         └────────────────────────────┘
```

### Flusso di comunicazione

**Da Charger a GUI (ricezione dati):**
1. Charger trasmette frame CAN → STM32 riceve dal bus
2. STM32 analizza pacchetto e lo inoltra in seriale → invia al PC
3. PC riceve stringa come regex → analizza il contenuto del pacchetto
4. GUI aggiorna visualizzazione con i dati decodificati

*TODO* **Da GUI a Charger (invio comando):**
1. GUI costruisce comando
2. Invio via porta seriale (USB) → STM32 riceve
3. STM32 analizza comando → costruisce frame CAN
4. Frame CAN trasmesso sul bus → Charger riceve ed esegue

---

## � Requisiti

- **Python 3.8+**
- **pyserial** per comunicazione seriale
- **PyQt6** per interfaccia grafica
- Interfaccia USB/seriale verso STM32

```bash
pip install pyserial PyQt6
```
Oppure scaricare il file .exe

---

## 📂 Struttura

```
GUI_EVO/
├── Charger EVO11KA GUI.py           # Launcher
├── charger_gui/
│   ├── main.py                      # GUI principale
│   ├── serial_handler.py            # Gestione seriale
│   ├── can_decoder.py               # Parser messaggi CAN
│   ├── tabs.py                      # Tabs x interfaccia
│   └── widgets.py                   # Widget usati
└── MT4404-D - EVO - CAN Bus Manual.pdf
```

---
## 📖 Documentazione Charger

**Manuale tecnico:** `MT4404-D - EVO - CAN Bus Manual.pdf`  
Contiene specifiche complete del protocollo CANbus, ID messaggi, struttura frame e parametri.

**Produttore:** [EDN Group](http://www.edngrup.com/) | sales@edngroup.com

---

## ⚠️ Sicurezza

> **ATTENZIONE:** Prima di usare su batterie reali:
> - Leggere il manuale CAN completo
> - Testare su banco prova con carichi resistivi
> - Non superare limiti di tensione/corrente
> - Predisporre interruttore di emergenza

---

## 📝 ToDo

- [ ] **Finestra invio messaggi CAN personalizzati**
  - Tab per costruzione manuale pacchetti CAN/CTL
  - Richiesta versione SW charger
  - Richiesta serial number charger
  - Log comandi inviati con timestamp



## 👥 Contatti

**Autore:** Alessandro Zingaretti 
**Email:**  zingaale@gmail.com
**Team:**   Polimarche Racing Team - UNIVPM

---

**Versione:** 1.0.0 | Novembre 2025

