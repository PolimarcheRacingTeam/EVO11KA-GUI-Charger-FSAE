/* =============================================================================
 *  FILE: utils_canBus_charger.c
 * =============================================================================
 *  
 *  EVO Charger CAN Bus Utilities - Level 1 
 *  Control and Real time diagnostic messages
 * 
 * =============================================================================
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>


/* CAN IDs */
#define CAN_ID_CTL   0x618  /* BMS → Charger - Control */
#define CAN_ID_STAT  0x610  /* Charger → BMS - Status */
#define CAN_ID_ACT1  0x611  /* Charger → BMS - Actual Values 1 */
#define CAN_ID_ACT2  0x614  /* Charger → BMS - Actual Values 2 */
#define CAN_ID_TST1  0x615  /* Charger → BMS - Test/Diagnostic */

/* CTL Packet (BMS → Charger) 
To Send every 100ms */
typedef struct {
    bool can_enable;          /* Ctl.CanEnable - Enable/Disable charger */
    bool led3_enable;         /* Ctl.LED3_A - Enable LED3 */
    float iac_max_A;          /* Ctl.IacMaxSet - Max AC input current [A] (0-500A) */
    float vout_max_V;         /* Ctl.VoutMaxSet - Max output voltage [V] (0-10000V) */
    float iout_max_A;         /* Ctl.IoutMaxSet - Max output current [A] (0-1500A) */
} CanPacket_Ctl_t;

/* STAT Packet (Charger → BMS) 
To receive every 1000ms */
typedef struct {
    bool power_enable;    /* Bit 7 - Hardware enable pin active */
    bool error_latch;     /* Bit 6 - Failure occurred */
    bool warn_limit;      /* Bit 5 - Warning condition */
    bool lim_temp;        /* Bit 3 - De-rating active */
    bool warning_hv;      /* Bit 1 - HV side not implemented */
    bool bulks;           /* Bit 0 - Bulk error */
} CanPacket_Stat_t;

/* ACT1 Packet (Charger → BMS) 
To receive every 100ms */
typedef struct {
    float iac_A;          /* AC Input Current [A] */
    float temp_C;         /* Temperature over Power Stage [°C] */
    float vout_V;         /* DC Output Voltage [V] */
    float iout_A;         /* DC Output Current [A] */
} CanPacket_Act1_t;

/* ACT2 Packet (Charger → BMS) 
To receive every 1000ms */
typedef struct {
    float temp_loglv_C;   /* Temperature Logic LV Stage [°C] */
    float ac_power_kW;    /* AC Input Power [kW] */
    float prox_limit_A;   /* Max AC current (Proximity) [A] */
    float pilot_limit_A;  /* Max AC current (Pilot) [A] */
} CanPacket_Act2_t;

/* TST1 Packet (Charger → BMS) - Test/Diagnostic flags 
To receive every 100ms*/
typedef struct {
    /* Byte 0 (D0) */
    bool ack;              /* Bit 7 - AC Mains connected */
    bool pr_compl;         /* Bit 6 - AC Mains precharge completed */
    bool pwr_ok;           /* Bit 5 - Charger providing output power */
    bool vout_ok;          /* Bit 4 - Output voltage present */
    bool neutral;          /* Bit 3 - Neutral connection OK */
    bool led3;             /* Bit 2 - LED3 active */
    bool led618;           /* Bit 1 - LED618 echo */
    /* Byte 1 (D1) */
    bool ovp;              /* Bit 15 - DC output over voltage */
    bool conn_open;        /* Bit 14 - Output connector not connected */
    bool ther_fail;        /* Bit 10 - De-rating condition */
    bool rx618_fail;       /* Bit 8 - Control message timeout (>600ms) */
    /* Byte 2 (D2) */
    bool bulk1_fail;       /* Bit 23 - Bulk channel 1 error */
    bool bulk2_fail;       /* Bit 22 - Bulk channel 2 error */
    bool bulk3_fail;       /* Bit 21 - Bulk channel 3 error */
    bool pump_on;          /* Bit 20 - Pump active (temp > 35°C) */
    bool fan_on;           /* Bit 19 - Fan active (temp > 40°C) */
    bool hv_rx_fail;       /* Bit 18 - HV communication error */
    bool cooling_fail;     /* Bit 17 - Cooling circuit fault */
    bool rx619_fail;       /* Bit 16 - RX ID619 fail flag */
    /* Byte 3 (D3) */
    bool neutro1;          /* Bit 31 - Neutral connection good */
    bool neutro2;          /* Bit 30 - Neutral detection system */
    bool three_phase;      /* Bit 29 - Three phase config (true=3ph, false=1ph) */
    bool iac_fail;         /* Bit 26 - AC current over max */
    bool ignition;         /* Bit 25 - Ignition wake signal */
    bool lv_battery_np;    /* Bit 24 - LV battery not present */
    /* Byte 4 (D4) */
    bool prox_ok;          /* Bit 39 - Proximity level correct */
    bool pilot_ok;         /* Bit 37 - Pilot signal correct */
    bool s2_ok;            /* Bit 35 - S2 switch closed */
    /* Byte 6 (D6) - Hours counter */
    uint16_t cnt_hours;    /* Bit 55-40 - Hours counter (0-65535) */
} CanPacket_Tst1_t;

/**
 * @brief Converte tensione output float (V) in valore uint16 per CAN
 * Formula: Ctl.VoutMaxSet = Vout x 10
 * Esempio: 360V → 360 x 10 = 3600 = 0x0E10
 * Range: 0-10000V → 0-100000 (0x0000-0x186A0)
 */
static uint16_t Voltage_ToRawCan(float voltage_V) {
    if (voltage_V < 0.0f) voltage_V = 0.0f;
    if (voltage_V > 10000.0f) voltage_V = 10000.0f;
    return (uint16_t)(voltage_V *10);
}

/**
 * @brief Converte corrente output float (A) in valore uint16 per CAN
 * Formula: Ctl.IoutMaxSet = Iout x 10
 * Esempio: 17A → 17 x 10 = 170 = 0x00AA
 * Range: 0-1500A → 0-15000 (0x0000-0x3A98)
 */
static uint16_t CurrentOut_ToRawCan(float current_A) {
    if (current_A < 0.0f) current_A = 0.0f;
    if (current_A > 1500.0f) current_A = 1500.0f;
    return (uint16_t)(current_A *10);
}

/**
 * @brief Converte corrente AC input float (A) in valore uint16 per CAN
 * Formula: Ctl.IacMaxSet = Iac x 10
 * Esempio: 16A → 16 x 10 = 160 = 0x00A0
 * Range: 0-500A → 0-5000 (0x0000-0x1388)
 */
static uint16_t CurrentAC_ToRawCAN(float current_A) {
    if (current_A < 0.0f) current_A = 0.0f;
    if (current_A > 500.0f) current_A = 500.0f;
    return (uint16_t)(current_A * 10);
}

/**
 * @brief Crea un pacchetto CTL (Control) da inviare al caricatore
 * 
 * Formato pacchetto CTL (8 bytes) - ID 0x618:
 * ┌─────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 * │ D0  │  D1  │  D2  │  D3  │  D4  │  D5  │  D6  │  D7  │
 * ├─────┼──────┼──────┼──────┼──────┼──────┼──────┼──────┤
 * │Flags│ IacMaxSet   │ VoutMaxSet  │ IoutMaxSet  │ Empty│
 * │     │ MSB  │ LSB  │ MSB  │ LSB  │ MSB  │ LSB  │      │
 * └─────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 * 
 * D0 (Byte 0): Flags
 *   - Bit 7: Ctl.CanEnable (1=Enable, 0=Disable)
 *   - Bit 3: Ctl.LED3_A (1=Enable LED, 0=Disable)
 * 
 * D1-D2 (Byte 1-2): Ctl.IacMaxSet(Little Endian)
 *   Max AC input current = Iac × 10
 *   Range: 0-500A
 * 
 * D3-D4 (Byte 3-4): Ctl.VoutMaxSet (Little Endian)
 *   Max output voltage = Vout × 10
 *   Range: 0-10000V
 * 
 * D5-D6 (Byte 5-6): Ctl.IoutMaxSet (Little Endian)
 *   Max output current = Iout × 10
 *   Range: 0-1500A
 * 
 * D7 (Byte 7): Empty (0x00)
 * 
 * @param ctl Struttura con i parametri di controllo
 * @param data Array di 8 byte da riempire con i dati CAN (output)
 * @return true se successo, false se errore
 */
bool CanBus_CreatePacket_Ctl(const CanPacket_Ctl_t *ctl, uint8_t data[8]) {
    if (ctl == NULL || data == NULL) {
        return false;
    }
        memset(data, 0, 8);
    
    /* D0 (Byte 0): Flags */
    data[0] = 0x00;
    if (ctl->can_enable) {
        data[0] |= 0x80;  /* Bit 7: CanEnable */
    }
    if (ctl->led3_enable) {
        data[0] |= 0x08;  /* Bit 3: LED3_A */
    }
    
    /* D1-D2 (Byte 1-2): IacMaxSet - Corrente AC input */
    uint16_t iac_raw = CurrentAC_ToRawCAN(ctl->iac_max_A);
    data[1] = (uint8_t)((iac_raw >> 8) & 0xFF);  /* MSB */
    data[2] = (uint8_t)(iac_raw & 0xFF);         /* LSB */
    
    /* D3-D4 (Byte 3-4): VoutMaxSet - Tensione output */
    uint16_t vout_raw = Voltage_ToRawCan(ctl->vout_max_V);
    data[3] = (uint8_t)((vout_raw >> 8) & 0xFF);  /* MSB */
    data[4] = (uint8_t)(vout_raw & 0xFF);         /* LSB */
    
    /* D5-D6 (Byte 5-6): IoutMaxSet - Corrente output */
    uint16_t iout_raw = CurrentOut_ToRawCan(ctl->iout_max_A);
    data[5] = (uint8_t)((iout_raw >> 8) & 0xFF);  /* MSB */
    data[6] = (uint8_t)(iout_raw & 0xFF);         /* LSB */
    
    /* D7 (Byte 7): Empty */
    data[7] = 0x00;
    
    return true;
}


/**
 * @brief Crea un pacchetto CTL con parametri float (comodo)
 * 
 * @param can_enable Enable/Disable charger (true/false)
 * @param led3_enable Enable LED3 (true/false)
 * @param iac_max_A Corrente AC massima in Ampere (0-500A)
 * @param vout_max_V Tensione output massima in Volt (0-10000V)
 * @param iout_max_A Corrente output massima in Ampere (0-1500A)
 * @param data Array di 8 byte da riempire (output)
 * @return true se successo, false se errore
 */
bool CanBus_CreatePacket_Ctl_Simple(bool can_enable,
                                     bool led3_enable,
                                     float iac_max_A,
                                     float vout_max_V,
                                     float iout_max_A,
                                     uint8_t data[8]) {
    CanPacket_Ctl_t ctl;
    ctl.can_enable = can_enable;
    ctl.led3_enable = led3_enable;
    ctl.iac_max_A = iac_max_A;
    ctl.vout_max_V = vout_max_V;
    ctl.iout_max_A = iout_max_A;
    return CanBus_CreatePacket_Ctl(&ctl, data);
}


/* ============================================================================
 * FUNZIONI DI DECODIFICA PACCHETTI RICEVUTI DAL CHARGER
 * ============================================================================ */

/**
 * @brief Decodifica pacchetto STAT (Status) - ID 0x610
 * 
 * Formato: 4 byte (D0-D3)
 * D0: Flags status (bit field)
 * D1-D3: Empty
 */
bool CanBus_DecodePacket_Stat(const uint8_t data[8], CanPacket_Stat_t *stat) {
    if (stat == NULL || data == NULL) return false;
    
    stat->power_enable = (data[0] & 0x80) != 0;  /* Bit 7 */
    stat->error_latch  = (data[0] & 0x40) != 0;  /* Bit 6 */
    stat->warn_limit   = (data[0] & 0x20) != 0;  /* Bit 5 */
    stat->lim_temp     = (data[0] & 0x08) != 0;  /* Bit 3 */
    stat->warning_hv   = (data[0] & 0x02) != 0;  /* Bit 1 */
    stat->bulks        = (data[0] & 0x01) != 0;  /* Bit 0 */
    
    return true;
}

/**
 * @brief Decodifica pacchetto ACT1 (Actual Values 1) - ID 0x611
 * 
 * Formato: 8 byte
 * D0-D1: AC Input Current (Iac × 10)
 * D2-D3: Temperature (T × 0.005188 - 40)
 * D4-D5: DC Output Voltage (Vout × 10)
 * D6-D7: DC Output Current (Iout × 10)
 */
bool CanBus_DecodePacket_Act1(const uint8_t data[8], CanPacket_Act1_t *act1) {
    if (act1 == NULL || data == NULL) return false;
    
    uint16_t iac_raw = (data[0] << 8) | data[1];
    uint16_t temp_raw = (data[2] << 8) | data[3];
    uint16_t vout_raw = (data[4] << 8) | data[5];
    uint16_t iout_raw = (data[6] << 8) | data[7];
    
    act1->iac_A = iac_raw / 10.0f;
    act1->temp_C = (temp_raw * 0.005188f) - 40.0f;
    act1->vout_V = vout_raw / 10.0f;
    act1->iout_A = iout_raw / 10.0f;
    
    return true;
}

/**
 * @brief Decodifica pacchetto ACT2 (Actual Values 2) - ID 0x614
 * 
 * Formato: 8 byte
 * D0-D1: Temp Logic LV (T × 0.005188 - 40)
 * D2-D3: AC Power (P × 0.01 kW)
 * D4-D5: Proximity Limit (I × 10)
 * D6-D7: Pilot Limit (I × 10)
 */
bool CanBus_DecodePacket_Act2(const uint8_t data[8], CanPacket_Act2_t *act2) {
    if (act2 == NULL || data == NULL) return false;
    
    uint16_t temp_raw = (data[0] << 8) | data[1];
    uint16_t power_raw = (data[2] << 8) | data[3];
    uint16_t prox_raw = (data[4] << 8) | data[5];
    uint16_t pilot_raw = (data[6] << 8) | data[7];
    
    act2->temp_loglv_C = (temp_raw * 0.005188f) - 40.0f;
    act2->ac_power_kW = power_raw * 0.01f;
    act2->prox_limit_A = prox_raw / 10.0f;
    act2->pilot_limit_A = pilot_raw / 10.0f;
    
    return true;
}

/**
 * @brief Decodifica pacchetto TST1 (Test/Diagnostic) - ID 0x615
 * 
 * Formato: 8 byte con 27 flag diagnostici + contatore ore (16-bit)
 * D0: Bit 7-0 (ACok, PrCompl, PwrOk, VoutOk, Neutral, LED3, LED618, -)
 * D1: Bit 15-8 (ovp, connOpen, -, -, TherFail, -, rx618Fail, -)
 * D2: Bit 23-16 (bulk1/2/3_fail, PUMPon, FANon, HVrxFail, CoolingFail, Rx619fail)
 * D3: Bit 31-24 (Neutro1, Neutro2, ThreePhase, -, IacFail, Ignition, LVBatteryNP, -)
 * D4: Bit 39-32 (ProxOk, -, PilotOk, -, S2Ok, -, -, -)
 * D5: Empty
 * D6-D7: Hours counter (16-bit, LSB first)
 */
bool CanBus_DecodePacket_Tst1(const uint8_t data[8], CanPacket_Tst1_t *tst) {
    if (tst == NULL || data == NULL) return false;
    
    /* Byte 0 (D0) */
    tst->ack      = (data[0] & 0x80) != 0;  /* Bit 7 */
    tst->pr_compl = (data[0] & 0x40) != 0;  /* Bit 6 */
    tst->pwr_ok   = (data[0] & 0x20) != 0;  /* Bit 5 */
    tst->vout_ok  = (data[0] & 0x10) != 0;  /* Bit 4 */
    tst->neutral  = (data[0] & 0x08) != 0;  /* Bit 3 */
    tst->led3     = (data[0] & 0x04) != 0;  /* Bit 2 */
    tst->led618   = (data[0] & 0x02) != 0;  /* Bit 1 */
    
    /* Byte 1 (D1) */
    tst->ovp        = (data[1] & 0x80) != 0;  /* Bit 15 */
    tst->conn_open  = (data[1] & 0x40) != 0;  /* Bit 14 */
    tst->ther_fail  = (data[1] & 0x04) != 0;  /* Bit 10 */
    tst->rx618_fail = (data[1] & 0x01) != 0;  /* Bit 8 */
    
    /* Byte 2 (D2) */
    tst->bulk1_fail   = (data[2] & 0x80) != 0;  /* Bit 23 */
    tst->bulk2_fail   = (data[2] & 0x40) != 0;  /* Bit 22 */
    tst->bulk3_fail   = (data[2] & 0x20) != 0;  /* Bit 21 */
    tst->pump_on      = (data[2] & 0x10) != 0;  /* Bit 20 */
    tst->fan_on       = (data[2] & 0x08) != 0;  /* Bit 19 */
    tst->hv_rx_fail   = (data[2] & 0x04) != 0;  /* Bit 18 */
    tst->cooling_fail = (data[2] & 0x02) != 0;  /* Bit 17 */
    tst->rx619_fail   = (data[2] & 0x01) != 0;  /* Bit 16 */
    
    /* Byte 3 (D3) */
    tst->neutro1       = (data[3] & 0x80) != 0;  /* Bit 31 */
    tst->neutro2       = (data[3] & 0x40) != 0;  /* Bit 30 */
    tst->three_phase   = (data[3] & 0x20) != 0;  /* Bit 29 */
    tst->iac_fail      = (data[3] & 0x04) != 0;  /* Bit 26 */
    tst->ignition      = (data[3] & 0x02) != 0;  /* Bit 25 */
    tst->lv_battery_np = (data[3] & 0x01) != 0;  /* Bit 24 */
    
    /* Byte 4 (D4) */
    tst->prox_ok  = (data[4] & 0x80) != 0;  /* Bit 39 */
    tst->pilot_ok = (data[4] & 0x20) != 0;  /* Bit 37 */
    tst->s2_ok    = (data[4] & 0x08) != 0;  /* Bit 35 */
    
    /* Byte 6-7 (D6-D7): Hours counter (16-bit) */
    tst->cnt_hours = (data[6] << 8) | data[7];
    
    return true;
}


































/**
 * @brief Stampa un pacchetto CTL decodificato (per debug)
 * 
 * @param data Array di 8 byte del pacchetto CTL
 * 
 */
void CanBus_Debug_PrintCtl(const uint8_t data[8]) {
    printf("\n\rCTL Packet Decoded:\n");
    
    /* Stampa i byte in HEX */
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    
    /* D0: Flags */
    bool can_enable = (data[0] & 0x80) != 0;
    bool led3 = (data[0] & 0x08) != 0;
    printf("  CanEnable: %s\n", can_enable ? "true" : "false");
    printf("  LED3: %s\n", led3 ? "true" : "false");
    
    /* D1-D2: IacMax */
    uint16_t iac_raw = (data[1] << 8) | data[2];
    float iac = iac_raw / 10.0f;
    printf("  IacMax: %.1f A (raw: 0x%04X = %u)\n", iac, iac_raw, iac_raw);
    
    /* D3-D4: VoutMax */
    uint16_t vout_raw = (data[3] << 8) | data[4];
    float vout = vout_raw / 10.0f;
    printf("  VoutMax: %.1f V (raw: 0x%04X = %u)\n", vout, vout_raw, vout_raw);
    
    /* D5-D6: IoutMax */
    uint16_t iout_raw = (data[5] << 8) | data[6];
    float iout = iout_raw / 10.0f;
    printf("  IoutMax: %.1f A (raw: 0x%04X = %u)\n", iout, iout_raw, iout_raw);
}

/** Stampa pacchetto STAT decodificato */
void CanBus_Debug_PrintStat(const uint8_t data[8]) {
    CanPacket_Stat_t stat;
    CanBus_DecodePacket_Stat(data, &stat);
    
    printf("\n\rSTAT Packet Decoded:\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 4; i++) {
        printf("%02X", data[i]);
        if (i < 3) printf(", ");
    }
    printf("]\n");
    
    printf("  PowerEnable: %s\n", stat.power_enable ? "true" : "false");
    printf("  ErrorLatch: %s\n", stat.error_latch ? "true" : "false");
    printf("  WarnLimit: %s\n", stat.warn_limit ? "true" : "false");
    printf("  LimTemp: %s\n", stat.lim_temp ? "true" : "false");
    printf("  WarningHV: %s\n", stat.warning_hv ? "true" : "false");
    printf("  Bulks: %s\n", stat.bulks ? "true" : "false");
}

/** Stampa pacchetto ACT1 decodificato */
void CanBus_Debug_PrintAct1(const uint8_t data[8]) {
    CanPacket_Act1_t act1;
    CanBus_DecodePacket_Act1(data, &act1);
    
    printf("\n\rACT1 Packet Decoded:\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    
    printf("  AC Input Current: %.1f A\n", act1.iac_A);
    printf("  Temperature: %.1f .C\n", act1.temp_C);
    printf("  DC Output Voltage: %.1f V\n", act1.vout_V);
    printf("  DC Output Current: %.1f A\n", act1.iout_A);
    printf("  DC Output Power: %.1f W\n", act1.vout_V * act1.iout_A);
}

/** Stampa pacchetto ACT2 decodificato */
void CanBus_Debug_PrintAct2(const uint8_t data[8]) {
    CanPacket_Act2_t act2;
    CanBus_DecodePacket_Act2(data, &act2);
    
    printf("\n\rACT2 Packet Decoded:\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    
    printf("  Temp Logic LV: %.1f .C\n", act2.temp_loglv_C);
    printf("  AC Power: %.2f kW\n", act2.ac_power_kW);
    printf("  Proximity Limit: %.1f A\n", act2.prox_limit_A);
    printf("  Pilot Limit: %.1f A\n", act2.pilot_limit_A);
}

/** Stampa pacchetto TST1 decodificato */
void CanBus_Debug_PrintTst1(const uint8_t data[8]) {
    CanPacket_Tst1_t tst;
    CanBus_DecodePacket_Tst1(data, &tst);
    
    printf("\n\rTST1 Packet Decoded:\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    
    /* Status flags */
    printf("  === Status Flags ===\n");
    printf("  ACok: %s\n", tst.ack ? "true" : "false");
    printf("  PrechargeCompleted: %s\n", tst.pr_compl ? "true" : "false");
    printf("  PowerOk: %s\n", tst.pwr_ok ? "true" : "false");
    printf("  VoutOk: %s\n", tst.vout_ok ? "true" : "false");
    printf("  Neutral: %s\n", tst.neutral ? "true" : "false");
    printf("  LED3: %s\n", tst.led3 ? "true" : "false");
    printf("  LED618: %s\n", tst.led618 ? "true" : "false");
    
    /* Error flags */
    printf("  === Error Flags ===\n");
    printf("  OverVoltage: %s\n", tst.ovp ? "true" : "false");
    printf("  ConnectorOpen: %s\n", tst.conn_open ? "true" : "false");
    printf("  ThermalFail: %s\n", tst.ther_fail ? "true" : "false");
    printf("  RX618Timeout: %s\n", tst.rx618_fail ? "true" : "false");
    printf("  Bulk1Fail: %s\n", tst.bulk1_fail ? "true" : "false");
    printf("  Bulk2Fail: %s\n", tst.bulk2_fail ? "true" : "false");
    printf("  Bulk3Fail: %s\n", tst.bulk3_fail ? "true" : "false");
    printf("  HV_RX_Fail: %s\n", tst.hv_rx_fail ? "true" : "false");
    printf("  CoolingFail: %s\n", tst.cooling_fail ? "true" : "false");
    printf("  RX619Fail: %s\n", tst.rx619_fail ? "true" : "false");
    
    /* Cooling system */
    printf("  === Cooling ===\n");
    printf("  PumpOn: %s\n", tst.pump_on ? "true" : "false");
    printf("  FanOn: %s\n", tst.fan_on ? "true" : "false");
    
    /* AC configuration */
    printf("  === AC Config ===\n");
    printf("  ThreePhase: %s\n", tst.three_phase ? "true (3-phase)" : "false (1-phase)");
    printf("  Neutro1: %s\n", tst.neutro1 ? "true" : "false");
    printf("  Neutro2: %s\n", tst.neutro2 ? "true" : "false");
    printf("  IacFail: %s\n", tst.iac_fail ? "true" : "false");
    printf("  ProxOk: %s\n", tst.prox_ok ? "true" : "false");
    printf("  PilotOk: %s\n", tst.pilot_ok ? "true" : "false");
    printf("  S2Ok: %s\n", tst.s2_ok ? "true" : "false");
    
    /* Other */
    printf("  === Other ===\n");
    printf("  Ignition: %s\n", tst.ignition ? "true" : "false");
    printf("  LV_BatteryNP: %s\n", tst.lv_battery_np ? "true" : "false");
    printf("  HoursCounter: %u hours\n", tst.cnt_hours);
}










































/**
 * @brief Genera un pacchetto CAN con 8 byte di bit casuali (0-1)
 * 
 * Utile per testing e debug. Ogni byte è riempito con valori casuali
 * generati da rand().
 * 
 * @param data Array di 8 byte da riempire con valori casuali (output)
 * 
 * Esempio:
 *   uint8_t test_data[8];
 *   CanBus_GenerateRandomPacket(test_data);
 *   // test_data contiene ora 8 byte casuali
 */
void CanBus_GenerateRandomPacket(uint8_t data[8]) {
    static bool seeded = false;
    
    /* Inizializza il generatore random solo una volta */
    if (!seeded) {
        srand((unsigned int)time(NULL));
        seeded = true;
    }
    
    /* Genera 8 byte casuali */
    for (int i = 0; i < 8; i++) {
        data[i] = (uint8_t)(rand() & 0xFF);
    }
}

/** ESEMPIO 1: Creazione pacchetto base */
void Example_BasicCtlPacket(void) {
    uint8_t can_data[8];
    /* Crea pacchetto: Enable charger, 16A AC, 360V, 17A */
    CanBus_CreatePacket_Ctl_Simple(
        true,      /* can_enable */
        false,     /* led3_disabled */
        16.0f,     /* iac_max_A */
        360.0f,    /* vout_max_V */
        17.0f,     /* iout_max_A */
        can_data
    );
    
    CanBus_Debug_PrintCtl(can_data);
    /* Risultato atteso: [0x80, 0xA0, 0x00, 0x10, 0x0E, 0xAA, 0x00, 0x00] */
    
    /* Ora puoi inviare can_data tramite CAN */
    // CAN_Transmit(0x618, can_data, 8);
}

/**
 * ESEMPIO 2: Creazione pacchetto con LED abilitato
 */
void Example_CtlPacketWithLED(void) {
    uint8_t can_data[8];
    
    /* Crea pacchetto: Enable charger + LED, 16A AC, 500V, 17A */
    CanBus_CreatePacket_Ctl_Simple(
        true,      /* can_enable */
        true,      /* led3_enable - ABILITATO */
        16.0f,     /* iac_max_A */
        500.0f,    /* vout_max_V */
        17.0f,     /* iout_max_A */
        can_data
    );
    CanBus_Debug_PrintCtl(can_data);
    
    /* Risultato atteso: [0x88, 0xA0, 0x00, 0x88, 0x13, 0xAA, 0x00, 0x00] */
    
    // CAN_Transmit(0x618, can_data, 8);
}

/**
 * ESEMPIO 3: Disable charger
 */
void Example_DisableCharger(void) {
    uint8_t can_data[8];
    
    /* Crea pacchetto con CanEnable = false */
    CanBus_CreatePacket_Ctl_Simple(
        false,     /* can_enable - DISABILITATO */
        false,     /* led3_enable */
        0.0f,      /* iac_max_A */
        0.0f,      /* vout_max_V */
        0.0f,      /* iout_max_A */
        can_data
    );
    
    CanBus_Debug_PrintCtl(can_data);
    /* Risultato atteso: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] */
    
    // CAN_Transmit(0x618, can_data, 8);
}



/**
 * ESEMPIO 4: Generazione pacchetto casuale per testing
 * Genera UN pacchetto random e lo testa con TUTTI i decoder
 */
void Example_RandomPacket(void) {
    uint8_t random_data[8];
    
    /* Genera UN SOLO pacchetto con byte casuali */
    CanBus_GenerateRandomPacket(random_data);
    
    printf("\n\r=== RANDOM PACKET GENERATED ===\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", random_data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    
    /* Testa lo stesso pacchetto con TUTTI i decoder */
    printf("\n\r--- Interpreting as CTL (0x618) ---");
    CanBus_Debug_PrintCtl(random_data);
    
    printf("\n\r--- Interpreting as STAT (0x610) ---");
    CanBus_Debug_PrintStat(random_data);
    
    printf("\n\r--- Interpreting as ACT1 (0x611) ---");
    CanBus_Debug_PrintAct1(random_data);
    
    printf("\n\r--- Interpreting as ACT2 (0x614) ---");
    CanBus_Debug_PrintAct2(random_data);
    
    printf("\n\r--- Interpreting as TST1 (0x615) ---");
    CanBus_Debug_PrintTst1(random_data);
}

int main(void) {
    Example_BasicCtlPacket();
    printf("\n\r###########################\n\r");
    Example_CtlPacketWithLED();
    printf("\n\r###########################\n\r");
    Example_DisableCharger();
    printf("\n\r###########################\n\r");
    uint8_t can_data_ctl[8];
    CanBus_CreatePacket_Ctl_Simple(true, true, 32.0f, 420.0f, 25.0f, can_data_ctl);
    CanBus_Debug_PrintCtl(can_data_ctl);
    printf("\n\r###########################\n\r");

    /* Esempio con pacchetto casuale */
    Example_RandomPacket();
    
    return 0;
}