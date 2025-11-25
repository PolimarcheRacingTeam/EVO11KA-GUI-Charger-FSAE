/* =============================================================================
 *  FILE: utils_canBus_charger_level3.c
 * =============================================================================
 *
 *  EVO Charger CAN Bus Utilities - Level 3 
 *  CAN Service messages (AC currents, Temperatures, Diagnostics)
 * 
 * =============================================================================
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


/* CAN IDs - Level 3 (tutti Charger → BMS) */
#define CAN_ID_ACT3  0x712  /* AC Input Current of each module */
#define CAN_ID_TEMP  0x713  /* Temperature of each thermal sensor */
#define CAN_ID_STST1 0x715  /* Extra Real Time diagnostic */
#define CAN_ID_ACT4  0x714  /* Temperature FAN */

/* ACT3 Packet - ID 0x712 (Charger → BMS)
   Transmit every 100ms */
typedef struct {
    float fan_voltage_V;     /* DC voltage applied to FAN (0-30V) */
    float iacm1_A;           /* AC Input Current Phase 1 (0-50A) */
    float iacm2_A;           /* AC Input Current Phase 2 (0-50A) */
    float iacm3_A;           /* AC Input Current Phase 3 (0-50A) */
} CanPacket_Act3_t;

/* TEMP Packet - ID 0x713 (Charger → BMS)
   Transmit every 100ms */
typedef struct {
    float temp_loghv_C;      /* Temperature over Logic Board HV Side (-40 to +300°C) */
    float temp_power1_C;     /* Temperature over Power Stage 1 (-40 to +300°C) */
    float temp_power2_C;     /* Temperature over Power Stage 2 (-40 to +300°C) */
    float temp_power3_C;     /* Temperature over Power Stage 3 (-40 to +300°C) */
} CanPacket_Temp_t;

/* STST1 Packet - ID 0x715 (Charger → BMS) - Real Time Diagnostic
   Transmit every 100ms */
typedef struct {
    bool pfc_enable;         /* PFC Stage enable flag */
    bool log_temp_high;      /* Logic temperature over max */
    bool log_temp_low;       /* Logic temperature under min */
    bool uvlo_log;           /* UVLO (under voltage LV logic) flag */
    bool ther_low_fail;      /* Temperature error at -40°C */
    bool rx618_fail;         /* RX internal ID618 fail flag */
    bool bulk1_fail;         /* DC bulk1 fail flag */
    bool bulk2_fail;         /* DC bulk2 fail flag */
    bool bulk3_fail;         /* DC bulk3 fail flag */
    bool cooling_fail1;      /* Cooling fail over power stage 1 */
    bool cooling_fail2;      /* Cooling fail over power stage 2 */
    bool cooling_fail3;      /* Cooling fail over power stage 3 */
    bool uvlo_log_lv;        /* UVLO (under voltage LV logic) flag */
    bool bat_over;           /* Always hot Battery over 32Vdc (only when EN61851/SAEJ1772 enabled)*/
    bool bat_under;          /* Always hot Battery under 8Vdc (only when EN61851/SAEJ1772 enabled)*/
} CanPacket_Stst1_t;

/* ACT4 Packet - ID 0x714 (Charger → BMS)
   Transmit every 100ms */
typedef struct {
    float temp_logfan_C;     /* Temperature over Logic Board FAN Side (-40 to +300°C) */
    uint16_t iout1_raw;      /* Output current channel 1 (raw value 0-65535) */
    uint16_t iout2_raw;      /* Output current channel 2 (raw value 0-65535) */
    uint16_t iout3_raw;      /* Output current channel 3 (raw value 0-65535) */
} CanPacket_Act4_t;


/* ============================================================================
 * DECODER FUNCTIONS (Charger → BMS)
 * ============================================================================ */

/**
 * @brief Decodifica pacchetto ACT3 - ID 0x712
 * 
 * Formato: 8 byte
 * D0-D1: FAN Voltage (0-30V, resolution 0.1V)
 * D2-D3: AC Current Module 1 (0-50A, resolution 0.1A)
 * D4-D5: AC Current Module 2 (0-50A, resolution 0.1A)
 * D6-D7: AC Current Module 3 (0-50A, resolution 0.1A)
 * 
 * Formula: value = raw / 10
 * 
 * @param data Array di 8 byte ricevuti
 * @param act3 Struttura act3 da riempire (output)
 * @return true se successo
 */
bool CanBus_DecodePacket_Act3(const uint8_t data[8], CanPacket_Act3_t *act3) {
    if (act3 == NULL || data == NULL) return false;
    
    /* D0-D1: FAN Voltage  */
    uint16_t fan_raw = (data[0] << 8) | data[1];
    act3->fan_voltage_V = fan_raw * 0.1f;
    
    /* D2-D3: AC Current Module 1 */
    uint16_t iacm1_raw = (data[2] << 8) | data[3];
    act3->iacm1_A = iacm1_raw * 0.1f;
    
    /* D4-D5: AC Current Module 2  */
    uint16_t iacm2_raw = (data[4] << 8) | data[5];
    act3->iacm2_A = iacm2_raw * 0.1f;
    
    /* D6-D7: AC Current Module 3  */
    uint16_t iacm3_raw = (data[6] << 8) | data[7];
    act3->iacm3_A = iacm3_raw * 0.1f;
    
    return true;
}

/**
 * @brief Decodifica pacchetto TEMP - ID 0x713
 * 
 * Formato: 8 byte
 * D0-D1: Temp Logic HV (resolution 0.005188, offset -40°C)
 * D2-D3: Temp Power Stage 1 (resolution 0.005188, offset -40°C)
 * D4-D5: Temp Power Stage 2 (resolution 0.005188, offset -40°C)
 * D6-D7: Temp Power Stage 3 (resolution 0.005188, offset -40°C)
 * 
 * Formula: temp_C = (raw × 0.005188) - 40
 * 
 * @param data Array di 8 byte ricevuti
 * @param temp Struttura temp da riempire (output)
 * @return true se successo
 */
bool CanBus_DecodePacket_Temp(const uint8_t data[8], CanPacket_Temp_t *temp) {
    if (temp == NULL || data == NULL) return false;
    
    /* D0-D1: Temp Logic HV */
    uint16_t loghv_raw = (data[0] << 8) | data[1];
    temp->temp_loghv_C = (loghv_raw * 0.005188f) - 40.0f;
    
    /* D2-D3: Temp Power Stage 1 */
    uint16_t power1_raw = (data[2] << 8) | data[3];
    temp->temp_power1_C = (power1_raw * 0.005188f) - 40.0f;
    
    /* D4-D5: Temp Power Stage 2 */
    uint16_t power2_raw = (data[4] << 8) | data[5];
    temp->temp_power2_C = (power2_raw * 0.005188f) - 40.0f;
    
    /* D6-D7: Temp Power Stage 3 */
    uint16_t power3_raw = (data[6] << 8) | data[7];
    temp->temp_power3_C = (power3_raw * 0.005188f) - 40.0f;
    
    return true;
}

/**
 * @brief Decodifica pacchetto STST1 (Real Time Diagnostic) - ID 0x715
 * 
 * Formato: 8 byte con flag diagnostici
 * Byte 0 (D0): Bit 2, 13, 12, 11, 10, 8
 * Byte 1 (D1): Bit 23, 22, 21, 20, 19, 18
 * Byte 2 (D2): Bit 27, 25, 24
 * Byte 3 (D3): Empty
 * 
 * Nota: I bit nella tabella sono numerati con Start Byte e Start Bit
 * dove Start Bit è la posizione all'interno del byte (0-7)
 * 
 * @param data Array di 8 byte ricevuti
 * @param stst Struttura stst1 da riempire (output)
 * @return true se successo
 */
bool CanBus_DecodePacket_Stst1(const uint8_t data[8], CanPacket_Stst1_t *stst) {
    if (stst == NULL || data == NULL) return false;
    
    /* Byte 0 (D0) - Start Bit indica la posizione nel byte */
    stst->pfc_enable    = (data[0] & (1 << 2)) != 0;   /* Start Byte=0, Start Bit=2 */
    
    /* Byte 1 (D1) */
    stst->log_temp_high = (data[1] & (1 << 5)) != 0;   /* Start Byte=1, Start Bit=13 → bit 5 del byte 1 */
    stst->log_temp_low  = (data[1] & (1 << 4)) != 0;   /* Start Byte=1, Start Bit=12 → bit 4 del byte 1 */
    stst->uvlo_log      = (data[1] & (1 << 3)) != 0;   /* Start Byte=1, Start Bit=11 → bit 3 del byte 1 */
    stst->ther_low_fail = (data[1] & (1 << 2)) != 0;   /* Start Byte=1, Start Bit=10 → bit 2 del byte 1 */
    stst->rx618_fail    = (data[1] & (1 << 0)) != 0;   /* Start Byte=1, Start Bit=8 → bit 0 del byte 1 */
    
    /* Byte 2 (D2) */
    stst->bulk1_fail    = (data[2] & (1 << 7)) != 0;   /* Start Byte=2, Start Bit=23 → bit 7 del byte 2 */
    stst->bulk2_fail    = (data[2] & (1 << 6)) != 0;   /* Start Byte=2, Start Bit=22 → bit 6 del byte 2 */
    stst->bulk3_fail    = (data[2] & (1 << 5)) != 0;   /* Start Byte=2, Start Bit=21 → bit 5 del byte 2 */
    stst->cooling_fail1 = (data[2] & (1 << 4)) != 0;   /* Start Byte=2, Start Bit=20 → bit 4 del byte 2 */
    stst->cooling_fail2 = (data[2] & (1 << 3)) != 0;   /* Start Byte=2, Start Bit=19 → bit 3 del byte 2 */
    stst->cooling_fail3 = (data[2] & (1 << 2)) != 0;   /* Start Byte=2, Start Bit=18 → bit 2 del byte 2 */
    
    /* Byte 3 (D3) */
    stst->uvlo_log_lv   = (data[3] & (1 << 3)) != 0;   /* Start Byte=3, Start Bit=27 → bit 3 del byte 3 */
    stst->bat_over      = (data[3] & (1 << 1)) != 0;   /* Start Byte=3, Start Bit=25 → bit 1 del byte 3 */
    stst->bat_under     = (data[3] & (1 << 0)) != 0;   /* Start Byte=3, Start Bit=24 → bit 0 del byte 3 */
    
    return true;
}

/**
 * @brief Decodifica pacchetto ACT4 - ID 0x714
 * 
 * Formato: 8 byte
 * D0-D1: Temp Logic FAN (resolution 0.005188, offset -40°C)
 * D2-D3: Output current channel 1 (raw value, resolution 1)
 * D4-D5: Output current channel 2 (raw value, resolution 1)
 * D6-D7: Output current channel 3 (raw value, resolution 1)
 * 
 * @param data Array di 8 byte ricevuti
 * @param act4 Struttura act4 da riempire (output)
 * @return true se successo
 */
bool CanBus_DecodePacket_Act4(const uint8_t data[8], CanPacket_Act4_t *act4) {
    if (act4 == NULL || data == NULL) return false;
    
    /* D0-D1: Temp Logic FAN */
    uint16_t temp_raw = (data[0] << 8) | data[1];
    act4->temp_logfan_C = (temp_raw * 0.005188f) - 40.0f;
    
    /* D2-D3: Output current channel 1 */
    act4->iout1_raw = (data[2] << 8) | data[3];
    
    /* D4-D5: Output current channel 2 */
    act4->iout2_raw = (data[4] << 8) | data[5];
    
    /* D6-D7: Output current channel 3 */
    act4->iout3_raw = (data[6] << 8) | data[7];
    
    return true;
}


/* ============================================================================
 * DEBUG FUNCTIONS
 * ============================================================================ */

/**
 * @brief Stampa pacchetto ACT3 decodificato
 */
void CanBus_Debug_PrintAct3(const uint8_t data[8]) {
    CanPacket_Act3_t act3;
    CanBus_DecodePacket_Act3(data, &act3);
    
    printf("\n\rACT3 Packet Decoded:\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    
    printf("  FAN Voltage: %.1f V\n", act3.fan_voltage_V);
    printf("  AC Current Module 1: %.1f A\n", act3.iacm1_A);
    printf("  AC Current Module 2: %.1f A\n", act3.iacm2_A);
    printf("  AC Current Module 3: %.1f A\n", act3.iacm3_A);
    printf("  Total AC Current: %.1f A\n", act3.iacm1_A + act3.iacm2_A + act3.iacm3_A);
}

/**
 * @brief Stampa pacchetto TEMP decodificato
 */
void CanBus_Debug_PrintTemp(const uint8_t data[8]) {
    CanPacket_Temp_t temp;
    CanBus_DecodePacket_Temp(data, &temp);
    
    printf("\n\rTEMP Packet Decoded:\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    
    printf("  Logic Board HV Temp: %.1f .C\n", temp.temp_loghv_C);
    printf("  Power Stage 1 Temp: %.1f .C\n", temp.temp_power1_C);
    printf("  Power Stage 2 Temp: %.1f .C\n", temp.temp_power2_C);
    printf("  Power Stage 3 Temp: %.1f .C\n", temp.temp_power3_C);
    
    /* Trova temperatura massima */
    float max_temp = temp.temp_power1_C;
    if (temp.temp_power2_C > max_temp) max_temp = temp.temp_power2_C;
    if (temp.temp_power3_C > max_temp) max_temp = temp.temp_power3_C;
    printf("  Max Power Stage Temp: %.1f .C\n", max_temp);
}

/**
 * @brief Stampa pacchetto STST1 decodificato
 */
void CanBus_Debug_PrintStst1(const uint8_t data[8]) {
    CanPacket_Stst1_t stst;
    CanBus_DecodePacket_Stst1(data, &stst);
    
    printf("\n\rSTST1 Packet Decoded:\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    
    printf("  === PFC Status ===\n");
    printf("  PFC Enable: %s\n", stst.pfc_enable ? "true" : "false");
    
    printf("  === Temperature Flags ===\n");
    printf("  Logic Temp High: %s\n", stst.log_temp_high ? "true" : "false");
    printf("  Logic Temp Low: %s\n", stst.log_temp_low ? "true" : "false");
    printf("  Thermal Low Fail (-40°C): %s\n", stst.ther_low_fail ? "true" : "false");
    
    printf("  === Bulk Flags ===\n");
    printf("  Bulk1 Fail: %s\n", stst.bulk1_fail ? "true" : "false");
    printf("  Bulk2 Fail: %s\n", stst.bulk2_fail ? "true" : "false");
    printf("  Bulk3 Fail: %s\n", stst.bulk3_fail ? "true" : "false");
    
    printf("  === Cooling Flags ===\n");
    printf("  Cooling Fail Stage 1: %s\n", stst.cooling_fail1 ? "true" : "false");
    printf("  Cooling Fail Stage 2: %s\n", stst.cooling_fail2 ? "true" : "false");
    printf("  Cooling Fail Stage 3: %s\n", stst.cooling_fail3 ? "true" : "false");
    
    printf("  === Voltage Flags ===\n");
    printf("  UVLO Logic: %s\n", stst.uvlo_log ? "true" : "false");
    printf("  UVLO Logic LV: %s\n", stst.uvlo_log_lv ? "true" : "false");
    printf("  Battery Over (>32V): %s\n", stst.bat_over ? "true" : "false");
    printf("  Battery Under (<8V): %s\n", stst.bat_under ? "true" : "false");
    
    printf("  === Communication ===\n");
    printf("  RX618 Fail: %s\n", stst.rx618_fail ? "true" : "false");
}

/**
 * @brief Stampa pacchetto ACT4 decodificato
 */
void CanBus_Debug_PrintAct4(const uint8_t data[8]) {
    CanPacket_Act4_t act4;
    CanBus_DecodePacket_Act4(data, &act4);
    
    printf("\n\rACT4 Packet Decoded:\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    
    printf("  Logic FAN Temp: %.1f .C\n", act4.temp_logfan_C);
    printf("  Output Current Ch1 (raw): %u\n", act4.iout1_raw);
    printf("  Output Current Ch2 (raw): %u\n", act4.iout2_raw);
    printf("  Output Current Ch3 (raw): %u\n", act4.iout3_raw);
}


/* ============================================================================
 * EXAMPLES
 * ============================================================================ */

/**
 * ESEMPIO 1: Decodifica ACT3 - AC Currents
 */
void Example_DecodeAct3(void) {
    /* Esempio: FAN 12V, Phase1=10A, Phase2=10A, Phase3=10A */
    uint8_t act3_data[8] = {0x00, 0x78, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64};
    
    printf("\n\r=== DECODE ACT3 EXAMPLE ===\n");
    CanBus_Debug_PrintAct3(act3_data);
}

/**
 * ESEMPIO 2: Decodifica TEMP - Temperatures
 */
void Example_DecodeTemp(void) {
    /* Esempio: tutte le temperature a 25°C
     * Formula: raw = (temp + 40) / 0.005188 = (25 + 40) / 0.005188 ≈ 12535 = 0x30F7
     */
    uint8_t temp_data[8] = {0x30, 0xF7, 0x30, 0xF7, 0x30, 0xF7, 0x30, 0xF7};
    
    printf("\n\r=== DECODE TEMP EXAMPLE ===\n");
    CanBus_Debug_PrintTemp(temp_data);
}

/**
 * ESEMPIO 3: Decodifica STST1 - Diagnostics
 */
void Example_DecodeStst1(void) {
    /* Esempio: PFC enabled, tutto OK (nessun errore) */
    uint8_t stst_data[8] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    printf("\n\r=== DECODE STST1 EXAMPLE ===\n");
    CanBus_Debug_PrintStst1(stst_data);
}

/**
 * ESEMPIO 4: Decodifica ACT4 - FAN Temp + Output Currents
 */
void Example_DecodeAct4(void) {
    /* Esempio: FAN temp 30°C, correnti raw 100, 150, 200 */
    uint8_t act4_data[8] = {0x36, 0x70, 0x00, 0x64, 0x00, 0x96, 0x00, 0xC8};
    
    printf("\n\r=== DECODE ACT4 EXAMPLE ===\n");
    CanBus_Debug_PrintAct4(act4_data);
}


int main(void) {
    printf("\n\r========================================\n");
    printf("  EVO Charger - CAN Bus Level 3 Test\n");
    printf("========================================\n");
    
    Example_DecodeAct3();
    printf("\n\r###########################\n");
    
    Example_DecodeTemp();
    printf("\n\r###########################\n");
    
    Example_DecodeStst1();
    printf("\n\r###########################\n");
    
    Example_DecodeAct4();
    
    return 0;
}
