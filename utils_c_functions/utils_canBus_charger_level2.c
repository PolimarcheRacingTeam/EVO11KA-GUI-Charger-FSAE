/* =============================================================================
 *  FILE: utils_canBus_charger_level2.c
 * =============================================================================
 *
 *  EVO Charger CAN Bus Utilities - Level 2 
 *  On-demand diagnostic messages (Fault codes, Software, Serial Number)
 * 
 * =============================================================================
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


/* CAN IDs - Level 2 */
#define CAN_ID_REQ   0x61B  /* BMS → Charger - Request diagnostic */
#define CAN_ID_FLTA  0x61D  /* Charger → BMS - Fault Active */
#define CAN_ID_FLTP  0x61C  /* Charger → BMS - Fault Passive (Inactive) */
#define CAN_ID_SW    0x61E  /* Charger → BMS - Software Version */
#define CAN_ID_SN    0x61F  /* Charger → BMS - Serial Number */

/* Request Types */
typedef enum {
    REQ_FAULT_INACTIVE = 0x1C,  /* Request inactive faults (ID 0x61C) */
    REQ_FAULT_ACTIVE   = 0x1D,  /* Request active faults (ID 0x61D) */
    REQ_SOFTWARE       = 0x1E,  /* Request software version (ID 0x61E) */
    REQ_SERIAL_NUMBER  = 0x1F   /* Request serial number (ID 0x61F) - sent only once at startup */
} RequestType_t;

/* Failure Level */
typedef enum {
    FAILURE_WARNING      = 1,   /* Warning - charger works normally but de-rated */
    FAILURE_SOFT         = 10,  /* Soft Failure - charger stops, restarts when fault clears */
    FAILURE_HARD         = 11   /* Hard Failure - charger stops, needs AC disconnect/reconnect */
} FailureLevel_t;

/* Frame Type */
typedef enum {
    FRAME_SINGLE = 1,  /* Single frame (only one fault) */
    FRAME_MULTI  = 2   /* Multi frame (more than one fault) */
} FrameType_t;

/* REQ Packet (BMS → Charger) - Request diagnostic info */
typedef struct {
    bool enable;           /* Request Enable (bit 7 of D0) */
    uint16_t id_requested; /* ID requested (0x61C, 0x61D, 0x61E, 0x61F) */
} CanPacket_Req_t;

/* Fault Packet Structure (Active or Passive) - ID 0x61D (Active) or 0x61C (Passive) */
typedef struct {
    FrameType_t frame_type;      /* 1=Single frame, 2=Multi frame */
    uint8_t total_errors;        /* Total number of faults (0-63) */
    uint8_t frame_number;        /* Frame number in transmission (0-63) */
    uint8_t fault_code;          /* Fault code (0x00-0xFF) - See Table 4.6 */
    uint8_t occurrence;          /* Number of occurrences (0-63) */
    FailureLevel_t failure_level;/* 1=Warning, 10=Soft, 11=Hard */
    uint16_t first_time_h;       /* First time fault occurred (hours, 0-65535) */
    uint16_t last_time_h;        /* Last time fault occurred (hours, 0-65535) */
} CanPacket_Fault_t;

/* Software Version Packet - ID 0x61E */
typedef struct {
    char version[9];  /* 8 ASCII characters + null terminator */
} CanPacket_Software_t;

/* Serial Number Packet - ID 0x61F */
typedef struct {
    char serial[9];  /* 8 ASCII characters + null terminator */
} CanPacket_SerialNumber_t;

/* Fault Code Definitions (from Table 4.6) */
typedef enum {
    FAULT_A0_BULK1_VOLTAGE     = 0xA0,
    FAULT_A1_BULK2_VOLTAGE     = 0xA1,
    FAULT_A2_BULK3_VOLTAGE     = 0xA2,
    FAULT_A3_BULK_ERROR        = 0xA3,
    FAULT_A4_CAN_REGISTERS     = 0xA4,
    FAULT_A5_CAN_COMMAND       = 0xA5,
    FAULT_A6_TEMP_LOW          = 0xA6,
    FAULT_A7_TEMP_DERATING     = 0xA7,
    FAULT_A8_TEMP_HIGH         = 0xA8,
    FAULT_A9_TEMP_FAILED       = 0xA9,
    FAULT_AA_INPUT_CURRENT_MAX = 0xAA,
    FAULT_AB_HVIL_INTERLOCK    = 0xAB,
    FAULT_AC_LOGIC_TEMP        = 0xAC,
    FAULT_AD_OUTPUT_OVERVOLT   = 0xAD
} FaultCode_t;


/* ============================================================================
 * ENCODER FUNCTIONS (BMS → Charger)
 * ============================================================================ */

/**
 * @brief Crea un pacchetto Request (REQ) per richiedere informazioni diagnostiche
 * 
 * Formato pacchetto REQ (4 bytes) - ID 0x61B:
 * ┌────────┬────────┬────────┬────────┐
 * │   D0   │   D1   │   D2   │   D3   │
 * ├────────┼────────┼────────┼────────┤
 * │ Enable │  0x00  │ ID MSB │ ID LSB │
 * └────────┴────────┴────────┴────────┘
 * 
 * D0: Bit 7 = Request Enable (1=enabled)
 * D1: Always 0x00
 * D2: ID MSB (sempre 0x06)
 * D3: ID LSB (0x1C, 0x1D, 0x1E, 0x1F)
 *   - 0x1C = Inactive Faults
 *   - 0x1D = Active Faults
 *   - 0x1E = Software Version
 *   - 0x1F = Serial Number
 * 
 * @param enable Enable request (true/false)
 * @param request_type Type of request (REQ_FAULT_INACTIVE, REQ_FAULT_ACTIVE, etc.)
 * @param data Array di 8 byte da riempire (output)
 * @return true se successo
 */
bool CanBus_CreatePacket_Req(bool enable, RequestType_t request_type, uint8_t data[8]) {
    if (data == NULL) return false;
    
    memset(data, 0, 8);
    
    /* D0: Enable bit (bit 7) */
    data[0] = enable ? 0x80 : 0x00;
    
    /* D1: Always 0x00 */
    data[1] = 0x00;
    
    /* D2-D3: ID richiesto (MSB first) */
    data[2] = 0x06;              /* MSB: sempre 0x06 */
    data[3] = request_type;      /* LSB: 0x1C, 0x1D, 0x1E, 0x1F */
    
    return true;
}

/**
 * @brief Wrapper semplificato per richiedere fault attivi
 */
bool CanBus_Request_FaultActive(uint8_t data[8]) {
    return CanBus_CreatePacket_Req(true, REQ_FAULT_ACTIVE, data);
}

/**
 * @brief Wrapper semplificato per richiedere fault inattivi
 */
bool CanBus_Request_FaultInactive(uint8_t data[8]) {
    return CanBus_CreatePacket_Req(true, REQ_FAULT_INACTIVE, data);
}

/**
 * @brief Wrapper semplificato per richiedere versione software
 */
bool CanBus_Request_Software(uint8_t data[8]) {
    return CanBus_CreatePacket_Req(true, REQ_SOFTWARE, data);
}

/**
 * @brief Wrapper semplificato per richiedere serial number
 */
bool CanBus_Request_SerialNumber(uint8_t data[8]) {
    return CanBus_CreatePacket_Req(true, REQ_SERIAL_NUMBER, data);
}


/* ============================================================================
 * DECODER FUNCTIONS (Charger → BMS)
 * ============================================================================ */

/**
 * @brief Decodifica pacchetto Fault (Active o Passive) - ID 0x61D o 0x61C
 * 
 * Formato: 8 byte
 * D0: Frame type (bit 7-6) + Total errors (bit 5-0)
 * D1: Frame number (bit 7-2)
 * D2: Fault code (8 bit)
 * D3: Occurrence (bit 7-2) + Failure level (bit 1-0)
 * D4-D5: First time (16-bit, Big Endian)
 * D6-D7: Last time (16-bit, Big Endian)
 * 
 * @param data Array di 8 byte ricevuti
 * @param fault Struttura fault da riempire (output)
 * @return true se successo
 */
bool CanBus_DecodePacket_Fault(const uint8_t data[8], CanPacket_Fault_t *fault) {
    if (fault == NULL || data == NULL) return false;
    
    /* D0: Frame type (bit 7-6) + Total errors (bit 5-0) */
    fault->frame_type = (FrameType_t)((data[0] >> 6) & 0x03);
    fault->total_errors = data[0] & 0x3F;  /* 6 bit: 0-63 */
    
    /* D1: Frame number (bit 7-2) - Note: inizia da 1, non da 0 */
    fault->frame_number = (data[1] >> 2) & 0x3F;  /* 6 bit: 1-63 */
    
    /* D2: Fault code */
    fault->fault_code = data[2];
    
    /* D3: Occurrence (bit 7-2) + Failure level (bit 1-0) */
    fault->occurrence = (data[3] >> 2) & 0x3F;  /* 6 bit: 0-63 */
    uint8_t level_bits = data[3] & 0x03;
    
    /* Converti level bits in FailureLevel_t */
    if (level_bits == 0x00) {
        fault->failure_level = FAILURE_WARNING;      /* 01 = Warning */
    } else if (level_bits == 0x02) {
        fault->failure_level = FAILURE_SOFT;         /* 10 = Soft */
    } else if (level_bits == 0x03) {
        fault->failure_level = FAILURE_HARD;         /* 11 = Hard */
    }
    
    /* D4-D5: First time (Big Endian) */
    fault->first_time_h = (data[4] << 8) | data[5];
    
    /* D6-D7: Last time (Big Endian) */
    fault->last_time_h = (data[6] << 8) | data[7];
    
    return true;
}

/**
 * @brief Decodifica pacchetto Software Version - ID 0x61E
 * 
 * Formato: 8 byte ASCII
 * Esempio: "SW3225A5" = 0x53 0x57 0x33 0x32 0x32 0x35 0x41 0x35
 * 
 * @param data Array di 8 byte ricevuti
 * @param sw Struttura software da riempire (output)
 * @return true se successo
 */
bool CanBus_DecodePacket_Software(const uint8_t data[8], CanPacket_Software_t *sw) {
    if (sw == NULL || data == NULL) return false;
    
    /* Copia 8 byte ASCII */
    memcpy(sw->version, data, 8);
    sw->version[8] = '\0';  /* Null terminator */
    
    return true;
}

/**
 * @brief Decodifica pacchetto Serial Number - ID 0x61F
 * 
 * Formato: 8 byte ASCII
 * 
 * @param data Array di 8 byte ricevuti
 * @param sn Struttura serial number da riempire (output)
 * @return true se successo
 */
bool CanBus_DecodePacket_SerialNumber(const uint8_t data[8], CanPacket_SerialNumber_t *sn) {
    if (sn == NULL || data == NULL) return false;
    
    /* Copia 8 byte ASCII */
    memcpy(sn->serial, data, 8);
    sn->serial[8] = '\0';  /* Null terminator */
    
    return true;
}


/* ============================================================================
 * DEBUG FUNCTIONS
 * ============================================================================ */

/**
 * @brief Stampa pacchetto Request decodificato
 */
void CanBus_Debug_PrintReq(const uint8_t data[8]) {
    printf("\n\rREQ Packet Decoded:\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 4; i++) {
        printf("%02X", data[i]);
        if (i < 3) printf(", ");
    }
    printf("]\n");
    
    bool enable = (data[0] & 0x80) != 0;
    uint16_t id = (data[2] << 8) | data[3];  /* D2=MSB, D3=LSB */
    
    printf("  Request Enable: %s\n", enable ? "true" : "false");
    printf("  ID Requested: 0x%04X", id);
    
    switch (data[3]) {  /* LSB byte */
        case 0x1C: printf(" (Inactive Faults)\n"); break;
        case 0x1D: printf(" (Active Faults)\n"); break;
        case 0x1E: printf(" (Software Version)\n"); break;
        case 0x1F: printf(" (Serial Number)\n"); break;
        default: printf(" (Unknown)\n"); break;
    }
}

/**
 * @brief Ottiene il nome del fault code
 */
const char* CanBus_GetFaultName(uint8_t code) {
    switch (code) {
        case FAULT_A0_BULK1_VOLTAGE:     return "Bulk 1 Voltage";
        case FAULT_A1_BULK2_VOLTAGE:     return "Bulk 2 Voltage";
        case FAULT_A2_BULK3_VOLTAGE:     return "Bulk 3 Voltage";
        case FAULT_A3_BULK_ERROR:        return "Bulk Error";
        case FAULT_A4_CAN_REGISTERS:     return "CAN Registers";
        case FAULT_A5_CAN_COMMAND:       return "CAN Command";
        case FAULT_A6_TEMP_LOW:          return "Cold Plate Temp LOW";
        case FAULT_A7_TEMP_DERATING:     return "Cold Plate Temp DERATING";
        case FAULT_A8_TEMP_HIGH:         return "Cold Plate Temp HIGH";
        case FAULT_A9_TEMP_FAILED:       return "Cold Plate Temp FAILED";
        case FAULT_AA_INPUT_CURRENT_MAX: return "Input Current MAX";
        case FAULT_AB_HVIL_INTERLOCK:    return "HVIL Interlock Loop";
        case FAULT_AC_LOGIC_TEMP:        return "Logic Temperature";
        case FAULT_AD_OUTPUT_OVERVOLT:   return "Output Overvoltage";
        default: return "Unknown Fault";
    }
}

/**
 * @brief Ottiene la stringa del livello di errore
 */
const char* CanBus_GetFailureLevelStr(FailureLevel_t level) {
    switch (level) {
        case FAILURE_WARNING: return "Warning";
        case FAILURE_SOFT:    return "Soft Failure";
        case FAILURE_HARD:    return "Hard Failure";
        default: return "Unknown";
    }
}

/**
 * @brief Verifica se il pacchetto è "No Fault Detected"
 * Frame speciale: tutti i byte da D1 a D7 sono 0xFF
 */
bool CanBus_IsNoFaultDetected(const uint8_t data[8]) {
    /* Controlla se D1-D7 sono tutti 0xFF */
    for (int i = 1; i < 8; i++) {
        if (data[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Stampa pacchetto Fault decodificato (Active o Passive)
 * @param data Array di 8 byte del pacchetto Fault
 * @param is_active true se Active Fault (ID 0x61D), false se Passive Fault (ID 0x61C), indicare quale tipo di fault si sta stampando
 */
void CanBus_Debug_PrintFault(const uint8_t data[8], bool is_active) {
    printf("\n\r%s FAULT Packet Decoded:\n", is_active ? "ACTIVE" : "PASSIVE");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    
    /* Verifica se è il frame "No Fault Detected" */
    if (CanBus_IsNoFaultDetected(data)) {
        printf("  *** NO FAULT DETECTED ***\n");
        printf("  No faults stored in the charger.\n");
        return;
    }
    
    /* Decodifica normale */
    CanPacket_Fault_t fault;
    CanBus_DecodePacket_Fault(data, &fault);
    
    printf("  Frame Type: %s\n", fault.frame_type == FRAME_SINGLE ? "SINGLE" : "MULTI");
    printf("  Total Faults: %u\n", fault.total_errors);
    
    /* Mostra frame corrente e totale solo se MULTI frame */
    if (fault.frame_type == FRAME_MULTI) {
        printf("  Frame: %u of %u\n", fault.frame_number, fault.total_errors);
    } else {
        printf("  Frame: 1 of 1\n");
    }
    
    printf("  Fault Code: 0x%02X (%s)\n", fault.fault_code, CanBus_GetFaultName(fault.fault_code));
    printf("  Occurrence: %u times\n", fault.occurrence);
    printf("  Failure Level: %s\n", CanBus_GetFailureLevelStr(fault.failure_level));
    printf("  First Time: %u hours\n", fault.first_time_h);
    printf("  Last Time: %u hours\n", fault.last_time_h);
}

/**
 * @brief Stampa pacchetto Software decodificato
 */
void CanBus_Debug_PrintSoftware(const uint8_t data[8]) {
    CanPacket_Software_t sw;
    CanBus_DecodePacket_Software(data, &sw);
    
    printf("\n\rSOFTWARE Packet Decoded:\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    printf("  Software Version: %s\n", sw.version);
}

/**
 * @brief Stampa pacchetto Serial Number decodificato
 */
void CanBus_Debug_PrintSerialNumber(const uint8_t data[8]) {
    CanPacket_SerialNumber_t sn;
    CanBus_DecodePacket_SerialNumber(data, &sn);
    
    printf("\n\rSERIAL NUMBER Packet Decoded:\n");
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");
    printf("  Serial Number: %s\n", sn.serial);
}


/* ============================================================================
 * EXAMPLES
 * ============================================================================ */

/**
 * ESEMPIO 1: Richiesta fault attivi
 */
void Example_RequestActiveFaults(void) {
    uint8_t req_data[8];
    
    /* Crea richiesta fault attivi */
    CanBus_Request_FaultActive(req_data);
    
    printf("\n\r=== REQUEST ACTIVE FAULTS ===\n");
    CanBus_Debug_PrintReq(req_data);
    
    /* Invia su CAN */
    // CAN_Transmit(0x61B, req_data, 4);
}

/**
 * ESEMPIO 2: Decodifica risposta fault attivo
 */
void Example_DecodeActiveFault(void) {
    /* Esempio da documentazione: 
     * Single frame, 1 fault, code 0xA8, hard failure, occurrence 5
     * First time 30h, last time 120h 
     */
    uint8_t fault_data[8] = {0x41, 0x01, 0xA8, 0x17, 0x00, 0x1E, 0x00, 0x78};
    
    printf("\n\r=== DECODE ACTIVE FAULT EXAMPLE ===\n");
    CanBus_Debug_PrintFault(fault_data, true);
}

/**
 * ESEMPIO 3: Richiesta software version
 */
void Example_RequestSoftware(void) {
    uint8_t req_data[8];
    
    CanBus_Request_Software(req_data);
    
    printf("\n\r=== REQUEST SOFTWARE VERSION ===\n");
    CanBus_Debug_PrintReq(req_data);
    
    // CAN_Transmit(0x61B, req_data, 4);
    
    /* Simula risposta: "SW3225A5" */
    uint8_t sw_data[8] = {0x53, 0x57, 0x33, 0x32, 0x32, 0x35, 0x41, 0x35};
    CanBus_Debug_PrintSoftware(sw_data);
}

/**
 * ESEMPIO 4: Frame "No Fault Detected"
 */
void Example_NoFaultDetected(void) {
    /* Frame speciale per nessun fault: tutti 0xFF */
    uint8_t no_fault[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    printf("\n\r=== NO FAULT DETECTED ===\n");
    CanBus_Debug_PrintFault(no_fault, true);
}


int main(void) {
    printf("\n\r========================================\n");
    printf("  EVO Charger - CAN Bus Level 2 Test\n");
    printf("========================================\n");
    
    Example_RequestActiveFaults();
    printf("\n\r###########################\n");
    
    Example_DecodeActiveFault();
    printf("\n\r###########################\n");
    
    Example_RequestSoftware();
    printf("\n\r###########################\n");
    
    Example_NoFaultDetected();
    
    return 0;
}
