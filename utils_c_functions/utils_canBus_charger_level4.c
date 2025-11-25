/* =============================================================================
 *  FILE: utils_canBus_charger_level4.c
 * =============================================================================
 *
 *  EVO Charger CAN Bus Utilities - Level 4 
 *  SETUP CAN Specifications (Charger Configuration Message)
 * 
 *  Note: "Level 4 messages are reserved and normally not used by customers."
 *        This implementation only decodes the Tst2 message (ID 0x616) which
 *        contains charger configuration parameters defined during setup.
 * 
 * =============================================================================
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


/* CAN IDs - Level 4 */
#define CAN_ID_TST2    0x616  /* Charger → BMS - Charger Configuration */

/* Baudrate definitions */
typedef enum {
    BAUDRATE_500KBIT = 0,  /* 500 Kbit/s */
    BAUDRATE_250KBIT = 1,  /* 250 Kbit/s */
    BAUDRATE_125KBIT = 2,  /* 125 Kbit/s */
    BAUDRATE_1MBIT   = 3   /* 1 Mbit/s */
} BaudrateType_t;

/* ID Type definitions */
typedef enum {
    ID_STANDARD_11BIT = 0,  /* Standard Frame 11bit */
    ID_EXTENDED_29BIT = 1   /* Extended Frame 29bit */
} IdType_t;

/* AC Current Control definitions */
typedef enum {
    IAC_NOT_CONTROLLED = 0,  /* Not controlled (HW set) */
    IAC_SAEJ1772       = 1,  /* SAE J1772 Enabled */
    IAC_EN61851        = 2,  /* EN61851 Enabled */
    IAC_ID618          = 3   /* AC current controlled by ID618 */
} IacControlType_t;

/* Range definitions */
typedef enum {
    RANGE_R4_EVO_USERS = 0,  /* Range R4 */
    RANGE_R3           = 1,  /* Range R3 */
    RANGE_R2           = 2,  /* Range R2 */
    RANGE_R1           = 3   /* Range R1 */
} RangeType_t;

/* EVC Model definitions */
typedef enum {
    EVC_EVO11K  = 0,  /* EVO11K (liquid cooled) */
    EVC_EVO22K  = 1   /* EVO22K (air cooled) */
} EVCModelType_t;

/* ID Setting definitions */
typedef enum {
    ID_SINGLE_CHARGER = 0,   /* Single charger (used for several chargers in parallel) */
    ID_RANGE_1_16     = 1    /* ID from 1 to 16 (0b0000 to 0b1111) */
} IDSettingType_t;

/* TST2 Packet - ID 0x616 (Charger → BMS)
   Sent once when charger is switched on */
typedef struct {
    BaudrateType_t baudrate;       /* CAN Baudrate (500k, 250k, 125k, 1M) */
    IdType_t id_type;              /* ID Format (11bit or 29bit) */
    IacControlType_t iac_control;  /* AC current control method */
    RangeType_t range;             /* Output voltage range */
    bool three_phase;              /* Three-phase config (true=3ph, false=Y Grid) */
    bool slave;                    /* Slave mode (parallel chargers with EN61851/J1772) */
    EVCModelType_t evc_model;      /* EVO model (EVO22K or EVO11K) */
    IDSettingType_t id_setting;    /* ID setting for parallel chargers */
    bool air_cooler;               /* Cooling type (true=EVO11KA air, false=EVO11KL liquid) */
    bool parallel_ctrl;            /* Parallel control (same CAN command for multiple chargers) */
    float iacm_max_set_A;          /* Max AC input current [A] (0-51A, resolution 0.2A) */
    float vout_max_set_V;          /* Max DC output voltage [V] (0-1000V, resolution 0.1V) */
    float iout_max_set_A;          /* Max DC output current [A] (0-150A, resolution 0.1A) */
    uint8_t password;              /* System Password (0-255) - Factory: 0xA5 */
} CanPacket_Tst2_t;


/* ============================================================================
 * DECODER FUNCTIONS (Charger → BMS)
 * ============================================================================ */

/**
 * @brief Decodifica pacchetto TST2 (Charger Configuration) - ID 0x616
 * 
 * Formato: 8 byte con Start Byte e Start Bit come da tabella
 * 
 * Byte 0 (D0):
 *   - Start Byte=0, Start Bit=7, Length=2: Baudrate
 *   - Start Byte=0, Start Bit=5, Length=1: IDType
 *   - Start Byte=0, Start Bit=4, Length=2: IacControl
 *   - Start Byte=0, Start Bit=2, Length=2: Range
 * 
 * Byte 1 (D1):
 *   - Start Byte=0, Start Bit=0, Length=1: 3Pconfig
 *   - Start Byte=1, Start Bit=15, Length=1: Slave (bit 7 del byte 1)
 *   - Start Byte=1, Start Bit=14, Length=1: EVCmodel (bit 6 del byte 1)
 *   - Start Byte=1, Start Bit=13, Length=4: IDsetting (bit 5-2 del byte 1)
 *   - Start Byte=1, Start Bit=8, Length=1: AirCooler (bit 0 del byte 1)
 *   - Start Byte=1, Start Bit=9, Length=1: ParallelCtrl (bit 1 del byte 1)
 * 
 * Byte 2 (D2): IacmMaxSet (8 bit, resolution 0.2A)
 * Byte 3-4 (D3-D4): VoutMaxSet (16 bit Big Endian, resolution 0.1V)
 * Byte 5-6 (D5-D6): IoutMaxSet (16 bit Big Endian, resolution 0.1A)
 * Byte 7 (D7): Psw (8 bit, 0-255)
 * 
 * @param data Array di 8 byte ricevuti
 * @param tst2 Struttura tst2 da riempire (output)
 * @return true se successo
 */
bool CanBus_DecodePacket_Tst2(const uint8_t data[8], CanPacket_Tst2_t *tst2) {
    if (tst2 == NULL || data == NULL) return false;
    
    /* Byte 0 (D0) - Configuration bits */
    tst2->baudrate = (BaudrateType_t)((data[0] >> 6) & 0x03);      
    tst2->id_type = (IdType_t)((data[0] >> 5) & 0x01);             
    tst2->iac_control = (IacControlType_t)((data[0] >> 2) & 0x03); 
    tst2->range = (RangeType_t)(data[0] & 0x03);                   
    
    /* Byte 0 (D0) - bit 0 e Byte 1 (D1) */
    tst2->three_phase = (data[0] & (1 << 0)) != 0;                 
    
    /* Byte 1 (D1) - More configuration bits */
    tst2->slave = (data[1] & (1 << 7)) != 0;                        
    tst2->evc_model = (EVCModelType_t)((data[1] >> 6) & 0x01);     
    tst2->id_setting = (IDSettingType_t)((data[1] >> 2) & 0x0F);   
    tst2->parallel_ctrl = (data[1] & (1 << 1)) != 0;               
    tst2->air_cooler = (data[1] & (1 << 0)) != 0;                  
    
    /* Byte 2 (D2) - IacmMaxSet (8 bit, resolution 0.2A) */
    tst2->iacm_max_set_A = data[2] * 0.2f;                         
    
    /* Byte 3-4 (D3-D4) - VoutMaxSet (16 bit Big Endian, resolution 0.1V) */
    uint16_t vout_raw = (data[3] << 8) | data[4];                 
    tst2->vout_max_set_V = vout_raw * 0.1f;
    
    /* Byte 5-6 (D5-D6) - IoutMaxSet (16 bit Big Endian, resolution 0.1A) */
    uint16_t iout_raw = (data[5] << 8) | data[6];                  
    tst2->iout_max_set_A = iout_raw * 0.1f;
    
    /* Byte 7 (D7) - Password (8 bit, 0-255) */
    tst2->password = data[7];                                      
    
    return true;
}


/* ============================================================================
 * HELPER FUNCTIONS
 * ============================================================================ */

/**
 * @brief Converte baudrate enum in stringa leggibile
 */
const char* CanBus_GetBaudrateStr(BaudrateType_t baudrate) {
    switch (baudrate) {
        case BAUDRATE_500KBIT: return "500 Kbit/s";
        case BAUDRATE_250KBIT: return "250 Kbit/s";
        case BAUDRATE_125KBIT: return "125 Kbit/s";
        case BAUDRATE_1MBIT:   return "1 Mbit/s";
        default: return "Unknown";
    }
}

/**
 * @brief Converte IAC control enum in stringa leggibile
 */
const char* CanBus_GetIacControlStr(IacControlType_t iac_control) {
    switch (iac_control) {
        case IAC_NOT_CONTROLLED: return "Not controlled (HW set)";
        case IAC_SAEJ1772:       return "SAE J1772 Enabled";
        case IAC_EN61851:        return "EN61851 Enabled";
        case IAC_ID618:          return "AC current controlled by ID618";
        default: return "Unknown";
    }
}

/**
 * @brief Converte range enum in stringa leggibile
 */
const char* CanBus_GetRangeStr(RangeType_t range) {
    switch (range) {
        case RANGE_R4_EVO_USERS: return "R4 (EVO Users Manual)";
        case RANGE_R3:           return "R3";
        case RANGE_R2:           return "R2";
        case RANGE_R1:           return "R1";
        default: return "Unknown";
    }
}

/**
 * @brief Converte EVC model enum in stringa leggibile
 */
const char* CanBus_GetEVCModelStr(EVCModelType_t model) {
    switch (model) {
        case EVC_EVO11K: return "EVO11K (liquid)";
        case EVC_EVO22K: return "EVO22K (air)";
        default: return "Unknown";
    }
}


/* ============================================================================
 * DEBUG FUNCTIONS
 * ============================================================================ */

/**
 * @brief Stampa pacchetto TST2 decodificato
 */
void CanBus_Debug_PrintTst2(const uint8_t data[8]) {
    CanPacket_Tst2_t tst2;
    CanBus_DecodePacket_Tst2(data, &tst2);
    
    printf("\n\r========================================\n");
    printf("TST2 Packet Decoded (Charger Configuration):\n");
    printf("========================================\n");
    
    printf("  CAN Data (HEX): [");
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n\n");
    
    printf("  === Communication Settings ===\n");
    printf("  Baudrate: %s\n", CanBus_GetBaudrateStr(tst2.baudrate));
    printf("  ID Type: %s\n", tst2.id_type == ID_STANDARD_11BIT ? "Standard 11bit" : "Extended 29bit");
    
    printf("\n  === Current Control ===\n");
    printf("  AC Current Control: %s\n", CanBus_GetIacControlStr(tst2.iac_control));
    printf("  Max AC Input Current: %.1f A\n", tst2.iacm_max_set_A);
    
    printf("\n  === Voltage/Current Limits ===\n");
    printf("  Range: %s\n", CanBus_GetRangeStr(tst2.range));
    printf("  Max DC Output Voltage: %.1f V\n", tst2.vout_max_set_V);
    printf("  Max DC Output Current: %.1f A\n", tst2.iout_max_set_A);
    
    printf("\n  === Charger Configuration ===\n");
    printf("  Model: %s\n", CanBus_GetEVCModelStr(tst2.evc_model));
    printf("  Three-Phase: %s\n", tst2.three_phase ? "Yes (3-phase)" : "No (Y Grid)");
    printf("  Cooling: %s\n", tst2.air_cooler ? "Air (EVO11KA)" : "Liquid (EVO11KL)");
    
    printf("\n  === Parallel Operation ===\n");
    printf("  Slave Mode: %s\n", tst2.slave ? "Yes (slave)" : "No (master/single)");
    printf("  Parallel Control: %s\n", tst2.parallel_ctrl ? "Yes (same CAN cmd)" : "No");
    printf("  ID Setting: %s\n", tst2.id_setting == ID_SINGLE_CHARGER ? "Single charger" : "ID 1-16");
    
    printf("\n  === Security ===\n");
    printf("  Password: 0x%02X (%u)\n", tst2.password, tst2.password);
    if (tst2.password == 0xA5) {
        printf("  (Factory default password)\n");
    }
    
    printf("========================================\n");
}


/* ============================================================================
 * EXAMPLES
 * ============================================================================ */

/**
 * ESEMPIO 1: Decodifica TST2 - Charger Configuration
 * 
 * Configurazione esempio:
 * - Baudrate: 500 Kbit/s
 * - ID Type: Standard 11bit
 * - IAC Control: SAE J1772
 * - Range: R4
 * - Single phase (Y Grid)
 * - Not slave
 * - EVO22K (air cooled)
 * - Single charger
 * - Max AC: 32A
 * - Max DC Voltage: 400V
 * - Max DC Current: 100A
 * - Password: 0xA5 (factory)
 */
void Example_DecodeTst2_Default(void) {
    /* Esempio con configurazione default */
    uint8_t tst2_data[8] = {
        0x04,  /* D0: Baudrate=500k, IDType=Std, IacControl=SAE J1772, Range=R4 */
        0x00,  /* D1: Single phase, not slave, EVO22K, single charger */
        0xA0,  /* D2: IacmMaxSet = 32A (160 * 0.2 = 32) */
        0x0F,  /* D3: VoutMaxSet MSB */
        0xA0,  /* D4: VoutMaxSet LSB (4000 * 0.1 = 400V) */
        0x03,  /* D5: IoutMaxSet MSB */
        0xE8,  /* D6: IoutMaxSet LSB (1000 * 0.1 = 100A) */
        0xA5   /* D7: Password (factory default) */
    };
    
    printf("\n\r=== DECODE TST2 EXAMPLE - Default Configuration ===\n");
    CanBus_Debug_PrintTst2(tst2_data);
}

/**
 * ESEMPIO 2: Decodifica TST2 - Three Phase Configuration
 */
void Example_DecodeTst2_ThreePhase(void) {
    /* Configurazione three-phase con EN61851 */
    uint8_t tst2_data[8] = {
        0x08,  /* D0: Baudrate=500k, IDType=Std, IacControl=EN61851, Range=R4 */
        0x01,  /* D1: Three phase, not slave, EVO22K, single charger */
        0xC8,  /* D2: IacmMaxSet = 40A (200 * 0.2 = 40) */
        0x0F,  /* D3: VoutMaxSet MSB */
        0xA0,  /* D4: VoutMaxSet LSB (4000 * 0.1 = 400V) */
        0x05,  /* D5: IoutMaxSet MSB */
        0xDC,  /* D6: IoutMaxSet LSB (1500 * 0.1 = 150A) */
        0xA5   /* D7: Password */
    };
    
    printf("\n\r=== DECODE TST2 EXAMPLE - Three Phase Configuration ===\n");
    CanBus_Debug_PrintTst2(tst2_data);
}


int main(void) {
    printf("\n\r========================================\n");
    printf("  EVO Charger - CAN Bus Level 4 Test\n");
    printf("  (SETUP/Configuration Messages)\n");
    printf("========================================\n");
    
    Example_DecodeTst2_Default();
    printf("\n\r###########################\n");
    
    Example_DecodeTst2_ThreePhase();
    
    return 0;
}
