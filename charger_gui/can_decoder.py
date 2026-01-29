from dataclasses import dataclass
from enum import Enum
from typing import List, Optional


# ============================================================================
# LEVEL 2 - Enums and Types
# ============================================================================

class RequestType(Enum):
    """Request Types for Level 2"""
    FAULT_INACTIVE = 0x1C
    FAULT_ACTIVE = 0x1D
    SOFTWARE = 0x1E
    SERIAL_NUMBER = 0x1F


class FailureLevel(Enum):
    """Failure Level for Faults"""
    WARNING = 1      # Warning - charger works normally but de-rated
    SOFT = 10        # Soft Failure - charger stops, restarts when fault clears
    HARD = 11        # Hard Failure - charger stops, needs AC disconnect/reconnect


class FrameType(Enum):
    """Frame Type for Fault messages"""
    SINGLE = 1       # Single frame (only one fault)
    MULTI = 2        # Multi frame (more than one fault)


class FaultCode(Enum):
    """Fault Code Definitions (from Table 4.6)"""
    BULK1_VOLTAGE = 0xA0
    BULK2_VOLTAGE = 0xA1
    BULK3_VOLTAGE = 0xA2
    BULK_ERROR = 0xA3
    CAN_REGISTERS = 0xA4
    CAN_COMMAND = 0xA5
    TEMP_LOW = 0xA6
    TEMP_DERATING = 0xA7
    TEMP_HIGH = 0xA8
    TEMP_FAILED = 0xA9
    INPUT_CURRENT_MAX = 0xAA
    HVIL_INTERLOCK = 0xAB
    LOGIC_TEMP = 0xAC
    OUTPUT_OVERVOLT = 0xAD


# ============================================================================
# LEVEL 4 - Enums
# ============================================================================

class BaudrateType(Enum):
    """Baudrate definitions"""
    BAUDRATE_500KBIT = 0
    BAUDRATE_250KBIT = 1
    BAUDRATE_125KBIT = 2
    BAUDRATE_1MBIT = 3


class IdType(Enum):
    """ID Type definitions"""
    STANDARD_11BIT = 0
    EXTENDED_29BIT = 1


class IacControlType(Enum):
    """AC Current Control definitions"""
    NOT_CONTROLLED = 0
    SAEJ1772 = 1
    EN61851 = 2
    ID618 = 3


class RangeType(Enum):
    """Range definitions"""
    R4_EVO_USERS = 0
    R3 = 1
    R2 = 2
    R1 = 3


class EVCModelType(Enum):
    """EVC Model definitions"""
    EVO11K = 0  # liquid cooled
    EVO22K = 1  # air cooled


class IDSettingType(Enum):
    """ID Setting definitions"""
    SINGLE_CHARGER = 0
    RANGE_1_16 = 1


# ============================================================================
# Level 1 - Data Structures
# ============================================================================

@dataclass
class CtlPacket:
    """CTL Packet (BMS → Charger) - ID 0x618"""
    can_enable: bool
    led3_enable: bool
    iac_max_A: float
    vout_max_V: float
    iout_max_A: float


@dataclass
class StatPacket:
    """STAT Packet (Charger → BMS) - ID 0x610"""
    power_enable: bool
    error_latch: bool
    warn_limit: bool
    lim_temp: bool
    warning_hv: bool
    bulks: bool


@dataclass
class Act1Packet:
    """ACT1 Packet (Charger → BMS) - ID 0x611"""
    iac_A: float
    temp_C: float
    vout_V: float
    iout_A: float


@dataclass
class Act2Packet:
    """ACT2 Packet (Charger → BMS) - ID 0x614"""
    temp_loglv_C: float
    ac_power_kW: float
    prox_limit_A: float
    pilot_limit_A: float


@dataclass
class Tst1Packet:
    """TST1 Packet (Charger → BMS) - ID 0x615"""
    # Byte 0
    ack: bool
    pr_compl: bool
    pwr_ok: bool
    vout_ok: bool
    neutral: bool
    led3: bool
    led618: bool
    # Byte 1
    ovp: bool
    conn_open: bool
    ther_fail: bool
    rx618_fail: bool
    # Byte 2
    bulk1_fail: bool
    bulk2_fail: bool
    bulk3_fail: bool
    pump_on: bool
    fan_on: bool
    hv_rx_fail: bool
    cooling_fail: bool
    rx619_fail: bool
    # Byte 3
    neutro1: bool
    neutro2: bool
    three_phase: bool
    iac_fail: bool
    ignition: bool
    lv_battery_np: bool
    # Byte 4
    prox_ok: bool
    pilot_ok: bool
    s2_ok: bool
    # Byte 6
    cnt_hours: int


# ============================================================================
# Level 2 - Data Structures
# ============================================================================

@dataclass
class ReqPacket:
    """REQ Packet (BMS → Charger) - ID 0x61B"""
    enable: bool
    id_requested: int


@dataclass
class FaultPacket:
    """Pacchetto Fault (Active or Passive) - ID 0x61D or 0x61C"""
    frame_type: FrameType
    total_errors: int
    frame_number: int
    fault_code: int
    occurrence: int
    failure_level: FailureLevel
    first_time_h: int
    last_time_h: int


@dataclass
class SoftwarePacket:
    """Pacchetto Software Version - ID 0x61E"""
    version: str


@dataclass
class SerialNumberPacket:
    """Pacchetto Serial Number - ID 0x61F"""
    serial: str


# ============================================================================
# Level 3 - Data Structures
# ============================================================================

@dataclass
class Act3Packet:
    """ACT3 Packet (Charger → BMS) - ID 0x712"""
    fan_voltage_V: float
    iacm1_A: float
    iacm2_A: float
    iacm3_A: float


@dataclass
class TempPacket:
    """TEMP Packet (Charger → BMS) - ID 0x713"""
    temp_loghv_C: float
    temp_power1_C: float
    temp_power2_C: float
    temp_power3_C: float


@dataclass
class Stst1Packet:
    """STST1 Packet (Charger → BMS) - ID 0x715"""
    pfc_enable: bool
    log_temp_high: bool
    log_temp_low: bool
    uvlo_log: bool
    ther_low_fail: bool
    rx618_fail: bool
    bulk1_fail: bool
    bulk2_fail: bool
    bulk3_fail: bool
    cooling_fail1: bool
    cooling_fail2: bool
    cooling_fail3: bool
    uvlo_log_lv: bool
    bat_over: bool
    bat_under: bool


@dataclass
class Act4Packet:
    """ACT4 Packet (Charger → BMS) - ID 0x717"""
    temp_logfan_C: float
    iout1_raw: int
    iout2_raw: int
    iout3_raw: int


# ============================================================================
# Level 4 - Data Structures
# ============================================================================

@dataclass
class Tst2Packet:
    """TST2 Packet (Charger → BMS) - ID 0x716"""
    baudrate: BaudrateType
    id_type: IdType
    iac_control: IacControlType
    range: RangeType
    three_phase: bool
    slave: bool
    evc_model: EVCModelType
    id_setting: IDSettingType
    air_cooler: bool
    parallel_ctrl: bool
    iacm_max_set_A: float
    vout_max_set_V: float
    iout_max_set_A: float
    password: int


# ============================================================================
# DECODER CLASS
# ============================================================================

class CANDecoder:
    """Decoder per i messaggi CAN LIVELLI: 1, 2, 3 ,4 """
    
    # CAN IDs

    CAN_ID_CTL = 0x618          #lv1
    CAN_ID_STAT = 0x610         #lv1
    CAN_ID_ACT1 = 0x611         #lv1
    CAN_ID_ACT2 = 0x614         #lv1
    CAN_ID_TST1 = 0x615         #lv1
    CAN_ID_REQ = 0x61B          #lv2
    CAN_ID_FLTP = 0x61C         #lv2
    CAN_ID_FLTA = 0x61D         #lv2
    CAN_ID_SW = 0x61E           #lv2
    CAN_ID_SN = 0x61F           #lv2
    CAN_ID_TST2 = 0x616         #lv4
    CAN_ID_ACT3 = 0x712         #lv3
    CAN_ID_TEMP = 0x713         #lv3
    CAN_ID_ACT4 = 0x714         #lv3
    CAN_ID_STST1 = 0x715        #lv3
    
    # ========================================================================
    # LEVEL 1 - Decoders
    # ========================================================================
    
    @staticmethod
    def decode_ctl(data: List[int]) -> CtlPacket:
        """Decode CTL packet - ID 0x618 (BMS → Charger)"""
        can_enable = bool(data[0] & 0x80)
        led3_enable = bool(data[0] & 0x40)
        iac_max_A = data[1] * 0.1
        vout_max_V = ((data[2] << 8) | data[3]) * 0.1
        iout_max_A = ((data[4] << 8) | data[5]) * 0.1
        
        return CtlPacket(can_enable, led3_enable, iac_max_A, vout_max_V, iout_max_A)
    
    @staticmethod
    def decode_stat(data: List[int]) -> StatPacket:
        """Decode STAT packet - ID 0x610 (Charger → BMS)"""
        return StatPacket(
            power_enable=bool(data[0] & (1 << 7)),
            error_latch=bool(data[0] & (1 << 6)),
            warn_limit=bool(data[0] & (1 << 5)),
            lim_temp=bool(data[0] & (1 << 3)),
            warning_hv=bool(data[0] & (1 << 1)),
            bulks=bool(data[0] & (1 << 0))
        )
    
    @staticmethod
    def decode_act1(data: List[int]) -> Act1Packet:
        """Decode ACT1 packet - ID 0x611 (Charger → BMS)"""
        iac_raw = (data[0] << 8) | data[1]
        temp_raw = (data[2] << 8) | data[3]
        vout_raw = (data[4] << 8) | data[5]
        iout_raw = (data[6] << 8) | data[7]
        
        return Act1Packet(
            iac_A=iac_raw * 0.1,
            temp_C=(temp_raw * 0.005188) - 40.0,
            vout_V=vout_raw * 0.1,
            iout_A=iout_raw * 0.1
        )
    
    @staticmethod
    def decode_act2(data: List[int]) -> Act2Packet:
        """Decode ACT2 packet - ID 0x614 (Charger → BMS)"""
        temp_raw = (data[0] << 8) | data[1]
        power_raw = (data[2] << 8) | data[3]
        prox_raw = (data[4] << 8) | data[5]
        pilot_raw = (data[6] << 8) | data[7]
        
        return Act2Packet(
            temp_loglv_C=(temp_raw * 0.005188) - 40.0,
            ac_power_kW=power_raw * 0.05,
            prox_limit_A=prox_raw * 0.1,
            pilot_limit_A=pilot_raw * 0.1
        )
    
    @staticmethod
    def decode_tst1(data: List[int]) -> Tst1Packet:
        """Decode TST1 packet - ID 0x615 (Charger → BMS)"""
        return Tst1Packet(
            # Byte 0
            ack=bool(data[0] & (1 << 7)),
            pr_compl=bool(data[0] & (1 << 6)),
            pwr_ok=bool(data[0] & (1 << 5)),
            vout_ok=bool(data[0] & (1 << 4)),
            neutral=bool(data[0] & (1 << 3)),
            led3=bool(data[0] & (1 << 2)),
            led618=bool(data[0] & (1 << 1)),
            # Byte 1
            ovp=bool(data[1] & (1 << 7)),
            conn_open=bool(data[1] & (1 << 6)),
            ther_fail=bool(data[1] & (1 << 2)),
            rx618_fail=bool(data[1] & (1 << 0)),
            # Byte 2
            bulk1_fail=bool(data[2] & (1 << 7)),
            bulk2_fail=bool(data[2] & (1 << 6)),
            bulk3_fail=bool(data[2] & (1 << 5)),
            pump_on=bool(data[2] & (1 << 4)),
            fan_on=bool(data[2] & (1 << 3)),
            hv_rx_fail=bool(data[2] & (1 << 2)),
            cooling_fail=bool(data[2] & (1 << 1)),
            rx619_fail=bool(data[2] & (1 << 0)),
            # Byte 3
            neutro1=bool(data[3] & (1 << 7)),
            neutro2=bool(data[3] & (1 << 6)),
            three_phase=bool(data[3] & (1 << 5)),
            iac_fail=bool(data[3] & (1 << 2)),
            ignition=bool(data[3] & (1 << 1)),
            lv_battery_np=bool(data[3] & (1 << 0)),
            # Byte 4
            prox_ok=bool(data[4] & (1 << 7)),
            pilot_ok=bool(data[4] & (1 << 5)),
            s2_ok=bool(data[4] & (1<<3)),
            #Byte6
            cnt_hours= int((data[6]<<8) | data[7])
        )
    
    # ========================================================================
    # LEVEL 2 - Decoders
    # ========================================================================
    
    @staticmethod
    def decode_req(data: List[int]) -> ReqPacket:
        """Decode REQ packet - ID 0x61B (BMS → Charger)"""
        enable = bool(data[0] & 0x80)
        id_requested = (data[2] << 8) | data[3]
        return ReqPacket(enable, id_requested)
    
    @staticmethod
    def decode_fault(data: List[int]) -> Optional[FaultPacket]:
        """Decode Fault packet - ID 0x61D (Active) or 0x61C (Passive)"""
        # Check for "No Fault Detected" (all bytes from D1 to D7 are 0xFF)
        if all(b == 0xFF for b in data[1:8]):
            return None
        
        frame_type = FrameType((data[0] >> 6) & 0x03)
        total_errors = data[0] & 0x3F
        frame_number = (data[1] >> 2) & 0x3F
        fault_code = data[2]
        occurrence = (data[3] >> 2) & 0x3F
        
        level_bits = data[3] & 0x03
        if level_bits == 0x01:
            failure_level = FailureLevel.WARNING
        elif level_bits == 0x02:
            failure_level = FailureLevel.SOFT
        elif level_bits == 0x03:
            failure_level = FailureLevel.HARD
        else:
            failure_level = FailureLevel.WARNING
        
        first_time_h = (data[4] << 8) | data[5]
        last_time_h = (data[6] << 8) | data[7]
        
        return FaultPacket(
            frame_type, total_errors, frame_number, fault_code,
            occurrence, failure_level, first_time_h, last_time_h
        )
    
    @staticmethod
    def decode_software(data: List[int]) -> SoftwarePacket:
        """Decode Software Version packet - ID 0x61E"""
        version = ''.join(chr(b) for b in data[:8])
        return SoftwarePacket(version)
    
    @staticmethod
    def decode_serial_number(data: List[int]) -> SerialNumberPacket:
        """Decode Serial Number packet - ID 0x61F"""
        serial = ''.join(chr(b) for b in data[:8])
        return SerialNumberPacket(serial)
    
    # ========================================================================
    # LEVEL 3 - Decoders
    # ========================================================================
    
    @staticmethod
    def decode_act3(data: List[int]) -> Act3Packet:
        """Decode ACT3 packet - ID 0x712"""
        fan_voltage_V = ((data[0] << 8) | data[1]) * 0.1
        iacm1_A = ((data[2] << 8) | data[3]) * 0.1
        iacm2_A = ((data[4] << 8) | data[5]) * 0.1
        iacm3_A = ((data[6] << 8) | data[7]) * 0.1
        
        return Act3Packet(fan_voltage_V, iacm1_A, iacm2_A, iacm3_A)
    
    @staticmethod
    def decode_temp(data: List[int]) -> TempPacket:
        """Decode TEMP packet - ID 0x713"""
        loghv_raw = (data[0] << 8) | data[1]
        power1_raw = (data[2] << 8) | data[3]
        power2_raw = (data[4] << 8) | data[5]
        power3_raw = (data[6] << 8) | data[7]
        
        return TempPacket(
            temp_loghv_C=(loghv_raw * 0.005188) - 40.0,
            temp_power1_C=(power1_raw * 0.005188) - 40.0,
            temp_power2_C=(power2_raw * 0.005188) - 40.0,
            temp_power3_C=(power3_raw * 0.005188) - 40.0
        )
    
    @staticmethod
    def decode_stst1(data: List[int]) -> Stst1Packet:
        """Decode STST1 packet - ID 0x715"""
        return Stst1Packet(
            pfc_enable=bool(data[0] & (1 << 2)),
            log_temp_high=bool(data[1] & (1 << 5)),
            log_temp_low=bool(data[1] & (1 << 4)),
            uvlo_log=bool(data[1] & (1 << 3)),
            ther_low_fail=bool(data[1] & (1 << 2)),
            rx618_fail=bool(data[1] & (1 << 0)),
            bulk1_fail=bool(data[2] & (1 << 7)),
            bulk2_fail=bool(data[2] & (1 << 6)),
            bulk3_fail=bool(data[2] & (1 << 5)),
            cooling_fail1=bool(data[2] & (1 << 4)),
            cooling_fail2=bool(data[2] & (1 << 3)),
            cooling_fail3=bool(data[2] & (1 << 2)),
            uvlo_log_lv=bool(data[3] & (1 << 3)),
            bat_over=bool(data[3] & (1 << 1)),
            bat_under=bool(data[3] & (1 << 0))
        )
    
    @staticmethod
    def decode_act4(data: List[int]) -> Act4Packet:
        """Decode ACT4 packet - ID 0x714"""
        temp_raw = (data[0] << 8) | data[1]
        iout1_raw = (data[2] << 8) | data[3]
        iout2_raw = (data[4] << 8) | data[5]
        iout3_raw = (data[6] << 8) | data[7]
        
        return Act4Packet(
            temp_logfan_C=(temp_raw * 0.005188) - 40.0,
            iout1_raw=iout1_raw,
            iout2_raw=iout2_raw,
            iout3_raw=iout3_raw
        )
    
    # ========================================================================
    # LEVEL 4 - Decoders
    # ========================================================================
    
    @staticmethod
    def decode_tst2(data: List[int]) -> Tst2Packet:
        """Decode TST2 packet - ID 0x616"""
        baudrate = BaudrateType((data[0] >> 6) & 0x03)
        id_type = IdType((data[0] >> 5) & 0x01)
        iac_control = IacControlType((data[0] >> 2) & 0x03)
        range_val = RangeType(data[0] & 0x03)
        three_phase = bool(data[0] & (1 << 0))
        
        slave = bool(data[1] & (1 << 7))
        evc_model = EVCModelType((data[1] >> 6) & 0x01)
        id_setting = IDSettingType((data[1] >> 2) & 0x0F)
        parallel_ctrl = bool(data[1] & (1 << 1))
        air_cooler = bool(data[1] & (1 << 0))
        
        iacm_max_set_A = data[2] * 0.2
        vout_max_set_V = ((data[3] << 8) | data[4]) * 0.1
        iout_max_set_A = ((data[5] << 8) | data[6]) * 0.1
        password = data[7]
        
        return Tst2Packet(
            baudrate, id_type, iac_control, range_val, three_phase,
            slave, evc_model, id_setting, air_cooler, parallel_ctrl,
            iacm_max_set_A, vout_max_set_V, iout_max_set_A, password
        )
    
    # ========================================================================
    # Main Decode Function
    # ========================================================================
    
    @classmethod
    def decode_message(cls, can_id: int, data: List[int]):
        """Decode CAN message based on ID"""
        decoders = {
            cls.CAN_ID_CTL: cls.decode_ctl,
            cls.CAN_ID_STAT: cls.decode_stat,
            cls.CAN_ID_ACT1: cls.decode_act1,
            cls.CAN_ID_ACT2: cls.decode_act2,
            cls.CAN_ID_TST1: cls.decode_tst1,
            cls.CAN_ID_REQ: cls.decode_req,
            cls.CAN_ID_FLTP: cls.decode_fault,
            cls.CAN_ID_FLTA: cls.decode_fault,
            cls.CAN_ID_SW: cls.decode_software,
            cls.CAN_ID_SN: cls.decode_serial_number,
            cls.CAN_ID_TST2: cls.decode_tst2,
            cls.CAN_ID_ACT3: cls.decode_act3,
            cls.CAN_ID_TEMP: cls.decode_temp,
            cls.CAN_ID_ACT4: cls.decode_act4,
            cls.CAN_ID_STST1: cls.decode_stst1,
        }
        
        decoder = decoders.get(can_id)
        if decoder:
            return decoder(data)
        return None
    
    @classmethod
    def get_message_name(cls, can_id: int) -> str:
        """Get message name from CAN ID"""
        names = {
            cls.CAN_ID_CTL: "CTL (Control)",
            cls.CAN_ID_STAT: "STAT (Status)",
            cls.CAN_ID_ACT1: "ACT1 (Actual Values 1)",
            cls.CAN_ID_ACT2: "ACT2 (Actual Values 2)",
            cls.CAN_ID_TST1: "TST1 (Test/Diagnostic)",
            cls.CAN_ID_REQ: "REQ (Request)",
            cls.CAN_ID_FLTP: "FLTP (Fault Passive)",
            cls.CAN_ID_FLTA: "FLTA (Fault Active)",
            cls.CAN_ID_SW: "SW (Software Version)",
            cls.CAN_ID_SN: "SN (Serial Number)",
            cls.CAN_ID_TST2: "TST2 (Configuration)",
            cls.CAN_ID_ACT3: "ACT3 (AC Currents)",
            cls.CAN_ID_TEMP: "TEMP (Temperatures)",
            cls.CAN_ID_ACT4: "ACT4 (Temperature FAN)",
            cls.CAN_ID_STST1: "STST1 (Real Time Diagnostic)",
        }
        return names.get(can_id, f"Unknown (0x{can_id:03X})")
