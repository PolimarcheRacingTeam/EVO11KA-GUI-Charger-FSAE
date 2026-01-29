import re
from typing import Optional, List
from PyQt6.QtCore import QThread, pyqtSignal
import serial
import serial.tools.list_ports


class SerialMessage:
    def __init__(self, can_id: int, data: List[int], direction: str = "RX", raw: str = ""):
        self.direction = direction  # "RX" o "TX"
        self.can_id = can_id
        self.data = data
        self.raw = raw if raw else self._format_raw()
    
    def _format_raw(self):
        data_hex = ' '.join(f'{b:02X}' for b in self.data)
        return f"CanBus {self.direction} 0x{self.can_id:03X} {data_hex}"
    
    def __repr__(self):
        data_hex = ' '.join(f'{b:02X}' for b in self.data)
        return f"CAN {self.direction} ID=0x{self.can_id:03X} Data=[{data_hex}]"


class SerialHandler(QThread):
    """Thread to handle serial communication"""
    
    # PyQt Signals
    message_received = pyqtSignal(SerialMessage)
    connection_status = pyqtSignal(bool, str)  # (connected, message)
    error_occurred = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.serial_port: Optional[serial.Serial] = None
        self.running = False
        self.port_name = ""
        self.baudrate = 115200
        
        # Pattern espressione regolare: "CanBus Rx/Tx {ID} {Contenuto}"
        # Esempi:
        # "CanBus Rx 0x618 12 34 56 78 9A BC DE F0"
        # "CanBus Tx 0x610 AA BB CC DD"
        self.pattern = re.compile(
            r'CanBus\s+(Rx|Tx)\s+(?:0x)?([0-9A-Fa-f]+)\s+((?:[0-9A-Fa-f]{2}\s*)+)',
            re.IGNORECASE
        )
    
    def set_port(self, port_name: str, baudrate: int = 115200):
        """Set serial port and baudrate"""
        self.port_name = port_name
        self.baudrate = baudrate
    
    def connect(self) -> bool:
        """Connect to serial port"""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=0.1
            )
            
            self.connection_status.emit(True, f"Connesso a {self.port_name}")
            return True
            
        except serial.SerialException as e:
            self.connection_status.emit(False, f"Errore connessione: {e}")
            self.error_occurred.emit(str(e))
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.connection_status.emit(False, "Disconnesso")
    
    def send_message(self, message: str):
        """Invia un messaggio sulla seriale"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{message}\n".encode())
            except serial.SerialException as e:
                self.error_occurred.emit(f"Errore invio: {e}")
    
    def parse_message(self, line: str) -> Optional[SerialMessage]:
        """
        Verify if it's an expected RE
        Parse a line received from serial
        
        Expected format: "CanBus Rx/Tx {ID} {Content}"
        Examples:
            "CanBus Rx 0x618 12 34 56 78 9A BC DE F0"
            "CanBus Tx 610 AA BB CC DD EE FF"
        """
        match = self.pattern.match(line.strip())
        if not match:
            return None
        direction = match.group(1)  # "Rx" o "Tx"
        can_id_str = match.group(2)
        data_str = match.group(3).strip()
        
        try:
            # Convert ID (can be hex or decimal)
            can_id = int(can_id_str, 16)
            
            # Convert data (space separated)
            data_bytes = [int(b, 16) for b in data_str.split()]
            
            return SerialMessage(direction, can_id, data_bytes, line)
            
        except ValueError as e:
            self.error_occurred.emit(f"Errore parsing: {e} - Riga: {line}")
            return None
    
    def run(self):
        """Main thread for serial reading"""
        self.running = True
        buffer = ""
        
        while self.running:
            if not self.serial_port or not self.serial_port.is_open:
                self.msleep(50)    #delay
                continue
            
            try:
                # Read from serial
                if self.serial_port.in_waiting > 0:         #in buffer
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer += data.decode('utf-8', errors='ignore')
                    
                    # Process complete lines separated by newline
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            # Parse the message
                            msg = self.parse_message(line)
                            if msg:
                                self.message_received.emit(msg)
                
                self.msleep(10)  # Small pause to avoid CPU overload
                
            except serial.SerialException as e:
                self.error_occurred.emit(f"Errore lettura: {e}")
                self.disconnect()
                break
            except Exception as e:
                self.error_occurred.emit(f"Errore inaspettato: {e}")
    
    def stop(self):
        """Stop the thread"""
        self.running = False
        self.disconnect()
        self.wait()


def list_serial_ports() -> List[str]:
    """Return list of available (open) serial ports"""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]