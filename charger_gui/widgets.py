from PyQt6.QtWidgets import (QWidget, QLabel, QVBoxLayout, QHBoxLayout,
                              QGroupBox, QGridLayout, QFrame)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont, QColor, QPalette
from datetime import datetime


class ParameterDisplay(QWidget):
    """Widget per visualizzare un singolo parametro con label e valore"""
    
    def __init__(self, name: str, unit: str = "", decimals: int = 1):
        super().__init__()
        self.name = name
        self.unit = unit
        self.decimals = decimals
        
        self.setup_ui()
    
    def setup_ui(self):
        layout = QHBoxLayout()
        layout.setContentsMargins(15, 2, 5, 2)
        
        # Label nome
        self.name_label = QLabel(f"{self.name}:")
        self.name_label.setMinimumWidth(150)
        
        # Label valore
        self.value_label = QLabel("---")
        self.value_label.setMinimumWidth(80)
        font = QFont()
        font.setBold(True)
        font.setPointSize(10)
        self.value_label.setFont(font)
        
        layout.addWidget(self.name_label)
        layout.addWidget(self.value_label)
        layout.addStretch()
        
        self.setLayout(layout)
    
    def set_value(self, value: float):
        """Aggiorna il valore visualizzato"""
        if self.unit:
            text = f"{value:.{self.decimals}f} {self.unit}"
        else:
            text = f"{value:.{self.decimals}f}"
        
        self.value_label.setText(text)
    
    def clear(self):
        """Reset del valore"""
        self.value_label.setText("---")
        self.value_label.setStyleSheet("")


class BooleanIndicator(QWidget):
    """Widget per visualizzare uno stato booleano con LED colorato (x flags)"""
    
    def __init__(self, name: str, true_color: str = "green", false_color: str = "gray"):
        super().__init__()
        self.name = name
        self.true_color = true_color
        self.false_color = false_color
        self.state = False
        
        self.setup_ui()
    
    def setup_ui(self):
        layout = QHBoxLayout()
        layout.setContentsMargins(5, 2, 5, 2)
        
        # "LED"
        self.led = QLabel("●")      # ●
        font = QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.led.setFont(font)
        
        # Label nome
        self.name_label = QLabel(self.name)
        
        layout.addWidget(self.led)
        layout.addWidget(self.name_label)
        layout.addStretch()
        
        self.setLayout(layout)
        self.set_state(False)
    
    def set_state(self, state: bool):
        self.state = state
        color = self.true_color if state else self.false_color
        self.led.setStyleSheet(f"color: {color};")
    
    def clear(self):
        self.set_state(False)


class GroupPanel(QGroupBox):
    """Pannello raggruppato per organizzare parametri correlati e dividere pacchetti diversi"""
    
    def __init__(self, title: str):
        super().__init__(title)
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        
        # Stile
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #cccccc;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
    
    def add_widget(self, widget: QWidget):
        self.layout.addWidget(widget)
    
    def add_separator(self):
        """Aggiunge una linea separatrice"""
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        self.layout.addWidget(line)


class MessageInfoPanel(QWidget):
    """Pannello per visualizzare info generali sul messaggio"""
    
    def __init__(self):
        super().__init__()
        self.setup_ui()
    
    def setup_ui(self):
        layout = QGridLayout()
        
        # Timestamp
        self.timestamp_label = QLabel("Last Update: ---")
        font = QFont()
        self.timestamp_label.setFont(font)
        
        # CAN ID
        self.can_id_label = QLabel("CAN ID: ---")

        layout.addWidget(self.timestamp_label, 0, 0)
        layout.addWidget(self.can_id_label, 0, 1)
        
        self.setLayout(layout)
    
    def update_info(self, can_id: int, message_name: str):
        self.timestamp_label.setText(f"Last Update: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        self.can_id_label.setText(f"CAN ID: 0x{can_id:03X} - {message_name}")
    
    def reset(self):
        self.timestamp_label.setText("Last Update: ---")
        self.can_id_label.setText("CAN ID: ---")


class FaultListWidget(QWidget):
    """Widget per visualizzare lista di fault correnti"""
    
    def __init__(self):
        super().__init__()
        self.faults = []
        self.fault_info = []  # Store fault metadata
        self.setup_ui()
    
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # Label titolo
        title = QLabel("Fault History")
        font = QFont()
        font.setBold(True)
        font.setPointSize(11)
        title.setFont(font)
        
        # Fault area
        self.fault_container = QVBoxLayout()
        
        # Label "no faults"
        self.no_fault_label = QLabel("No faults detected")
        self.no_fault_label.setStyleSheet("color: green; font-style: italic;")
        self.fault_container.addWidget(self.no_fault_label)
        
        layout.addWidget(title)
        layout.addLayout(self.fault_container)
        layout.addStretch()
        
        self.setLayout(layout)
    
    def add_fault(self, fault_code: int, fault_name: str, occurrence: int, 
                  failure_level: str, last_time_h: int):

        if self.no_fault_label.parent():
            self.fault_container.removeWidget(self.no_fault_label)
            self.no_fault_label.hide()
        
        # Store fault metadata (check if already exists)
        fault_key = (fault_code, fault_name)
        existing_index = None
        for i, info in enumerate(self.fault_info):
            if (info['code'], info['name']) == fault_key:
                existing_index = i
                break
        
        if existing_index is not None:
            # Update existing fault
            self.fault_info[existing_index].update({
                'occurrence': occurrence,
                'level': failure_level,
                'last_time_h': last_time_h
            })
            # Update widget display
            fault_widget = self.faults[existing_index]
            fault_widget.setTitle(f"Fault 0x{fault_code:02X}: {fault_name}")
            details_label = fault_widget.findChild(QLabel)
            if details_label:
                details_label.setText(
                    f"Occurrences: {occurrence}\n"
                    f"Level: {failure_level}\n"
                    f"Last: {last_time_h}h ago"
                )
        else:
            # Add new fault
            self.fault_info.append({
                'code': fault_code,
                'name': fault_name,
                'occurrence': occurrence,
                'level': failure_level,
                'last_time_h': last_time_h,
                'is_active': 'HARD' in failure_level.upper()  # Assume HARD = active
            })
            
            fault_widget = QGroupBox(f"Fault 0x{fault_code:02X}: {fault_name}")
            fault_layout = QVBoxLayout()

            details = QLabel(
                f"Occurrences: {occurrence}\n"
                f"Level: {failure_level}\n"
                f"Last: {last_time_h}h ago"
            )

            if "HARD" in failure_level.upper():
                fault_widget.setStyleSheet("border: 2px solid red;")
            elif "SOFT" in failure_level.upper():
                fault_widget.setStyleSheet("border: 2px solid orange;")
            else:
                fault_widget.setStyleSheet("border: 2px solid yellow;")
            
            fault_layout.addWidget(details)
            fault_widget.setLayout(fault_layout)
            
            self.fault_container.addWidget(fault_widget)
            self.faults.append(fault_widget)
    
    def clear_faults(self):
        for fault in self.faults:
            self.fault_container.removeWidget(fault)
            fault.deleteLater()
        
        self.faults.clear()
        self.fault_info.clear()
        self.no_fault_label.show()
        self.fault_container.addWidget(self.no_fault_label)
    
    def get_fault_count(self) -> int:
        """Return the total number of faults"""
        return len(self.fault_info)
    
    def get_active_fault_count(self) -> int:
        """Return the number of active (HARD) faults"""
        return sum(1 for fault in self.fault_info if fault.get('is_active', False))


class RawDataDisplay(QWidget):
    """Widget per visualizzare i dati grezzo del messaggio CAN (valore pacchetto)"""
    
    def __init__(self):
        super().__init__()
        self.setup_ui()
    
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # Titolo
        title = QLabel("Raw CAN Data")
        
        # Data label
        self.data_label = QLabel("---")
        self.data_label.setFont(QFont("Courier New", 10))
        self.data_label.setStyleSheet("background-color: #f0f0f0;color: black; padding: 5px;")
        
        layout.addWidget(title)
        layout.addWidget(self.data_label)
        
        self.setLayout(layout)
    
    def update_data(self, data: list):
        hex_str = ' '.join(f'{b:02X}' for b in data)
        self.data_label.setText(f"[{hex_str}]")
    
    def clear(self):
        """Reset dei dati"""
        self.data_label.setText("---")
