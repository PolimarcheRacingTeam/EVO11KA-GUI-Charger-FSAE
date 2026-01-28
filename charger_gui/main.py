#!/usr/bin/env python3

import sys, os

import serial
from PyQt6.QtWidgets import (QApplication, QMainWindow, QTabWidget, QWidget,
                              QVBoxLayout, QHBoxLayout, QPushButton, QComboBox,
                              QLabel, QStatusBar, QMenuBar, QMenu, QMessageBox,
                              QDialog, QDialogButtonBox, QFormLayout, QSpinBox,
                              QCheckBox, QDoubleSpinBox)
from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtGui import QAction, QIcon
from .tabs import Level1Tab, Level2Tab, Level3Tab, Level4Tab
from .serial_handler import SerialHandler, SerialMessage, list_serial_ports
from .can_decoder import CANDecoder


class ControlDialog(QDialog):
    """Dialog per inviare messaggi di controllo al charger"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Send Control Message (CTL)")
        self.setup_ui()
    
    def setup_ui(self):
        layout = QFormLayout()
        
        # Enable checkboxes
        self.can_enable_cb = QCheckBox()
        self.can_enable_cb.setChecked(True)
        layout.addRow("CAN Enable:", self.can_enable_cb)
        
        self.led3_enable_cb = QCheckBox()
        layout.addRow("LED3 Enable:", self.led3_enable_cb)
        
        # Current/Voltage spinboxes
        self.iac_max_spin = QDoubleSpinBox()
        self.iac_max_spin.setRange(0, 500)
        self.iac_max_spin.setValue(32.0)
        self.iac_max_spin.setSuffix(" A")
        layout.addRow("Max AC Current:", self.iac_max_spin)
        
        self.vout_max_spin = QDoubleSpinBox()
        self.vout_max_spin.setRange(0, 10000)
        self.vout_max_spin.setValue(400.0)
        self.vout_max_spin.setSuffix(" V")
        layout.addRow("Max Output Voltage:", self.vout_max_spin)
        
        self.iout_max_spin = QDoubleSpinBox()
        self.iout_max_spin.setRange(0, 1500)
        self.iout_max_spin.setValue(100.0)
        self.iout_max_spin.setSuffix(" A")
        layout.addRow("Max Output Current:", self.iout_max_spin)
        
        # Buttons
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | 
            QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        
        layout.addRow(buttons)
        self.setLayout(layout)
    
    def get_values(self):
        return {
            'can_enable': self.can_enable_cb.isChecked(),
            'led3_enable': self.led3_enable_cb.isChecked(),
            'iac_max_A': self.iac_max_spin.value(),
            'vout_max_V': self.vout_max_spin.value(),
            'iout_max_A': self.iout_max_spin.value()
        }


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EVO Charger CAN Bus Monitor")
        self.setGeometry(100, 100, 1200, 800)

        # Serial handler
        self.serial_handler = SerialHandler()
        self.serial_handler.message_received.connect(self.on_message_received)
        self.serial_handler.connection_status.connect(self.on_connection_status)
        self.serial_handler.error_occurred.connect(self.on_error)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        icon_path = os.path.join(script_dir, "logoGUI.ico")
        self.setWindowIcon(QIcon(icon_path))

        self.setup_ui()
        self.refresh_ports()

    def setup_ui(self):
        """Setup interfaccia utente"""
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Toolbar
        toolbar_layout = QHBoxLayout()

        # Port selection
        toolbar_layout.addWidget(QLabel("Serial Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        toolbar_layout.addWidget(self.port_combo)

        # Baudrate selection
        toolbar_layout.addWidget(QLabel("Baudrate:"))
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.setMinimumWidth(150)
        self.baudrate_combo.setEditable(True)
        self.baudrate_combo.addItems([str(b) for b in serial.Serial.BAUDRATES])
        self.baudrate_combo.setCurrentIndex(16)         #imposta 115200 predefinito
        toolbar_layout.addWidget(self.baudrate_combo)

        # Refresh button
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        toolbar_layout.addWidget(self.refresh_btn)

        # Connect/Disconnect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        toolbar_layout.addWidget(self.connect_btn)

        toolbar_layout.addStretch()
        main_layout.addLayout(toolbar_layout)

        # Tab Widget
        self.tab_widget = QTabWidget()

        # Create tabs
        self.level1_tab = Level1Tab()
        self.level2_tab = Level2Tab()
        self.level3_tab = Level3Tab()
        self.level4_tab = Level4Tab()

        self.tab_widget.addTab(self.level1_tab, "Level 1 - Control & RT Diagnostic")
        self.tab_widget.addTab(self.level2_tab, "Level 2 - Faults & Info")
        self.tab_widget.addTab(self.level3_tab, "Level 3 - Service Messages")
        self.tab_widget.addTab(self.level4_tab, "Level 4 - Configuration")

        main_layout.addWidget(self.tab_widget)

        # Status Bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready - Not Connected")

        # Menu Bar
        self.create_menu_bar()

    def create_menu_bar(self):
        """Create menu bar"""
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("File")

        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # Tools menu
        tools_menu = menubar.addMenu("Tools")

        clear_action = QAction("Clear All Data", self)
        clear_action.triggered.connect(self.clear_all_data)
        tools_menu.addAction(clear_action)

        # Help menu
        help_menu = menubar.addMenu("Help")

        about_action = QAction("About", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

    def refresh_ports(self):
        """Aggiorna elenco porte seriali (COM) disponibili/aperte"""
        self.port_combo.clear()
        ports = list_serial_ports()
        if ports:
            self.port_combo.addItems(ports)
        else:
            self.port_combo.addItem("No ports found")

    def toggle_connection(self):
        """Apre/Chiude la connessione con la porta seriale selezionata"""
        if not self.serial_handler.running:
            # Connect
            port = self.port_combo.currentText()
            if port == "No ports found":
                QMessageBox.warning(self, "Error", "No serial port selected")
                return

            self.serial_handler.set_port(port, int(self.baudrate_combo.currentText()))
            if self.serial_handler.connect():
                self.serial_handler.start()
                self.connect_btn.setText("Disconnect")
                self.connect_btn.setStyleSheet("background-color: #f44336; color: white;")
                self.port_combo.setEnabled(False)
                self.refresh_btn.setEnabled(False)

        else:
            # Disconnect
            self.serial_handler.stop()
            self.connect_btn.setText("Connect")
            self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white;")
            self.port_combo.setEnabled(True)
            self.refresh_btn.setEnabled(True)


    @pyqtSlot(SerialMessage)
    def on_message_received(self, msg: SerialMessage):
        """Handle ha ricevuto un messaggio CAN"""

        decoded = CANDecoder.decode_message(msg.can_id, msg.data)

        if decoded is None:
            return

        if msg.can_id == CANDecoder.CAN_ID_CTL:
            self.level1_tab.update_ctl(decoded,msg.can_id, msg.data)
        elif msg.can_id == CANDecoder.CAN_ID_ACT1:
            self.level1_tab.update_act1(decoded, msg.can_id, msg.data)
        elif msg.can_id == CANDecoder.CAN_ID_STAT:
            self.level1_tab.update_stat(decoded, msg.can_id, msg.data)
        elif msg.can_id == CANDecoder.CAN_ID_ACT2:
            self.level1_tab.update_act2(decoded, msg.can_id, msg.data)
        elif msg.can_id == CANDecoder.CAN_ID_TST1:
            self.level1_tab.update_tst1(decoded, msg.can_id, msg.data)
        elif msg.can_id in [CANDecoder.CAN_ID_FLTA, CANDecoder.CAN_ID_FLTP]:
            self.level2_tab.update_fault(decoded, msg.can_id, msg.data)
        elif msg.can_id == CANDecoder.CAN_ID_SW:
            self.level2_tab.update_software(decoded, msg.can_id, msg.data)
        elif msg.can_id == CANDecoder.CAN_ID_SN:
            self.level2_tab.update_serial(decoded, msg.can_id, msg.data)
        elif msg.can_id == CANDecoder.CAN_ID_ACT3:
            self.level3_tab.update_act3(decoded, msg.can_id, msg.data)
        elif msg.can_id == CANDecoder.CAN_ID_TEMP:
            self.level3_tab.update_temp(decoded, msg.can_id, msg.data)
        elif msg.can_id == CANDecoder.CAN_ID_STST1:
            self.level3_tab.update_stst1(decoded, msg.can_id, msg.data)
        elif msg.can_id == CANDecoder.CAN_ID_ACT4:
            self.level3_tab.update_act4(decoded, msg.can_id, msg.data)
        elif msg.can_id == CANDecoder.CAN_ID_TST2:
            self.level4_tab.update_tst2(decoded, msg.can_id, msg.data)

        # Aggiorna status bar
        msg_name = CANDecoder.get_message_name(msg.can_id)
        self.status_bar.showMessage(f"Last message: {msg_name} ({msg.direction})")

    @pyqtSlot(bool, str)
    def on_connection_status(self, connected: bool, message: str):
        """Handle.connection_status è cambiato"""
        if connected:
            self.status_bar.showMessage(f"Connected: {message}")
        else:
            self.status_bar.showMessage(f"Not Connected: {message}")

    @pyqtSlot(str)
    def on_error(self, error_msg: str):
        """Handle messaggio di errore"""
        self.status_bar.showMessage(f"ERROR: {error_msg}")
        QMessageBox.warning(self, "Communication Error", error_msg)

    def clear_all_data(self):
        # This would require adding clear() methods to all tabs
        QMessageBox.information(self, "Clear Data",
                                "Data clearing not yet implemented.\nRestart the application to clear all data.")

    def show_about(self):
        QMessageBox.about(self, "About EVO Charger Monitor/Debug",
                          "<h3>EVO Charger CAN Bus Monitor</h3>"
                          "<p>Version 1.0</p>"
                          "<p>Monitor and control EVO Charger via CAN Bus over serial connection.</p>"
                          "<p>Supports 4 levels of CAN messages:</p>"
                          "<ul>"    #elenco
                          "<li>Level 1: Control & Real-time Diagnostic</li>"
                          "<li>Level 2: Faults & Information</li>"
                          "<li>Level 3: Service Messages</li>"
                          "<li>Level 4: Configuration</li>"
                          "</ul>"
                          )

    def closeEvent(self, event):
        """Handle window close event"""
        if self.serial_handler.running:
            self.serial_handler.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
