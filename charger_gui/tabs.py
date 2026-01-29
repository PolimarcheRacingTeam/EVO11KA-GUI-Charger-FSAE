from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QScrollArea, 
                              QFrame, QPushButton, QLabel, QGridLayout, QMessageBox)
from PyQt6.QtCore import Qt
from .widgets import (ParameterDisplay, BooleanIndicator, GroupPanel, 
                      MessageInfoPanel, FaultListWidget, RawDataDisplay)
from .can_decoder import *


class Level1Tab(QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()
    
    def setup_ui(self):
        main_layout = QVBoxLayout()
        
        # Info banner at the top
        info_banner = QLabel("Level 1 - Real-time Control & Diagnostics | Monitor charger operational status and parameters")
        info_banner.setStyleSheet("""
            QLabel {
                color: #1976D2;
                background-color: #E3F2FD;
                padding: 10px;
                border-radius: 4px;
                font-size: 11px;
                font-weight: bold;
                border-left: 4px solid #1976D2;
            }
        """)
        info_banner.setWordWrap(True)
        main_layout.addWidget(info_banner)
        
        # Scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        layout = QVBoxLayout(scroll_content)

        # CTL Panel - Control Packet (ID 0x618)
        self.ctl_panel = GroupPanel("CTL - Control messages (100ms) - TX")
        self.ctl_info = MessageInfoPanel()
        self.ctl_can = BooleanIndicator("CanBus Enable", "green", "red")
        self.ctl_led3 = BooleanIndicator("LED3 Enable", "green", "gray")
        self.ctl_iac = ParameterDisplay("Max AC Input current", "A",1)
        self.ctl_vout = ParameterDisplay("Max output voltage","V",1)
        self.ctl_iOut = ParameterDisplay("Max output current", "A",1)
        self.ctl_raw = RawDataDisplay()
        self.ctl_panel.add_widget(self.ctl_iac)
        self.ctl_panel.add_widget(self.ctl_vout)
        self.ctl_panel.add_widget(self.ctl_iOut)
        self.ctl_panel.add_separator()
        self.ctl_panel.add_widget(self.ctl_can)
        self.ctl_panel.add_widget(self.ctl_led3)

        # ACT1 Panel - Actual Values 1 (ID 0x611)
        self.act1_panel = GroupPanel("ACT1 - Real-time Values (100ms) - RX")
        self.act1_info = MessageInfoPanel()
        self.act1_iac = ParameterDisplay("AC Input Current", "A", 1)
        self.act1_temp = ParameterDisplay("Temperature (Power Stage)", "°C", 1)
        self.act1_vout = ParameterDisplay("DC Output Voltage", "V", 1)
        self.act1_iout = ParameterDisplay("DC Output Current", "A", 1)
        self.act1_raw = RawDataDisplay()
        
        self.act1_panel.add_widget(self.act1_info)
        self.act1_panel.add_separator()
        self.act1_panel.add_widget(self.act1_iac)
        self.act1_panel.add_widget(self.act1_temp)
        self.act1_panel.add_widget(self.act1_vout)
        self.act1_panel.add_widget(self.act1_iout)
        self.act1_panel.add_widget(self.act1_raw)
        
        #  STAT Panel - Status (ID 0x610) 
        self.stat_panel = GroupPanel("STAT - Charger Status (1000ms) - RX")
        self.stat_info = MessageInfoPanel()
        self.stat_power_enable = BooleanIndicator("Hardware Enable Pin Active", "green", "gray")
        self.stat_error_latch = BooleanIndicator("Failure Occurred", "red", "gray")
        self.stat_warn_limit = BooleanIndicator("Warning Condition", "orange", "gray")
        self.stat_lim_temp = BooleanIndicator("De-rating Active", "orange", "gray")
        self.stat_warning_hv = BooleanIndicator("HV Warning", "orange", "gray")
        self.stat_bulks = BooleanIndicator("Bulk Error", "red", "gray")
        self.stat_raw = RawDataDisplay()
        
        # Info label for errorLatch
        self.error_latch_info = QLabel("INFO: When 'Failure Occurred' is active, check Level 2 tab for fault details")
        self.error_latch_info.setStyleSheet("""
            QLabel {
                color: #FF9800;
                background-color: #FFF3E0;
                padding: 8px;
                border-radius: 4px;
                font-size: 10px;
                font-weight: bold;
            }
        """)
        self.error_latch_info.setVisible(False)
        
        self.stat_panel.add_widget(self.stat_info)
        self.stat_panel.add_separator()
        self.stat_panel.add_widget(self.stat_power_enable)
        self.stat_panel.add_widget(self.stat_error_latch)
        self.stat_panel.add_widget(self.error_latch_info)
        self.stat_panel.add_widget(self.stat_warn_limit)
        self.stat_panel.add_widget(self.stat_lim_temp)
        self.stat_panel.add_widget(self.stat_warning_hv)
        self.stat_panel.add_widget(self.stat_bulks)
        self.stat_panel.add_widget(self.stat_raw)
        
        #  ACT2 Panel - Actual Values 2 (ID 0x614) 
        self.act2_panel = GroupPanel("ACT2 - Additional Values (1000ms) - RX")
        self.act2_info = MessageInfoPanel()
        self.act2_temp_loglv = ParameterDisplay("Temp Logic LV Stage", "°C", 1)
        self.act2_ac_power = ParameterDisplay("AC Input Power", "kW", 2)
        self.act2_prox_limit = ParameterDisplay("Max AC Current (Proximity)", "A", 1)
        self.act2_pilot_limit = ParameterDisplay("Max AC Current (Pilot)", "A", 1)
        self.act2_raw = RawDataDisplay()
        
        self.act2_panel.add_widget(self.act2_info)
        self.act2_panel.add_separator()
        self.act2_panel.add_widget(self.act2_temp_loglv)
        self.act2_panel.add_widget(self.act2_ac_power)
        self.act2_panel.add_widget(self.act2_prox_limit)
        self.act2_panel.add_widget(self.act2_pilot_limit)
        self.act2_panel.add_widget(self.act2_raw)
        
        #  TST1 Panel - Test/Diagnostic (ID 0x615) 
        self.tst1_panel = GroupPanel("TST1 - Diagnostic Flags (100ms) - RX")
        self.tst1_info = MessageInfoPanel()
        
        # Status flags
        self.tst1_ack = BooleanIndicator("AC Mains Connected", "green", "gray")
        self.tst1_pr_compl = BooleanIndicator("AC Precharge Completed", "green", "gray")
        self.tst1_pwr_ok = BooleanIndicator("Output Power OK", "green", "gray")
        self.tst1_vout_ok = BooleanIndicator("Output Voltage OK", "green", "gray")
        
        # Error flags
        self.tst1_ovp = BooleanIndicator("DC Over Voltage", "red", "gray")
        self.tst1_conn_open = BooleanIndicator("Connector Not Connected", "orange", "gray")
        self.tst1_rx618_fail = BooleanIndicator("Control Timeout (>600ms)", "red", "gray")
        
        # Bulk failures
        self.tst1_bulk1_fail = BooleanIndicator("Bulk 1 Error", "red", "gray")
        self.tst1_bulk2_fail = BooleanIndicator("Bulk 2 Error", "red", "gray")
        self.tst1_bulk3_fail = BooleanIndicator("Bulk 3 Error", "red", "gray")
        
        # Cooling
        self.tst1_pump_on = BooleanIndicator("Pump Active (>35°C)", "blue", "gray")
        self.tst1_fan_on = BooleanIndicator("Fan Active (>40°C)", "blue", "gray")
        self.tst1_cooling_fail = BooleanIndicator("Cooling Fault", "red", "gray")
        
        self.tst1_raw = RawDataDisplay()
        
        self.tst1_panel.add_widget(self.tst1_info)
        self.tst1_panel.add_separator()
        self.tst1_panel.add_widget(self.tst1_ack)
        self.tst1_panel.add_widget(self.tst1_pr_compl)
        self.tst1_panel.add_widget(self.tst1_pwr_ok)
        self.tst1_panel.add_widget(self.tst1_vout_ok)
        self.tst1_panel.add_separator()
        self.tst1_panel.add_widget(self.tst1_ovp)
        self.tst1_panel.add_widget(self.tst1_conn_open)
        self.tst1_panel.add_widget(self.tst1_rx618_fail)
        self.tst1_panel.add_separator()
        self.tst1_panel.add_widget(self.tst1_bulk1_fail)
        self.tst1_panel.add_widget(self.tst1_bulk2_fail)
        self.tst1_panel.add_widget(self.tst1_bulk3_fail)
        self.tst1_panel.add_separator()
        self.tst1_panel.add_widget(self.tst1_pump_on)
        self.tst1_panel.add_widget(self.tst1_fan_on)
        self.tst1_panel.add_widget(self.tst1_cooling_fail)
        self.tst1_panel.add_widget(self.tst1_raw)
        
        # Add all panels
        layout.addWidget(self.ctl_panel)
        layout.addWidget(self.act1_panel)
        layout.addWidget(self.stat_panel)
        layout.addWidget(self.act2_panel)
        layout.addWidget(self.tst1_panel)
        layout.addStretch()
        
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)

    def update_ctl(self,packet: CtlPacket, can_id:int, raw_data:list):
        "Aggiorna info CTL trasmesse BMS -> Charger"
        self.ctl_info.update_info(can_id,"CTL - Control Values")
        self.ctl_iac.set_value(packet.iac_max_A)
        self.ctl_vout.set_value(packet.vout_max_V)
        self.ctl_iOut.set_value(packet.iout_max_A)
        self.ctl_can.set_state(packet.can_enable)
        self.ctl_led3.set_state(packet.led3_enable)

    def update_act1(self, packet: Act1Packet, can_id: int, raw_data: list):
        """Update ACT1 display"""
        self.act1_info.update_info(can_id, "ACT1 - Actual Values 1")
        self.act1_iac.set_value(packet.iac_A)
        self.act1_temp.set_value(packet.temp_C)
        self.act1_vout.set_value(packet.vout_V)
        self.act1_iout.set_value(packet.iout_A)
        self.act1_raw.update_data(raw_data)
    
    def update_stat(self, packet: StatPacket, can_id: int, raw_data: list):
        """Update STAT display"""
        self.stat_info.update_info(can_id, "STAT - Status")
        self.stat_power_enable.set_state(packet.power_enable)
        self.stat_error_latch.set_state(packet.error_latch)
        self.stat_warn_limit.set_state(packet.warn_limit)
        self.stat_lim_temp.set_state(packet.lim_temp)
        self.stat_warning_hv.set_state(packet.warning_hv)
        self.stat_bulks.set_state(packet.bulks)
        self.stat_raw.update_data(raw_data)
        
        # Show/hide info message based on errorLatch state
        self.error_latch_info.setVisible(packet.error_latch)
    
    def update_act2(self, packet: Act2Packet, can_id: int, raw_data: list):
        """Update ACT2 display"""
        self.act2_info.update_info(can_id, "ACT2 - Actual Values 2")
        self.act2_temp_loglv.set_value(packet.temp_loglv_C)
        self.act2_ac_power.set_value(packet.ac_power_kW)
        self.act2_prox_limit.set_value(packet.prox_limit_A)
        self.act2_pilot_limit.set_value(packet.pilot_limit_A)
        self.act2_raw.update_data(raw_data)
    
    def update_tst1(self, packet: Tst1Packet, can_id: int, raw_data: list):
        """Update TST1 display"""
        self.tst1_info.update_info(can_id, "TST1 - Test/Diagnostic")
        self.tst1_ack.set_state(packet.ack)
        self.tst1_pr_compl.set_state(packet.pr_compl)
        self.tst1_pwr_ok.set_state(packet.pwr_ok)
        self.tst1_vout_ok.set_state(packet.vout_ok)
        self.tst1_ovp.set_state(packet.ovp)
        self.tst1_conn_open.set_state(packet.conn_open)
        self.tst1_rx618_fail.set_state(packet.rx618_fail)
        self.tst1_bulk1_fail.set_state(packet.bulk1_fail)
        self.tst1_bulk2_fail.set_state(packet.bulk2_fail)
        self.tst1_bulk3_fail.set_state(packet.bulk3_fail)
        self.tst1_pump_on.set_state(packet.pump_on)
        self.tst1_fan_on.set_state(packet.fan_on)
        self.tst1_cooling_fail.set_state(packet.cooling_fail)
        self.tst1_raw.update_data(raw_data)

########################################################################################################
########################################################################################################
########################################################################################################

class Level2Tab(QWidget):
    def __init__(self):
        super().__init__()
        self.total_faults = 0
        self.active_faults = 0
        self.passive_faults = 0
        self.setup_ui()
    
    def setup_ui(self):
        main_layout = QVBoxLayout()
        
        # Status summary at the top
        summary_frame = QFrame()
        summary_frame.setFrameStyle(QFrame.Shape.StyledPanel | QFrame.Shadow.Raised)
        summary_layout = QHBoxLayout(summary_frame)
        summary_layout.setContentsMargins(10, 10, 10, 10)
        
        # Summary title
        summary_title = QLabel("Fault Summary")
        summary_title.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        summary_layout.addWidget(summary_title)
        
        summary_layout.addStretch()
        
        # Fault counters
        self.total_faults_label = QLabel("Total: 0")
        self.total_faults_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.total_faults_label.setStyleSheet("color: #333; padding: 5px 10px; background-color: #e0e0e0; border-radius: 3px;")
        summary_layout.addWidget(self.total_faults_label)
        
        self.active_faults_label = QLabel("Active: 0")
        self.active_faults_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.active_faults_label.setStyleSheet("color: white; padding: 5px 10px; background-color: #f44336; border-radius: 3px;")
        summary_layout.addWidget(self.active_faults_label)
        
        self.passive_faults_label = QLabel("Passive: 0")
        self.passive_faults_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.passive_faults_label.setStyleSheet("color: white; padding: 5px 10px; background-color: #ff9800; border-radius: 3px;")
        summary_layout.addWidget(self.passive_faults_label)
        
        # Status indicator
        self.status_indicator = QLabel("● System OK")
        self.status_indicator.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        self.status_indicator.setStyleSheet("color: #4CAF50; padding: 5px 15px;")
        summary_layout.addWidget(self.status_indicator)
        
        main_layout.addWidget(summary_frame)
        
        # Info banner about automatic fault reporting
        info_banner = QLabel("INFO: Fault details are automatically requested by the microcontroller when errors are detected")
        info_banner.setStyleSheet("""
            QLabel {
                color: #1976D2;
                background-color: #E3F2FD;
                padding: 8px;
                border-radius: 4px;
                font-size: 10px;
                border-left: 4px solid #1976D2;
            }
        """)
        info_banner.setWordWrap(True)
        main_layout.addWidget(info_banner)
        
        # Scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        layout = QVBoxLayout(scroll_content)
        
        #  Fault Panel 
        self.fault_panel = GroupPanel("FAULT - Active/Passive Faults - RX")
        self.fault_info = MessageInfoPanel()
        
        # Button controls in horizontal layout
        button_layout = QHBoxLayout()
        self.clear_faults_button = QPushButton("Clear All Faults")
        self.clear_faults_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
        """)
        self.clear_faults_button.setMaximumWidth(200)
        self.clear_faults_button.clicked.connect(self._clear_faults_with_confirmation)
        button_layout.addWidget(self.clear_faults_button)
        button_layout.addStretch()
        
        self.fault_list = FaultListWidget()
        self.fault_raw = RawDataDisplay()
        
        self.fault_panel.add_widget(self.fault_info)
        self.fault_panel.layout.addLayout(button_layout)
        self.fault_panel.add_widget(self.fault_list)
        self.fault_panel.add_widget(self.fault_raw)
        
        #  Info Panel (Software Version & Serial Number in grid) 
        self.info_panel = GroupPanel("DEVICE INFO - Software & Serial Number - RX")
        info_grid = QGridLayout()
        
        # Software Version
        sw_label = QLabel(" Software Version:")
        sw_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.sw_version_value = QLabel("N/A")
        self.sw_version_value.setFont(QFont("Arial", 10))
        self.sw_version_value.setStyleSheet("color: #2196F3; padding: 5px;")
        info_grid.addWidget(sw_label, 0, 0)
        info_grid.addWidget(self.sw_version_value, 0, 1)
        
        # Serial Number
        sn_label = QLabel("Serial Number:")
        sn_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.sn_number_value = QLabel("N/A")
        self.sn_number_value.setFont(QFont("Arial", 10))
        self.sn_number_value.setStyleSheet("color: #4CAF50; padding: 5px;")
        info_grid.addWidget(sn_label, 1, 0)
        info_grid.addWidget(self.sn_number_value, 1, 1)
        
        info_grid.setColumnStretch(1, 1)
        self.info_panel.layout.addLayout(info_grid)
        
        # Raw data displays for SW and SN
        self.sw_raw = RawDataDisplay()
        self.sn_raw = RawDataDisplay()
        
        # Add panels
        layout.addWidget(self.fault_panel)
        layout.addWidget(self.info_panel)
        layout.addStretch()
        
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)
    
    def update_fault(self, packet: FaultPacket, can_id: int, raw_data: list):
        """Update Fault display"""
        is_active = can_id == 0x61D
        msg_name = "FLTA - Active Fault" if is_active else "FLTP - Passive Fault"
        self.fault_info.update_info(can_id, msg_name)
        
        if packet:  # Non "No Fault Detected"
            fault_name = self.get_fault_name(packet.fault_code)
            level_str = packet.failure_level.name
            
            self.fault_list.add_fault(
                packet.fault_code,
                fault_name,
                packet.occurrence,
                level_str,
                packet.last_time_h
            )
        else:
            self.fault_list.clear_faults()
        
        self.fault_raw.update_data(raw_data)
        self._update_fault_counters()
    
    def update_software(self, packet: SoftwarePacket, can_id: int, raw_data: list):
        """Update Software Version display"""
        self.sw_version_value.setText(f"v{packet.version}")
        self.sw_raw.update_data(raw_data)
    
    def update_serial(self, packet: SerialNumberPacket, can_id: int, raw_data: list):
        """Update Serial Number display"""
        self.sn_number_value.setText(packet.serial)
        self.sn_raw.update_data(raw_data)
    
    def _clear_faults_with_confirmation(self):
        """Clear faults with confirmation dialog"""
        if self.fault_list.get_fault_count() == 0:
            QMessageBox.information(
                self,
                "No Faults",
                "There are no faults to clear."
            )
            return
        
        reply = QMessageBox.question(
            self,
            "Clear Faults",
            f"Are you sure you want to clear all {self.fault_list.get_fault_count()} fault(s)?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            self.fault_list.clear_faults()
            self._update_fault_counters()
            QMessageBox.information(
                self,
                "Faults Cleared",
                "All faults have been cleared successfully."
            )
    
    def _update_fault_counters(self):
        """Update fault counter displays"""
        total = self.fault_list.get_fault_count()
        active = self.fault_list.get_active_fault_count()
        passive = total - active
        
        self.total_faults = total
        self.active_faults = active
        self.passive_faults = passive
        
        # Update labels
        self.total_faults_label.setText(f"Total: {total}")
        self.active_faults_label.setText(f"Active: {active}")
        self.passive_faults_label.setText(f"Passive: {passive}")
        
        # Update status indicator
        if total == 0:
            self.status_indicator.setText("● System OK")
            self.status_indicator.setStyleSheet("color: #4CAF50; padding: 5px 15px; font-weight: bold;")
        elif active > 0:
            self.status_indicator.setText("● Active Faults Detected")
            self.status_indicator.setStyleSheet("color: #f44336; padding: 5px 15px; font-weight: bold;")
        else:
            self.status_indicator.setText("● Passive Faults Only")
            self.status_indicator.setStyleSheet("color: #ff9800; padding: 5px 15px; font-weight: bold;")
    
    @staticmethod
    def get_fault_name(code: int) -> str:
        fault_names = {
            0xA0: "Bulk1 Voltage",
            0xA1: "Bulk2 Voltage",
            0xA2: "Bulk3 Voltage",
            0xA3: "Bulk Error",
            0xA4: "CAN Registers",
            0xA5: "CAN Command",
            0xA6: "Temp Low",
            0xA7: "Temp Derating",
            0xA8: "Temp High",
            0xA9: "Temp Failed",
            0xAA: "Input Current Max",
            0xAB: "HVIL Interlock",
            0xAC: "Logic Temp",
            0xAD: "Output Overvolt"
        }
        return fault_names.get(code, f"Unknown (0x{code:02X})")


########################################################################################################
########################################################################################################
########################################################################################################


class Level3Tab(QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()
    
    def setup_ui(self):
        main_layout = QVBoxLayout()
        
        # Summary frame at the top
        summary_frame = QFrame()
        summary_frame.setFrameStyle(QFrame.Shape.StyledPanel | QFrame.Shadow.Raised)
        summary_layout = QHBoxLayout(summary_frame)
        summary_layout.setContentsMargins(10, 10, 10, 10)
        
        summary_title = QLabel("Service Diagnostics")
        summary_title.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        summary_layout.addWidget(summary_title)
        summary_layout.addStretch()
        
        # Temperature status
        self.temp_status_label = QLabel("Temp: --°C")
        self.temp_status_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.temp_status_label.setStyleSheet("color: #333; padding: 5px 10px; background-color: #e0e0e0; border-radius: 3px;")
        summary_layout.addWidget(self.temp_status_label)
        
        # Current status
        self.current_status_label = QLabel("AC Current: --A")
        self.current_status_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.current_status_label.setStyleSheet("color: #333; padding: 5px 10px; background-color: #e0e0e0; border-radius: 3px;")
        summary_layout.addWidget(self.current_status_label)
        
        main_layout.addWidget(summary_frame)
        
        # Info banner
        info_banner = QLabel("Level 3 - Service Messages | Detailed temperature monitoring and AC phase diagnostics")
        info_banner.setStyleSheet("""
            QLabel {
                color: #1976D2;
                background-color: #E3F2FD;
                padding: 8px;
                border-radius: 4px;
                font-size: 10px;
                border-left: 4px solid #1976D2;
            }
        """)
        info_banner.setWordWrap(True)
        main_layout.addWidget(info_banner)
        
        # Scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        layout = QVBoxLayout(scroll_content)
        
        #  ACT3 Panel - AC Currents 
        self.act3_panel = GroupPanel("ACT3 - AC Input Currents (100ms) - RX")
        self.act3_info = MessageInfoPanel()
        self.act3_fan_voltage = ParameterDisplay("FAN Voltage", "V", 1)
        self.act3_iacm1 = ParameterDisplay("AC Current Module 1", "A", 1)
        self.act3_iacm2 = ParameterDisplay("AC Current Module 2", "A", 1)
        self.act3_iacm3 = ParameterDisplay("AC Current Module 3", "A", 1)
        self.act3_total = ParameterDisplay("Total AC Current", "A", 1)
        self.act3_raw = RawDataDisplay()
        
        self.act3_panel.add_widget(self.act3_info)
        self.act3_panel.add_separator()
        self.act3_panel.add_widget(self.act3_fan_voltage)
        self.act3_panel.add_widget(self.act3_iacm1)
        self.act3_panel.add_widget(self.act3_iacm2)
        self.act3_panel.add_widget(self.act3_iacm3)
        self.act3_panel.add_widget(self.act3_total)
        self.act3_panel.add_widget(self.act3_raw)
        
        #  TEMP Panel - Temperatures 
        self.temp_panel = GroupPanel("TEMP - Temperature Sensors (100ms) - RX")
        self.temp_info = MessageInfoPanel()
        self.temp_loghv = ParameterDisplay("Logic Board HV", "°C", 1)
        self.temp_power1 = ParameterDisplay("Power Stage 1", "°C", 1)
        self.temp_power2 = ParameterDisplay("Power Stage 2", "°C", 1)
        self.temp_power3 = ParameterDisplay("Power Stage 3", "°C", 1)
        self.temp_max = ParameterDisplay("Max Power Stage Temp", "°C", 1)
        self.temp_raw = RawDataDisplay()
        
        self.temp_panel.add_widget(self.temp_info)
        self.temp_panel.add_separator()
        self.temp_panel.add_widget(self.temp_loghv)
        self.temp_panel.add_widget(self.temp_power1)
        self.temp_panel.add_widget(self.temp_power2)
        self.temp_panel.add_widget(self.temp_power3)
        self.temp_panel.add_widget(self.temp_max)
        self.temp_panel.add_widget(self.temp_raw)
        
        #  STST1 Panel - Real Time Diagnostic 
        self.stst1_panel = GroupPanel("STST1 - Real Time Diagnostic (100ms) - RX")
        self.stst1_info = MessageInfoPanel()
        self.stst1_pfc_enable = BooleanIndicator("PFC Enable", "green", "gray")
        self.stst1_log_temp_high = BooleanIndicator("Logic Temp High", "red", "gray")
        self.stst1_log_temp_low = BooleanIndicator("Logic Temp Low", "orange", "gray")
        self.stst1_bulk1_fail = BooleanIndicator("Bulk 1 Fail", "red", "gray")
        self.stst1_bulk2_fail = BooleanIndicator("Bulk 2 Fail", "red", "gray")
        self.stst1_bulk3_fail = BooleanIndicator("Bulk 3 Fail", "red", "gray")
        self.stst1_cooling_fail1 = BooleanIndicator("Cooling Fail Stage 1", "red", "gray")
        self.stst1_cooling_fail2 = BooleanIndicator("Cooling Fail Stage 2", "red", "gray")
        self.stst1_cooling_fail3 = BooleanIndicator("Cooling Fail Stage 3", "red", "gray")
        self.stst1_raw = RawDataDisplay()
        
        self.stst1_panel.add_widget(self.stst1_info)
        self.stst1_panel.add_separator()
        self.stst1_panel.add_widget(self.stst1_pfc_enable)
        self.stst1_panel.add_widget(self.stst1_log_temp_high)
        self.stst1_panel.add_widget(self.stst1_log_temp_low)
        self.stst1_panel.add_separator()
        self.stst1_panel.add_widget(self.stst1_bulk1_fail)
        self.stst1_panel.add_widget(self.stst1_bulk2_fail)
        self.stst1_panel.add_widget(self.stst1_bulk3_fail)
        self.stst1_panel.add_separator()
        self.stst1_panel.add_widget(self.stst1_cooling_fail1)
        self.stst1_panel.add_widget(self.stst1_cooling_fail2)
        self.stst1_panel.add_widget(self.stst1_cooling_fail3)
        self.stst1_panel.add_widget(self.stst1_raw)
        
        #  ACT4 Panel - Temperature FAN 
        self.act4_panel = GroupPanel("ACT4 - Temperature FAN (100ms) - RX")
        self.act4_info = MessageInfoPanel()
        self.act4_temp_logfan = ParameterDisplay("Logic Board FAN Temp", "°C", 1)
        self.act4_raw = RawDataDisplay()
        
        self.act4_panel.add_widget(self.act4_info)
        self.act4_panel.add_widget(self.act4_temp_logfan)
        self.act4_panel.add_widget(self.act4_raw)
        
        # Add panels
        layout.addWidget(self.act3_panel)
        layout.addWidget(self.temp_panel)
        layout.addWidget(self.stst1_panel)
        layout.addWidget(self.act4_panel)
        layout.addStretch()
        
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)
    
    def update_act3(self, packet: Act3Packet, can_id: int, raw_data: list):
        """Update ACT3 display"""
        self.act3_info.update_info(can_id, "ACT3 - AC Currents")
        self.act3_fan_voltage.set_value(packet.fan_voltage_V)
        self.act3_iacm1.set_value(packet.iacm1_A)
        self.act3_iacm2.set_value(packet.iacm2_A)
        self.act3_iacm3.set_value(packet.iacm3_A)
        total_current = packet.iacm1_A + packet.iacm2_A + packet.iacm3_A
        self.act3_total.set_value(total_current)
        self.act3_raw.update_data(raw_data)
        
        # Update summary
        self.current_status_label.setText(f"AC Current: {total_current:.1f}A")
    
    def update_temp(self, packet: TempPacket, can_id: int, raw_data: list):
        """Update TEMP display"""
        self.temp_info.update_info(can_id, "TEMP - Temperatures")
        self.temp_loghv.set_value(packet.temp_loghv_C)
        self.temp_power1.set_value(packet.temp_power1_C)
        self.temp_power2.set_value(packet.temp_power2_C)
        self.temp_power3.set_value(packet.temp_power3_C)
        max_temp = max(packet.temp_power1_C, packet.temp_power2_C, packet.temp_power3_C)
        self.temp_max.set_value(max_temp)
        self.temp_raw.update_data(raw_data)
        
        # Update summary with color coding
        temp_color = "#4CAF50" if max_temp < 60 else "#FF9800" if max_temp < 80 else "#f44336"
        self.temp_status_label.setText(f"Max Temp: {max_temp:.1f}°C")
        self.temp_status_label.setStyleSheet(f"color: white; padding: 5px 10px; background-color: {temp_color}; border-radius: 3px; font-weight: bold;")
    
    def update_stst1(self, packet: Stst1Packet, can_id: int, raw_data: list):
        """Update STST1 display"""
        self.stst1_info.update_info(can_id, "STST1 - Real Time Diagnostic")
        self.stst1_pfc_enable.set_state(packet.pfc_enable)
        self.stst1_log_temp_high.set_state(packet.log_temp_high)
        self.stst1_log_temp_low.set_state(packet.log_temp_low)
        self.stst1_bulk1_fail.set_state(packet.bulk1_fail)
        self.stst1_bulk2_fail.set_state(packet.bulk2_fail)
        self.stst1_bulk3_fail.set_state(packet.bulk3_fail)
        self.stst1_cooling_fail1.set_state(packet.cooling_fail1)
        self.stst1_cooling_fail2.set_state(packet.cooling_fail2)
        self.stst1_cooling_fail3.set_state(packet.cooling_fail3)
        self.stst1_raw.update_data(raw_data)
    
    def update_act4(self, packet: Act4Packet, can_id: int, raw_data: list):
        """Update ACT4 display"""
        self.act4_info.update_info(can_id, "ACT4 - Temperature FAN")
        self.act4_temp_logfan.set_value(packet.temp_logfan_C)
        self.act4_raw.update_data(raw_data)


########################################################################################################
########################################################################################################
########################################################################################################


class Level4Tab(QWidget):
    def __init__(self):
        super().__init__()
        self.config_received = False
        self.setup_ui()
    
    def setup_ui(self):
        main_layout = QVBoxLayout()
        
        # Summary frame at the top
        summary_frame = QFrame()
        summary_frame.setFrameStyle(QFrame.Shape.StyledPanel | QFrame.Shadow.Raised)
        summary_layout = QHBoxLayout(summary_frame)
        summary_layout.setContentsMargins(10, 10, 10, 10)
        
        summary_title = QLabel("Charger Configuration")
        summary_title.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        summary_layout.addWidget(summary_title)
        summary_layout.addStretch()
        
        # Config status
        self.config_status_label = QLabel("Waiting for configuration...")
        self.config_status_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.config_status_label.setStyleSheet("color: white; padding: 5px 10px; background-color: #FF9800; border-radius: 3px;")
        summary_layout.addWidget(self.config_status_label)
        
        main_layout.addWidget(summary_frame)
        
        # Info banner
        info_banner = QLabel("Level 4 - Configuration | Charger settings sent once at startup")
        info_banner.setStyleSheet("""
            QLabel {
                color: #1976D2;
                background-color: #E3F2FD;
                padding: 8px;
                border-radius: 4px;
                font-size: 10px;
                border-left: 4px solid #1976D2;
            }
        """)
        info_banner.setWordWrap(True)
        main_layout.addWidget(info_banner)
        
        # Scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        layout = QVBoxLayout(scroll_content)
        
        #  TST2 Panel - Configuration 
        self.tst2_panel = GroupPanel("TST2 - Charger Configuration - RX (once at the startup)")
        self.tst2_info = MessageInfoPanel()
        
        self.tst2_comm_title = QLabel("Communication Settings")
        font = QFont()
        font.setBold(True)
        self.tst2_comm_title.setFont(font)
        self.tst2_baudrate = QLabel("Baudrate: ---")
        self.tst2_id_type = QLabel("ID Type: ---")
        
        self.tst2_ctrl_title = QLabel("Current Control")
        self.tst2_ctrl_title.setFont(font)
        self.tst2_iac_control = QLabel("AC Current Control: ---")
        self.tst2_iacm_max = ParameterDisplay("Max AC Input Current", "A", 1)
        
        self.tst2_limits_title = QLabel("Voltage/Current Limits")
        self.tst2_limits_title.setFont(font)
        self.tst2_range = QLabel("Range: ---")
        self.tst2_vout_max = ParameterDisplay("Max DC Output Voltage", "V", 1)
        self.tst2_iout_max = ParameterDisplay("Max DC Output Current", "A", 1)

        self.tst2_config_title = QLabel("Charger Configuration")
        self.tst2_config_title.setFont(font)
        self.tst2_model = QLabel("Model: ---")
        self.tst2_three_phase = BooleanIndicator("Three-Phase", "green", "gray")
        self.tst2_cooling = QLabel("Cooling: ---")

        self.tst2_parallel_title = QLabel("Parallel Operation")
        self.tst2_parallel_title.setFont(font)
        self.tst2_slave = BooleanIndicator("Slave Mode", "blue", "gray")
        self.tst2_parallel_ctrl = BooleanIndicator("Parallel Control", "blue", "gray")
        
        self.tst2_raw = RawDataDisplay()
        
        self.tst2_panel.add_widget(self.tst2_info)
        self.tst2_panel.add_separator()
        self.tst2_panel.add_widget(self.tst2_comm_title)
        self.tst2_panel.add_widget(self.tst2_baudrate)
        self.tst2_panel.add_widget(self.tst2_id_type)
        self.tst2_panel.add_separator()
        self.tst2_panel.add_widget(self.tst2_ctrl_title)
        self.tst2_panel.add_widget(self.tst2_iac_control)
        self.tst2_panel.add_widget(self.tst2_iacm_max)
        self.tst2_panel.add_separator()
        self.tst2_panel.add_widget(self.tst2_limits_title)
        self.tst2_panel.add_widget(self.tst2_range)
        self.tst2_panel.add_widget(self.tst2_vout_max)
        self.tst2_panel.add_widget(self.tst2_iout_max)
        self.tst2_panel.add_separator()
        self.tst2_panel.add_widget(self.tst2_config_title)
        self.tst2_panel.add_widget(self.tst2_model)
        self.tst2_panel.add_widget(self.tst2_three_phase)
        self.tst2_panel.add_widget(self.tst2_cooling)
        self.tst2_panel.add_separator()
        self.tst2_panel.add_widget(self.tst2_parallel_title)
        self.tst2_panel.add_widget(self.tst2_slave)
        self.tst2_panel.add_widget(self.tst2_parallel_ctrl)
        self.tst2_panel.add_widget(self.tst2_raw)
        
        layout.addWidget(self.tst2_panel)
        layout.addStretch()
        
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)
    
    def update_tst2(self, packet: Tst2Packet, can_id: int, raw_data: list):
        """Update TST2 display"""
        self.tst2_info.update_info(can_id, "TST2 - Configuration")
        
        # Update config status
        if not self.config_received:
            self.config_received = True
            self.config_status_label.setText("Configuration Received")
            self.config_status_label.setStyleSheet("color: white; padding: 5px 10px; background-color: #4CAF50; border-radius: 3px; font-weight: bold;")
        
        # Communication
        baudrate_str = {
            BaudrateType.BAUDRATE_500KBIT: "500 Kbit/s",
            BaudrateType.BAUDRATE_250KBIT: "250 Kbit/s",
            BaudrateType.BAUDRATE_125KBIT: "125 Kbit/s",
            BaudrateType.BAUDRATE_1MBIT: "1 Mbit/s"
        }.get(packet.baudrate, "Unknown")
        self.tst2_baudrate.setText(f"Baudrate: {baudrate_str}")
        
        id_type_str = "Standard 11bit" if packet.id_type == IdType.STANDARD_11BIT else "Extended 29bit"
        self.tst2_id_type.setText(f"ID Type: {id_type_str}")
        
        # Current Control
        iac_ctrl_str = {
            IacControlType.NOT_CONTROLLED: "Not controlled (HW set)",
            IacControlType.SAEJ1772: "SAE J1772",
            IacControlType.EN61851: "EN61851",
            IacControlType.ID618: "ID 0x618"
        }.get(packet.iac_control, "Unknown")
        self.tst2_iac_control.setText(f"AC Current Control: {iac_ctrl_str}")
        self.tst2_iacm_max.set_value(packet.iacm_max_set_A)
        
        # Limits
        range_str = {
            RangeType.R4_EVO_USERS: "R4 (EVO Users Manual)",
            RangeType.R3: "R3",
            RangeType.R2: "R2",
            RangeType.R1: "R1"
        }.get(packet.range, "Unknown")
        self.tst2_range.setText(f"Range: {range_str}")
        self.tst2_vout_max.set_value(packet.vout_max_set_V)
        self.tst2_iout_max.set_value(packet.iout_max_set_A)
        
        # Configuration
        model_str = "EVO11K (liquid)" if packet.evc_model == EVCModelType.EVO11K else "EVO22K (air)"
        self.tst2_model.setText(f"Model: {model_str}")
        self.tst2_three_phase.set_state(packet.three_phase)
        cooling_str = "Air (EVO11KA)" if packet.air_cooler else "Liquid (EVO11KL)"
        self.tst2_cooling.setText(f"Cooling: {cooling_str}")
        
        # Parallel
        self.tst2_slave.set_state(packet.slave)
        self.tst2_parallel_ctrl.set_state(packet.parallel_ctrl)
        
        self.tst2_raw.update_data(raw_data)
