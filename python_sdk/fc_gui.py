import sys
from time import sleep, time

from fc_gui.ui_gui import Ui_MainWindow
from FlightController import FC_Controller, FC_State_Struct

""" qtdesigner file """

import qdarktheme
from PySide6 import QtSerialPort
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtUiTools import *
from PySide6.QtWidgets import *


class MainWindow(Ui_MainWindow, QMainWindow):
    def __init__(self) -> None:
        super(MainWindow, self).__init__()
        self.setupUi(self)
        self.connect_signals()
        self.init_misc()
        self.fc = None
        self.updating_serial = False
        self.speed_xyzYaw = [0, 0, 0, 0]

    def init_misc(self) -> None:
        self.text_log.clear()
        self.line_info.clear()
        self.timer = QTimer()
        self.timer.timeout.connect(self.fc_timer_update)
        self.timer.start(100)

    def print_log(self, msg: str) -> None:
        msg = msg.replace("\n", " ")
        time_str = QDateTime.currentDateTime().toString("[hh:mm:ss] ")
        self.text_log.appendPlainText(f"{time_str}{msg}")

    def connect_signals(self) -> None:
        pass

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.fc is not None:
            self.fc.quit()
        return super().closeEvent(event)

    def showEvent(self, event: QShowEvent) -> None:
        return super().showEvent(event)

    @Slot()
    def on_btn_serial_update_clicked(self) -> None:
        self.updating_serial = True
        self.combo_serial.clear()
        self.combo_serial.addItem("断开连接")
        self.combo_serial.setCurrentIndex(0)
        ports = QtSerialPort.QSerialPortInfo.availablePorts()
        self.combo_serial.addItems([port.portName() for port in ports])
        self.print_log(f"找到{len(ports)}个串口")
        self.updating_serial = False

    @Slot()
    def on_combo_serial_currentIndexChanged(self) -> None:
        if self.updating_serial:
            return
        text = self.combo_serial.currentText()
        if text == "断开连接" or self.fc is not None:
            if self.fc is not None:
                self.print_log("正在断开连接...")
                self.fc.quit()
                self.fc = None
                self.print_log("断开连接成功")
                self.line_info.setText("")
        if text != "断开连接":
            try:
                self.print_log("正在连接...")
                self.fc = FC_Controller(text, 500000)
                self.fc.start_listen_serial(
                    callback=self.update_fc_state, print_state=False
                )
                self.print_log("连接成功")
            except Exception as e:
                self.print_log(f"连接失败, {e}")
                self.fc = None
        if self.fc is not None:
            self.btn_serial_update.setEnabled(False)
        else:
            self.btn_serial_update.setEnabled(True)

    def fc_timer_update(self) -> None:
        if self.fc is None:
            return
        if not self.fc.state.unlock.value:
            return
        if self.fc.state.mode.value != self.fc.PROGRAM_MODE:
            self.fc.realtime_control(*self.speed_xyzYaw)

    def update_fc_state(self, state: FC_State_Struct) -> None:
        self.lcd_h.display(f"{state.alt_add.value/100:9.02f}")
        self.lcd_d.display(f"{state.yaw.value:9.02f}")
        self.lcd_v.display(f"{state.bat.value:9.02f}")
        self.lcd_x.display(f"{state.vel_x.value/100:9.02f}")
        self.lcd_y.display(f"{state.vel_y.value/100:9.02f}")
        self.lcd_z.display(f"{state.vel_z.value/100:9.02f}")
        state_text = ""
        if state.unlock.value:
            state_text += "电机已解锁  "
        else:
            state_text += "电机已锁定  "
        mode_dict = {1: "定高", 2: "定点", 3: "程控"}
        state_text += f"模式: {mode_dict[state.mode.value]}  "
        state_text += f"指令状态: {state.cid.value:02X} / {state.cmd_0.value:02X} / {state.cmd_1.value:02X} /"
        if self.fc.last_command_done:
            state_text += " 已完成  "
        else:
            state_text += " 进行中  "
        if self.fc.is_stablizing:
            state_text += "机体已稳定"
        elif not state.unlock.value:
            state_text += "等待起飞"
        else:
            state_text += "机体运动中"
        self.line_info.setText(state_text)

    @Slot()
    def on_btn_takeoff_clicked(self) -> None:
        if self.fc is None:
            return
        self.fc.take_off()

    @Slot()
    def on_btn_land_clicked(self) -> None:
        if self.fc is None:
            return
        self.fc.land()

    @Slot()
    def on_btn_unlock_clicked(self) -> None:
        if self.fc is None:
            return
        self.fc.unlock()

    @Slot()
    def on_btn_lock_clicked(self) -> None:
        if self.fc is None:
            return
        self.fc.lock()

    @Slot()
    def on_buttonGroup_buttonClicked(self) -> None:
        if self.fc is None:
            return
        if self.radio_control_realtime.isChecked():
            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        else:
            self.fc.set_flight_mode(self.fc.PROGRAM_MODE)

    @Slot()
    def on_btn_forward_pressed(self) -> None:
        self.speed_xyzYaw[0] = self.box_hori_param.value()

    @Slot()
    def on_btn_forward_released(self) -> None:
        self.speed_xyzYaw[0] = 0

    @Slot()
    def on_btn_backward_pressed(self) -> None:
        self.speed_xyzYaw[0] = -self.box_hori_param.value()

    @Slot()
    def on_btn_backward_released(self) -> None:
        self.speed_xyzYaw[0] = 0

    @Slot()
    def on_btn_left_pressed(self) -> None:
        self.speed_xyzYaw[1] = self.box_hori_param.value()

    @Slot()
    def on_btn_left_released(self) -> None:
        self.speed_xyzYaw[1] = 0

    @Slot()
    def on_btn_right_pressed(self) -> None:
        self.speed_xyzYaw[1] = -self.box_hori_param.value()

    @Slot()
    def on_btn_right_released(self) -> None:
        self.speed_xyzYaw[1] = 0

    @Slot()
    def on_btn_l_turn_pressed(self) -> None:
        self.speed_xyzYaw[3] = -self.box_spin_param.value()

    @Slot()
    def on_btn_l_turn_released(self) -> None:
        self.speed_xyzYaw[3] = 0

    @Slot()
    def on_btn_r_turn_pressed(self) -> None:
        self.speed_xyzYaw[3] = self.box_spin_param.value()

    @Slot()
    def on_btn_r_turn_released(self) -> None:
        self.speed_xyzYaw[3] = 0

    @Slot()
    def on_btn_up_pressed(self) -> None:
        self.speed_xyzYaw[2] = self.box_vert_param.value()

    @Slot()
    def on_btn_up_released(self) -> None:
        self.speed_xyzYaw[2] = 0

    @Slot()
    def on_btn_down_pressed(self) -> None:
        self.speed_xyzYaw[2] = -self.box_vert_param.value()

    @Slot()
    def on_btn_down_released(self) -> None:
        self.speed_xyzYaw[2] = 0

    @Slot()
    def on_btn_stop_clicked(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw = [0, 0, 0, 0]
        self.fc.stablize()

    @Slot()
    def on_btn_set_h_clicked(self) -> None:
        if self.fc is None:
            return
        self.fc.set_height(1, self.box_height.value(),self.box_vert_param.value())


def main():
    app = QApplication(sys.argv)

    mwin = MainWindow()
    app.setStyleSheet(qdarktheme.load_stylesheet())
    mwin.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
