import sys
from time import sleep, time

from fc_gui.ui_gui import Ui_MainWindow
from FlightController import FC_Client, FC_Controller
from FlightController.Base import FC_State_Struct

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
        self.server_ip = "127.0.0.1"
        self.updating_serial = False
        self.speed_xyzYaw = [0, 0, 0, 0]
        self.horizonal_distance = 50
        self.vertical_distance = 50
        self.yaw_degree = 30
        self.btn_serial_update.click()

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
        # if self.fc is not None:
        #     self.fc.quit()
        return super().closeEvent(event)

    def showEvent(self, event: QShowEvent) -> None:
        return super().showEvent(event)

    @Slot()
    def on_btn_serial_update_clicked(self) -> None:
        self.updating_serial = True
        self.combo_serial.clear()
        self.combo_serial.addItem("断开连接")
        self.combo_serial.addItem("远端服务")
        self.combo_serial.setCurrentIndex(0)
        ports = QtSerialPort.QSerialPortInfo.availablePorts()
        add_ports = []
        for port in ports:
            if int(port.portName().strip("COM")) < 64:
                add_ports.append(port.portName())
        self.combo_serial.addItems(add_ports)
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
                sleep(0.1)
                self.fc = None
                self.print_log("断开连接成功")
                self.line_info.setText("")
        if text == "远端服务":
            try:
                self.server_ip, ok = QInputDialog.getText(
                    self, "远端服务", "请输入IP地址:", QLineEdit.Normal, self.server_ip
                )
                if not ok:
                    return
                self.print_log("正在连接...")
                self.fc = FC_Client()
                self.fc.connect(self.server_ip)
                self.fc.start_sync_state(
                    callback=self.update_fc_state, print_state=False
                )
                self.print_log("连接成功")
            except Exception as e:
                self.print_log(f"连接失败, {e}")
                self.fc = None
        elif text != "断开连接":
            try:
                self.print_log("正在连接...")
                self.fc = FC_Controller()
                self.fc.start_listen_serial(
                    text, 500000, callback=self.update_fc_state, print_state=False
                )
                self.print_log("串口连接成功")
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
        if self.check_enable_ctl.isChecked():
            if self.fc.state.mode.value != self.fc.PROGRAM_MODE:
                self.fc.send_realtime_control_data(*self.speed_xyzYaw)

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
        state_text += f"指令状态: x{state.cid.value:02X} / x{state.cmd_0.value:02X} / x{state.cmd_1.value:02X} /"
        if self.fc.last_command_done:
            state_text += " 已完成  "
        else:
            state_text += " 进行中  "
        if self.fc.hovering:
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
        self.print_log("起飞")

    @Slot()
    def on_btn_land_clicked(self) -> None:
        if self.fc is None:
            return
        self.fc.land()
        self.print_log("降落")

    @Slot()
    def on_btn_unlock_clicked(self) -> None:
        if self.fc is None:
            return
        self.fc.unlock()
        self.print_log("解锁电机")
        if self.fc.state.mode.value == self.fc.HOLD_ALT_MODE:
            self.print_log("警告: 当前处于定高模式")

    @Slot()
    def on_btn_lock_clicked(self) -> None:
        if self.fc is None:
            return
        self.fc.lock()
        self.print_log("锁定电机")

    @Slot()
    def on_buttonGroup_buttonClicked(self) -> None:
        if self.fc is None:
            return
        if self.radio_control_realtime.isChecked():
            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
            self.print_log("飞控切换到定点模式")
        else:
            self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
            self.print_log("飞控切换到程控模式")

    @Slot()
    def on_btn_forward_pressed(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[0] = self.box_hori_param.value()

    @Slot()
    def on_btn_forward_released(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[0] = 0
        if self.fc.state.mode.value == self.fc.PROGRAM_MODE:
            distance = self.horizonal_distance
            speed = self.box_hori_param.value()
            self.fc.horizontal_move(distance, speed, 0)
            self.print_log(f"向前移动 {distance} 厘米, 平均速度 {speed}")

    @Slot()
    def on_btn_backward_pressed(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[0] = -self.box_hori_param.value()

    @Slot()
    def on_btn_backward_released(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[0] = 0
        if self.fc.state.mode.value == self.fc.PROGRAM_MODE:
            distance = self.horizonal_distance
            speed = self.box_hori_param.value()
            self.fc.horizontal_move(distance, speed, 180)
            self.print_log(f"向后移动 {distance} 厘米, 平均速度 {speed}")

    @Slot()
    def on_btn_left_pressed(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[1] = self.box_hori_param.value()

    @Slot()
    def on_btn_left_released(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[1] = 0
        if self.fc.state.mode.value == self.fc.PROGRAM_MODE:
            distance = self.horizonal_distance
            speed = self.box_hori_param.value()
            self.fc.horizontal_move(distance, speed, 270)
            self.print_log(f"向左移动 {distance} 厘米, 平均速度 {speed}")

    @Slot()
    def on_btn_right_pressed(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[1] = -self.box_hori_param.value()

    @Slot()
    def on_btn_right_released(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[1] = 0
        if self.fc.state.mode.value == self.fc.PROGRAM_MODE:
            distance = self.horizonal_distance
            speed = self.box_hori_param.value()
            self.fc.horizontal_move(distance, speed, 90)
            self.print_log(f"向右移动 {distance} 厘米, 平均速度 {speed}")

    @Slot()
    def on_btn_l_turn_pressed(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[3] = -self.box_spin_param.value()

    @Slot()
    def on_btn_l_turn_released(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[3] = 0
        if self.fc.state.mode.value == self.fc.PROGRAM_MODE:
            degree = self.yaw_degree
            speed = self.box_spin_param.value()
            self.fc.turn_left(degree, speed)
            self.print_log(f"向左转 {degree} 度, 平均速度 {speed}")

    @Slot()
    def on_btn_r_turn_pressed(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[3] = self.box_spin_param.value()

    @Slot()
    def on_btn_r_turn_released(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[3] = 0
        if self.fc.state.mode.value == self.fc.PROGRAM_MODE:
            degree = self.yaw_degree
            speed = self.box_spin_param.value()
            self.fc.turn_right(degree, speed)
            self.print_log(f"向右转 {degree} 度, 平均速度 {speed}")

    @Slot()
    def on_btn_up_pressed(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[2] = self.box_vert_param.value()

    @Slot()
    def on_btn_up_released(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[2] = 0
        if self.fc.state.mode.value == self.fc.PROGRAM_MODE:
            distance = self.vertical_distance
            speed = self.box_vert_param.value()
            self.fc.go_up(distance, speed)
            self.print_log(f"向上移动 {distance} 厘米, 平均速度 {speed}")

    @Slot()
    def on_btn_down_pressed(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[2] = -self.box_vert_param.value()

    @Slot()
    def on_btn_down_released(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw[2] = 0
        if self.fc.state.mode.value == self.fc.PROGRAM_MODE:
            distance = self.vertical_distance
            speed = self.box_vert_param.value()
            self.fc.go_down(distance, speed)
            self.print_log(f"向下移动 {distance} 厘米, 平均速度 {speed}")

    @Slot()
    def on_btn_stop_clicked(self) -> None:
        if self.fc is None:
            return
        self.speed_xyzYaw = [0, 0, 0, 0]
        self.fc.stablize()
        self.print_log("停止运动")

    @Slot()
    def on_btn_set_h_clicked(self) -> None:
        if self.fc is None:
            return
        if self.fc.state.mode.value != self.fc.PROGRAM_MODE:
            self.print_log("请先进入程控模式")
            return
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        self.fc.set_height(1, self.box_height.value(), self.box_vert_param.value())
        self.print_log(
            f"设置目标高度为 {self.box_height.value()} 厘米, 平均速度 {self.box_vert_param.value()}"
        )

    @Slot()
    def on_btn_rgb_clicked(self) -> None:
        if self.fc is None:
            return
        s, ok = QInputDialog.getText(
            self, "设置WS2812 RGB", "RGB(hex):", QLineEdit.Normal, "#000000"
        )
        if not ok:
            return
        try:
            r, g, b = int(s[1:3], 16), int(s[3:5], 16), int(s[5:7], 16)
            self.fc.set_rgb_led(r, g, b)
        except:
            self.print_log("输入错误")

    def _set_io(self, io):
        if self.fc is None:
            return
        s, ok = QInputDialog.getItem(self, f"设置IO:{io}", "操作:", ["开", "关"], 0, False)
        if not ok:
            return
        if s == "开":
            self.fc.set_digital_output(io, True)
        else:
            self.fc.set_digital_output(io, False)

    def _set_pwm(self, channel):
        if self.fc is None:
            return
        get, ok = QInputDialog.getDouble(self, f"设置PWM:{channel}", "占空比:", 0, 0, 100, 1)
        if not ok:
            return
        self.fc.set_PWM_output(channel, get)

    @Slot()
    def on_btn_pod_clicked(self) -> None:
        if self.fc is None:
            return
        s, ok = QInputDialog.getItem(self, "设置吊舱", "操作:", ["上升", "下降"], 0, False)
        if not ok:
            return
        if s == "上升":
            self.fc.set_pod(2, 20000)
        else:
            get, ok = QInputDialog.getInt(self, "设置吊舱", "放线时间(ms):", 0, 0, 20000, 1)
            if not ok:
                return
            self.fc.set_pod(1, get)

    @Slot()
    def on_btn_io_0_clicked(self) -> None:
        self._set_io(0)

    @Slot()
    def on_btn_io_1_clicked(self) -> None:
        self._set_io(1)

    @Slot()
    def on_btn_io_2_clicked(self) -> None:
        self._set_io(2)

    @Slot()
    def on_btn_io_3_clicked(self) -> None:
        self._set_io(3)

    @Slot()
    def on_btn_pwm_0_clicked(self) -> None:
        self._set_pwm(0)

    @Slot()
    def on_btn_pwm_1_clicked(self) -> None:
        self._set_pwm(1)

    @Slot()
    def on_btn_pwm_2_clicked(self) -> None:
        self._set_pwm(2)

    @Slot()
    def on_btn_pwm_3_clicked(self) -> None:
        self._set_pwm(3)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        key = event.key()
        if key == Qt.Key_W:
            self.on_btn_forward_pressed()
        elif key == Qt.Key_S:
            self.on_btn_backward_pressed()
        elif key == Qt.Key_A:
            self.on_btn_left_pressed()
        elif key == Qt.Key_D:
            self.on_btn_right_pressed()
        elif key == Qt.Key_Q:
            self.on_btn_l_turn_pressed()
        elif key == Qt.Key_E:
            self.on_btn_r_turn_pressed()
        elif key == Qt.Key_R:
            self.on_btn_up_pressed()
        elif key == Qt.Key_F:
            self.on_btn_down_pressed()
        elif key == Qt.Key_Space:
            self.on_btn_stop_clicked()
        elif key == Qt.Key_Z:
            self.on_btn_takeoff_clicked()
        elif key == Qt.Key_X:
            self.on_btn_unlock_clicked()
        elif key == Qt.Key_C:
            self.on_btn_land_clicked()
        elif key == Qt.Key_Escape:
            self.on_btn_lock_clicked()
        return super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent) -> None:
        key = event.key()
        if key == Qt.Key_W:
            self.on_btn_forward_released()
        elif key == Qt.Key_S:
            self.on_btn_backward_released()
        elif key == Qt.Key_A:
            self.on_btn_left_released()
        elif key == Qt.Key_D:
            self.on_btn_right_released()
        elif key == Qt.Key_Q:
            self.on_btn_l_turn_released()
        elif key == Qt.Key_E:
            self.on_btn_r_turn_released()
        elif key == Qt.Key_R:
            self.on_btn_up_released()
        elif key == Qt.Key_F:
            self.on_btn_down_released()
        return super().keyReleaseEvent(event)


def main():
    app = QApplication(sys.argv)

    mwin = MainWindow()
    app.setStyleSheet(qdarktheme.load_stylesheet())
    mwin.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
