import sys

from fc_gui.ui_gui import Ui_MainWindow
from FlightController import FlightController

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
        self.init_slots()
        self.init_misc()
        self.fc = None
        self.updating_serial = False

    def init_slots(self) -> None:
        pass

    def init_misc(self) -> None:
        self.text_log.clear()
        self.line_info.clear()

    def print_log(self, msg: str) -> None:
        msg = msg.replace("\n", " ")
        time_str = QDateTime.currentDateTime().toString("[hh:mm:ss] ")
        self.text_log.appendPlainText(f"{time_str}{msg}")

    def connect_signals(self) -> None:
        QMetaObject.connectSlotsByName(self)

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
        if text != "断开连接":
            try:
                self.print_log("正在连接...")
                self.fc = FlightController(text)
                self.fc.start_listen_serial(callback=self.update_fc_state)
                self.print_log("连接成功")
            except Exception as e:
                self.print_log(f"连接失败, {e}")
                self.fc = None
        if self.fc is not None:
            self.btn_serial_update.setEnabled(False)
        else:
            self.btn_serial_update.setEnabled(True)

    def update_fc_state(self, state: dict) -> None:
        pass


def main():
    app = QApplication(sys.argv)

    mwin = MainWindow()
    app.setStyleSheet(qdarktheme.load_stylesheet())
    mwin.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
