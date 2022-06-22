# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'guisumzrU.ui'
##
## Created by: Qt User Interface Compiler version 6.3.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QAbstractSpinBox, QApplication, QButtonGroup, QComboBox,
    QDoubleSpinBox, QFrame, QHBoxLayout, QLCDNumber,
    QLabel, QLineEdit, QMainWindow, QPlainTextEdit,
    QPushButton, QRadioButton, QSizePolicy, QVBoxLayout,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(644, 503)
        MainWindow.setMinimumSize(QSize(644, 503))
        MainWindow.setMaximumSize(QSize(644, 503))
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout_8 = QVBoxLayout(self.centralwidget)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        sizePolicy = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setMinimumSize(QSize(80, 0))
        font = QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.label.setFont(font)
        self.label.setFrameShape(QFrame.NoFrame)
        self.label.setMidLineWidth(0)
        self.label.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.label)

        self.lcd_h = QLCDNumber(self.centralwidget)
        self.lcd_h.setObjectName(u"lcd_h")
        self.lcd_h.setMinimumSize(QSize(0, 40))
        font1 = QFont()
        font1.setPointSize(7)
        self.lcd_h.setFont(font1)
        self.lcd_h.setFrameShape(QFrame.StyledPanel)
        self.lcd_h.setSmallDecimalPoint(True)
        self.lcd_h.setDigitCount(9)
        self.lcd_h.setMode(QLCDNumber.Dec)
        self.lcd_h.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_h.setProperty("value", 0.000000000000000)
        self.lcd_h.setProperty("intValue", 0)

        self.horizontalLayout.addWidget(self.lcd_h)

        self.horizontalLayout.setStretch(1, 1)

        self.verticalLayout.addLayout(self.horizontalLayout)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setMinimumSize(QSize(80, 0))
        self.label_2.setFont(font)
        self.label_2.setFrameShape(QFrame.NoFrame)
        self.label_2.setMidLineWidth(0)
        self.label_2.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_2.addWidget(self.label_2)

        self.lcd_d = QLCDNumber(self.centralwidget)
        self.lcd_d.setObjectName(u"lcd_d")
        self.lcd_d.setMinimumSize(QSize(0, 40))
        self.lcd_d.setFont(font1)
        self.lcd_d.setFrameShape(QFrame.StyledPanel)
        self.lcd_d.setSmallDecimalPoint(True)
        self.lcd_d.setDigitCount(9)
        self.lcd_d.setMode(QLCDNumber.Dec)
        self.lcd_d.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_d.setProperty("value", 0.000000000000000)
        self.lcd_d.setProperty("intValue", 0)

        self.horizontalLayout_2.addWidget(self.lcd_d)

        self.horizontalLayout_2.setStretch(1, 1)

        self.verticalLayout.addLayout(self.horizontalLayout_2)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_3 = QLabel(self.centralwidget)
        self.label_3.setObjectName(u"label_3")
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        self.label_3.setMinimumSize(QSize(80, 0))
        self.label_3.setFont(font)
        self.label_3.setFrameShape(QFrame.NoFrame)
        self.label_3.setMidLineWidth(0)
        self.label_3.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_3.addWidget(self.label_3)

        self.lcd_v = QLCDNumber(self.centralwidget)
        self.lcd_v.setObjectName(u"lcd_v")
        self.lcd_v.setMinimumSize(QSize(0, 40))
        self.lcd_v.setFont(font1)
        self.lcd_v.setFrameShape(QFrame.StyledPanel)
        self.lcd_v.setSmallDecimalPoint(True)
        self.lcd_v.setDigitCount(9)
        self.lcd_v.setMode(QLCDNumber.Dec)
        self.lcd_v.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_v.setProperty("value", 0.000000000000000)
        self.lcd_v.setProperty("intValue", 0)

        self.horizontalLayout_3.addWidget(self.lcd_v)

        self.horizontalLayout_3.setStretch(1, 1)

        self.verticalLayout.addLayout(self.horizontalLayout_3)


        self.horizontalLayout_7.addLayout(self.verticalLayout)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.label_6 = QLabel(self.centralwidget)
        self.label_6.setObjectName(u"label_6")
        sizePolicy.setHeightForWidth(self.label_6.sizePolicy().hasHeightForWidth())
        self.label_6.setSizePolicy(sizePolicy)
        self.label_6.setMinimumSize(QSize(80, 0))
        self.label_6.setFont(font)
        self.label_6.setFrameShape(QFrame.NoFrame)
        self.label_6.setMidLineWidth(0)
        self.label_6.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_6.addWidget(self.label_6)

        self.lcd_x = QLCDNumber(self.centralwidget)
        self.lcd_x.setObjectName(u"lcd_x")
        self.lcd_x.setMinimumSize(QSize(0, 40))
        self.lcd_x.setFont(font1)
        self.lcd_x.setFrameShape(QFrame.StyledPanel)
        self.lcd_x.setSmallDecimalPoint(True)
        self.lcd_x.setDigitCount(9)
        self.lcd_x.setMode(QLCDNumber.Dec)
        self.lcd_x.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_x.setProperty("value", 0.000000000000000)
        self.lcd_x.setProperty("intValue", 0)

        self.horizontalLayout_6.addWidget(self.lcd_x)

        self.horizontalLayout_6.setStretch(1, 1)

        self.verticalLayout_2.addLayout(self.horizontalLayout_6)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.label_4 = QLabel(self.centralwidget)
        self.label_4.setObjectName(u"label_4")
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        self.label_4.setMinimumSize(QSize(80, 0))
        self.label_4.setFont(font)
        self.label_4.setFrameShape(QFrame.NoFrame)
        self.label_4.setMidLineWidth(0)
        self.label_4.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_4.addWidget(self.label_4)

        self.lcd_y = QLCDNumber(self.centralwidget)
        self.lcd_y.setObjectName(u"lcd_y")
        self.lcd_y.setMinimumSize(QSize(0, 40))
        self.lcd_y.setFont(font1)
        self.lcd_y.setFrameShape(QFrame.StyledPanel)
        self.lcd_y.setSmallDecimalPoint(True)
        self.lcd_y.setDigitCount(9)
        self.lcd_y.setMode(QLCDNumber.Dec)
        self.lcd_y.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_y.setProperty("value", 0.000000000000000)
        self.lcd_y.setProperty("intValue", 0)

        self.horizontalLayout_4.addWidget(self.lcd_y)

        self.horizontalLayout_4.setStretch(1, 1)

        self.verticalLayout_2.addLayout(self.horizontalLayout_4)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.label_5 = QLabel(self.centralwidget)
        self.label_5.setObjectName(u"label_5")
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        self.label_5.setMinimumSize(QSize(80, 0))
        self.label_5.setFont(font)
        self.label_5.setFrameShape(QFrame.NoFrame)
        self.label_5.setMidLineWidth(0)
        self.label_5.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_5.addWidget(self.label_5)

        self.lcd_z = QLCDNumber(self.centralwidget)
        self.lcd_z.setObjectName(u"lcd_z")
        self.lcd_z.setMinimumSize(QSize(0, 40))
        self.lcd_z.setFont(font1)
        self.lcd_z.setFrameShape(QFrame.StyledPanel)
        self.lcd_z.setSmallDecimalPoint(True)
        self.lcd_z.setDigitCount(9)
        self.lcd_z.setMode(QLCDNumber.Dec)
        self.lcd_z.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_z.setProperty("value", 0.000000000000000)
        self.lcd_z.setProperty("intValue", 0)

        self.horizontalLayout_5.addWidget(self.lcd_z)

        self.horizontalLayout_5.setStretch(1, 1)

        self.verticalLayout_2.addLayout(self.horizontalLayout_5)


        self.horizontalLayout_7.addLayout(self.verticalLayout_2)


        self.verticalLayout_8.addLayout(self.horizontalLayout_7)

        self.horizontalLayout_15 = QHBoxLayout()
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.label_10 = QLabel(self.centralwidget)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setMinimumSize(QSize(82, 0))
        font2 = QFont()
        font2.setPointSize(11)
        self.label_10.setFont(font2)
        self.label_10.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_15.addWidget(self.label_10)

        self.line_info = QLineEdit(self.centralwidget)
        self.line_info.setObjectName(u"line_info")
        self.line_info.setFont(font2)
        self.line_info.setDragEnabled(True)
        self.line_info.setReadOnly(True)

        self.horizontalLayout_15.addWidget(self.line_info)

        self.horizontalLayout_15.setStretch(1, 1)

        self.verticalLayout_8.addLayout(self.horizontalLayout_15)

        self.horizontalLayout_16 = QHBoxLayout()
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.label_11 = QLabel(self.centralwidget)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setMinimumSize(QSize(82, 0))
        self.label_11.setFont(font2)
        self.label_11.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_16.addWidget(self.label_11)

        self.text_log = QPlainTextEdit(self.centralwidget)
        self.text_log.setObjectName(u"text_log")
        self.text_log.setFont(font2)
        self.text_log.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.text_log.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.text_log.setUndoRedoEnabled(False)
        self.text_log.setReadOnly(True)

        self.horizontalLayout_16.addWidget(self.text_log)

        self.btn_rgb = QPushButton(self.centralwidget)
        self.btn_rgb.setObjectName(u"btn_rgb")
        self.btn_rgb.setMinimumSize(QSize(0, 70))
        font3 = QFont()
        font3.setPointSize(11)
        font3.setBold(False)
        self.btn_rgb.setFont(font3)

        self.horizontalLayout_16.addWidget(self.btn_rgb)


        self.verticalLayout_8.addLayout(self.horizontalLayout_16)

        self.horizontalLayout_14 = QHBoxLayout()
        self.horizontalLayout_14.setObjectName(u"horizontalLayout_14")
        self.horizontalLayout_10 = QHBoxLayout()
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.btn_takeoff = QPushButton(self.centralwidget)
        self.btn_takeoff.setObjectName(u"btn_takeoff")
        self.btn_takeoff.setMinimumSize(QSize(0, 50))
        font4 = QFont()
        font4.setPointSize(12)
        font4.setBold(False)
        self.btn_takeoff.setFont(font4)

        self.verticalLayout_4.addWidget(self.btn_takeoff)

        self.btn_land = QPushButton(self.centralwidget)
        self.btn_land.setObjectName(u"btn_land")
        self.btn_land.setMinimumSize(QSize(0, 50))
        self.btn_land.setFont(font4)

        self.verticalLayout_4.addWidget(self.btn_land)

        self.btn_unlock = QPushButton(self.centralwidget)
        self.btn_unlock.setObjectName(u"btn_unlock")
        self.btn_unlock.setMinimumSize(QSize(0, 50))
        font5 = QFont()
        font5.setPointSize(13)
        font5.setBold(True)
        font5.setItalic(True)
        font5.setStyleStrategy(QFont.PreferAntialias)
        self.btn_unlock.setFont(font5)

        self.verticalLayout_4.addWidget(self.btn_unlock)

        self.btn_lock = QPushButton(self.centralwidget)
        self.btn_lock.setObjectName(u"btn_lock")
        self.btn_lock.setMinimumSize(QSize(0, 50))
        self.btn_lock.setFont(font5)

        self.verticalLayout_4.addWidget(self.btn_lock)


        self.horizontalLayout_10.addLayout(self.verticalLayout_4)

        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.horizontalLayout_8 = QHBoxLayout()
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.btn_l_turn = QPushButton(self.centralwidget)
        self.btn_l_turn.setObjectName(u"btn_l_turn")
        self.btn_l_turn.setMinimumSize(QSize(0, 70))
        font6 = QFont()
        font6.setPointSize(20)
        font6.setBold(False)
        self.btn_l_turn.setFont(font6)

        self.horizontalLayout_8.addWidget(self.btn_l_turn)

        self.btn_forward = QPushButton(self.centralwidget)
        self.btn_forward.setObjectName(u"btn_forward")
        self.btn_forward.setMinimumSize(QSize(0, 70))
        self.btn_forward.setFont(font6)

        self.horizontalLayout_8.addWidget(self.btn_forward)

        self.btn_r_turn = QPushButton(self.centralwidget)
        self.btn_r_turn.setObjectName(u"btn_r_turn")
        self.btn_r_turn.setMinimumSize(QSize(0, 70))
        self.btn_r_turn.setFont(font6)

        self.horizontalLayout_8.addWidget(self.btn_r_turn)


        self.verticalLayout_5.addLayout(self.horizontalLayout_8)

        self.horizontalLayout_9 = QHBoxLayout()
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.btn_left = QPushButton(self.centralwidget)
        self.btn_left.setObjectName(u"btn_left")
        self.btn_left.setMinimumSize(QSize(0, 70))
        self.btn_left.setFont(font6)

        self.horizontalLayout_9.addWidget(self.btn_left)

        self.btn_backward = QPushButton(self.centralwidget)
        self.btn_backward.setObjectName(u"btn_backward")
        self.btn_backward.setMinimumSize(QSize(0, 70))
        self.btn_backward.setFont(font6)

        self.horizontalLayout_9.addWidget(self.btn_backward)

        self.btn_right = QPushButton(self.centralwidget)
        self.btn_right.setObjectName(u"btn_right")
        self.btn_right.setMinimumSize(QSize(0, 70))
        self.btn_right.setFont(font6)

        self.horizontalLayout_9.addWidget(self.btn_right)


        self.verticalLayout_5.addLayout(self.horizontalLayout_9)


        self.horizontalLayout_10.addLayout(self.verticalLayout_5)

        self.verticalLayout_6 = QVBoxLayout()
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.btn_up = QPushButton(self.centralwidget)
        self.btn_up.setObjectName(u"btn_up")
        self.btn_up.setMinimumSize(QSize(0, 50))
        self.btn_up.setFont(font6)

        self.verticalLayout_6.addWidget(self.btn_up)

        self.btn_stop = QPushButton(self.centralwidget)
        self.btn_stop.setObjectName(u"btn_stop")
        self.btn_stop.setMinimumSize(QSize(0, 50))
        self.btn_stop.setFont(font6)

        self.verticalLayout_6.addWidget(self.btn_stop)

        self.btn_down = QPushButton(self.centralwidget)
        self.btn_down.setObjectName(u"btn_down")
        self.btn_down.setMinimumSize(QSize(0, 50))
        self.btn_down.setFont(font6)

        self.verticalLayout_6.addWidget(self.btn_down)


        self.horizontalLayout_10.addLayout(self.verticalLayout_6)

        self.verticalLayout_7 = QVBoxLayout()
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.horizontalLayout_17 = QHBoxLayout()
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.combo_serial = QComboBox(self.centralwidget)
        self.combo_serial.addItem("")
        self.combo_serial.setObjectName(u"combo_serial")
        font7 = QFont()
        font7.setPointSize(9)
        self.combo_serial.setFont(font7)

        self.horizontalLayout_17.addWidget(self.combo_serial)

        self.btn_serial_update = QPushButton(self.centralwidget)
        self.btn_serial_update.setObjectName(u"btn_serial_update")
        font8 = QFont()
        font8.setPointSize(10)
        font8.setKerning(True)
        self.btn_serial_update.setFont(font8)
        self.btn_serial_update.setIconSize(QSize(0, 0))

        self.horizontalLayout_17.addWidget(self.btn_serial_update)

        self.horizontalLayout_17.setStretch(0, 1)

        self.verticalLayout_7.addLayout(self.horizontalLayout_17)

        self.horizontalLayout_20 = QHBoxLayout()
        self.horizontalLayout_20.setObjectName(u"horizontalLayout_20")
        self.radio_control_realtime = QRadioButton(self.centralwidget)
        self.buttonGroup = QButtonGroup(MainWindow)
        self.buttonGroup.setObjectName(u"buttonGroup")
        self.buttonGroup.addButton(self.radio_control_realtime)
        self.radio_control_realtime.setObjectName(u"radio_control_realtime")
        self.radio_control_realtime.setChecked(True)

        self.horizontalLayout_20.addWidget(self.radio_control_realtime)

        self.radio_control_flow = QRadioButton(self.centralwidget)
        self.buttonGroup.addButton(self.radio_control_flow)
        self.radio_control_flow.setObjectName(u"radio_control_flow")

        self.horizontalLayout_20.addWidget(self.radio_control_flow)


        self.verticalLayout_7.addLayout(self.horizontalLayout_20)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.label_7 = QLabel(self.centralwidget)
        self.label_7.setObjectName(u"label_7")
        font9 = QFont()
        font9.setPointSize(12)
        self.label_7.setFont(font9)

        self.horizontalLayout_11.addWidget(self.label_7)

        self.box_hori_param = QDoubleSpinBox(self.centralwidget)
        self.box_hori_param.setObjectName(u"box_hori_param")
        self.box_hori_param.setFont(font9)
        self.box_hori_param.setDecimals(1)
        self.box_hori_param.setMaximum(100.000000000000000)
        self.box_hori_param.setSingleStep(5.000000000000000)
        self.box_hori_param.setValue(20.000000000000000)

        self.horizontalLayout_11.addWidget(self.box_hori_param)

        self.horizontalLayout_11.setStretch(1, 1)

        self.verticalLayout_7.addLayout(self.horizontalLayout_11)

        self.horizontalLayout_12 = QHBoxLayout()
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.label_8 = QLabel(self.centralwidget)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setFont(font9)

        self.horizontalLayout_12.addWidget(self.label_8)

        self.box_vert_param = QDoubleSpinBox(self.centralwidget)
        self.box_vert_param.setObjectName(u"box_vert_param")
        self.box_vert_param.setMinimumSize(QSize(100, 0))
        self.box_vert_param.setFont(font9)
        self.box_vert_param.setDecimals(1)
        self.box_vert_param.setMaximum(100.000000000000000)
        self.box_vert_param.setSingleStep(5.000000000000000)
        self.box_vert_param.setValue(20.000000000000000)

        self.horizontalLayout_12.addWidget(self.box_vert_param)

        self.horizontalLayout_12.setStretch(1, 1)

        self.verticalLayout_7.addLayout(self.horizontalLayout_12)

        self.horizontalLayout_21 = QHBoxLayout()
        self.horizontalLayout_21.setObjectName(u"horizontalLayout_21")
        self.label_12 = QLabel(self.centralwidget)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setFont(font9)

        self.horizontalLayout_21.addWidget(self.label_12)

        self.box_spin_param = QDoubleSpinBox(self.centralwidget)
        self.box_spin_param.setObjectName(u"box_spin_param")
        self.box_spin_param.setFont(font9)
        self.box_spin_param.setDecimals(1)
        self.box_spin_param.setMaximum(100.000000000000000)
        self.box_spin_param.setSingleStep(5.000000000000000)
        self.box_spin_param.setValue(30.000000000000000)

        self.horizontalLayout_21.addWidget(self.box_spin_param)

        self.horizontalLayout_21.setStretch(1, 1)

        self.verticalLayout_7.addLayout(self.horizontalLayout_21)

        self.horizontalLayout_13 = QHBoxLayout()
        self.horizontalLayout_13.setObjectName(u"horizontalLayout_13")
        self.label_9 = QLabel(self.centralwidget)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setFont(font9)

        self.horizontalLayout_13.addWidget(self.label_9)

        self.box_height = QDoubleSpinBox(self.centralwidget)
        self.box_height.setObjectName(u"box_height")
        self.box_height.setMinimumSize(QSize(100, 0))
        self.box_height.setFont(font9)
        self.box_height.setDecimals(0)
        self.box_height.setMaximum(200.000000000000000)
        self.box_height.setSingleStep(10.000000000000000)
        self.box_height.setStepType(QAbstractSpinBox.DefaultStepType)
        self.box_height.setValue(50.000000000000000)

        self.horizontalLayout_13.addWidget(self.box_height)

        self.horizontalLayout_13.setStretch(1, 1)

        self.verticalLayout_7.addLayout(self.horizontalLayout_13)

        self.btn_set_h = QPushButton(self.centralwidget)
        self.btn_set_h.setObjectName(u"btn_set_h")
        font10 = QFont()
        font10.setPointSize(10)
        self.btn_set_h.setFont(font10)
        self.btn_set_h.setIconSize(QSize(8, 16))

        self.verticalLayout_7.addWidget(self.btn_set_h)


        self.horizontalLayout_10.addLayout(self.verticalLayout_7)

        self.horizontalLayout_10.setStretch(0, 1)
        self.horizontalLayout_10.setStretch(1, 3)
        self.horizontalLayout_10.setStretch(2, 1)

        self.horizontalLayout_14.addLayout(self.horizontalLayout_10)

        self.horizontalLayout_14.setStretch(0, 1)

        self.verticalLayout_8.addLayout(self.horizontalLayout_14)

        self.verticalLayout_8.setStretch(2, 1)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"\u4e0a\u4f4d\u673a", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"\u6fc0\u5149\u9ad8\u5ea6", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"\u822a\u5411\u89d2", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"\u7535\u6c60\u7535\u538b", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"X-\u901f\u5ea6", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Y-\u901f\u5ea6", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Z-\u901f\u5ea6", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"\u5176\u4ed6\u72b6\u6001:", None))
        self.line_info.setText(QCoreApplication.translate("MainWindow", u"\u6d4b\u8bd5\u6587\u672c", None))
        self.line_info.setPlaceholderText("")
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"\u7cfb\u7edf\u65e5\u5fd7:", None))
        self.text_log.setPlainText(QCoreApplication.translate("MainWindow", u"\u6d4b\u8bd5\u6587\u672c", None))
        self.btn_rgb.setText(QCoreApplication.translate("MainWindow", u"\u8bbe\u7f6eRGB", None))
#if QT_CONFIG(shortcut)
        self.btn_rgb.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_takeoff.setText(QCoreApplication.translate("MainWindow", u"\u8d77\u98de", None))
#if QT_CONFIG(shortcut)
        self.btn_takeoff.setShortcut(QCoreApplication.translate("MainWindow", u"X", None))
#endif // QT_CONFIG(shortcut)
        self.btn_land.setText(QCoreApplication.translate("MainWindow", u"\u7740\u9646", None))
#if QT_CONFIG(shortcut)
        self.btn_land.setShortcut(QCoreApplication.translate("MainWindow", u"C", None))
#endif // QT_CONFIG(shortcut)
        self.btn_unlock.setText(QCoreApplication.translate("MainWindow", u"\u89e3\u9501", None))
#if QT_CONFIG(shortcut)
        self.btn_unlock.setShortcut(QCoreApplication.translate("MainWindow", u"O", None))
#endif // QT_CONFIG(shortcut)
        self.btn_lock.setText(QCoreApplication.translate("MainWindow", u"\u9501\u5b9a", None))
#if QT_CONFIG(shortcut)
        self.btn_lock.setShortcut(QCoreApplication.translate("MainWindow", u"P", None))
#endif // QT_CONFIG(shortcut)
        self.btn_l_turn.setText(QCoreApplication.translate("MainWindow", u"\u21ba", None))
#if QT_CONFIG(shortcut)
        self.btn_l_turn.setShortcut(QCoreApplication.translate("MainWindow", u"Q", None))
#endif // QT_CONFIG(shortcut)
        self.btn_forward.setText(QCoreApplication.translate("MainWindow", u"\u2191", None))
#if QT_CONFIG(shortcut)
        self.btn_forward.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_r_turn.setText(QCoreApplication.translate("MainWindow", u"\u21bb", None))
#if QT_CONFIG(shortcut)
        self.btn_r_turn.setShortcut(QCoreApplication.translate("MainWindow", u"E", None))
#endif // QT_CONFIG(shortcut)
        self.btn_left.setText(QCoreApplication.translate("MainWindow", u"\u2190", None))
#if QT_CONFIG(shortcut)
        self.btn_left.setShortcut(QCoreApplication.translate("MainWindow", u"A", None))
#endif // QT_CONFIG(shortcut)
        self.btn_backward.setText(QCoreApplication.translate("MainWindow", u"\u2193", None))
#if QT_CONFIG(shortcut)
        self.btn_backward.setShortcut(QCoreApplication.translate("MainWindow", u"S", None))
#endif // QT_CONFIG(shortcut)
        self.btn_right.setText(QCoreApplication.translate("MainWindow", u"\u2192", None))
#if QT_CONFIG(shortcut)
        self.btn_right.setShortcut(QCoreApplication.translate("MainWindow", u"D", None))
#endif // QT_CONFIG(shortcut)
#if QT_CONFIG(tooltip)
        self.btn_up.setToolTip(QCoreApplication.translate("MainWindow", u"\u4e0a\u5347", None))
#endif // QT_CONFIG(tooltip)
        self.btn_up.setText(QCoreApplication.translate("MainWindow", u"\u21e1", None))
#if QT_CONFIG(shortcut)
        self.btn_up.setShortcut(QCoreApplication.translate("MainWindow", u"R", None))
#endif // QT_CONFIG(shortcut)
#if QT_CONFIG(tooltip)
        self.btn_stop.setToolTip(QCoreApplication.translate("MainWindow", u"\u505c\u6b62\u8fd0\u52a8", None))
#endif // QT_CONFIG(tooltip)
        self.btn_stop.setText(QCoreApplication.translate("MainWindow", u"\u00d7", None))
#if QT_CONFIG(shortcut)
        self.btn_stop.setShortcut(QCoreApplication.translate("MainWindow", u"Z", None))
#endif // QT_CONFIG(shortcut)
#if QT_CONFIG(tooltip)
        self.btn_down.setToolTip(QCoreApplication.translate("MainWindow", u"\u4e0b\u964d", None))
#endif // QT_CONFIG(tooltip)
        self.btn_down.setText(QCoreApplication.translate("MainWindow", u"\u21e3", None))
#if QT_CONFIG(shortcut)
        self.btn_down.setShortcut(QCoreApplication.translate("MainWindow", u"F", None))
#endif // QT_CONFIG(shortcut)
        self.combo_serial.setItemText(0, QCoreApplication.translate("MainWindow", u"\u65ad\u5f00\u8fde\u63a5", None))

        self.btn_serial_update.setText(QCoreApplication.translate("MainWindow", u"\u5237\u65b0", None))
        self.radio_control_realtime.setText(QCoreApplication.translate("MainWindow", u"\u5b9e\u65f6\u63a7\u5236", None))
#if QT_CONFIG(shortcut)
        self.radio_control_realtime.setShortcut(QCoreApplication.translate("MainWindow", u"2", None))
#endif // QT_CONFIG(shortcut)
        self.radio_control_flow.setText(QCoreApplication.translate("MainWindow", u"\u6d41\u7a0b\u63a7\u5236", None))
#if QT_CONFIG(shortcut)
        self.radio_control_flow.setShortcut(QCoreApplication.translate("MainWindow", u"3", None))
#endif // QT_CONFIG(shortcut)
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"\u6c34\u5e73\u901f\u5ea6", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"\u5782\u76f4\u901f\u5ea6", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"\u65cb\u8f6c\u901f\u5ea6", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"\u5b9a\u9ad8\u9ad8\u5ea6", None))
        self.btn_set_h.setText(QCoreApplication.translate("MainWindow", u"\u5b9a\u9ad8\u4f5c\u52a8", None))
    # retranslateUi

