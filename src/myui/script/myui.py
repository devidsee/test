# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'myui.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1402, 935)
        MainWindow.setStyleSheet("color: rgb(0, 0, 0);\n"
"background-color: rgb(211, 215, 207);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pic1 = QtWidgets.QGraphicsView(self.centralwidget)
        self.pic1.setGeometry(QtCore.QRect(20, 30, 251, 221))
        self.pic1.setObjectName("pic1")
        self.tab_control = QtWidgets.QTabWidget(self.centralwidget)
        self.tab_control.setGeometry(QtCore.QRect(350, 0, 411, 391))
        self.tab_control.setObjectName("tab_control")
        self.tab_con1 = QtWidgets.QWidget()
        self.tab_con1.setObjectName("tab_con1")
        self.rb_switch = SwitchBtn(self.tab_con1)
        self.rb_switch.setGeometry(QtCore.QRect(10, 70, 71, 21))
        self.rb_switch.setSizeIncrement(QtCore.QSize(20, 10))
        self.rb_switch.setBaseSize(QtCore.QSize(20, 10))
        self.rb_switch.setObjectName("rb_switch")
        self.rb_push = SwitchBtn(self.tab_con1)
        self.rb_push.setGeometry(QtCore.QRect(120, 70, 81, 21))
        self.rb_push.setObjectName("rb_push")
        self.rb_temp = SwitchBtn(self.tab_con1)
        self.rb_temp.setGeometry(QtCore.QRect(10, 190, 71, 21))
        self.rb_temp.setObjectName("rb_temp")
        self.rb_layer = SwitchBtn(self.tab_con1)
        self.rb_layer.setGeometry(QtCore.QRect(260, 70, 91, 21))
        self.rb_layer.setObjectName("rb_layer")
        self.rb_scan = SwitchBtn(self.tab_con1)
        self.rb_scan.setGeometry(QtCore.QRect(120, 190, 81, 21))
        self.rb_scan.setObjectName("rb_scan")
        self.label_2 = QtWidgets.QLabel(self.tab_con1)
        self.label_2.setGeometry(QtCore.QRect(140, 40, 30, 23))
        self.label_2.setObjectName("label_2")
        self.label_5 = QtWidgets.QLabel(self.tab_con1)
        self.label_5.setGeometry(QtCore.QRect(30, 150, 30, 23))
        self.label_5.setObjectName("label_5")
        self.label = QtWidgets.QLabel(self.tab_con1)
        self.label.setGeometry(QtCore.QRect(290, 40, 30, 23))
        self.label.setObjectName("label")
        self.label_20 = QtWidgets.QLabel(self.tab_con1)
        self.label_20.setGeometry(QtCore.QRect(140, 150, 30, 23))
        self.label_20.setObjectName("label_20")
        self.label_7 = QtWidgets.QLabel(self.tab_con1)
        self.label_7.setGeometry(QtCore.QRect(30, 40, 45, 23))
        self.label_7.setSizeIncrement(QtCore.QSize(40, 20))
        self.label_7.setBaseSize(QtCore.QSize(40, 20))
        self.label_7.setObjectName("label_7")
        self.push_sb = QtWidgets.QSpinBox(self.tab_con1)
        self.push_sb.setGeometry(QtCore.QRect(190, 100, 44, 25))
        self.push_sb.setObjectName("push_sb")
        self.tem = QtWidgets.QLabel(self.tab_con1)
        self.tem.setGeometry(QtCore.QRect(20, 230, 56, 17))
        self.tem.setObjectName("tem")
        self.layer_sb = QtWidgets.QSpinBox(self.tab_con1)
        self.layer_sb.setGeometry(QtCore.QRect(350, 100, 44, 25))
        self.layer_sb.setObjectName("layer_sb")
        self.rob_start = QtWidgets.QPushButton(self.tab_con1)
        self.rob_start.setGeometry(QtCore.QRect(20, 100, 61, 31))
        self.rob_start.setObjectName("rob_start")
        self.layer_pro = QtWidgets.QProgressBar(self.tab_con1)
        self.layer_pro.setGeometry(QtCore.QRect(240, 100, 95, 25))
        self.layer_pro.setProperty("value", 24)
        self.layer_pro.setObjectName("layer_pro")
        self.push_pro = QtWidgets.QProgressBar(self.tab_con1)
        self.push_pro.setGeometry(QtCore.QRect(100, 100, 81, 25))
        self.push_pro.setProperty("value", 24)
        self.push_pro.setObjectName("push_pro")
        self.tab_control.addTab(self.tab_con1, "")
        self.tab_con2 = QtWidgets.QWidget()
        self.tab_con2.setObjectName("tab_con2")
        self.layoutWidget = QtWidgets.QWidget(self.tab_con2)
        self.layoutWidget.setGeometry(QtCore.QRect(20, 20, 341, 315))
        self.layoutWidget.setObjectName("layoutWidget")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.layoutWidget)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.label_3 = QtWidgets.QLabel(self.layoutWidget)
        self.label_3.setObjectName("label_3")
        self.gridLayout_3.addWidget(self.label_3, 0, 0, 1, 1)
        self.ip_internet = QtWidgets.QLabel(self.layoutWidget)
        self.ip_internet.setObjectName("ip_internet")
        self.gridLayout_3.addWidget(self.ip_internet, 0, 1, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.layoutWidget)
        self.label_4.setObjectName("label_4")
        self.gridLayout_3.addWidget(self.label_4, 1, 0, 1, 1)
        self.ip_i6217 = QtWidgets.QLabel(self.layoutWidget)
        self.ip_i6217.setObjectName("ip_i6217")
        self.gridLayout_3.addWidget(self.ip_i6217, 1, 1, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.layoutWidget)
        self.label_6.setObjectName("label_6")
        self.gridLayout_3.addWidget(self.label_6, 2, 0, 1, 1)
        self.ip_o6217 = QtWidgets.QLabel(self.layoutWidget)
        self.ip_o6217.setObjectName("ip_o6217")
        self.gridLayout_3.addWidget(self.ip_o6217, 2, 1, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.layoutWidget)
        self.label_8.setObjectName("label_8")
        self.gridLayout_3.addWidget(self.label_8, 3, 0, 1, 1)
        self.io_ip100 = QtWidgets.QLabel(self.layoutWidget)
        self.io_ip100.setObjectName("io_ip100")
        self.gridLayout_3.addWidget(self.io_ip100, 3, 1, 1, 1)
        self.label_17 = QtWidgets.QLabel(self.layoutWidget)
        self.label_17.setObjectName("label_17")
        self.gridLayout_3.addWidget(self.label_17, 4, 0, 1, 1)
        self.io_ip130 = QtWidgets.QLabel(self.layoutWidget)
        self.io_ip130.setObjectName("io_ip130")
        self.gridLayout_3.addWidget(self.io_ip130, 4, 1, 1, 1)
        self.label_18 = QtWidgets.QLabel(self.layoutWidget)
        self.label_18.setObjectName("label_18")
        self.gridLayout_3.addWidget(self.label_18, 5, 0, 1, 1)
        self.io_ip140 = QtWidgets.QLabel(self.layoutWidget)
        self.io_ip140.setObjectName("io_ip140")
        self.gridLayout_3.addWidget(self.io_ip140, 5, 1, 1, 1)
        self.label_19 = QtWidgets.QLabel(self.layoutWidget)
        self.label_19.setObjectName("label_19")
        self.gridLayout_3.addWidget(self.label_19, 6, 0, 1, 1)
        self.io_ip150 = QtWidgets.QLabel(self.layoutWidget)
        self.io_ip150.setObjectName("io_ip150")
        self.gridLayout_3.addWidget(self.io_ip150, 6, 1, 1, 1)
        self.label_23 = QtWidgets.QLabel(self.layoutWidget)
        self.label_23.setObjectName("label_23")
        self.gridLayout_3.addWidget(self.label_23, 7, 0, 1, 1)
        self.io_ip150_2 = QtWidgets.QLabel(self.layoutWidget)
        self.io_ip150_2.setObjectName("io_ip150_2")
        self.gridLayout_3.addWidget(self.io_ip150_2, 7, 1, 1, 1)
        self.label_24 = QtWidgets.QLabel(self.layoutWidget)
        self.label_24.setObjectName("label_24")
        self.gridLayout_3.addWidget(self.label_24, 8, 0, 1, 1)
        self.io_ip150_3 = QtWidgets.QLabel(self.layoutWidget)
        self.io_ip150_3.setObjectName("io_ip150_3")
        self.gridLayout_3.addWidget(self.io_ip150_3, 8, 1, 1, 1)
        self.label_25 = QtWidgets.QLabel(self.layoutWidget)
        self.label_25.setObjectName("label_25")
        self.gridLayout_3.addWidget(self.label_25, 9, 0, 1, 1)
        self.io_ip150_4 = QtWidgets.QLabel(self.layoutWidget)
        self.io_ip150_4.setObjectName("io_ip150_4")
        self.gridLayout_3.addWidget(self.io_ip150_4, 9, 1, 1, 1)
        self.label_26 = QtWidgets.QLabel(self.layoutWidget)
        self.label_26.setObjectName("label_26")
        self.gridLayout_3.addWidget(self.label_26, 10, 0, 1, 1)
        self.io_ip150_5 = QtWidgets.QLabel(self.layoutWidget)
        self.io_ip150_5.setObjectName("io_ip150_5")
        self.gridLayout_3.addWidget(self.io_ip150_5, 10, 1, 1, 1)
        self.tab_control.addTab(self.tab_con2, "")
        self.pic2 = QtWidgets.QGraphicsView(self.centralwidget)
        self.pic2.setGeometry(QtCore.QRect(20, 300, 251, 241))
        self.pic2.setObjectName("pic2")
        self.label_21 = QtWidgets.QLabel(self.centralwidget)
        self.label_21.setGeometry(QtCore.QRect(30, 10, 67, 17))
        self.label_21.setObjectName("label_21")
        self.label_22 = QtWidgets.QLabel(self.centralwidget)
        self.label_22.setGeometry(QtCore.QRect(30, 270, 67, 17))
        self.label_22.setObjectName("label_22")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1402, 28))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tab_control.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_2.setText(_translate("MainWindow", "送粉"))
        self.label_5.setText(_translate("MainWindow", "温度"))
        self.label.setText(_translate("MainWindow", "激光"))
        self.label_20.setText(_translate("MainWindow", "扫描"))
        self.label_7.setText(_translate("MainWindow", "机器人"))
        self.tem.setText(_translate("MainWindow", "            °C"))
        self.rob_start.setText(_translate("MainWindow", "开始"))
        self.tab_control.setTabText(self.tab_control.indexOf(self.tab_con1), _translate("MainWindow", "控制"))
        self.label_3.setText(_translate("MainWindow", "以太网串口"))
        self.ip_internet.setText(_translate("MainWindow", "192.168.0.178"))
        self.label_4.setText(_translate("MainWindow", "远程模拟输入6217"))
        self.ip_i6217.setText(_translate("MainWindow", "192.168.0.100"))
        self.label_6.setText(_translate("MainWindow", "远程模拟输出6224"))
        self.ip_o6217.setText(_translate("MainWindow", "192.168.0.120"))
        self.label_8.setText(_translate("MainWindow", "远程数字I/O6052"))
        self.io_ip100.setText(_translate("MainWindow", "192.168.0.110"))
        self.label_17.setText(_translate("MainWindow", "远程数字I/O6052"))
        self.io_ip130.setText(_translate("MainWindow", "192.168.0.130"))
        self.label_18.setText(_translate("MainWindow", "远程数字I/O6052"))
        self.io_ip140.setText(_translate("MainWindow", "192.168.0.140"))
        self.label_19.setText(_translate("MainWindow", "远程数字I/O6052"))
        self.io_ip150.setText(_translate("MainWindow", "192.168.0.150"))
        self.label_23.setText(_translate("MainWindow", "相机1"))
        self.io_ip150_2.setText(_translate("MainWindow", "192.168.0.300"))
        self.label_24.setText(_translate("MainWindow", "相机2"))
        self.io_ip150_3.setText(_translate("MainWindow", "192.168.0.400"))
        self.label_25.setText(_translate("MainWindow", "机器人"))
        self.io_ip150_4.setText(_translate("MainWindow", "192.168.0.500"))
        self.label_26.setText(_translate("MainWindow", "激光扫描"))
        self.io_ip150_5.setText(_translate("MainWindow", "192.168.0.600"))
        self.tab_control.setTabText(self.tab_control.indexOf(self.tab_con2), _translate("MainWindow", "连接"))
        self.label_21.setText(_translate("MainWindow", "相机1图片"))
        self.label_22.setText(_translate("MainWindow", "相机2图片"))
from SwitchBtn import SwitchBtn
