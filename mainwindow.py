# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created: Thu May  3 14:44:22 2018
#      by: PyQt5 UI code generator 5.3.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 420)
        self.centralWidget = QtWidgets.QWidget(MainWindow)
        self.centralWidget.setObjectName("centralWidget")
        self.RPM = QtWidgets.QLCDNumber(self.centralWidget)
        self.RPM.setGeometry(QtCore.QRect(260, 150, 81, 51))
        self.RPM.setFrameShape(QtWidgets.QFrame.Panel)
        self.RPM.setFrameShadow(QtWidgets.QFrame.Raised)
        self.RPM.setLineWidth(5)
        self.RPM.setSmallDecimalPoint(True)
        self.RPM.setDigitCount(4)
        self.RPM.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.RPM.setObjectName("RPM")
        self.light = QtWidgets.QCheckBox(self.centralWidget)
        self.light.setGeometry(QtCore.QRect(610, 40, 100, 51))
        self.light.setSizeIncrement(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        font.setStrikeOut(False)
        self.light.setFont(font)
        self.light.setIconSize(QtCore.QSize(50, 50))
        self.light.setCheckable(True)
        self.light.setChecked(False)
        self.light.setAutoRepeat(False)
        self.light.setAutoExclusive(False)
        self.light.setTristate(False)
        self.light.setObjectName("light")
        self.waterTemp = QtWidgets.QLCDNumber(self.centralWidget)
        self.waterTemp.setGeometry(QtCore.QRect(70, 260, 81, 51))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(12)
        font.setItalic(False)
        self.waterTemp.setFont(font)
        self.waterTemp.setMouseTracking(False)
        self.waterTemp.setInputMethodHints(QtCore.Qt.ImhNone)
        self.waterTemp.setFrameShape(QtWidgets.QFrame.Panel)
        self.waterTemp.setFrameShadow(QtWidgets.QFrame.Raised)
        self.waterTemp.setLineWidth(5)
        self.waterTemp.setMidLineWidth(0)
        self.waterTemp.setSmallDecimalPoint(True)
        self.waterTemp.setDigitCount(3)
        self.waterTemp.setMode(QtWidgets.QLCDNumber.Dec)
        self.waterTemp.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.waterTemp.setProperty("value", 0.0)
        self.waterTemp.setProperty("intValue", 0)
        self.waterTemp.setObjectName("waterTemp")
        self.label_3 = QtWidgets.QLabel(self.centralWidget)
        self.label_3.setGeometry(QtCore.QRect(30, 220, 161, 31))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.label_2 = QtWidgets.QLabel(self.centralWidget)
        self.label_2.setGeometry(QtCore.QRect(50, 110, 121, 31))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.label = QtWidgets.QLabel(self.centralWidget)
        self.label.setGeometry(QtCore.QRect(70, 0, 91, 31))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.airTemp = QtWidgets.QLCDNumber(self.centralWidget)
        self.airTemp.setGeometry(QtCore.QRect(70, 150, 81, 51))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(12)
        font.setItalic(False)
        self.airTemp.setFont(font)
        self.airTemp.setMouseTracking(False)
        self.airTemp.setInputMethodHints(QtCore.Qt.ImhNone)
        self.airTemp.setFrameShape(QtWidgets.QFrame.Panel)
        self.airTemp.setFrameShadow(QtWidgets.QFrame.Raised)
        self.airTemp.setLineWidth(5)
        self.airTemp.setMidLineWidth(0)
        self.airTemp.setSmallDecimalPoint(True)
        self.airTemp.setDigitCount(3)
        self.airTemp.setMode(QtWidgets.QLCDNumber.Dec)
        self.airTemp.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.airTemp.setProperty("value", 0.0)
        self.airTemp.setProperty("intValue", 0)
        self.airTemp.setObjectName("airTemp")
        self.label_4 = QtWidgets.QLabel(self.centralWidget)
        self.label_4.setGeometry(QtCore.QRect(280, 0, 41, 31))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.pH = QtWidgets.QLCDNumber(self.centralWidget)
        self.pH.setGeometry(QtCore.QRect(260, 40, 81, 51))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(12)
        font.setItalic(False)
        self.pH.setFont(font)
        self.pH.setMouseTracking(False)
        self.pH.setInputMethodHints(QtCore.Qt.ImhNone)
        self.pH.setFrameShape(QtWidgets.QFrame.Panel)
        self.pH.setFrameShadow(QtWidgets.QFrame.Raised)
        self.pH.setLineWidth(5)
        self.pH.setMidLineWidth(0)
        self.pH.setSmallDecimalPoint(True)
        self.pH.setDigitCount(3)
        self.pH.setMode(QtWidgets.QLCDNumber.Dec)
        self.pH.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.pH.setProperty("value", 0.0)
        self.pH.setProperty("intValue", 0)
        self.pH.setObjectName("pH")
        self.weight = QtWidgets.QLCDNumber(self.centralWidget)
        self.weight.setGeometry(QtCore.QRect(70, 40, 81, 51))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(12)
        font.setItalic(False)
        self.weight.setFont(font)
        self.weight.setMouseTracking(False)
        self.weight.setInputMethodHints(QtCore.Qt.ImhNone)
        self.weight.setFrameShape(QtWidgets.QFrame.Panel)
        self.weight.setFrameShadow(QtWidgets.QFrame.Raised)
        self.weight.setLineWidth(5)
        self.weight.setMidLineWidth(0)
        self.weight.setSmallDecimalPoint(True)
        self.weight.setDigitCount(3)
        self.weight.setMode(QtWidgets.QLCDNumber.Dec)
        self.weight.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.weight.setProperty("value", 0.0)
        self.weight.setProperty("intValue", 0)
        self.weight.setObjectName("weight")
        self.label_5 = QtWidgets.QLabel(self.centralWidget)
        self.label_5.setGeometry(QtCore.QRect(270, 120, 61, 31))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.flow = QtWidgets.QLCDNumber(self.centralWidget)
        self.flow.setGeometry(QtCore.QRect(260, 260, 81, 51))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(12)
        font.setItalic(False)
        self.flow.setFont(font)
        self.flow.setMouseTracking(False)
        self.flow.setInputMethodHints(QtCore.Qt.ImhNone)
        self.flow.setFrameShape(QtWidgets.QFrame.Panel)
        self.flow.setFrameShadow(QtWidgets.QFrame.Raised)
        self.flow.setLineWidth(5)
        self.flow.setMidLineWidth(0)
        self.flow.setSmallDecimalPoint(True)
        self.flow.setDigitCount(3)
        self.flow.setMode(QtWidgets.QLCDNumber.Dec)
        self.flow.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.flow.setProperty("value", 0.0)
        self.flow.setProperty("intValue", 0)
        self.flow.setObjectName("flow")
        self.label_6 = QtWidgets.QLabel(self.centralWidget)
        self.label_6.setGeometry(QtCore.QRect(270, 220, 61, 31))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.pDiff = QtWidgets.QLCDNumber(self.centralWidget)
        self.pDiff.setGeometry(QtCore.QRect(410, 150, 81, 51))
        self.pDiff.setFrameShape(QtWidgets.QFrame.Panel)
        self.pDiff.setFrameShadow(QtWidgets.QFrame.Raised)
        self.pDiff.setLineWidth(5)
        self.pDiff.setSmallDecimalPoint(True)
        self.pDiff.setDigitCount(4)
        self.pDiff.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.pDiff.setObjectName("pDiff")
        self.label_7 = QtWidgets.QLabel(self.centralWidget)
        self.label_7.setGeometry(QtCore.QRect(390, 90, 121, 31))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.centralWidget)
        self.label_8.setGeometry(QtCore.QRect(380, 120, 141, 31))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.pump = QtWidgets.QCheckBox(self.centralWidget)
        self.pump.setGeometry(QtCore.QRect(610, 120, 100, 51))
        self.pump.setSizeIncrement(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        font.setStrikeOut(False)
        self.pump.setFont(font)
        self.pump.setIconSize(QtCore.QSize(50, 50))
        self.pump.setCheckable(True)
        self.pump.setChecked(False)
        self.pump.setAutoRepeat(False)
        self.pump.setAutoExclusive(False)
        self.pump.setTristate(False)
        self.pump.setObjectName("pump")
        self.label_9 = QtWidgets.QLabel(self.centralWidget)
        self.label_9.setGeometry(QtCore.QRect(380, 220, 141, 31))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.lux = QtWidgets.QLCDNumber(self.centralWidget)
        self.lux.setGeometry(QtCore.QRect(410, 260, 81, 51))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(12)
        font.setItalic(False)
        self.lux.setFont(font)
        self.lux.setMouseTracking(False)
        self.lux.setInputMethodHints(QtCore.Qt.ImhNone)
        self.lux.setFrameShape(QtWidgets.QFrame.Panel)
        self.lux.setFrameShadow(QtWidgets.QFrame.Raised)
        self.lux.setLineWidth(5)
        self.lux.setMidLineWidth(0)
        self.lux.setSmallDecimalPoint(True)
        self.lux.setDigitCount(3)
        self.lux.setMode(QtWidgets.QLCDNumber.Dec)
        self.lux.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lux.setProperty("value", 0.0)
        self.lux.setProperty("intValue", 0)
        self.lux.setObjectName("lux")
        self.fan = QtWidgets.QCheckBox(self.centralWidget)
        self.fan.setGeometry(QtCore.QRect(610, 190, 100, 51))
        self.fan.setSizeIncrement(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        font.setStrikeOut(False)
        self.fan.setFont(font)
        self.fan.setIconSize(QtCore.QSize(50, 50))
        self.fan.setCheckable(True)
        self.fan.setChecked(False)
        self.fan.setAutoRepeat(False)
        self.fan.setAutoExclusive(False)
        self.fan.setTristate(False)
        self.fan.setObjectName("fan")
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtWidgets.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 800, 27))
        self.menuBar.setObjectName("menuBar")
        MainWindow.setMenuBar(self.menuBar)
        self.mainToolBar = QtWidgets.QToolBar(MainWindow)
        self.mainToolBar.setObjectName("mainToolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.light.setText(_translate("MainWindow", "Lights"))
        self.label_3.setText(_translate("MainWindow", "Water Temp"))
        self.label_2.setText(_translate("MainWindow", "Air Temp"))
        self.label.setText(_translate("MainWindow", "Weight"))
        self.label_4.setText(_translate("MainWindow", "pH"))
        self.label_5.setText(_translate("MainWindow", "RPM"))
        self.label_6.setText(_translate("MainWindow", "Flow"))
        self.label_7.setText(_translate("MainWindow", "Pressure"))
        self.label_8.setText(_translate("MainWindow", "Difference"))
        self.pump.setText(_translate("MainWindow", "Pump"))
        self.label_9.setText(_translate("MainWindow", "Lux Levels"))
        self.fan.setText(_translate("MainWindow", "Fan"))

