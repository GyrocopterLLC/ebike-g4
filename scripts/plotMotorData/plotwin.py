# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\plotMotorData\plotwin.ui',
# licensing of '.\plotMotorData\plotwin.ui' applies.
#
# Created: Sat Aug 17 14:34:33 2019
#      by: pyside2-uic  running on PySide2 5.12.2
#
# WARNING! All changes made in this file will be lost!

from PySide2 import QtCore, QtGui, QtWidgets
from PySide2 import QtCharts

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(749, 455)
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setObjectName("gridLayout")
        self.btnStop = QtWidgets.QPushButton(Form)
        self.btnStop.setObjectName("btnStop")
        self.gridLayout.addWidget(self.btnStop, 1, 1, 1, 1)
        self.btnStart = QtWidgets.QPushButton(Form)
        self.btnStart.setObjectName("btnStart")
        self.gridLayout.addWidget(self.btnStart, 1, 0, 1, 1)
        self.listwPorts = QtWidgets.QListWidget(Form)
        self.listwPorts.setObjectName("listwPorts")
        self.gridLayout.addWidget(self.listwPorts, 3, 0, 1, 2)
        self.btnListPorts = QtWidgets.QPushButton(Form)
        self.btnListPorts.setObjectName("btnListPorts")
        self.gridLayout.addWidget(self.btnListPorts, 4, 0, 1, 2)
        self.btnSetParams = QtWidgets.QPushButton(Form)
        self.btnSetParams.setObjectName("btnSetParams")
        self.gridLayout.addWidget(self.btnSetParams, 5, 0, 1, 2)
        self.btnOpen = QtWidgets.QPushButton(Form)
        self.btnOpen.setObjectName("btnOpen")
        self.gridLayout.addWidget(self.btnOpen, 0, 0, 1, 2)
        self.chartview = QtCharts.QtCharts.QChartView(Form)
        self.chartview.setObjectName("chartview")
        self.gridLayout.addWidget(self.chartview, 0, 2, 4, 1)
        self.btnAutoScale = QtWidgets.QPushButton(Form)
        self.btnAutoScale.setObjectName("btnAutoScale")
        self.gridLayout.addWidget(self.btnAutoScale, 4, 2, 1, 1)
        self.btnSaveData = QtWidgets.QPushButton(Form)
        self.btnSaveData.setObjectName("btnSaveData")
        self.gridLayout.addWidget(self.btnSaveData, 5, 2, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)
        Form.setTabOrder(self.btnOpen, self.btnStart)
        Form.setTabOrder(self.btnStart, self.btnStop)
        Form.setTabOrder(self.btnStop, self.listwPorts)
        Form.setTabOrder(self.listwPorts, self.btnListPorts)
        Form.setTabOrder(self.btnListPorts, self.btnSetParams)

    def retranslateUi(self, Form):
        Form.setWindowTitle(QtWidgets.QApplication.translate("Form", "Live Plotting", None, -1))
        self.btnStop.setText(QtWidgets.QApplication.translate("Form", "Stop Comms", None, -1))
        self.btnStart.setText(QtWidgets.QApplication.translate("Form", "Start Comms", None, -1))
        self.btnListPorts.setText(QtWidgets.QApplication.translate("Form", "List Comm Ports", None, -1))
        self.btnSetParams.setText(QtWidgets.QApplication.translate("Form", "Set Params", None, -1))
        self.btnOpen.setText(QtWidgets.QApplication.translate("Form", "Open Serial Port", None, -1))
        self.btnAutoScale.setText(QtWidgets.QApplication.translate("Form", "Auto Scale", None, -1))
        self.btnSaveData.setText(QtWidgets.QApplication.translate("Form", "Save Data", None, -1))

