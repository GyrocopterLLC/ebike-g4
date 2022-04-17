#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Measure motor parameters live.
Works with packet data version of the controller.

"""

from PySide2 import QtGui, QtCore, QtWidgets, QtGui
from PySide2 import QtCharts
from PySide2.QtCore import QPointF, QRectF

import random
from collections import deque

import numpy as np
# import pyqtgraph as pg
import serial
from serial.tools import list_ports
import sys, os
import struct
from ebike_controller_packet import Packet, ebike_pack, ebike_unpack

from plotwin import Ui_Form
from params import Ui_ParamsForm

PACKETID_GETRAM = 0x01
PACKETID_SETRAM = 0x02
PACKETID_RAMRESULT = 0x81
PACKETID_ACK = 0x91
PACKETID_NACK = 0x92
CONFIGID_NUM_USB_OUTPUTS = 0x0204
CONFIGID_USB_SPEED = 0x0205
CONFIGID_USB_CHOICE_1 = 0x0206
CONFIGID_USB_CHOICE_2 = 0x0207
CONFIGID_USB_CHOICE_3 = 0x0208
CONFIGID_USB_CHOICE_4 = 0x0209
CONFIGID_USB_CHOICE_5 = 0x020A
CONFIGID_USB_CHOICE_6 = 0x020B
CONFIGID_USB_CHOICE_7 = 0x020C
CONFIGID_USB_CHOICE_8 = 0x020D
CONFIGID_USB_CHOICE_9 = 0x020E
CONFIGID_USB_CHOICE_10 = 0x020F
PACKETID_STREAMDATA = 0x88
PACKETID_ENABLE_FEATURE = 0x05
PACKETID_DISABLE_FEATURE = 0x06
FEATUREID_STREAMDATA = 0x0001

PLOT_COLORS = [(0,0,255),(255,0,0),(0,255,0),(0,0,0),(255,26,184),(255,211,0),(0,87,0),(131,131,255),(158,79,70),(0,255,193)]

# Returns the packet from the serial port plus any additional bytes
def ReadAPacket(all_bytes):
	pkt, sop_loc, pkt_length = ebike_unpack(all_bytes)
	if(sop_loc > -1):
		# valid SOP found
		if(pkt_length > 0):
			# valid full packet
			# return the packet and any remaining data in the byte string
			return(pkt, all_bytes[sop_loc+pkt_length:])
		else:
			# SOP was valid, but maybe the packet wasn't fully received yet
			# at least trim off the bytes before SOP
			# pkt will be zero, so the calling function can know it failed
			return(pkt, all_bytes[sop_loc:])
	else:
		# No sop was found
		# Return an empty string
		return(pkt, b'')

def WaitForAck(serial_port, msg, timeout_ms):
	ack_happened = False
	the_time = QtCore.QTime()
	the_time.start()
	tempstring = b''
	serial_port.write(msg)
	while(the_time.elapsed() < timeout_ms and not ack_happened):
		tempstring += serial_port.read_all()
		pkt, tempstring = ReadAPacket(tempstring)
		if(pkt.PacketID == PACKETID_ACK):
			ack_happened = True
			break
		if(pkt.PacketID == PACKETID_NACK):
			print('nacked')
			print('data was '+str(msg))
			print('reply was (ID-'+str(pkt.PacketID)+', Data-'+str(pkt.Data)+')')
			ack_happened = False
			break
	return ack_happened


class ListenToComThread(QtCore.QThread):
	size_arrays = 2000
	num_plots = 5
	new_data = False
	def __init__(self, numPlots, sizeArrays):
		super().__init__()
		self.size_arrays = sizeArrays
		self.num_plots = numPlots
		self.tempstring = b''
		self.data = np.zeros(shape=(self.num_plots, self.size_arrays))
		self.timestamp = np.zeros(self.size_arrays)
	def setNumPlots(self, numPlots = 10):
		self.num_plots = numPlots
		self.data = np.zeros(shape=(self.num_plots, self.size_arrays))
	def setSizeArrays(self, sizeArrays = 2000):
		self.size_arrays = sizeArrays
		self.data = np.zeros(shape=(self.num_plots, self.size_arrays))
	def setSerialPort(self,ser):
		self.ser = ser
	def exit_now(self):
		self.exiting = True
	def run(self):
		self.exiting = False
		print("Listening for data...")
		while self.ser.is_open and not self.exiting:
			try:
				self.tempstring += self.ser.read_all()
				if(len(self.tempstring) > 0):
					(pkt, self.tempstring) = ReadAPacket(self.tempstring)
					if(pkt.PacketID == PACKETID_STREAMDATA):
						try:
							num_data = int(len(pkt.Data)/4)-1
							if(num_data != self.num_plots):
								raise ValueError('Mismatch in data length.\n'+
									'Packet data length is: '+str(len(pkt.Data)) +'\n'+
									'Expected: '+str((self.num_plots)*4+4))
							converted_data = struct.unpack('>i' + str(num_data)+'f', pkt.Data)
							dataList = np.array(converted_data[1:]).reshape(num_data,1)
							self.timestamp = np.hstack((self.timestamp[1:],converted_data[0]))
							self.data[:num_data,:] = np.hstack((self.data[:num_data,1:self.size_arrays],dataList))
							self.new_data = True
						except ValueError as err:
							print('In ListenToComThread: {}'.format(err))
						except struct.error as err:
							print('Struct error in ListenToComThread: {}'.format(err))
			except serial.SerialException:
				print("Serial port "+self.ser.port+" was interrupted.")
				self.ser.close()
				self.exiting=True
			self.yieldCurrentThread()
		print('stopped listening.')

class MainWindow(QtWidgets.QWidget):
	chart_size = 2000
	num_plots = 5
	collecting_data = False
	def __init__(self):
		super().__init__()
		self.ser = serial.Serial()
		self.win = Ui_Form()
		self.win.setupUi(self)
		
		self.chart = QtCharts.QtCharts.QChart()
		# self.chart.legend().hide()
		self.chart.legend().setAlignment(QtCore.Qt.AlignBottom)
		self.chart.setTitle("Live Updating Data")
		self.win.chartview.setChart(self.chart)
		self.dataGrabber = ListenToComThread(self.num_plots, self.chart_size)
		self.params = PopupParamSetter()
		self.resetPlots()

		self.timer=QtCore.QTimer()
		self.timer.timeout.connect(self.updateChart)
		self.timer.start(50)
		
		self.win.btnListPorts.clicked.connect(self.listPorts)
		self.win.btnSetParams.clicked.connect(self.ParamsClicked)
		self.win.btnOpen.clicked.connect(self.OpenClicked)
		self.win.btnStart.clicked.connect(self.StartClicked)
		self.win.btnStop.clicked.connect(self.StopClicked)
		self.win.btnAutoScale.clicked.connect(self.AutoScale)
		self.win.btnSaveData.clicked.connect(self.SaveData)

		self.listPorts()
		
		self.show()
	def resetPlots(self):
		self.chart.removeAllSeries()
		self.lines = [QtCharts.QtCharts.QLineSeries() for i in range(self.num_plots)]
		self.linedata = [deque([QPointF(i,0) for i in range(self.chart_size)]) for j in range(self.num_plots)]
		# self.ydata = [deque([0]*self.ChartSize) for i in range(self.numPlots)]
		for i in range(self.num_plots):
			linename = self.params.PARAM_NAMES[self.params.selected_params[i]]
			self.lines[i].setName(linename)
			self.lines[i].setColor(QtGui.QColor.fromRgb(
				PLOT_COLORS[i][0],PLOT_COLORS[i][1],PLOT_COLORS[i][2]))
			self.lines[i].replace(self.linedata[i])

		for i in range(self.num_plots):
			self.chart.addSeries(self.lines[i])
		self.chart.createDefaultAxes()
	def updatePlotNumberAndLegend(self):
		self.num_plots = self.params.num_vars	
		self.dataGrabber.setNumPlots(self.num_plots)
		self.resetPlots()
	def updateChart(self):
		if(self.dataGrabber.new_data):
			self.dataGrabber.new_data = False
			for i in range(self.num_plots):
				for j in range(self.chart_size):
					self.linedata[i][j].setY(self.dataGrabber.data[i][j])
				self.lines[i].replace(self.linedata[i])
	def StartClicked(self):
		if(self.ser.is_open):
			self.collecting_data = True
			self.dataGrabber.start()
			self.ser.write(ebike_pack(PACKETID_ENABLE_FEATURE, struct.pack('>h',FEATUREID_STREAMDATA)))
	def StopClicked(self):
		if(self.ser.is_open):
			self.collecting_data = False
			self.ser.write(ebike_pack(PACKETID_DISABLE_FEATURE, struct.pack('>h',FEATUREID_STREAMDATA)))
			self.dataGrabber.exit_now()
	# def zoomToEnd(self):
	# 	self.axes = self.chart.axes(QtCore.Qt.Horizontal)
	# 	for i in range(self.numPlots):
	# 		self.lines[i].detachAxis(self.axes[0])
	# 	self.chart.removeAxis(self.axes[0])
	# 	self.axes[0].setMin(self.ChartSize - 50)
	# 	self.chart.addAxis(self.axes[0],QtCore.Qt.AlignBottom)
	# 	for i in range(self.numPlots):
	# 		self.lines[i].attachAxis(self.axes[0])
	# def unZoom(self):
	# 	self.axes = self.chart.axes(QtCore.Qt.Horizontal)
	# 	for i in range(self.numPlots):
	# 		self.lines[i].detachAxis(self.axes[0])
	# 	self.chart.removeAxis(self.axes[0])
	# 	self.axes[0].setMin(0)
	# 	self.chart.addAxis(self.axes[0],QtCore.Qt.AlignBottom)
	# 	for i in range(self.numPlots):
	# 		self.lines[i].attachAxis(self.axes[0])

	def listPorts(self):
		## Fill the list widget with available serial ports
		self.win.listwPorts.clear()
		genny = sorted(serial.tools.list_ports.comports())
		for id, desc, hwid in genny:
			self.win.listwPorts.addItem(id+": "+desc)
	def openPort(self, portName):
		if(self.collecting_data):
			self.StopClicked()
		self.ser.port = portName
		# print("Trying to open "+self.ser.port)
		self.ser.open()
		# if(self.ser.is_open):
		# 	print("Success!")
		self.ser.reset_input_buffer()
		self.ser.reset_output_buffer()
		self.win.btnOpen.setText(QtWidgets.QApplication.translate("Form", "Close Serial Port", None, -1))
		self.dataGrabber.setSerialPort(self.ser)
	def closePort(self):
		if(self.collecting_data):
			self.StopClicked()
		self.ser.close()
		self.win.btnOpen.setText(QtWidgets.QApplication.translate("Form", "Open Serial Port", None, -1))
	def OpenClicked(self):
		try:
			if(self.ser.is_open):
				self.closePort()
			else:
				self.openPort(self.win.listwPorts.currentItem().text().split(':')[0])
		except AttributeError as e:
			print("Unable to open port")

	def ParamsClicked(self):
		if(self.collecting_data):
			self.StopClicked()
		self.params.setSerialPort(self.ser)
		self.params.showForm()
		self.params.window_closed.connect(self.updatePlotNumberAndLegend)
	def AutoScale(self):
		axes = self.chart.axes(QtCore.Qt.Vertical)[0]
		for i in range(self.num_plots):
			self.lines[i].detachAxis(axes)
		self.chart.removeAxis(axes)
		axes.setMin(np.min(self.dataGrabber.data))
		axes.setMax(np.max(self.dataGrabber.data))
		self.chart.addAxis(axes,QtCore.Qt.AlignLeft)
		for i in range(self.num_plots):
			self.lines[i].attachAxis(axes)
	def SaveData(self):
		if(self.collecting_data):
			self.StopClicked()
			restart_after = True
		else:
			restart_after = False
		save_file_name = QtWidgets.QFileDialog.getSaveFileName(self, "Save File", os.getcwd(), "Comma-separated variable (*.csv);;All files (*.*)")
		if(len(save_file_name[0]) > 0):
			header_line = 'Index,Timestamp,'
			for i in range(self.num_plots):
				header_line = header_line + self.params.PARAM_NAMES[self.params.selected_params[i]] + ','
			np.savetxt(save_file_name[0], np.transpose(np.vstack((np.arange(0, self.chart_size),self.dataGrabber.timestamp,self.dataGrabber.data))),
			delimiter=',',header=header_line,comments='')

class GetParamData(QtCore.QThread):
	got_data = QtCore.Signal(int,int)
	tempstring = b''
	state = 0
	num_vars = 12
	var_ids = [CONFIGID_NUM_USB_OUTPUTS, CONFIGID_USB_SPEED, 
	CONFIGID_USB_CHOICE_1, CONFIGID_USB_CHOICE_2, CONFIGID_USB_CHOICE_3,
	CONFIGID_USB_CHOICE_4, CONFIGID_USB_CHOICE_5, CONFIGID_USB_CHOICE_6,
	CONFIGID_USB_CHOICE_7, CONFIGID_USB_CHOICE_8, CONFIGID_USB_CHOICE_9,
	CONFIGID_USB_CHOICE_10]
	var_types = ['h','h','h','h','h','h','h','h','h','h','h','h']
	def __init__(self, serial_port):
		super().__init__()
		self.ser = serial_port
	def run(self):
		self.exiting = False
		# Send first request
		if(getattr(self.ser,"is_open",False)):
			self.ser.write(ebike_pack(PACKETID_GETRAM,struct.pack(
				'>h',self.var_ids[self.state])))
		# Wait for responses
		while(getattr(self.ser, "is_open", False) and not self.exiting):
			if(self.state >= self.num_vars):
				self.exiting = True
			else:
				try:
					self.tempstring += self.ser.read_all()
					if(len(self.tempstring) > 0):
						pkt=Packet(0,b'')
						(pkt, self.tempstring) = ReadAPacket(self.tempstring)
						if(pkt.PacketID == PACKETID_RAMRESULT):
							self.got_data.emit(self.state, struct.unpack(
								'>'+self.var_types[self.state],pkt.Data)[0])
							self.state += 1
							if(self.state < self.num_vars):
								self.ser.write(ebike_pack(PACKETID_GETRAM,struct.pack(
									'>h',self.var_ids[self.state])))
				except (serial.SerialException, struct.error) as e:
					if(type(e) == serial.SerialException):
						print("Serial port "+self.ser.port+" was interrupted.")
						self.ser.close()
					else:
						print("Incorrect data format")
					self.exiting=True

class SetParamData(QtCore.QThread):
	completed = False
	state = 0
	num_vars = 12
	var_ids = [CONFIGID_NUM_USB_OUTPUTS, CONFIGID_USB_SPEED, 
	CONFIGID_USB_CHOICE_1, CONFIGID_USB_CHOICE_2, CONFIGID_USB_CHOICE_3,
	CONFIGID_USB_CHOICE_4, CONFIGID_USB_CHOICE_5, CONFIGID_USB_CHOICE_6,
	CONFIGID_USB_CHOICE_7, CONFIGID_USB_CHOICE_8, CONFIGID_USB_CHOICE_9,
	CONFIGID_USB_CHOICE_10]
	var_types = ['h','h','h','h','h','h','h','h','h','h','h','h']
	def __init__(self, serial_port, new_variables):
		super().__init__()
		self.ser = serial_port
		self.new_data = new_variables
	def run(self):
		success = False
		if(getattr(self.ser, "is_open", False)):
			for i in range(self.num_vars):
				retry_counter = 0
				while(not success and retry_counter < 3):
					success = WaitForAck(self.ser,ebike_pack(PACKETID_SETRAM,struct.pack(
						'>hh',self.var_ids[i], self.new_data[i])), 50)
					retry_counter += 1
				if(not success):
					break
				self.yieldCurrentThread()
		if(success):
			self.completed = True

class PopupParamSetter(QtWidgets.QWidget):
	window_closed = QtCore.Signal()
	current_data_rate = 0
	num_vars = 5
	DATA_RATES = [50,100,200,500,1000,5000]
	PARAM_NAMES = ['Ia','Ib','Ic','Ta','Tb','Tc','Throttle','HallAngle',\
			'HallSpeed','HallAccel','HallState','Vbus','Id','Iq','Td','Tq',\
				'Va','Vb','Vc','Fet Temp','ErrorCode','RawThrottle']
	selected_params = [0, 1, 2, 6, 7, 3, 4, 5, 8, 10]
	ser = None
	
	def __init__(self):
		super().__init__()

		self.win = Ui_ParamsForm()
		self.win.setupUi(self)
		self.win.spinNumVar.setValue(self.num_vars)
		self.win.dialDataRate.valueChanged.connect(self.dataRateChanged)
		self.win.spinNumVar.valueChanged.connect(self.numVarsChanged)

		self.paramLabels = [getattr(self.win,'lbVar'+str(i+1)) for i in range(10)]
		self.paramCombos = [getattr(self.win,'cbVar'+str(i+1)) for i in range(10)]
		for i in range(10):
			for j in range(len(self.PARAM_NAMES)):
				self.paramCombos[i].insertItem(j, self.PARAM_NAMES[j])
			self.paramCombos[i].setCurrentIndex(self.selected_params[i])
		self.numVarsChanged()
		self.win.btnCancel.clicked.connect(self.close)
		self.win.btnOk.clicked.connect(self.confirm)
		self.win.btnOk.setDisabled(True)
	def showForm(self):
		self.show()

		# Get current settings from the controller
		self.commThread = GetParamData(self.ser)
		self.commThread.got_data.connect(self.getSettings)
		self.commThread.finished.connect(self.commFinished)
		self.commThread.start()
	def setSerialPort(self, serial_port):
		self.ser = serial_port
	def commFinished(self):
		self.win.btnOk.setDisabled(False)
	def dataRateChanged(self):
		self.current_data_rate = self.win.dialDataRate.value()
		self.win.lbDataRate.setText('Data Rate: '+str(self.DATA_RATES[self.current_data_rate])+'Hz')
	def numVarsChanged(self):
		self.num_vars = self.win.spinNumVar.value()
		for i in range(10):
			self.paramLabels[i].show()
			self.paramCombos[i].show()
		for i in range(10-self.num_vars):
			self.paramLabels[i+self.num_vars].hide()
			self.paramCombos[i+self.num_vars].hide()
	def confirm(self):
		self.num_vars = self.win.spinNumVar.value()
		self.current_data_rate = self.win.dialDataRate.value()
		new_data = [self.num_vars, self.current_data_rate]
		for i in range(10):
			self.selected_params[i] = self.paramCombos[i].currentIndex()
			new_data.append(self.selected_params[i]+1)
		self.finishThread = SetParamData(self.ser, new_data)
		self.finishThread.finished.connect(self.confirmFinished)
		self.finishThread.start()
	def confirmFinished(self):
		if(self.finishThread.completed):
			QtWidgets.QMessageBox.information(self,"Completed","New data sent successfully")
			self.close()
		else:
			QtWidgets.QMessageBox.warning(self,"Failed","New data unable to be sent")
	@QtCore.Slot(int, int)
	def getSettings(self, var_id, var_value):
		if(var_id == 0):
			# Num vars
			self.win.spinNumVar.setValue(var_value)
			self.numVarsChanged()
		if(var_id == 1):
			# Data speed
			self.win.dialDataRate.setValue(var_value)
			self.dataRateChanged()
		if(var_id >= 2 and var_id < 12):
			# Variable selections
			self.paramCombos[var_id-2].setCurrentIndex(var_value-1)
			self.selected_params[var_id-2] = var_value-1
	def closeEvent(self, event):
		self.window_closed.emit()
		event.accept()

if __name__ == '__main__':
	app = QtWidgets.QApplication(sys.argv)
	mywin = MainWindow()
	sys.exit(app.exec_())