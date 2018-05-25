# GUI and Modbus integration Program
# Written by Eoin Fitzgerald C13319946 - Last Update 21/06/2018
# Course/Year: DT021A/4
# Supervisor: Frank Duignan

# import neccessariy libraries
from PyQt5.QtWidgets import QApplication, QDialog, QMessageBox
from PyQt5 import Qt
import sys 
from mainwindow import Ui_MainWindow
from threading import Timer
import pymodbus
import serial
import time
from pymodbus.pdu import ModbusRequest
from pymodbus.client.sync import ModbusSerialClient as ModbusClient #initialize a serial RTU client instance
from pymodbus.transaction import ModbusRtuFramer


# class developed for GUI
class MyGUI(Qt.QMainWindow):
	ui=None
	# Callback function for updating GUI
	def TimerCallback(self):
		lightState=self.ui.light.checkState()
		fanState=self.ui.fan.checkState()
		self.client.write_registers(14,[lightState, fanState],unit=1)
		time.sleep(0.1)
		pumpState=self.ui.pump.checkState()
		self.client.write_register(10,pumpState,unit=2)
		time.sleep(0.1)
		result=self.client.read_holding_registers(9,10,unit=1)
		self.ui.airTemp.setProperty("value",result.registers[0])
		self.ui.lux.setProperty("value",result.registers[1])
		self.ui.RPM.setProperty("value",result.registers[2])
		self.ui.weight.setProperty("value",result.registers[3])
		self.ui.pDiff.setProperty("value",result.registers[7])
		time.sleep(0.1)
		result2=self.client.read_holding_registers(9,4,unit=2)
		self.ui.waterTemp.setProperty("value",result2.registers[0])
		self.ui.flow.setProperty("value",result2.registers[2])
		self.ui.pH.setProperty("value",((result2.registers[3])/100))
		t=Timer(.1,self.TimerCallback)  # Callback repeatedly calls itself
		t.start()

	# class constructor 			
	def __init__(self):
		super(MyGUI,self).__init__()
		self.ui=Ui_MainWindow()
		self.ui.setupUi(self)
		t=Timer(1,self.TimerCallback) # Callback is called initially
		t.start()
		pymodbus.constants.Defaults.Timeout=0.3 # Modbus Timeout is 0.3 seconds
		self.client= ModbusClient(method = "rtu", port="/dev/ttyUSB0",stopbits = 1, bytesize = 8, parity = 'N', baudrate= 9600)

		#Connect to the serial modbus server
		self.connection = self.client.connect()

def main():
    # We instantiate a QApplication passing the arguments of the script to it:
    a = Qt.QApplication(sys.argv)
    w = MyGUI() # object created for updating Modbus and GUI
    w.show()    # GUI is displayed
    # Now we can start it.
    a.exec_()

# This section starts the main function on load
if __name__ == "__main__":
    main()
