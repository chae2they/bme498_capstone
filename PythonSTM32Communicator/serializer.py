import serial
import serial.tools.list_ports
import threading
import time


# Serializer Module
# Primary Goal: Send data to STM32

class STM32_Serializer:

	def __init__(self, 
		        COM_Port=None, 
		        start_sig={1:0, 2:0, 3:0, 4:0, 5:0, 6:0, 7:0, 8:0}, 
		        send_freq=1, 
		        baudrate=19200, 
		        write_timeout=0):

		# STM32 Port Finding + Automatic Setup
		self.COM_Port = COM_Port
		if(COM_Port == None):
			ports = serial.tools.list_ports.comports()

			for port in ports:
				# VID and PID comes from STM32 Firmware Programming
				if((port.vid == 1155) and (port.pid == 22336)):
					self.COM_Port = port.device

		# Data Transmit Settings
		self.data = start_sig									# self.data is a dictionary
		self.data_frequency = min(100, max(1,send_freq)) 		# Force max frequency to be 100Hz, min frequency to be 1Hz (arbitrarily picked for now)

		# Set up PySerial
		self.ser = serial.Serial()
		self.ser.port = self.COM_Port
		self.ser.baudrate = baudrate			# Note that Baudrate really does not matter because OS recognizes STM32 as USB CDC (USB Protocol is used)
		self.ser.write_timeout = write_timeout	# Default value is 0, which makes it non-blocking

		# Set up Threading
		self.running = False
		self.thread = None

	def update_signal(self, new_data):
		self.data = new_data

	def binarize_data(self):
		# self.data is expected to be a LIBRARY of Keys 1 to 8 and Values between 0 and 1 (both inclusive)

		bin_data = []
		for i in range(8):
			bin_data.append(int(self.data[i+1] * 255))

		return bytearray(bin_data)

	def send_data(self):
		binary_data = self.binarize_data()
		self.ser.write(binary_data)
		print("Sent data of: [{} {} {} {} {} {} {} {}]\n".format(binary_data[0], binary_data[1], binary_data[2], binary_data[3], binary_data[4], binary_data[5], binary_data[6], binary_data[7]))

	def start(self):
		if not self.running:
			self.running = True
			self.ser.open()
			print("Creating Task in start()")
			self.thread = threading.Thread(target=self.send_loop)
			self.thread.start()


	def send_loop(self):
		while self.running:
			self.send_data()
			time.sleep(1/self.data_frequency)

	def end(self):
		self.running = False
		if self.thread:
			self.thread.join()


	def test_function(self):
		print("Assumed to send {} on Port: {}".format(self.data, self.COM_Port))
		self.ser.write(self.data)