# Standard Python Libraries
import time
import math

# Custom Python Libraries
import serializer


# Simple goal: Just send a Sine wave to Microcontroller for now
if __name__ == "__main__":

	sample_data = {1:0, 2:0, 3:0, 4:0, 5:0, 6:0, 7:0, 8:0}
	uart_serializer = serializer.STM32_Serializer(start_sig=sample_data, send_freq=30)		# Initialize to send data at 30Hz
	uart_serializer.start()
	i = 0
	while True:

		for k in sample_data:
			# Expect values to go up by the multiples of the key
			# (All value divided by 255)
			# 1: 0, 1,  2,  3,  4,  5,  6,  7,  8,  9, 10
			# 2: 0, 2,  4,  6,  8, 10, 12, 14, 16, 18, 20
			# ...
			# 8: 0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80
			sample_data[k] = (sample_data[k] + k)
			if(sample_data[k] == 9*k):
				sample_data[k] = k

		# Just dividing values by 255 so that library can use a value between (0,1) as expected
		temp_buffer = {}
		for k in sample_data:
			temp_buffer[k] = sample_data[k]/255
		uart_serializer.update_signal(temp_buffer)
		print("Updated Data to Send: ")
		print(sample_data)

		# Value updates every 1 second, so we expect to see the "sent data" message ~30 times per data point
		time.sleep(1)
		i = i + 1
		if(i>=20):
			break

	uart_serializer.end()
