# STM32 Communicator Module

This short Python Module was created for the purpose of BME498 Capstone Project.
Its purpose is to streamline the serial communication between any Python program to the STM32 Microcontroller System

## Python Version and Packages

This module was tested on Python 3.13.0

pip freeze
> asyncio==3.4.3
> pyserial==3.5
> setuptools==75.1.0
> wheel==0.44.0

## Quick Start

Have the `serializer.py` copied/moved to the same folder as your Python Program.

> import serializer
> 
> uart_serializer = serializer.STM32_Serializer()							# This creates an instance of STM32_Serializer
>
> uart_serializer.update_signal({1:0, 2:0, 3:0, 4:0, 5:0, 6:0, 7:0, 8:0})	# Use update_signal() to update data that the serializer sends to STM32
>
> uart_serializer.start()													# This starts the STM32 Serializer, it continuously sends data 
> 
> while (program):
> 	updated_data = function()												# During runtime, just call update_signal() to update data to send
> 	uart_serializer.update_signal(updated_data)
>
> uart_serializer.end()														# When finished, call end() to stop uart_serializer from sending data

Please see sample.py for a more detailed example of how this module can be used.

## Data Accepted by Serilaizer

Data you update to the STM32_Serializer() should be a Python Dictionary.

Keys have to be an integer from 1 to 8, representing the channel of stimulation.

Values have to be a floating point value between 0 to 1.
Ideally, this 0 to 1 value has to represent the **duty cycle** of the PWM. 0 would represent just the lowest voltage value to be outputted, 1 would represent the highest voltage value to be outputted.

## STM32_Serializer Parameters

> def __init__(self, 
>		       COM_Port=None, 
>		       start_sig={1:0, 2:0, 3:0, 4:0, 5:0, 6:0, 7:0, 8:0}, 
>		       send_freq=1, 
>		       baudrate=19200, 
>		       write_timeout=0):

COM_Port - STM32 Device will be recognized as a COM Device on Windows. Setting this to None when instantiating will automatically find the STM32 Device. Going through Device Driver to manually find the COM_Port and setting this will also work. 

start_sig - Just initializes the data to send to some value.

send_freq - This is the \[Number of times data is sent\]/sec. 1 would be 1Hz (data is sent every 1 second). 30 would be 30Hz (data is sent every 1/30 seconds).

baudrate - Used in PySerial. Unimportant parameter for USB Communication.

write_timeout - Set this to 0 for non-blocking serial send. Setting this to 1 will do a blocking serial send. 