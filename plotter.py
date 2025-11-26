#!/usr/bin/env python3

import serial
import time

arduino = serial.Serial(port='/dev/cu.usbmodem1101', baudrate=9600, timeout=.1)
def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data


write_read("L20\nR-300\nL-20\nR300")
# while True:
#     num = input("Enter a command: ")
#     value = write_read(num)
#     print(value)
