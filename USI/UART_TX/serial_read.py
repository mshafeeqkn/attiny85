# File:   serial_read.py
# Author: Mohammed Shafeeque
# 
# Description:
# This file contains the code to read the data from the serial port
# 

# Copyright (c) 2024 Mohammed Shafeeque
# 
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
# 
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
# 
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.

import serial
import time
import sys

# Define the serial port name and baudrate
serial_port = '/dev/tty.usbserial-1410'  # TODO: Change this to your serial port name
baud_rate = 2400                         # Change this to match your device's baud rate

# Initialize the serial port
ser = serial.Serial(serial_port, baud_rate)

# Process command line arguments
print_hex = False
if len(sys.argv) > 1 and sys.argv[1] == '-h':
    print_hex = True

# Read serial port data and print
try:
    while True:
        if print_hex:
            char = ser.read(1).hex()
            print('0x' + char, end=" ")
        else:
            char = ser.read(1)
            try:
                print(char.decode('utf-8'), end="")
            except UnicodeDecodeError:
                pass
        sys.stdout.flush()

except KeyboardInterrupt:
    ser.close()
    print("\n\nSerial port closed\n\n")
except OSError:
    print("\n\nDevice unplugged\n\n")
finally:
    print("   GOOD BYE\n\n")
