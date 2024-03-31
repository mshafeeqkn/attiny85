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
import struct

# Define the serial port name and baudrate
serial_port = '/dev/tty.usbserial-1410'  # TODO: Change this to your serial port name
baud_rate = 2400                         # Change this to match your device's baud rate

if (len(sys.argv)) > 1:
    data = int(sys.argv[1])
else:
    data = 0

# Initialize the serial port
ser = serial.Serial(serial_port, baud_rate)

# Read serial port data and print
try:
    data_to_send = struct.pack('B', data)
    # char_to_send = 'b';
    # data_to_send = char_to_send.encode()
    ser.write(data_to_send);

    # char = ser.read(1).hex()
    # print('0x' + char, end=" ")

except KeyboardInterrupt:
    print("\n\nSerial port closed\n\n")
except OSError:
    print("\n\nDevice unplugged\n\n")
finally:
    ser.close()
    print("   GOOD BYE\n\n")
