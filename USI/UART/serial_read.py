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
