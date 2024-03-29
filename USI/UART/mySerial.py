import serial
import time
import sys

# Define the serial port name and baudrate
serial_port = '/dev/tty.usbserial-1410'  # Change this to your serial port name
baud_rate = 2400  # Change this to match your device's baud rate

# Initialize the serial port
ser = serial.Serial(serial_port, baud_rate)
print_hex = False
try:
    while True:
        # ser.write(b'Hello, Arduino!\n')  # Change the message as needed

        # Wait for a short delay (optional)
        # ser.flush()  # Ensure all data is sent before continuing

        # Print a confirmation message
        # print("Data sent successfully")
        if print_hex:
            char = ser.read(1).hex()
            print('0x' + char, end=" ")
        else:
            char = ser.read(1)
            print(char.decode('utf-8'), end="")
        sys.stdout.flush()
        # print(strdata.ljust(30), " hex: ", data)

except KeyboardInterrupt:
    ser.close()
    print("\n\nSerial port closed\n\n")
except OSError:
    print("\n\nDevice unplugged\n\n")
finally:
    print("   GOOD BYE\n\n")
