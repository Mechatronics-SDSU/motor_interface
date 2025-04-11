import serial
import struct
import numpy

baud_rate = 115200
usb_port = None
srl = None

# Try multiple ports
for port in ["COM3", "/dev/tty.usbmodem205939804E301"]:
    try:
        srl = serial.Serial(port, baud_rate)
        usb_port = port
        print(f"Connected on {usb_port}")
        break
    except serial.SerialException as e:
        print(f"Failed to connect on {port}: {e}")

# Check if connection was successful
if srl is None:
    print("❌ Unable to connect to any serial port.")
else:
    # Proceed with transmitting if serial port is valid
    def usb_transmit(num_array):
        packed_data = b''
        for num in num_array:
            packed_data += struct.pack('<i', num)
        srl.write(packed_data)
        print(f"Transmitted: {num_array}")

    def print_tx(num_array):  # DEBUG FUNCTION
        print(num_array)

    # Optional test call
    # usb_transmit([100, 0, 0, 0, 0, 0, 0, 0])
