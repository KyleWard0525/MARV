"""
    Listen on MARV's serial port and write output to file
"""
import serial

# Set up serial port and baud rate
serial_port = 'COM8'
baud_rate = 9600

# Open output file and serial port
out_file = open('output.txt', 'w+')

try:
    serial_conn = serial.Serial(serial_port, baud_rate)

    print(f"\nListening on {serial_port} at {baud_rate} baud...\n\n")

    while True:
        line = serial_conn.readline().decode('windows-1252')
        print(f"{line}")
        out_file.write(line)

except serial.serialutil.SerialException:
    print(f"\nError: Could not open {serial_port} at {baud_rate} baud.\n")
    out_file.close()
except KeyboardInterrupt:
    print(f"\nExiting...\n")
    out_file.close()
    
