"""
    Listen on MARV's serial port and write output to file
    
    kward
"""
import sys
import serial

# Set up serial port and baud rate
serial_port = 'COM8'
baud_rate = 115200

output_path = 'data/imu/outputs/' + str(sys.argv[1])

# Open output file
if not output_path:
    out_file = open('output.txt', 'w')
else:
    out_file = open(output_path, 'w')

# Try to open serial port connection
try:
    serial_conn = serial.Serial(serial_port, baud_rate)
    print(f"\nListening on {serial_port} at {baud_rate} baud...\n\n")
    
    
    bytes_received = 0
    
    while True:
            # Read data from serial port
            line = serial_conn.readline().decode('windows-1252')
            bytes_received += sys.getsizeof(line)
            print(f"{bytes_received} bytes received", end='\r')
            
            # Check for end of transmission flag
            if "<END>" in str(line):
                print("\nEnd of transmission. Closing connection...")
                serial_conn.close()
                out_file.close()
                sys.exit(0)
            
            out_file.write(line)
    
except serial.serialutil.SerialException:
    print(f"\nError: Could not open {serial_port} at {baud_rate} baud.\n")
    out_file.close()

    
out_file.close()