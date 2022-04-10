"""
    Listen on MARV's serial port and write output to file
"""
import sys
import serial

# Set up serial port and baud rate
serial_port = 'COM8'
baud_rate = 115200


# Try to open serial port connection
try:
    serial_conn = serial.Serial(serial_port, baud_rate)
    print(f"\nListening on {serial_port} at {baud_rate} baud...\n\n")
    
    # Open output file
    out_file = open('output.txt', 'w')
    
    while True:
        try:
            # Read data from serial port
            line = serial_conn.readline().decode('windows-1252')
            print(f"{sys.getsizeof(line)} bytes received")
            
            # Check for end of transmission flag
            if "<END>" in str(line):
                print("\nEnd of transmission. Closing connection...")
                serial_conn.close()
                out_file.close()
                sys.exit(0)
            
            out_file.write(line)
        except Exception:
            print(f"\nExiting...\n")
            serial_conn.close()
            out_file.close()
            sys.exit(0)
    
except serial.serialutil.SerialException:
    print(f"\nError: Could not open {serial_port} at {baud_rate} baud.\n")
    out_file.close()

    
