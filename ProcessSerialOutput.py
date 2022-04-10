"""
    A script to process the output of from the serial port.
    
    kward
"""
import numpy as np
import matplotlib.pyplot as plt

filename = 'output.txt'

# Tags for the different types of data
data_tags = ["<IMU_DATA>"]

imu_data = {}

# Parse data in
def read_data():
    global filename
    
    # Open the file and read lines
    file = open(filename, 'r')
    lines = file.read()
    file.close()
    
    # Extract lines of imu data
    imu_lines = ' '.join(lines.split(data_tags[0])[1].split("\n")).split()
    
    # Iterate through lines and extract data
    for line in imu_lines:
        # Split into seperate measurements
        metrics = line.split(',')
        
        for metric in metrics:
            header, value = metric.split('=')
            
            if not header in imu_data:
                imu_data[header] = []
                imu_data[header].append(float(value))
            else:
                imu_data[header].append(float(value))
    
# Normalize data in a given range
def normalize(data, min_val=0, max_val=1):
    return (data - np.min(data)) / (np.max(data) - np.min(data)) * (max_val - min_val) + min_val

# Plot acceleration data
def plot_acceleration(filename='accelerationPlots.png'):
    global imu_data
    x_axis = imu_data['Time']
    
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 10))
    
    # Plot lateral acceleration
    axs[0].plot(x_axis, imu_data['Ax'])
    axs[0].set_title('Lateral Acceleration')
    axs[0].set_ylabel('Ax (g)')
    
    # Plot longitudinal acceleration
    axs[1].plot(x_axis, imu_data['Ay'])
    axs[1].set_title('Longitudinal Acceleration')
    axs[1].set_ylabel('Ay (g)')
    
    # Plot vertical acceleration
    axs[2].plot(x_axis, imu_data['Az'])
    axs[2].set_title('Vertical Acceleration')
    axs[2].set_ylabel('Az (g)')
    axs[2].set_xlabel('Time (s)')
    
    fig.suptitle('MARV Acceleration Plots')
    plt.show()
    fig.savefig(filename)

read_data()
plot_acceleration('data/imu/plots/30cmSqr_200Hz_Acceleration.png')
