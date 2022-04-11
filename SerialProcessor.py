"""
    A script to process the output of from the serial port.
    
    kward
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

txt_path = "data/imu/outputs/"
plot_path = "data/imu/plots/"
filename = txt_path + str(sys.argv[1]) + ".txt"

# Tags for the different types of data
data_tags = ["<IMU_DATA>"]

# Parse data in
def read_data(filename): 
    data = {
        'sample_rate': 0,
        'raw': {},
        'lowpass': {},
        'highpass': {},
    }
    
    # Open the file and read lines
    file = open(filename, 'r')
    lines = file.read()
    file.close()
    
    # Extract lines of imu data
    imu_lines = ' '.join(lines.split(data_tags[0])[1].split("\n")).split()
    data['sample_rate'] = int(imu_lines[0].split('=')[1])
    
    # Iterate through lines and extract data
    for line in imu_lines[1:]:
        # Split into seperate measurements
        metrics = line.split(',')
        
        # Loop through metrics
        for metric in metrics:
            # Split into name and value
            header, value = metric.split('=')
            
            # Check if header is in file_data
            if not header in data['raw']:
                data['raw'][header] = []
                data['raw'][header].append(float(value))
            else:
                data['raw'][header].append(float(value))


    # Copy raw data to lowpass and highpass arrays
    data['lowpass'] = dict.copy(data['raw'])
    data['highpass'] = dict.copy(data['raw'])

    # Loop through file data and apply a high-pass filter
    for header in data['raw']:
        if not header in ['Time', 'Velocity', 'Distance']:    
            data['lowpass'][header] = butter_filter(data['lowpass'][header], sampling_rate=data['sample_rate'], cutoff=0.5, type='low')            
            data['highpass'][header] = butter_filter(data['highpass'][header], sampling_rate=data['sample_rate'], type='high')
            
    return data

# Apply a butterworth filter to the data
def butter_filter(data, sampling_rate, type, cutoff=0.5, ord=5):
    if type == 'low':
        b,a = signal.butter(ord, cutoff, fs=sampling_rate, btype=type, analog=False)
    elif type == 'high':
        b,a = signal.butter(ord, cutoff, fs=sampling_rate, btype=type, analog=False)
    
    # Apply the filter to the data
    return signal.filtfilt(b, a, data)
        

# Plot acceleration and gyro data 
def plot_data(data, save=True, filename='accelerationPlots.png'):
    x_axis = data['Time']
    
    fig, axs = plt.subplots(3, 2, sharex=True, figsize=(15, 10))
    
    # Plot lateral acceleration
    axs[0,0].plot(x_axis, data['Ax'])
    axs[0,0].set_title('Lateral Acceleration')
    axs[0,0].set_ylabel('Ax (g)')
    
    # Plot longitudinal acceleration
    axs[1,0].plot(x_axis, data['Ay'])
    axs[1,0].set_title('Longitudinal Acceleration')
    axs[1,0].set_ylabel('Ay (g)')
    
    # Plot vertical acceleration
    axs[2,0].plot(x_axis, data['Az'])
    axs[2,0].set_title('Vertical Acceleration')
    axs[2,0].set_ylabel('Az (g)')
    axs[2,0].set_xlabel('Time (s)')
    
    # Plot vertical angular rate (acceleration about the z-axis)
    axs[0,1].plot(x_axis, data['Gz'], 'r')
    axs[0,1].set_title('Vertical Angular Rate')
    axs[0,1].set_ylabel('Gz (deg/s)')
    
    # Plot pitch
    axs[1,1].plot(x_axis, data['Pitch'], 'g')
    axs[1,1].set_title('Pitch')
    axs[1,1].set_ylabel('Angle (deg)')
    
    # Plot roll
    axs[2,1].plot(x_axis, data['Roll'], 'g')
    axs[2,1].set_title('Roll')
    axs[2,1].set_ylabel('Angle (deg)')

    fig.suptitle('MARV IMU Plots: ' + str(filename.split('.')[0]))
    plt.show()
    
    if save:
        fig.savefig(plot_path + filename)
    

data = read_data(filename)

for header in data:
    if not header == 'sample_rate':
        plot_data(data[header], True, f"straightLine12cm{data['sample_rate']}HzTest_{header}.png")
#plot_data(data, True, str(sys.argv[1]) + '.png')
