# ECE 16 Fall 2018 Final Project Code
# Jun Young Jang (A12298630)
# Taylor Hua (A12907514)


# ========================================
# FOR FIREBASE PURPOSES ONLY
# Place this code at the top of your file

import pyrebase
import time

config = {
  "apiKey": "AIzaSyBCuJvm-DPjvS6P9TQ-sXhs01g76e9aWto",
  "authDomain": "ece16-fall18.firebaseapp.com",
  "databaseURL": "https://ece16-fall18.firebaseio.com",
  "storageBucket": "ece16-fall18.appspot.com",
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()

last_time = time.time()

# =========================================


#                               Libraries
import scipy.signal as sig
import serial
import sys
import requests
import math
from time import sleep
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np

#                               Globals
# These variables are just defined commands for the BLE module
set_clear = "AT+CLEAR"
set_renew = "AT+RENEW"
set_reset = "AT+RESET"
set_role= "AT+ROLE1"
set_imme = "AT+IMME1"
set_adty = "AT+ADTY3"
set_AT = "AT"
set_connect = "AT+CON3403DE349771"
BLE_port = '/dev/cu.usbserial'

connection_secure = False
N = 400                                         # number of samples plotted
NS = 20                                         # number of samples to read per iteration
sample_count = 0                                # current sample count
filter_order = 3
freq = 200
LPF_cutoff = 10.0/100
HPF_cutoff = 0.5/100
thresh = 800
step = 0
heartrate = 0
avg_heartrate = 0
HRthresh = 5
first = False
times, ay_values, ir_values = np.zeros((3, N))     # data vectors
ay_flt, loc, locations = np.zeros((3,N))
filter_ICs = np.zeros((2, filter_order))
filter_coeffs = np.zeros((4, filter_order + 1))
irSignal = np.ndarray((2,N))

#                               Methods
# Call this function whenever you want to write to the db. Note that there is a maximum of 2 writes per second
def write_to_pyrebase(teamID, hr, steps):
    assert isinstance(teamID, str)
    assert isinstance(hr, int)
    assert isinstance(steps, int)
    
    global last_time
    current_time = time.time()
    if (current_time - last_time >= 0.5):
        last_time = current_time
        data = {"teamID": teamID, "hr": hr, "steps": steps, "timestamp": current_time}
        db.child("readings").push(data)
# Reads input buffer from BLE
def read_BLE (ser):
    msg = ""
    if (ser.in_waiting > 0):
        msg = ser.readline(ser.in_waiting).decode('utf-8')
    return msg
# Initializes settings on BLE for central
def initialize_BLE (ser):
    print("Initializing BLE...")
    ser.reset_input_buffer()

    ser.write(set_AT.encode())
    sleep(0.5)
    print("> AT: " + read_BLE(ser))
    
    ser.write(set_clear.encode())
    sleep(0.5)
    print("> CLEAR: " + read_BLE(ser))
    
    ser.write(set_reset.encode())
    sleep(0.5)
    print("> RESET: " + read_BLE(ser))
    sleep(2)

    ser.write(set_renew.encode())
    sleep(0.5)
    print("> RENEW: " + read_BLE(ser))
    sleep(2)

    ser.write(set_reset.encode())
    sleep(0.5)
    print("> RESET: " + read_BLE(ser))
    sleep(2)

    ser.write(set_imme.encode())
    sleep(0.5)
    print("> IMME: " + read_BLE(ser))

    ser.write(set_adty.encode())
    sleep(0.5)
    print("> ADTY: " + read_BLE(ser))

    ser.write(set_role.encode())
    sleep(0.5)
    print("> ROLE: " + read_BLE(ser))
    sleep(2)

    ser.write(set_reset.encode())
    sleep(0.5)
    print("> RESET: " + read_BLE(ser))
    sleep(2)
# Attempts to establish connection to peripheral HM-10
def connect_BLE (ser):
    global connection_secure
    ser.write(set_connect.encode())
    sleep(0.5)
    ble_in = read_BLE(ser)
    print("> CONNECT: " + ble_in)
    print("> Attempting to Connect...")
    sleep(15)
    ble_in = read_BLE(ser)
    if ( ble_in.find('OK+CONNF') > -1 ):
        print("Connection Failed")
        connection_secure = False
    else:
        print("Connection Successful")
        connection_secure = True
        ble_in = ""
# Attempts to reconnect to HM-10 in the case of failed connection
def reconnect_BLE (ser):
    global connection_secure
    ser.reset_input_buffer()
    command = input("> 1.) Hit ENTER to try Reconnecting\n> 2.) Enter QUIT to exit:\t")
    if (command == ""):
        ser.write(set_connect.encode())
        sleep(0.5)
        ble_in = read_BLE(ser)
        print("> " + ble_in)
        print("> Attempting to Reconnect...")
        sleep(15)
        ble_in = read_BLE(ser)
        print("> " + ble_in)
        if ( ble_in.find('OK+CONNF') > -1 ):
            print("> Reconnection Unsuccessful")
        else:
            print("> Reconnection Successful")
            connection_secure = True
    if (command == "QUIT" or command == "quit"):
        sys.exit(0)
# Applies a Band-pass filter on data signal
def filterSignal(data, filter_coeffs, filter_ICs):
    global first
    # ==================== Step 1 ====================
    if (first == True) :
        LPF, zi_new_LPF = sig.lfilter(filter_coeffs[3, :], filter_coeffs[2, :], data, zi = filter_ICs[1, :]*data[0])
        filter_ICs[1, :] = zi_new_LPF[:]
    else :
        LPF, zi_new_LPF = sig.lfilter(filter_coeffs[3, :], filter_coeffs[2, :], data, zi = filter_ICs[1, :])
        filter_ICs[1, :] = zi_new_LPF[:]

    # ==================== Step 2 ====================
    # Apply HPF filter
    if (first == True) :
        proc_data, zi_new_HPF = sig.lfilter(filter_coeffs[1, :], filter_coeffs[0, :], LPF, zi = filter_ICs[0, :]*LPF[0])
        filter_ICs[0, :] = zi_new_HPF[:]
    else :
        proc_data, zi_new_HPF = sig.lfilter(filter_coeffs[1, :], filter_coeffs[0, :], LPF, zi = filter_ICs[0, :])
        filter_ICs[0, :] = zi_new_HPF[:]

    return proc_data, filter_ICs
# Calculates the HR given the input data signal
def calculate_hr(ecg_signal):
    
    # Variable to contain the heartbeat locations
    hb_loc = np.zeros((np.size(ecg_signal, 0), np.size(ecg_signal, 1)))

    # Calculate signal statistics and choose an adaptive threshold
    meanval = np.mean(ecg_signal[1,:])
    stdval = np.std(ecg_signal[1,:])
    thresh = meanval + 2*stdval

    # Find beginning of beat_loc (count as heartbeat) by looking at the difference operator (approximate gradient)
    beat_loc = ecg_signal[1,:] > thresh
    beat_loc_flip = -(beat_loc-1)
    hb_loc[1, 1:] = beat_loc[1:]*beat_loc_flip[:-1]
    hb_loc[0, :] = ecg_signal[0, :]

    # Calculate the HR by looking at the time difference between successive heartbeats
    hb_times = hb_loc[0, np.where(hb_loc[1, 1:] == 1)][0]
    hb_time_dif = hb_times[1:] - hb_times[:-1]
    hr_at_hb = 60 / hb_time_dif
    
    # Set HR as the average HR over this window of data
    HR = np.mean(hr_at_hb)
    print("HR: ", HR)
    return HR, beat_loc * np.max(ecg_signal[1, :])
# Function to grab data samples from Arduino
def grab_samples(n_samples) :
    global sample_count, filter_coeffs, filter_ICs, first

    # Create local arrays of size(n_samples) to store new data
    times, ay_values, ir_values =  np.zeros((3,n_samples))
    ay_flt = np.zeros(n_samples)

    print("Grabbing Samples...")
    i = 0
    ser.write(b'(')
    ser.reset_input_buffer()
    while i < n_samples :                       # while we still need samples, keep reading
        try:
            data = ser.readline().strip().decode('utf-8')
            t, ay, ir = data.split(' ')
            t = float(t)
            ay = float(ay)
            ir = float(ir)
        except :                                # report error if we failed
            print('Invalid data: ', data)
            continue
        # Store the new values into appropriate local arrays
        times[i] = t / 1000                     # convert time to seconds from ms
        ay_values[i] = ay
        ir_values[i] = ir
        i += 1
    ser.write(b')')
    ser.reset_input_buffer()
    ay_flt, filter_ICs = filterSignal(ay_values, filter_coeffs, filter_ICs)
    print("Done Grabbing Samples!")
    sample_count += n_samples           # Increment sample_count by amount of n_samples
    return times, ay_values, ay_flt, ir_values            # Return two arrays (size n_sample) with new values
# Function to update data samples and other parameters (HR, stepCount, etc.)
def update_plots(ser):
    global times, ay_values, ir_values, locations
    global ay_flt, filter_coeffs, filter_ICs, loc
    global thresh, step, heartrate, avg_heartrate
    print("Updating Plots...")
    # shift samples left by 'NS'
    times[:N-NS] = times[NS:]
    ay_values[:N-NS] = ay_values[NS:]
    ir_values[:N-NS] = ir_values[NS:]
    ay_flt[:N-NS] = ay_flt[NS:]
    loc[:N-NS] = loc[NS:]
    locations[:N-NS] = locations[NS:]

    # grab new samples
    times[N-NS:], ay_values[N-NS:], ay_flt[N-NS:], ir_values[N-NS:] = grab_samples(NS)
    # store times and values into irSignal (ndarray)
    irSignal[0, :] = times[:]
    irSignal[1, :] = ir_values[:]
    # calculate hr and locations
    print(avg_heartrate)
    heartrate, locations = calculate_hr(irSignal)
    if (math.isnan(heartrate) is True):
        heartrate_str = str(int(avg_heartrate))
    else:
        heartrate_str = str(int(heartrate))
    # If avg_heartrate has been reset, store new HR value into avg_heartrate
    if avg_heartrate == 0:
        avg_heartrate = heartrate
    # If spike in HR is detected, send the '*' char to Arduino and reset avg_heartrate
    if heartrate >= (avg_heartrate + HRthresh):
        print("SPIKE DETECTED")
        ser.write(b'*')
        avg_heartrate = 0
    # send heartrate to Arduino via BLE
    ser.write(b'$')
    ser.write(heartrate_str.encode())
    ser.write(b'>')
    
    loc[N-NS:] = 0
    print("Detecting Peaks")
    peaks, props = sig.find_peaks(ay_flt[N-NS:], height=500, distance=10, width=(None,20))
    ser.reset_input_buffer()
    print("Incrementing Steps...")
    for i in peaks:
        loc[N-NS+i] = ay_flt[N-NS+i]
        step = step + 1
        ser.write(b'#')
    
    if (math.isnan(heartrate) is True):
        write_to_pyrebase('SoggyPasta', int(avg_heartrate), step)
    else:
        heartrate_str = str(int(heartrate))
        write_to_pyrebase('SoggyPasta', int(heartrate), step)
    
if (__name__ == "__main__"):
    with serial.Serial(port = BLE_port, baudrate = 9600, timeout = 1) as ser:
        try:
            sleep(2)
            # Initialize the BLE module to our preferred settings
            initialize_BLE(ser)

            # Establishes a connection with the BLE module
            connect_BLE(ser)

            # This loop will execute until a connection has been reestablished
            while (connection_secure == False):
                reconnect_BLE(ser)

            print("Get HPF LPF Coefficients...")
            # get HPF and LPF coefficients
            filter_coeffs[3, :], filter_coeffs[2, :] = sig.butter(filter_order, LPF_cutoff, btype='lowpass', analog=False)
            filter_coeffs[1, :], filter_coeffs[0, :] = sig.butter(filter_order, HPF_cutoff, btype='highpass', analog=False)
            print("Get HPF LP initial conditions....")
            # get HPF and LPF initial conditions
            filter_ICs[1, :] = sig.lfilter_zi( filter_coeffs[3, :], filter_coeffs[2, :] )
            filter_ICs[0, :] = sig.lfilter_zi( filter_coeffs[1, :], filter_coeffs[0, :] )

            # Apply initial filter
            first = True
            times, ay_values, ay_flt, ir_values = grab_samples(N)
            first = False
            peaks, props = sig.find_peaks(ay_flt, height=500, distance=10, width=(None,10))
            # For each step detected, send a '#' code to Arduino side
            for i in peaks:
                loc[i] = ay_flt[i]
                step = step + 1
                ser.write(b'#')
            # store times and values into irSignal (ndarray)
            irSignal[0, :] = times[:]
            irSignal[1, :] = ir_values[:]
            # calculate hr and locations
            heartrate, locations = calculate_hr(irSignal)
            avg_heartrate = heartrate
            heartrate_str = str(int(heartrate))
            # send heartrate to Arduino via Serial
            ser.write(b'$')
            ser.write(heartrate_str.encode())
            ser.write(b'>')

            # Send initial readings to pyrebase
            write_to_pyrebase('SoggyPasta', int(heartrate), step)
            print("Done Initializing")
            while (connection_secure == True):
                # Forever call the following method:
                update_plots(ser)
        finally:
            ser.flush()