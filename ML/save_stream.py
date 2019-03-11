import serial
import time
import csv
ser = serial.Serial('COM3', 115200)

# create csv file
with open('EMG_data/test_data.csv', 'w') as new_file:
    csv_writer = csv.writer(new_file)
    emg_0_buffer = []
    emg_1_buffer = []
    while True:
        line = ser.readline()
        line = (line.decode('utf-8')).strip("\n")
        line = line.strip("\r")
        print(line)
        line = line.split(',')
        emg_0_buffer.append(line[0])
        emg_1_buffer.append(line[1])

        # print(int(line[0]))
        # print(int(line[1]))
        if len(emg_0_buffer) > 350 :
            csv_writer.writerow(emg_0_buffer)
            csv_writer.writerow(emg_1_buffer)
            emg_0_buffer = []
            emg_1_buffer = []
