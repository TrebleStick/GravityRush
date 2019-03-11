import serial
import time
import csv
ser = serial.Serial('COM3', 115200)
cnt=0
# create csv file
with open('EMG_data/ife_wrist_none.csv', 'w') as new_file:
    csv_writer = csv.writer(new_file)
    emg_0_buffer = []
    emg_1_buffer = []
    while True:
        line = ser.readline()
        line = (line.decode('utf-8')).strip("\n")
        line = line.strip("\r")
        # print(line)
        line = line.split(',')
        emg_0_buffer.append(line[0])
        emg_1_buffer.append(line[1])

        # print(int(line[0]))
        # print(int(line[1]))
        if len(emg_0_buffer) > 1000 :
            csv_writer.writerow(emg_0_buffer)
            csv_writer.writerow(emg_1_buffer)
            # print(emg_0_buffer)
            # print(emg_1_buffer)
            emg_0_buffer = []
            emg_1_buffer = []
            cnt +=1
            print(cnt)
