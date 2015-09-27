import csv
import time
from serial import Serial

s = Serial(port="/dev/ttyACM0", baudrate=115200,timeout=20)

#Want to save serial to file, one for each temperature sensor. each line has a different temperature sensor
for row in csv.reader(iter(s.readline, None)):
    #handle each set of data
    #print row

    should_break = 0
    #reject bad rows
    if len(row) != 19:
        with open("/home/dustin/data/error_data_0.txt",'a') as file_:
            file_.write( ', '.join(row))
            file_.write('\n')
        continue

    if len(row[0].split(".")) != 18:
        with open("/home/dustin/data/error_data_1.txt",'a') as file_:
            file_.write( ', '.join(row))
            file_.write('\n')
        continue

    #check length of each argument
    if len(row[0]) != 113:
        with open("/home/dustin/data/error_data_2.txt",'a') as file_:
            file_.write( ', '.join(row))
            file_.write('\n')
        continue

    if row[0].count(".") != 17:
        with open("/home/dustin/data/error_data_3.txt",'a') as file_:
            file_.write( ', '.join(row))
            file_.write('\n')
        continue

    for test in row[1:]:
        if len(test) != 2:
            #for floating point temperature
            if test.count(".") == 1:
                 continue
            print "length not 2\r\n"
            with open("/home/dustin/data/error_data_4.txt",'a') as file_:
                file_.write( ', '.join(row) + "\n")
                file_.write('\n')
            should_break = 1
            break

    if should_break == 1:
        continue
    defin = row[0].split(".")

    filename = '/home/dustin/data/' + time.strftime("%Y%m%d") + 'dev_' + row[1] +"_"+ row[4] + row[5] + row[6] + row[7] + row[8] + row[9] + row[10] + row[11] + '.csv'

    with open(filename, 'a') as file_:
        file_.write(row[12] + "," + \
                row[13] + "," + \
                row[14] + "," + \
                row[15] + "," + \
                row[16] + "," + \
                row[17] + "," + \
                row[18] + ",\n")

s.close()
