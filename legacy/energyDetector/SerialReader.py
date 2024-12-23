#Author: Sean O'Connor
#Date: August 20, 2019
#Description: This program reads serial from a teensy connected to an RF detector and write it to a file. input arguments are pathName, fileName, portName. For example, to take serial readings, one might make the call: python3 SerialReader.py /Users/seanoconnor/Desktop/ outputSerialTest1.txt /dev/tty.usbmodem57314001

import serial
import sys
import io

def serialReader(pathName, fileName, portName):
    while True:
        wFile = open(pathName + fileName, 'a+')
        ser = serial.Serial(portName, 9600)
        z = str(ser.readline().splitlines()).strip('b[]').replace("'","")
        print(z)
        sig_str = (-70) + (((float(z) * 1000) - 350)/20)
        print(sig_str)
        wFile.write(z + ',' + str(sig_str) + '\n')
    wFile.close()
    ser.close()

def main():
    serialReader(sys.argv[1],sys.argv[2],sys.argv[3])

if __name__ == '__main__':
    main()
