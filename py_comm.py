import time
from time import sleep
import serial
import pifacedigitalio

pfd = pifacedigitalio.PiFaceDigital()

pfd.relays[1].value = 1

sleep(2);

test=serial.Serial("/dev/ttyUSB0",baudrate=1000000)
test.open()

"""
# Tare robot state
command = "FF FF FE 45 83 1E 04 64 00 02 64 00 01 00 02 64 00 02 00 02 64 00 03 00 02 64 00 04 00 02 64 00 05 00 02 64 00 06 00 02 64 00 07 C3 07 64 00 08 41 07 64 00 09 00 02 64 00 0A 00 02 64 00 0B 00 02 64 00 0C 00 02 64"
test.write(bytearray.fromhex(command))
out = ''
sleep(1)
while test.inWaiting() > 0:
    out += test.read(1)
if out != '':
    print ">>" + out

while 1 :
    command = raw_input("hex input >> ")

#   Python 3 users
    #input = input(">> ")
    if command == 'exit':
        test.close()
        exit()
    else:
        print "received "  + command.decode('hex');
        test.write( command.decode('hex') )
        out = ''
        time.sleep(1)
        while test.inWaiting() > 0:
            out += test.read(1)
        if out != '':
            print ">>" + out
test.close()
"""
pfd.relays[1].value = 0
