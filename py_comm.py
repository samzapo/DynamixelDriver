import time
import serial

test=serial.Serial("/dev/ttyUSB0",baudrate=1000000)
test.open()

# Tare robot state
command = "FF FF FE 45 83 1E 4 64 0 2 64 0 1 0 2 64 0 2 0 2 64 0 3 0 2 64 0 4 0 2 64 0 5 0 2 64 0 6 0 2 64 0 7 C3 7 64 0 8 41 7 64 0 9 0 2 64 0 A 0 2 64 0 B 0 2 64 0 C 0 2 64"
#test.write( command.decode('hex') )
test.write(bytearray.fromhex(command))
out = ''
time.sleep(1)
while test.inWaiting() > 0:
    out += test.read(1)
if out != '':
    print ">>" + out

"""
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

"""

test.close()
