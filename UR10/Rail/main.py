# # echo-server.py

import socket
import time

HOST = "172.31.1.101"
PORT = 503

read = 0
write = 1

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

# Commands/arrays -------------------------------------------

status = [0, 0, 0, 0, 0, 13, 0, 43, 13, read, 0, 0, 96, 65, 0, 0, 0, 0, 2]
status_array = bytearray(status)

shutdown = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 6, 0]
shutdown_array = bytearray(shutdown)

switchOn = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 7, 0]
switchOn_array = bytearray(switchOn)

enableOperation = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 15, 0]
enableOperation_array = bytearray(enableOperation)

# Function for shutdown
def set_shdn():
    sendCommand(shutdown_array)
    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 6]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 22]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 2]):
        print("wait for shutdown")

        #1 Sekunde Verzoegerung
        #1 second delay
        time.sleep(1)

# Function for switching on
def set_swon():
    sendCommand(switchOn_array)
    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 6]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 22]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 2]):
        print("wait for switch on")

        #1 Sekunde Verzoegerung
        #1 second delay
        time.sleep(1)

# Function for enabling operation
def set_op_en():
    sendCommand(enableOperation_array)
    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 6]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 2]):
        print("wait for op en")

        #1 Sekunde Verzoegerung
        #1 second delay
        time.sleep(1)

def set_mode(mode):

    #Set operation modes in object 6060h Modes of Operation
    sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, mode]))
    while (sendCommand(bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1])) != [0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode]):

        print("wait for mode")

        #1 second delay
        time.sleep(1)

def startProcedure():
    reset = [0, 0, 0, 0, 0, 15, 0, 43, 13, write, 0, 0, 96, 64, 0, 0, 0, 0, 2, 0, 1]
    reset_array = bytearray(reset)
    sendCommand(reset_array)

    sendCommand(status_array)

    set_shdn()
    set_swon()
    set_op_en()

def getReadyToMove():
    set_mode(1)
    # # Set velocity 6081h
    # sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 129, 0, 0, 0, 0, 2, 244, 1]))

    # # Set acceleration 6083h
    # sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 131, 0, 0, 0, 0, 2, 184, 11]))

def targetPosition(target, rw=1):

    def extractBytes(integer):
        return divmod(integer, 0x100)

    # Check if target datavalue is within range
    if target > 0xffff:
        print("Invalid target specified")
    else:
        if target > 255:
            hex(target)
            targetleft = extractBytes(target)[1]
            targetright = extractBytes(target)[0]
            targetPos = [0, 0, 0, 0, 0, 15, 0, 43, 13, rw, 0, 0, 96, 122, 0, 0, 0, 0, 2, targetleft, targetright]
        elif target <= 255:
            targetPos = [0, 0, 0, 0, 0, 14, 0, 43, 13, rw, 0, 0, 96, 122, 0, 0, 0, 0, 1, target]

        targetPos_array = bytearray(targetPos)
        sendCommand(targetPos_array)

        # Execute command
        sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 31, 0]))

        time.sleep(1)

        # Check Statusword for target reached
        while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
            print("wait for next command")
            # 1 second delay
            time.sleep(1)

        sendCommand(enableOperation_array)

#Definition of the function to send and receive data
def sendCommand(data):
    #Create socket and send request
    s.send(data)
    res = s.recv(24)
    #Print response telegram
    print(list(res))
    return list(res)

def homing():
    set_mode(6)

    setHomingMethodLSP = [0, 0, 0, 0, 0, 14, 0, 43, 13, write, 0, 0, 96, 152, 0, 0, 0, 0, 1, 17]
    setHomingMethodLSP_array = bytearray(setHomingMethodLSP)
    sendCommand(setHomingMethodLSP_array)

    # Set 60A8 SI Unit position to multiplying factor 1 (positions and alike are now given in orders of 1mm)
    # sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, write, 0, 0, 96, 168, 0, 0, 0, 0, 4, 0, 0, 1, 249]))

    # # Set feed constant 6092h_01h
    # sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 146, 1, 0, 0, 0, 2, 64, 56]))

    # Set feed revolutions 6092h_02h
    # sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 146, 2, 0, 0, 0, 1, 1]))

    # # Set speeds 6099h
    sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 153, 1, 0, 0, 0, 1, 0xc8]))
    sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 153, 2, 0, 0, 0, 1, 0xc8]))

    # # Set acceleration 609Ah
    sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 154, 0, 0, 0, 0, 2, 0xe8, 0x3]))

    time.sleep(1)
    print("About to home")
    # Start Homing 6040h
    sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 31, 0]))

    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
        print("wait for Homing to end")
        # 1 second delay
        time.sleep(1)

    print("Homing complete")

    sendCommand(enableOperation_array)

# def togglePower():
#     digitalInput7 = [0, 0, 0, 0, 0, 17, 0, 43, 13, write, 0, 0, 32, 16, 0, 0, 0, 0, 4, 0, 0, 0, 102]
#     digitalInput7_array = bytearray(digitalInput7)
#     sendCommand(digitalInput7_array)
#
# togglePower()
startProcedure()

homing()

getReadyToMove()

targetPosition(10)