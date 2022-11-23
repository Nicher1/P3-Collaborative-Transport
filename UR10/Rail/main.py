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

enableOperation = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x40, 0, 0, 0, 0, 2, 15, 0]
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
        return divmod(integer, 0x100)[::-1]

    # Check if target datavalue is within range
    if target > 0xffff:
        print("Invalid target specified")
    else:
        if target > 255:
            target2Byt = extractBytes(target)
            targetPos = [0, 0, 0, 0, 0, 15, 0, 43, 13, rw, 0, 0, 96, 122, 0, 0, 0, 0, 4, target2Byt[0], target2Byt[1]]
        elif target <= 255:
            targetPos = [0, 0, 0, 0, 0, 14, 0, 43, 13, rw, 0, 0, 96, 122, 0, 0, 0, 0, 1, target]

        print(f"targetPos: {targetPos}")
        #targetPos = [0, 0, 0, 0, 0, 14, 0, 43, 13, rw, 0, 0, 0x60, 0x7a, 0, 0, 0, 0, 1, 250]
        targetPos_array = bytearray(targetPos)
        print("Sending targetPos_array")
        sendCommand(targetPos_array)

        # set velocity and acceleration of profile
        sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x81, 0, 0, 0, 0, 2, 0x2c, 0x1]))
        # Profile acceleration set below
        sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x83, 0, 0, 0, 0, 2, 0x2c, 0x1]))
        # Profile deacceleration set below
        sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x84, 0, 0, 0, 0, 2, 0x2c, 0x1]))

        # # set velocity target
        # sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0xFF, 0, 0, 0, 0, 2, 0x2c, 0x1]))
        
        print("My location:")
        sendCommand(bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 0x60, 0x64, 0, 0, 0, 0, 4]))
        print("targetpos:")
        sendCommand(bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 0x60, 0x7a, 0, 0, 0, 0, 4]))

        # Execute command
        sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, rw, 0, 0, 0x60, 0x40, 0, 0, 0, 0, 2, 0x1f, 0x0]))

        time.sleep(0.1)
        print("targetpos after execute command:")
        sendCommand(bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 0x60, 0x7a, 0, 0, 0, 0, 4]))

        # Check Statusword for target reached
        while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 0x60, 0x41, 0, 0, 0, 0, 2, 39, 22]):
            print("Wait for next command")

            print("targetpos in loop:")
            sendCommand(bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 0x60, 0x7a, 0, 0, 0, 0, 4]))
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

    # # Set homing speeds 6099h
    sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 0x60, 0x99, 0, 0, 0, 0, 1, 0x02]))
    sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 0x60, 0x99, 1, 0, 0, 0, 1, 0xfa]))
    sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 0x60, 0x99, 2, 0, 0, 0, 1, 0xc8]))

    # # Set acceleration 609Ah
    sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x9a, 0, 0, 0, 0, 2, 0xe8, 0x3]))

    time.sleep(1)
    print("About to home")
    # Start Homing 6040h
    sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x40, 0, 0, 0, 0, 2, 0x1f, 0]))
  
    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 0x60, 0x41, 0, 0, 0, 0, 2, 39, 22]):
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
targetPosition(1000)

