import socket
import time

# Addresses and ports
localAddress = "127.0.0.1"
buffersize = 9

SERVER_PORT = 20002  # Define this servers port number
CLIENT_PORT = 20001  # Define which port on the client the server is going to communicate with

#UR10.setup()

# Create socket and bind it to a port
server = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
server.bind((localAddress, SERVER_PORT))

class testClass:

    position = [0, 0, 0]
        
testUR10 = testClass

# -------------------------------------------------

# Extracts bytes from an integer and gives them as an array representing 32 bits, with bit 0 to the far right.
def extractBytes(integer):
    firstArray = divmod(integer, 0x100)
    secondArray = divmod(firstArray[0], 0x100)
    thirdArray = divmod(secondArray[0], 0x100)
    output = [thirdArray[0], thirdArray[1], secondArray[1], firstArray[1]]
    return output

# Function which combines entries from a byte array into a single integer.
def combineBytes(sign, bytes):
    output = bytes[0]*256**3 + bytes[1]*256**2 + bytes[2]*256 + bytes[3]
    if sign == 1:
        output = output*(-1)
    return output

# Function which formats data and sends it back to the client.
def respondUDP(object, subindex, readData):
    # Format data
    sign = 0
    if readData < 0:
        sign = 1
        readData = abs(readData)
    readData = extractBytes(readData)
    package = [object, subindex, 0, sign, readData[0], readData[1], readData[2], readData[3]]
    package_array = bytes(package)
    
    server.sendto(package_array, (localAddress, CLIENT_PORT))

# Function which links the information to the correct object and subindex.
def interpretUDPCommand(object, subindex, rw, information):
    if rw == 0:
        if object == 1:
            if subindex == 0:
                UR10.inverseTranMat()[subindex] = information
            if subindex == 1:
                UR10.inverseTranMat()[subindex] = information
            if subindex == 2:
                UR10.inverseTranMat()[subindex] = information
    elif rw == 1:
        if object == 1:
            if subindex == 3:
                UR10.moveRTC()[subindex] = information
            if subindex == 4:
                UR10.moveRTC()[subindex] = information

    else:
        print("Error - Invalid read/write command")

# Main function of the server side UDP. Receives the data, unpacks it, and launches the correct corresponding functions.
def recieveAndUnpack():
    data = server.recvfrom(buffersize)
    data = list(data[0])
    object = data[0]
    subindex = data[1]
    rw = data[2]
    sign = data[3]
    information = combineBytes(sign, data[4:8])
    followingMessages = data[8]

    readData = interpretUDPCommand(object, subindex, rw, information)
    if rw == 0:
        respondUDP(object, subindex, readData)

    if followingMessages > 0:
        recieveAndUnpack()

while True:
    recieveAndUnpack()
    print(testUR10.position)