import socket
import time
import URBasic
import time
import numpy as np
from URBasic import kinematic
from URBasic.kinematic import Invkine_manip, Tran_Mat2Pose, Pose2Tran_Mat

# Addresses and ports
localAddress = "127.0.0.1"
buffersize = 9

SERVER_PORT = 20002  # Define this servers port number
CLIENT_PORT = 20001  # Define which port on the client the server is going to communicate with

host = '172.31.1.115'  # E.g. a Universal Robot offline simulator, please adjust to match your IP
acc = 0.9
vel = 0.9


class Robot:

    def __init__(self):
        self.robotMod = URBasic.robotModel.RobotModel()
        self.robot = URBasic.urScriptExt.UrScriptExt(host=host, robotModel=self.robotMod)
        self.robot.reset_error()

    def setup(self):
        ur10Pose = np.array([[-0.7071, 0, -0.7071, -0.3],
                             [0.7071, 0, -0.7071, - 0.3],
                             [0, -1, 0, 0.3],
                             [0, 0, 0, 1]])
        self.robot.movej(pose=kinematic.Tran_Mat2Pose(ur10Pose), a=acc, v=vel)
        time.sleep(1)

    setPosition = [0, 0]

    def moveRTC(self, x, z):
        '''
        Real time movement given an x and z vector.
        '''
        rot = np.array([[0.7071, -0.7071, 0],
                        [0.7071, 0.7071, 0],
                        [0, 0, 1]])
        movementVec = np.array([x / 1000, 0, z / 1000])
        movementVec = np.matmul(rot, movementVec)
        currentPose = self.robot.get_actual_tcp_pose()
        currentPose[0:3] = currentPose[0:3] + movementVec
        self.robot.set_realtime_pose(currentPose)

    def getCurrentTranMat(self, coordinate):
        rotation = np.array([[0.7071, 0.7071, 0, 0],
                             [-0.7071, 0.7071, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        currentPose = self.robot.get_actual_tcp_pose()
        tranMat = kinematic.Pose2Tran_Mat(currentPose)
        tranMat = np.matmul(rotation, tranMat)
        print(tranMat)
        return int(round(tranMat[coordinate, -1] * 1000))

    def getCurrentOrientation(self, coordinate):
        currentPose = self.robot.get_actual_tcp_pose()
        return int(round(currentPose[coordinate]*1000))


UR10 = Robot()
UR10.setup()

# Create socket and bind it to a port
server = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
server.bind((localAddress, SERVER_PORT))


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
    output = bytes[0] * 256 ** 3 + bytes[1] * 256 ** 2 + bytes[2] * 256 + bytes[3]
    if sign == 1:
        output = output * (-1)
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
        readData = 0
        if object == 1:
            if subindex == 0:
                readData = UR10.getCurrentTranMat(subindex)
            if subindex == 1:
                readData = UR10.getCurrentTranMat(subindex)
            if subindex == 2:
                readData = UR10.getCurrentTranMat(subindex)
        if object == 2:
            if subindex == 3:
                readData = UR10.getCurrentOrientation(3)
            if subindex == 4:
                readData = UR10.getCurrentOrientation(4)
            if subindex == 5:
                readData = UR10.getCurrentOrientation(5)
        return readData
    elif rw == 1:
        if object == 1:
            if subindex == 3:
                UR10.setPosition[0] = information
            if subindex == 4:
                UR10.setPosition[1] = information
        if object == 2:
            if subindex == 0:
                UR10.moveRTC(UR10.setPosition[0], UR10.setPosition[1])

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
