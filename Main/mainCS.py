import socket
import time
import numpy as np
import sys
sys.path_insert(0, "..")
from PIDConroller.URScript.just_PID import PID

# Client code -------------------------------------------------------

# Addresses and ports
localAddress = "127.0.0.1"
buffersize = 9

# Main code that runs once, abselutly has to be there or there is no PID controller
constants_y = [1, 0.002, 0.01]
constants_xz = [1, 1, 1]
PIDy = PID(Kp=constants_y[0], Ki=constants_y[1], Kd=constants_y[2], lim_max=300, lim_min=0)
PIDxz = PID(Kp=constants_xz[0], Ki=constants_xz[1], Kd=constants_xz[2], lim_max=0.1,
            lim_min=0)  # A position of max 100 mm will give a velocity of 125 mm/s

class subsys:
    def __init__(self, CLIENT_PORT, SERVER_PORT):
        self.CLIENT_PORT = CLIENT_PORT
        self.SERVER_PORT = SERVER_PORT
        self.s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.s.bind((localAddress, self.CLIENT_PORT))

ur10 = subsys(20001, 20002)
rail = subsys(20003, 20004)
camera = subsys(20005, 20006)

def extractBytes(integer):
    firstArray = divmod(integer, 0x100)
    secondArray = divmod(firstArray[0], 0x100)
    thirdArray = divmod(secondArray[0], 0x100)
    output = [thirdArray[0], thirdArray[1], secondArray[1], firstArray[1]]
    return output

def combineBytes(sign, bytes):
    output = bytes[0]*256**3 + bytes[1]*256**2 + bytes[2]*256 + bytes[3]
    if sign == 1:
        output = output*(-1)
    return output

def communicateUDP(sub_system, object, subindex=0, rw=0, information=0, nr_of_following_messages=0):

    # Format data
    sign = 0
    if information < 0:
        sign = 1
        information = abs(information)
    information = extractBytes(information)
    package = [object, subindex, rw, sign, information[0], information[1], information[2], information[3], nr_of_following_messages]
    package_array = bytes(package)

    # Send data
    sub_system.s.sendto(package_array, (localAddress, sub_system.SERVER_PORT))

    # Receive data
    if rw == 0:
        recieved = sub_system.s.recvfrom(buffersize)
        recieved = list(recieved[0])
        recieved = combineBytes(recieved[3], recieved[4:8])
        return recieved

def recieveAndUnpack():
    data = server.recvfrom(buffersize)
    data = list(data[0])
    object = data[0]
    subindex = data[1]
    rw = data[2]
    sign = data[3]
    information = combineBytes(sign, data[4:8])
    followingMessages = data[8]

    interpretUDPCommand(object, subindex, rw, information)

    if followingMessages > 0:
        recieveAndUnpack()

# Function which links the information to the correct object and subindex.
def interpretUDPCommand(object, subindex, rw, information):
    if rw == 0:
        readData = 0
        return readData
    elif rw == 1:
        if object == 21:
            if subindex == 0:
                cameraData.cameraData[0] = information
            if subindex == 1:
                cameraData.cameraData[1] = information
            if subindex == 2:
                cameraData.cameraData[2] = information
        if object == 22:
            if subindex == 0:
                cameraData.cameraData = information

    else:
        print("Error - Invalid read/write command")

# End Client code ---------------------------------------------------------------

class cameraData:
    currentPos = [0, 0, 0]
    state = False

cameraData = cameraData()

T_camera_EE = np.array([1, 0, 0, -32],
                           [0, 1, 0, 48],
                           [0, 0, 1, -175],
                           [0, 0, 0, 1])

T_towel_EE = np.array([0, 21, -80])

T_EE_robotbase = np.array([0, 1, 0, 0],
                           [0, 0, -1, 0],
                           [-1, 0, 0, 0],
                           [0, 0, 0, 1])

T_robotbase_global = np.array([1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1])

while True:
    # Step 1: Update all variables (attain current position, and human position from camera)

    T_EE_robotbase[0, 3] = communicateUDP(ur10, 1, 0, nr_of_following_messages=2)
    T_EE_robotbase[1, 3] = communicateUDP(ur10, 1, 1, nr_of_following_messages=1)
    T_EE_robotbase[2, 3] = communicateUDP(ur10, 1, 2, nr_of_following_messages=0)

    T_robotbase_global[1, 3] = communicateUDP(rail, 11, 1, nr_of_following_messages=0)

    recieveAndUnpack()  # Collect the latest information from the camera, and store it in cameraData
    humanPosGlobal = cameraData.currentPos*T_camera_EE*T_EE_robotbase*T_robotbase_global

    towelPosGlobal = T_towel_EE*T_EE_robotbase*T_robotbase_global  # The current position of our towel/end effector in global frame.

    # Step 2: Calculate goal position and push it through PID controller for X, Y and Z axis.

    goalPos = humanPosGlobal + np.array([1000, 0, 0])  #GoalPos is given by a translation form the humanPos, which is our restrictions.
    error = goalPos - towelPosGlobal

    pid = PID(1, 0.002, 0.01, -300, 300)

    # Step 3: Push new information to rail and UR10.
    communicateUDP(rail, 12, rw=1, information=PIDoutput[1])  # Target velocity for rail
    communicateUDP(ur10, 1, 3, rw=1, information=PIDoutput[0], nr_of_following_messages=1)
    communicateUDP(ur10, 1, 4, rw=1, information=PIDoutput[2], nr_of_following_messages=0)
