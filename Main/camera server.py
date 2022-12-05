import cv2 as cv
import numpy as np
from pyk4a import PyK4A, connected_device_count
from pyk4a import Config
from time import perf_counter
import mediapipe as mp
from helpers import colorize
import pyk4a as pyk
import socket
import time

# Addresses and ports
localAddress = "127.0.0.1"
buffersize = 9

SERVER_PORT = 20006  # Define this servers port number
CLIENT_PORT = 20005  # Define which port on the client the server is going to communicate with

# Create socket and bind it to a port
server = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
server.bind((localAddress, SERVER_PORT))


#####################################################################
###########################    CLASSES    ###########################
#####################################################################

class camera:
    position = [0, 0, 0]


class currentState:
    state = False


#####################################################################
###########################    SETTINGS    ##########################
#####################################################################

# Print settings
printXYZ = False
printWristDist = False

# Depth image settings
drawCirclesDEPTH = True
showImageDEPTH = True

# RGB image settings
drawCirclesRGB = False
showImageRGB = False

#####################################################################
###########################     SETUP     ###########################
#####################################################################

# Class instances
pos = camera()
state = currentState()

# Setup constants for use in main(), which needs to be defined only once
InnerThresh = 0.10
OuterThresh = 0.20
col = (0, 255, 0)

# Definition of mediapipe tracking solutions and drawing styles
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose

# Detection of the connected Azure Kinect camera
cnt = connected_device_count()
if not cnt:
    print("No devices available")
    exit()
print(f"Available devices: {cnt}")
for device_id in range(cnt):
    device = PyK4A(device_id=device_id)
    device.open()
    print(f"{device_id}: {device.serial}")
    device.close()

# Camera configuration
k4a = PyK4A(
    Config(
        color_resolution=pyk.ColorResolution.RES_720P,
        camera_fps=pyk.FPS.FPS_15,
        depth_mode=pyk.DepthMode.WFOV_UNBINNED,
        synchronized_images_only=True
    )
)

# Hand detection setup params
mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=False,
                      max_num_hands=2,
                      min_detection_confidence=0.5,
                      min_tracking_confidence=0.5)

mpDraw = mp.solutions.drawing_utils
# Detect the joint of hands and return the position of finger tips
# https://google.github.io/mediapipe/solutions/hands.html

k4a.start()
k4a.whitebalance = 4500
assert k4a.whitebalance == 4500
k4a.whitebalance = 4510
assert k4a.whitebalance == 4510


#####################################################################
###########################   FUNCTIONS   ###########################
#####################################################################

def main():
    while True:
        cameraUI()
        recieveAndUnpack()
        print("Looped")
       


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
    if rw == 0:  # Determine if the information should be read or written to the object.
        readData = 0
        if object == 31:
            if subindex == 0:
                readData = pos.position[0]
            if subindex == 1:
                readData = pos.position[1]
            if subindex == 2:
                readData = pos.position[2]
        elif object == 32:
            if subindex == 0:
                readData = state.state
        return readData
    elif rw == 1:
        print("Write commandos are not compatible with the camera system. Read only.")
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

# primary function containing all main code
def cameraUI():
    k4aCapture = k4a.get_capture()
    if np.any(k4aCapture.color):
        capCol = cv.cvtColor(k4aCapture.color, cv.COLOR_BGRA2BGR)
        capTransDepth = k4aCapture.transformed_depth

        start = perf_counter()

        res, unpackPackage = detectHands(capCol, capTransDepth)

        res = cv.cvtColor(res, cv.COLOR_RGB2BGR)
        end = perf_counter()
        # cv.imshow("res",res)

        if showImageRGB == True:
            cv.imshow('RGB', capCol)
            cv.waitKey(1)

        if showImageDEPTH == True:
            cv.imshow("transformed col to depth persceptive", colorize(capTransDepth, (None, 5000), cv.COLORMAP_HSV))
            cv.waitKey(1)

        if np.any(unpackPackage) != 0:
            Center = unpackPackage[0]
            centerDiff = unpackPackage[1]
            meany = unpackPackage[2]
            meanx = unpackPackage[3]
            length = unpackPackage[4]

            # print(length)
            if printXYZ == True:
                global iteration
                iteration = iteration + 1
                if iteration % 10 == 0:
                    vect = pixelDist2EucDist(meanx, meany, length)
                    print(vect)
            else:
                vect = pixelDist2EucDist(meanx, meany, length)

            pos.position = [int(vect[0]), int(vect[1]), int(vect[2])]

            if meanx > Center[1] + (1 - OuterThresh) and meanx < Center[1] + (1 + OuterThresh) and meany > Center[0] - (
                    1 + OuterThresh) and meany < Center[0] + (1 + OuterThresh):
                if meanx > Center[1] + (1 - InnerThresh) and meanx < Center[1] + (1 + InnerThresh) and meany > Center[
                    0] - (1 + InnerThresh) and meany < Center[0] + (1 + InnerThresh):
                    state.state = 0
                else:
                    state.state = 0

            else:
                state.state = 1


# function for hand detection. Also included is processing of the wrists relation to eachother and the middlepoint in between the wrists positional error regarding that of the i
def detectHands(Input_img_col, Input_img_depth):
    # Hand detection
    imgCol = cv.cvtColor(Input_img_col, cv.COLOR_BGR2RGB)
    imgDepth = Input_img_depth

    results = hands.process(imgCol)
    # print(results.multi_hand_landmarks)

    # Unpacking Input_img shape pro
    hdep, wdep = Input_img_depth.shape
    h, w, c = Input_img_col.shape
    fingertips = np.zeros((5, 2))

    Center = [int(h / 2), int(w / 2)]

    OuterBox = [int(np.rint(Center[0] * (1 - OuterThresh))), int(np.rint(Center[1] * (1 - OuterThresh))),
                int(np.rint(Center[0] * (1 + OuterThresh))), int(np.rint(Center[1] * (1 + OuterThresh)))]
    InnerBox = [int(np.rint(Center[0] * (1 - InnerThresh))), int(np.rint(Center[1] * (1 - InnerThresh))),
                int(np.rint(Center[0] * (1 + InnerThresh))), int(np.rint(Center[1] * (1 + InnerThresh)))]

    col = 0
    if results.multi_hand_landmarks:
        handPos = []
        if printWristDist == True:
            print("-------------------------------")
        for handLms in results.multi_hand_landmarks:
            col += 1
            for id, hand in enumerate(handLms.landmark):
                # print(id,hand)
                cx, cy = int(hand.x * w), int(hand.y * h)

                if id in [0]:
                    if printWristDist == True:
                        print("Dist wrist", col, ": ", imgDepth[cy, cx])
                    handPos.append(cy)
                    handPos.append(cx)

        for i in range(5):
            fingertips[i][0] = handLms.landmark[(i + 1) * 4].x * w
            fingertips[i][1] = handLms.landmark[(i + 1) * 4].y * h

        if len(handPos) == 4:
            meany = int((handPos[0] + handPos[2]) / 2)
            meanx = int((handPos[1] + handPos[3]) / 2)
            centerDiffLenght = int(imgDepth[handPos[0], handPos[1]] + imgDepth[handPos[2], handPos[3]]) / 2
            if drawCirclesDEPTH == True:
                cv.circle(imgDepth, (meanx, meany), 4, (0, 255, 0), -1)
            if drawCirclesRGB == True:
                cv.circle(imgCol, (meanx, meany), 4, (0, 255, 0), -1)
            centerDiff = [Center[0] - meanx, Center[
                1] - meanx]  # Contains y and x coordinate difference between hands mean and center respectively
            returnPackage = [Center, centerDiff, meany, meanx, centerDiffLenght]

        if len(handPos) == 2:
            if drawCirclesDEPTH == True:
                cv.circle(imgDepth, (handPos[1], handPos[0]), 4,
                          (int(255 / 20) * (col * 4), 255 - int(255 / 20) * (col * 5), 255), cv.FILLED)
            if drawCirclesRGB == True:
                cv.circle(imgCol, (handPos[1], handPos[0]), 4,
                          (int(255 / 20) * (col * 4), 255 - int(255 / 20) * (col * 5), 255), cv.FILLED)

        if len(handPos) == 4:
            if drawCirclesDEPTH == True:
                cv.circle(imgDepth, (handPos[1], handPos[0]), 4,
                          (int(255 / 20) * (col * 4), 255 - int(255 / 20) * (col * 5), 255), cv.FILLED)
                cv.circle(imgDepth, (handPos[3], handPos[2]), 4,
                          (int(255 / 20) * (col * 4), 255 - int(255 / 20) * (col * 5), 255), cv.FILLED)
            if drawCirclesRGB == True:
                cv.circle(imgCol, (handPos[1], handPos[0]), 4,
                          (int(255 / 20) * (col * 4), 255 - int(255 / 20) * (col * 5), 255), cv.FILLED)
                cv.circle(imgCol, (handPos[3], handPos[2]), 4,
                          (int(255 / 20) * (col * 4), 255 - int(255 / 20) * (col * 5), 255), cv.FILLED)
            return imgCol, returnPackage

    returnPackage = [0, 0, 0, 0, 0]

    return imgCol, returnPackage


def pixelDist2EucDist(xp, yp, h, FOVx=(np.pi / 2), FOVy=1.03, xwidth=1280, yheight=720):
    # xp = x coordinate in pixels, yp = y coordinate in pixels. h = 3D distance to point.
    # All angles are in radians
    thetax = (FOVx / xwidth) * (xp - (xwidth / 2))
    thetay = (FOVy / yheight) * (yp - (yheight / 2))
    x = h * np.sin(thetax)
    y = h * np.sin(thetay)
    z = h * np.cos(thetax)
    pos = [x, y, z]
    return pos


if __name__ == '__main__':
    main()
