import cv2 as cv
import numpy as np
from pyk4a import PyK4A, connected_device_count
from pyk4a import Config
from time import perf_counter
import mediapipe as mp
from helpers import colorize
import pyk4a as pyk

#####################################################################
###########################     SETUP     ########################### 
#####################################################################

# Setup constants for use in main(), which needs to be defined only once
InnerThresh = 0.05
OuterThresh = 0.1
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
assert k4a.whitebalance==4500
k4a.whitebalance = 4510
assert k4a.whitebalance == 4510


#####################################################################
###########################   FUNCTIONS   ########################### 
#####################################################################

def main():
    while True:
        toDoOrNotToDo = cameraUI()
        print(toDoOrNotToDo)

# primary function containing all main code
def cameraUI():
    k4aCapture = k4a.get_capture()
    if np.any(k4aCapture.color):
        capCol = cv.cvtColor(k4aCapture.color, cv.COLOR_BGRA2BGR)
        capTransDepth = k4aCapture.transformed_depth

        start = perf_counter()
        
        res, fingertips, stuff = detectHands(capCol, capTransDepth)

        res = cv.cvtColor(res, cv.COLOR_RGB2BGR)
        end = perf_counter()
        # cv.imshow("res",res)
        cv.imshow("transformed col to depth persceptive", colorize(capTransDepth, (None, 5000), cv.COLORMAP_HSV))
        cv.waitKey(1)
    
        #if stuff[0] != 0 and stuff[1] != 0 and stuff[2] != 0 and stuff[3] != 0:
        if np.any(stuff) != 0:
            Center = stuff[0]
            centerDiff = stuff[1]
            meany = stuff[2]
            meanx = stuff[3]

            if meanx > Center[1]+(1-OuterThresh) and meanx < Center[1]+(1+OuterThresh) and meany > Center[0]-(1+OuterThresh) and meany < Center[0]+(1+OuterThresh):
                if meanx > Center[1]+(1-InnerThresh) and meanx < Center[1]+(1+InnerThresh) and meany > Center[0]-(1+InnerThresh) and meany < Center[0]+(1+InnerThresh):
                    doStuff = False
                else:
                    doStuff = False

            else:
                doStuff = True

            return doStuff

# function for hand detection. Also included is processing of the wrists relation to eachother and the middlepoint in between the wrists positional error regarding that of the i
def detectHands(Input_img_col, Input_img_depth):
    # Hand detection    
    imgCol = cv.cvtColor(Input_img_col, cv.COLOR_BGR2RGB)
    imgDepth = Input_img_depth

    results = hands.process(imgCol)
    #print(results.multi_hand_landmarks)

    # Unpacking Input_img shape pro
    hdep, wdep = Input_img_depth.shape
    h, w, c = Input_img_col.shape
    fingertips = np.zeros((5,2))

    Center = [int(h/2), int(w/2)]
    
    OuterBox = [int(np.rint(Center[0]*(1-OuterThresh))),int(np.rint(Center[1]*(1-OuterThresh))), int(np.rint(Center[0]*(1+OuterThresh))),int(np.rint(Center[1]*(1+OuterThresh)))]
    InnerBox = [int(np.rint(Center[0]*(1-InnerThresh))),int(np.rint(Center[1]*(1-InnerThresh))), int(np.rint(Center[0]*(1+InnerThresh))),int(np.rint(Center[1]*(1+InnerThresh)))]

    col = 0
    if results.multi_hand_landmarks:
        handPos = []
        print("-------------------------------")
        for handLms in results.multi_hand_landmarks:
            col += 1
            for id, hand in enumerate(handLms.landmark):
                #print(id,hand)
                cx, cy = int(hand.x *w), int(hand.y*h)

                if id in [0]:
                    print("Dist wrist", col, ": ", imgDepth[cy, cx])
                    cv.circle(imgDepth, (cx,cy), 4, (int(255/20)*(col*4), 255-int(255/20)*(col*5), 255), cv.FILLED)
                    handPos.append(cy)
                    handPos.append(cx)
        
        for i in range(5):
            fingertips[i][0] = handLms.landmark[(i+1)*4].x * w
            fingertips[i][1] = handLms.landmark[(i+1)*4].y * h

        if len(handPos) == 4:
            meany = int((handPos[0] + handPos[2])/2)
            meanx = int((handPos[1] + handPos[3])/2)
            cv.circle(imgDepth, (meanx, meany), 4, (0, 255, 0), -1)
            centerDiff = [Center[0]-meanx, Center[1]-meanx] # Contains y and x coordinate difference between hands mean and center respectively

            paperbin = [Center, centerDiff, meany, meanx]
            
            return imgCol, fingertips, paperbin

    paperbin = [0, 0, 0, 0]

    return imgCol, fingertips, paperbin


if __name__ == '__main__':
    main()