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
        depth_mode=pyk.DepthMode.NFOV_UNBINNED,
        synchronized_images_only=True,
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


#####################################################################
###########################   FUNCTIONS   ########################### 
#####################################################################

# primary function containing all main code
def main():
    k4a.start()
    k4a.whitebalance = 4500
    assert k4a.whitebalance==4500
    k4a.whitebalance = 4510
    assert k4a.whitebalance == 4510

    while True:
        k4aCapture = k4a.get_capture()
        if np.any(k4aCapture.color):
            tempCap = k4aCapture.color
            
            cap = cv.cvtColor(tempCap, cv.COLOR_BGRA2BGR)

            start = perf_counter()
            
            res, fingertips, stuff = detectHands(cap)

            res = cv.cvtColor(res, cv.COLOR_RGB2BGR)
            end = perf_counter()
            cv.imshow("res",res)
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

# function for hand detection. Also included is processing of the wrists relation to eachother and the middlepoint in between the wrists positional error regarding that of the i
def detectHands(Input_img):
    # Hand detection    
    forHand = cv.cvtColor(Input_img, cv.COLOR_BGR2RGB)
    results = hands.process(forHand)
    #print(results.multi_hand_landmarks)

    # Unpacking Input_img shape pro
    h, w, c = Input_img.shape
    fingertips = np.zeros((5,2))

    Center = [int(h/2), int(w/2)]
    
    OuterBox = [int(np.rint(Center[0]*(1-OuterThresh))),int(np.rint(Center[1]*(1-OuterThresh))), int(np.rint(Center[0]*(1+OuterThresh))),int(np.rint(Center[1]*(1+OuterThresh)))]
    InnerBox = [int(np.rint(Center[0]*(1-InnerThresh))),int(np.rint(Center[1]*(1-InnerThresh))), int(np.rint(Center[0]*(1+InnerThresh))),int(np.rint(Center[1]*(1+InnerThresh)))]

    
    col = 0
    if results.multi_hand_landmarks:
        handPos = []
        for handLms in results.multi_hand_landmarks:
            col += 2
            for id, hand in enumerate(handLms.landmark):
                #print(id,hand)
                cx, cy = int(hand.x *w), int(hand.y*h)

                if id in [0]:
                    cv.circle(forHand, (cx,cy), 4, (int(255/20)*(col*4), 255-int(255/20)*(col*5), 255), cv.FILLED)
                    handPos.append(cy)
                    handPos.append(cx)
            #mpDraw.draw_landmarks(forHand, handLms, mpHands.HAND_CONNECTIONS)
        #print(handPos)
        
      


        for i in range(5):
            fingertips[i][0] = handLms.landmark[(i+1)*4].x * w
            fingertips[i][1] = handLms.landmark[(i+1)*4].y * h

        if len(handPos) == 4:
            meany = int((handPos[0] + handPos[2])/2)
            meanx = int((handPos[1] + handPos[3])/2)
            cv.circle(forHand, (meanx, meany), 4, (0, 255, 0), -1)
            centerDiff = [Center[0]-meanx, Center[1]-meanx] # Contains y and x coordinate difference between hands mean and center respectively

            paperbin = [Center, centerDiff, meany, meanx]
            
            return forHand, fingertips, paperbin

        
        '''
        if averaveX > 288 and averaveX < 352 and averageY > 216 and averageY < 264:
            if averaveX > 304 and averaveX < 336 and averageY > 228 and averageY < 252:
                print("blå")
            else: 
                print("rød")
        else:
            print(Center[0]-averageY, Center[1]-averaveX)
        '''
    paperbin = [0, 0, 0, 0]

    return forHand, fingertips, paperbin

if __name__ == '__main__':
    main()


######################################################################
##########################   OLD SNIPPETS   ########################## 
######################################################################

'''
with mp_pose.Pose(
model_complexity=0,
static_image_mode=True,
enable_segmentation=True,
min_detection_confidence=0.85) as pose:

    result = pose.process(cap)

    LandResults = []
    Idx = []
    try:
        for id, lm in enumerate(result.pose_landmarks.landmark):
            LandResults.append([])
            x= int(lm.x*cap.shape[1])
            y= int(lm.y*cap.shape[0])
            LandResults[id].append(y)
            LandResults[id].append(x)
            Idx.append(id)

        DrawSelect =[LandResults[15],LandResults[16]]



        LandResults = []
        Idx = []
        for id, lm in enumerate(result.pose_landmarks.landmark):
            LandResults.append([])
            x= int(lm.x*cap.shape[1])
            y= int(lm.y*cap.shape[0])
            LandResults[id].append(y)
            LandResults[id].append(x)
            Idx.append(id)

        Center = [cap.shape[0]/2, cap.shape[1]/2]
        OuterBox = [int(np.rint(Center[0]*(1-OuterThresh))),int(np.rint(Center[1]*(1-OuterThresh))), int(np.rint(Center[0]*(1+OuterThresh))),int(np.rint(Center[1]*(1+OuterThresh)))]
        InnerBox = [int(np.rint(Center[0]*(1-InnerThresh))),int(np.rint(Center[1]*(1-InnerThresh))), int(np.rint(Center[0]*(1+InnerThresh))),int(np.rint(Center[1]*(1+InnerThresh)))]


        Center = [cap.shape[0]/2,cap.shape[1]/2]
        averaveX = int(np.rint((LandResults[15][1]+LandResults[16][1])/2))
        averageY = int(np.rint((LandResults[15][0]+LandResults[16][0])/2))

        cv.rectangle(cap, (OuterBox[1],OuterBox[0]),(OuterBox[3],OuterBox[2]),(0,0,255),-1)
        cv.rectangle(cap, (InnerBox[1],InnerBox[0]), (InnerBox[3],InnerBox[2]),(255,0,0),-1)
        cv.circle(cap, (averaveX, averageY), 5, (0,255,0), -1)

        if averaveX > 288 and averaveX < 352 and averageY > 216 and averageY < 264:
            if averaveX > 304 and averaveX < 336 and averageY > 228 and averageY < 252:
                print("blå")
            else: 
                print("rød")
        else:
            print(Center[0]-averageY, Center[1]-averaveX)
        
        for i in range(len(DrawSelect)):
            cv.circle(cap, (DrawSelect[i][1], DrawSelect[i][0]), 5, (0, 255, 0), -1)
        
    except AttributeError:
        continue
    end = perf_counter()
    print(end-start)
    cv.imshow("Result", cap)
    # cv.imshow("depth", colorize(k4aCapture.depth, (None, 5000), cv.COLORMAP_HSV))
    cv.waitKey(1)
'''