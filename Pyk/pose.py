import cv2 as cv
import numpy as np
import mediapipe as mp
import pyk4a as pyk
from pyk4a import PyK4A, connected_device_count
from pyk4a import Config
from time import perf_counter
import helpers

InnerThresh = 0.05
OuterThresh = 0.1

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose
'''
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
'''
def main():
    while True:

        cap = cv.VideoCapture(0)
        

        with mp_pose.Pose(
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as pose:

            while cap.isOpened():
                success, image=cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    continue
                temp = image.copy()

                image.flags.writeable=False
                image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
                result = pose.process(image)

                image.flags.writeable=True
                image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
                
                mp_drawing.draw_landmarks(
                    temp,
                    result.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS)
                
                # your code here
                LandResults = []
                Idx = []
                for id, lm in enumerate(result.pose_landmarks.landmark):
                    LandResults.append([])
                    x= int(lm.x*image.shape[1])
                    y= int(lm.y*image.shape[0])
                    LandResults[id].append(y)
                    LandResults[id].append(x)
                    Idx.append(id)



                DrawSelect =[LandResults[15],LandResults[16]]
                IdSelect = [Idx[11],Idx[12],Idx[13],Idx[14],Idx[15],Idx[16], Idx[23],Idx[24]]
                
                Center = [image.shape[0]/2,image.shape[1]/2]
                OuterBox = [int(np.rint(Center[0]*(1-OuterThresh))),int(np.rint(Center[1]*(1-OuterThresh))), int(np.rint(Center[0]*(1+OuterThresh))),int(np.rint(Center[1]*(1+OuterThresh)))]
                InnerBox = [int(np.rint(Center[0]*(1-InnerThresh))),int(np.rint(Center[1]*(1-InnerThresh))), int(np.rint(Center[0]*(1+InnerThresh))),int(np.rint(Center[1]*(1+InnerThresh)))]

                averaveX = int(np.rint((LandResults[15][1]+LandResults[16][1])/2))
                averageY = int(np.rint((LandResults[15][0]+LandResults[16][0])/2))
                cv.rectangle(image, (OuterBox[1],OuterBox[0]),(OuterBox[3],OuterBox[2]),(0,0,255),-1)
                cv.rectangle(image, (InnerBox[1],InnerBox[0]), (InnerBox[3],InnerBox[2]),(255,0,0),-1)
                cv.circle(image, (averaveX, averageY), 5, (0,255,0), -1)

                if averaveX > 288 and averaveX < 352 and averageY > 216 and averageY < 264:
                    if averaveX > 304 and averaveX < 336 and averageY > 228 and averageY < 252:
                        print("blå")
                    else: 
                        print("rød")
                else:
                    print(Center[0]-averageY, Center[1]-averaveX)

                for i in range(len(DrawSelect)):
                    cv.circle(image, (DrawSelect[i][1],DrawSelect[i][0]),5,(0,255,0),-1)
                    cv.putText(image, str(IdSelect[i]),(DrawSelect[i][1],DrawSelect[i][0]),cv.FONT_HERSHEY_PLAIN,1,(255,0,0),2)

                cv.imshow('MediaPipe', cv.flip(image,1))
                cv.imshow("temp", temp)
                if cv.waitKey(5) & 0xFF == 27:
                    quit()

if __name__=="__main__":
    main()
