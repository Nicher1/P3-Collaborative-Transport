import time
from pyexpat import model
import cv2 as cv
import numpy as np
import mediapipe as mp
import pyk4a as pyk
from pyk4a import PyK4A, connected_device_count
from pyk4a import Config
from time import perf_counter
from helpers import colorize


mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose

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

def main():
    k4a = PyK4A(
        Config(
            color_resolution=pyk.ColorResolution.RES_720P,
            depth_mode=pyk.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=True,
            camera_fps=pyk.FPS.FPS_5

        )
    )
    k4a.start()
    k4a.whitebalance = 4500
    assert k4a.whitebalance==4500
    k4a.whitebalance = 4510
    assert k4a.whitebalance == 4510

    while True:
        k4aCapture = k4a.get_capture()
        if np.any(k4aCapture.depth):
            cv.imshow("k4a", colorize(k4aCapture.depth, (None, 5000), cv.COLORMAP_HSV))
            cv.waitKey(0)
            cap = cv.imread(k4aCapture.depth)
            #cap = cv.VideoCapture(k4aCapture.depth)
            break

            with mp_hands.Hands(
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as hands:
                with mp_pose.Pose(
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as pose:

                    while cap.isOpened():
                        success, image=cap.read()
                        if not success:
                            print("Ignoring empty camera frame.")
                            continue

                        image.flags.writeable=False
                        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
                        resultHands = hands.process(image)
                        result = pose.process(image)

                        image.flags.writeable=True
                        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
                        if resultHands.multi_hand_landmarks:
                            for hand_landmarks in resultHands.multi_hand_landmarks:
                                mp_drawing.draw_landmarks(
                                    image,
                                    hand_landmarks,
                                    mp_hands.HAND_CONNECTIONS,
                                    mp_drawing_styles.get_default_hand_landmarks_style(),
                                    mp_drawing_styles.get_default_hand_connections_style())
                        mp_drawing.draw_landmarks(
                            image,
                            result.pose_landmarks,
                            mp_pose.POSE_CONNECTIONS,
                            landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())


                        # your code here


                # your code here
                LandResults = []
                for id, lm in enumerate(result.pose_landmarks.landmark):
                    LandResults.append([])
                    x= int(lm.x*image.shape[1])
                    y= int(lm.y*image.shape[0])
                    LandResults[id].append(y)
                    LandResults[id].append(x)
                    
                for y, row in enumerate(LandResults):
                    print(row)
                print((LandResults[0])[1])
                
                cv.imshow('MediaPipe', cv.flip(image,1))
                if cv.waitKey(5) & 0xFF == 27:
                    break

if __name__=="__main__":
    main()