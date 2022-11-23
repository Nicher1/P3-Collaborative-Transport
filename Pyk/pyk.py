import cv2 as cv
import numpy as np
from pyk4a import PyK4A, connected_device_count
from pyk4a import Config
from time import perf_counter
import mediapipe as mp
from helpers import colorize
import pyk4a as pyk


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
        camera_fps=pyk.FPS.FPS_5,
        depth_mode=pyk.DepthMode.NFOV_UNBINNED,
        synchronized_images_only=True,
    # pyk.ImageFormat
        )
    )

    k4a.start()
    k4a.whitebalance = 4500
    assert k4a.whitebalance==4500
    k4a.whitebalance = 4510
    assert k4a.whitebalance == 4510

    col = (0, 255, 0)

    while True:
        k4aCapture = k4a.get_capture()
        if np.any(k4aCapture.color):
            tempCap = k4aCapture.color
            
            cap = cv.cvtColor(tempCap, cv.COLOR_BGRA2BGR)

            start = perf_counter()
            
            with mp_pose.Pose(
            model_complexity=0,
            static_image_mode=True,
            enable_segmentation=True,
            min_detection_confidence=0.7) as pose:

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

                    Center = [cap.shape[0]/2,cap.shape[1]/2]
                    averaveX = int(np.rint((LandResults[15][1]+LandResults[16][1])/2))
                    averageY = int(np.rint((LandResults[15][0]+LandResults[16][0])/2))

                    if averaveX > 288 and averaveX < 352 and averageY > 216 and averageY < 264:
                        if averaveX > 304 and averaveX < 336 and averageY > 228 and averageY < 252:
                            print("blå")
                        else: 
                            print("rød")
                    else:
                        print(Center[0]-averageY, Center[1]-averaveX)

                    
                except AttributeError:
                    continue
                end = perf_counter()
                print(end-start)
                cv.imshow("Result", cap)
               # cv.imshow("depth", colorize(k4aCapture.depth, (None, 5000), cv.COLORMAP_HSV))
                cv.waitKey(1)


if __name__ == '__main__':
    main()
    fisk