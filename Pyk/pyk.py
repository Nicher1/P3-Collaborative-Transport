import cv2 as cv
import numpy as np
from pyk4a import PyK4A, connected_device_count
from pyk4a import Config
from time import perf_counter
import mediapipe as mp
import helpers
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
        camera_fps=pyk.FPS.FPS_30,
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

    BG_COLOR = (192,192,192) # gray

    while True:
        k4aCapture = k4a.get_capture()
        if np.any(k4aCapture.color):
            tempCap = k4aCapture.color
            
            cap = tempCap[:, :, 0:3]

            print("cap", cap)

            cv.imshow("cap", cap)
            cv.waitKey(0)

            with mp_hands.Hands(
                static_image_mode = True,
                max_num_hands=2,
                min_detection_confidence=0.5) as hands:
                with mp_pose.Pose(
                model_complexity=0,
                static_image_mode=True,
                enable_segmentation=True,
                min_detection_confidence=0.5) as pose:

                    for idx, file in enumerate(cap):
                        image_height, image_width, _ = cap.shape
                        resultPose= pose.process(cap)
                        resultHands = hands.process(cap)

                        if not resultPose.pose_landmarks:
                            continue
                        
                        annotated_image = cap.copy()

                        condition = np.stack((resultPose.segmentation_mask,)*3, axis=1) > 0.1
                        bg_image = np.zeros(cap.shape,dtype=np.uint8)
                        bg_image[:]=BG_COLOR

                        annotated_image = np.where(condition, annotated_image, bg_image)

                        mp_drawing.draw_landmarks(
                            annotated_image,
                            resultPose.pose_landmarks,
                            mp_pose.POSE_CONNECTIONS,
                            landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

                        LandResults = []
                        Idx = []
                        for id, lm in enumerate(resultPose.pose_landmarks.landmark):
                            LandResults.append([])
                            x= int(lm.x*cap.shape[1])
                            y= int(lm.y*cap.shape[0])
                            LandResults[id].append(y)
                            LandResults[id].append(x)
                            Idx.append(id)

                        Center = [image.shape[0]/2,image.shape[1]/2]
                        averaveX = int(np.rint((LandResults[15][1]+LandResults[16][1])/2))
                        averageY = int(np.rint((LandResults[15][0]+LandResults[16][0])/2))

                        if averaveX > 288 and averaveX < 352 and averageY > 216 and averageY < 264:
                            if averaveX > 304 and averaveX < 336 and averageY > 228 and averageY < 252:
                                print("blå")
                            else: 
                                print("rød")
                        else:
                            print(Center[0]-averageY, Center[1]-averaveX)
                            
                    cv.imshow("image", annotated_image)
                    cv.waitKey(0)


if __name__ == '__main__':
    main()