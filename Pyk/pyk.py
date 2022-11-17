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

    BG_COLOR = (192,192,192) # gray

    while True:
        k4aCapture = k4a.get_capture()
        if np.any(k4aCapture.depth):
            cv.imshow("k4a", helpers.color(k4aCapture.depth, (None, 5000), cv.COLORMAP_HSV))
            cv.waitKey(0)
            cap = cv.imread(k4aCapture.color)

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

                        if not resultHands.pose_landmarks:
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
                    cv.imshow("image", annotated_image)
                    cv.waitKey(0)

if __name__ == '__main__':
    main()