import cv2 as cv
import numpy as np
from pyk4a import PyK4A, connected_device_count
from pyk4a import Config
from time import perf_counter
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
mp_holistic = mp.solutions.holistic


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
    cap = k4a.get_capture()
    print(cap)

if __name__ == '__main__':
    main()