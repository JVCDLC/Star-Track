import os
import zwoasi as asi
import cv2
import numpy as np
import platform


def load_zwo_sdk():
    os_name = platform.system()

    if os_name == "Windows":
        sdk_path = r"C:\Program Files\ASIStudio\ASICamera2.dll"
    elif os_name == "Linux":
        sdk_path = "/usr/local/lib/libASICamera2.so"## Adjust if necessary
    else:
        raise Exception("OS non supporté")

    if not os.path.exists(sdk_path):
        raise FileNotFoundError(f"SDK introuvable : {sdk_path}")

    asi.init(sdk_path)
    print("SDK chargé :", sdk_path)

# Initialize SDK (Windows)
load_zwo_sdk()

# Open camera
camera = asi.Camera(0)
camera.set_control_value(asi.ASI_GAIN, 0)
camera.set_control_value(asi.ASI_EXPOSURE, 10000)  # microseconds
#for planetary imaging
camera.set_image_type(asi.ASI_IMG_RAW8)
#for deep sky imaging
#camera.set_image_type(asi.ASI_IMG_RAW16)

camera.start_video_capture()

# Get resolution
width, height = camera.get_roi_format()[:2]

# Video writer
writer = cv2.VideoWriter(
    "output.avi",
    cv2.VideoWriter_fourcc(*"MJPG"),
    30, # FPS
    (width, height), #Size
    False #color
)

while True:
    frame = camera.capture_video_frame()
    frame = np.frombuffer(frame, dtype=np.uint8).reshape(height, width)

    cv2.imshow("Live", frame)
    writer.write(frame)

    if cv2.waitKey(1) == 27:  # ESC
        break

writer.release()
camera.stop_video_capture()
camera.close()
cv2.destroyAllWindows()