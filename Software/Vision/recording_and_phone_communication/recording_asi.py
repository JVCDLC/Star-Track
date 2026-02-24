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
camera.set_control_value(asi.ASI_GAIN, 50)
camera.set_control_value(asi.ASI_EXPOSURE, 1000000)  # microseconds
#for planetary imaging
#camera.set_image_type(asi.ASI_IMG_RAW8)
#for deep sky imaging
camera.set_image_type(asi.ASI_IMG_RAW16) #erreur: vValueError: cannot reshape array of size 2545408 into shape (976,1304),   
#  frame = np.frombuffer(frame, dtype=np.uint8).reshape(height, width)
#           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#ValueError: cannot reshape array of size 2545408 into shape (976,1304)

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
    frame = np.frombuffer(frame, dtype=np.uint16).reshape(height, width)

    cv2.imshow("Live", frame)
    writer.write(frame)

    # if cv2.getWindowProperty("Live", cv2.WND_PROP_VISIBLE) < 100:
    #     break

    if cv2.waitKey(1) == 27 or cv2.waitKey(1) == ord('q')  : #esc key  q 
        break

   
writer.release()
camera.stop_video_capture()
camera.close()
cv2.destroyAllWindows()