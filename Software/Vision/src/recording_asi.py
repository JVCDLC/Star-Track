import os
import zwoasi as asi
import cv2
import numpy as np


def find_asicam_dll():
    """Locate ASICamera2.dll from environment or common locations on Windows.

    Set environment variable `ASICAM_DLL` (or `ASI_DLL_PATH`) to the full DLL path
    to override automatic discovery.
    """
    env = os.environ.get("ASICAM_DLL") or os.environ.get("ASI_DLL_PATH")
    candidates = []
    if env:
        candidates.append(env)

    candidates.extend([
        os.path.join(os.getcwd(), "ASICamera2.dll"),
        r"C:\Program Files\ZWO\ASI SDK\ASICamera2.dll",
        r"C:\Program Files\ZWO\ASICamera2\ASICamera2.dll",
        r"C:\Windows\System32\ASICamera2.dll",
        r"C:\Windows\SysWOW64\ASICamera2.dll",
    ])

    for p in candidates:
        try:
            if p and os.path.exists(p):
                return p
        except Exception:
            continue

    raise FileNotFoundError(
        "Could not find ASICamera2.dll. Tried: {}. "
        "Install ZWO ASI SDK and set environment var `ASICAM_DLL` to the DLL path.".format(
            ", ".join(candidates)
        )
    )


# Initialize SDK (Windows)
asi.init(find_asicam_dll())

# Open camera
camera = asi.Camera(0)
camera.set_control_value(asi.ASI_GAIN, 100)
camera.set_control_value(asi.ASI_EXPOSURE, 10000)  # microseconds
camera.set_image_type(asi.ASI_IMG_RAW8)

camera.start_video_capture()

# Get resolution
width, height = camera.get_roi_format()[:2]

# Video writer
writer = cv2.VideoWriter(
    "output.avi",
    cv2.VideoWriter_fourcc(*"MJPG"),
    30,
    (width, height),
    False
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