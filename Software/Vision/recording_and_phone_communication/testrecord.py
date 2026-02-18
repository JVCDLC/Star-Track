
import os
import time
import platform
import datetime
import numpy as np
import cv2
import zwoasi
from scipy.optimize import curve_fit

# -------------------------------------------------------
# CONFIGURATION GÉNÉRALE
# -------------------------------------------------------
SESSION_NAME = "Session_Astro"
MODE = "planetary"   # "planetary" ou "deep"
GAIN = 200
EXPOSURE_US = 8000 if MODE == "planetary" else 30000000
NB_FRAMES = 200 if MODE == "planetary" else 10

# -------------------------------------------------------
# CHARGEMENT AUTOMATIQUE DU SDK SELON L'OS
# -------------------------------------------------------
def load_zwo_sdk():
    os_name = platform.system()

    if os_name == "Windows":
        sdk_path = "C:\\Program Files\\ZWO\\ASI SDK\\ASICamera2.dll"
    elif os_name == "Linux":
        sdk_path = "/usr/local/lib/libASICamera2.so"
    else:
        raise Exception("OS non supporté :", os_name)

    if not os.path.exists(sdk_path):
        raise FileNotFoundError(f"SDK introuvable : {sdk_path}")

    zwoasi.init(sdk_path)
    print("SDK chargé :", sdk_path)

# -------------------------------------------------------
# INITIALISATION DE LA CAMÉRA
# -------------------------------------------------------
def init_camera():
    cameras = zwoasi.list_cameras()
    if not cameras:
        raise Exception("Aucune caméra ASI détectée")

    cam = zwoasi.Camera(cameras[0])
    info = cam.get_camera_property()

    cam.disable_dark_subtract()
    cam.set_control_value(zwoasi.ASI_GAIN, GAIN)
    cam.set_control_value(zwoasi.ASI_EXPOSURE, EXPOSURE_US)
    cam.set_control_value(zwoasi.ASI_BANDWIDTHOVERLOAD, 40)

    if MODE == "planetary":
        cam.set_image_type(zwoasi.ASI_IMG_RAW8)
    else:
        cam.set_image_type(zwoasi.ASI_IMG_RAW16)

    return cam, info
# -------------------------------------------------------
# AUTO-EXPOSITION INTELLIGENTE # à adapter a planitery ou deep sky
# -------------------------------------------------------
def auto_exposure(camera, info, target_level=0.6, max_iter=10, mode="deep"):
    exposure = camera.get_control_value(zwoasi.ASI_EXPOSURE)[0]
 

    for _ in range(max_iter):
        camera.set_control_value(zwoasi.ASI_EXPOSURE, exposure)
        camera.start_exposure()

        while camera.get_exposure_status() == zwoasi.ASI_EXP_WORKING:
            pass

        frame = camera.get_exposure_data()
        img = np.frombuffer(frame, dtype=np.uint16)
        img = img.reshape(info['MaxHeight'], info['MaxWidth'])

        max_val = np.percentile(img, 99.5)
        target = 65535 * target_level

        ratio = target / max_val
        exposure = int(exposure * ratio)
        exposure = max(1000, min(exposure, 60000000))

        print(f"[AutoExpo] max={max_val:.0f}, new expo={exposure/1e6:.2f}s")

        if abs(max_val - target) < 2000:
            break

    camera.set_control_value(zwoasi.ASI_EXPOSURE, exposure)
    return exposure

# -------------------------------------------------------
# AUTOFOCUS BASÉ SUR LA FWHM
# -------------------------------------------------------
def gaussian(x, a, x0, sigma, c):
    return a * np.exp(-(x - x0)**2 / (2 * sigma**2)) + c

def compute_fwhm(profile):
    x = np.arange(len(profile))
    y = profile

    x0 = np.argmax(y)
    a = y[x0]
    sigma = 2
    c = np.min(y)

    try:
        popt, _ = curve_fit(gaussian, x, y, p0=[a, x0, sigma, c])
        sigma = popt[2]
        return 2.355 * sigma
    except:
        return None

def autofocus(camera, info, focuser, steps=20, step_size=20):
    best_fwhm = 9999
    best_pos = focuser.get_position()

    for _ in range(steps):
        focuser.move_relative(step_size)

        camera.start_exposure()
        while camera.get_exposure_status() == zwoasi.ASI_EXP_WORKING:
            pass

        frame = camera.get_exposure_data()
        img = np.frombuffer(frame, dtype=np.uint16)
        img = img.reshape(info['MaxHeight'], info['MaxWidth'])

        y, x = np.unravel_index(np.argmax(img), img.shape)
        profile = img[y, max(0, x-20):min(x+20, info['MaxWidth'])]

        fwhm = compute_fwhm(profile)
        print(f"[AF] pos={focuser.get_position()} → FWHM={fwhm}")

        if fwhm and fwhm < best_fwhm:
            best_fwhm = fwhm
            best_pos = focuser.get_position()

    focuser.move_absolute(best_pos)
    print("Autofocus terminé → meilleure FWHM :", best_fwhm)

# -------------------------------------------------------
# LIVE VIEW AVEC OPENCV
# -------------------------------------------------------
def live_view(cam, info):
    print("Live view : appuie sur 'q' pour quitter")

    cam.start_video_capture()

    while True:
        frame = cam.capture_video_frame()

        if MODE == "planetary":
            img = np.frombuffer(frame, dtype=np.uint8)
        else:
            img = np.frombuffer(frame, dtype=np.uint16)
            img = (img / 256).astype(np.uint8)

        img = img.reshape(info['MaxHeight'], info['MaxWidth'])

        cv2.imshow("ASI Live View", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cam.stop_video_capture()
    cv2.destroyAllWindows()

# -------------------------------------------------------
# ACQUISITION PLANÉTAIRE
# -------------------------------------------------------
def capture_planetary(cam, info, save_dir):
    cam.start_video_capture()

    for i in range(NB_FRAMES):
        frame = cam.capture_video_frame()
        img = np.frombuffer(frame, dtype=np.uint8)
        img = img.reshape(info['MaxHeight'], info['MaxWidth'])
        cv2.imwrite(os.path.join(save_dir, f"frame_{i:04d}.png"), img)

    cam.stop_video_capture()

# -------------------------------------------------------
# ACQUISITION DEEP SKY
# -------------------------------------------------------
def capture_deep(cam, info, save_dir):
    for i in range(NB_FRAMES):
        print(f"Pose {i+1}/{NB_FRAMES}")

        cam.start_exposure()
        while cam.get_exposure_status() == zwoasi.ASI_EXP_WORKING:
            pass

        frame = cam.get_exposure_data()
        img = np.frombuffer(frame, dtype=np.uint16)
        img = img.reshape(info['MaxHeight'], info['MaxWidth'])
        cv2.imwrite(os.path.join(save_dir, f"deep_{i:03d}.png"), img)

# -------------------------------------------------------
# MAIN
# -------------------------------------------------------
if __name__ == "__main__":
    load_zwo_sdk()
    cam, info = init_camera()

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%M")
    save_dir = os.path.join("sessions", f"{SESSION_NAME}_{timestamp}")
    os.makedirs(save_dir, exist_ok=True)

    print("Ajustement automatique de l'exposition…")
    auto_exposure(cam, info)

    live_view(cam, info)

    if MODE == "planetary":
        capture_planetary(cam, info, save_dir)
    else:
        capture_deep(cam, info, save_dir)

    cam.close()
    print("Session terminée.")