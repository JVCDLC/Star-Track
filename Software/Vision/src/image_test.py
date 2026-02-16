import cv2
import numpy as np

def analyse(image_path):

    image = cv2.imread(image_path)
    if image is None:
        print("Erreur image")
        return None

    # 🔥 1) Downscale pour accélérer
    scale = 0.5
    small = cv2.resize(image, None, fx=scale, fy=scale)

    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

    # 🔥 2) Blur léger (pas trop fort pour garder le bord net)
    blurred = cv2.GaussianBlur(gray, (5,5), 1.5)

    # 🔥 3) Threshold automatique (Otsu)
    _, thresh = cv2.threshold(
        blurred, 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    # 🔥 4) Nettoyage morphologique
    kernel = np.ones((5,5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # 🔥 5) Trouver contours
    contours, _ = cv2.findContours(
        thresh,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        print("Pas de contour")
        return None

    # 🔥 6) Prendre le plus grand contour
    largest = max(contours, key=cv2.contourArea)

    # 🔥 7) Cercle minimum englobant
    (x, y), radius = cv2.minEnclosingCircle(largest)

    # 🔥 Remettre à l'échelle originale
    x = int(x / scale)
    y = int(y / scale)
    radius = int(radius / scale)

    centre = (x, y)
    diametre = 2 * radius

    print("Centre:", centre)
    print("Diametre:", diametre)

    # 🔥 Affichage debug
    output = image.copy()
    cv2.circle(output, centre, radius, (0,255,0), 2)
    cv2.circle(output, centre, 3, (0,0,255), -1)

    cv2.imshow("Detection", output)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return centre, diametre

# Exemple d'appel
if __name__ == "__main__":
    analyse("test_data/lune_2fev/2026-02-03-0416_6-CapObj_0001.PNG")
