import cv2
import numpy as np

img = cv2.imread("image_focus_moon.png", cv2.IMREAD_GRAYSCALE)

# Resize
scale = 0.3
resized = cv2.resize(
    img,
    (int(img.shape[1]*scale), int(img.shape[0]*scale)),
    interpolation=cv2.INTER_AREA
)

# Blur (remove noise)
blur = cv2.GaussianBlur(resized, (5, 5), 0)

# Binary
_, binary = cv2.threshold(
    blur, 0, 255,
    cv2.THRESH_BINARY + cv2.THRESH_OTSU
)

# Invert if needed (objet noir sur fond blanc)
# binary = cv2.bitwise_not(binary)

# Contours
contours, _ = cv2.findContours(
    binary,
    cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE
)

if len(contours) == 0:
    print("Aucun contour détecté")
    exit()

# Largest contour (circle / semi-circle)
cnt = max(contours, key=cv2.contourArea)

# Center (moments)
M = cv2.moments(cnt)
cx = int(M["m10"] / M["m00"])
cy = int(M["m01"] / M["m00"])

# Draw center
output = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
cv2.circle(output, (cx, cy), 6, (0, 0, 255), -1)

# Display
cv2.imshow("Binary", binary)
cv2.imshow("Detected center", output)
cv2.waitKey(0)
cv2.destroyAllWindows()
