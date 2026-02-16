import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Blur (remove noise)
    blur = cv2.GaussianBlur(gray_frame, (5, 5), 0)

    # Binary (make it black and white to optimise calculation)
    #this fonction is the real one. The other one is for test only
    #_, binary = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, binary = cv2.threshold(blur, 80, 255, cv2.THRESH_BINARY)
    #Bitwise is added to do test with a white paper
    binary = cv2.bitwise_not(binary)

    # Contours
    contours, _ = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        print("no contours found")
        exit()

    # Largest contour (circle / semi-circle)
    cnt = max(contours, key=cv2.contourArea)

    (x, y), radius = cv2.minEnclosingCircle(cnt)

    diametre = 2 * radius

    # Center (moments)
    M = cv2.moments(cnt)
    cx = int(M["m10"] / M["m00"]) # m00 is the aire and m10 is aire*cx
    cy = int(M["m01"] / M["m00"]) # m00 is the aire and m01 is aire*cy

    # Draw center
    output = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)
    cv2.circle(output, (cx, cy), 6, (0, 0, 255), -1)
    # Draw detection circle
    cv2.circle(output,(int(x), int(y)),int(radius),(0, 255, 0),2)
    # Draw size
    cv2.putText(output,f"D={diametre}px",(int(x - radius), int(y - radius - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,0.6,(255, 0, 0),2)

    # Display
    cv2.imshow("Webcam", binary)
    cv2.imshow("Tracking", output)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()


