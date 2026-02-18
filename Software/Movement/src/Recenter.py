import numpy as np
from . import astrononic_function as astro_fct
#import ESP-32 code.
#-----------------------------------------------------------#
#Variables for recentering the camera
camAngle = 0 # Angle actuel de la caméra en degrés 0 -> haut
currentTime = astro_fct.currentTime
observer = astro_fct.observer
pixelToDegree = 0.1 # Conversion de pixels à degrés, à ajuster en fonction de la caméra et de l'objectif utilisés
earthRotationSpeed = 360/24/3600 # degrees per sec : 0.0041667 min_motor_speed : 0,00833
manualSpeedMax = 10*earthRotationSpeed # degees per second 
maxInputFormJoyStick = [-2,2]
#-----------------------------------------------------------#
#camera spec
focalLength_mm = 400
cameraPixelSize_um = 5.2
PixelAngle_deg = ( (cameraPixelSize_um /1000 )/ focalLength_mm) * (180/np.pi) # Angle subtended by one pixel in degrees
xResolution = 1304
yResolution = 976
fieldOfViewX_deg = PixelAngle_deg * xResolution
fieldOfViewY_deg = PixelAngle_deg * yResolution



#-----------------------------------------------------------#

rotationXYtoALTAZ = np.array([[np.sin(np.radians(camAngle)),np.cos(np.radians(camAngle))],
                    [np.cos(np.radians(camAngle)),-np.sin(np.radians(camAngle))  ]])
X = 0
Y = 1

[alt,az]= rotationXYtoALTAZ @ np.array([X,Y]) # Vecteur pointant vers la droite de la caméra (1,0) transformé en coordonnées Alt/Az
print(f"Vecteur pointant vers la droite de la caméra en coordonnées Alt/Az : {alt:.2f}° Alt, {az:.2f}° Az")


#-----------------------------------------------------------#
#Functions to implement

def trackTarget():
 
    timeUpdate = 1 # seconds
    lastTime = astro_fct.currentTime
    if currentTime - lastTime  > timeUpdate: # Update every second
        lastTime = currentTime
       #send_request_motor(ser, motor: Motor, earthRotationSpeed/timeUpdate)
        


    pass


def setupCameraAngle():
    # Move in X,Y and compare with X,Y mesure from the camera
    # 
    #define center x,y of an objet
    #move in X,Y and compare with X,Y mesure from the camera

    pass

def recenterCamera():
    # Code pour recentrer la caméra sur la cible spécifiée par targetAlt et targetAz
    # Cela pourrait impliquer de calculer les mouvements nécessaires en fonction de l'angle actuel de la caméra
    pass

def moveManually(inputX, inputY):
    # Code pour permettre un contrôle manuel de la caméra, par exemple via une interface utilisateur ou des commandes clavier
    if inputX > maxInputFormJoyStick[1]:
        inputX = maxInputFormJoyStick[1]
    elif inputX < maxInputFormJoyStick[0]:
        inputX = maxInputFormJoyStick[0]
    if inputY > maxInputFormJoyStick[1]:
        inputY = maxInputFormJoyStick[1]
    elif inputY < maxInputFormJoyStick[0]:
        inputY = maxInputFormJoyStick[0]

    # Convert the joystick inputs to degrees of movement
    moveX = (inputX / (maxInputFormJoyStick[1] - maxInputFormJoyStick[0])) * manualSpeedMax
    moveY = (inputY / (maxInputFormJoyStick[1] - maxInputFormJoyStick[0])) * manualSpeedMax

    [alt, az] = astro_fct.ALTAZtoHADEC(rotationXYtoALTAZ @ np.array([moveX, moveY])) # Convert joystick movement to Alt/Az changes

    print(f"Camera moved manually: {moveX:.2f}° X, {moveY:.2f}° Y. New camera angle: {camAngle:.2f}°")


# -------------------------------------------------------
# MAIN
# -------------------------------------------------------
if __name__ == "__main__":
    pass