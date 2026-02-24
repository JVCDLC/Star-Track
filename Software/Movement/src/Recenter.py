import cv2
import numpy as np
import time
import astrononic_function as astro_fct
from skyfield.api import load
from enum import Enum
#import ESP-32 code.
#-----------------------------------------------------------#
#Variables for recentering the camera
camAngle = -90 # Angle actuel de la caméra en degrés 0 -> haut , 90 -> droite 
timeScale = load.timescale()
lauchTime = timeScale.now()
lastTime = lauchTime

#currentTime = astro_fct.currentTime
observer = astro_fct.observer
EARTH_ROTATION_SPEED = 360/24/3600 # degrees per sec : 0.0041667 min_motor_speed : 0,00833
MANUAL_SPEED_MAX = 5*EARTH_ROTATION_SPEED # degees per second 
MAX_INPUT_FROM_JOYSTICK = [-1,1]
#-----------------------------------------------------------#
#camera spec
FOCAL_LENGHT_mm = 400
CAMERA_PIXEL_SIZE_um = 5.2
PIXEL_TO_ANGLE_deg = ( (CAMERA_PIXEL_SIZE_um /1000 )/ FOCAL_LENGHT_mm) * (180/np.pi) # Angle subtended by one pixel in degrees
X_RESOLUTION = 1304
Y_RESOLUTION = 976
fieldOfViewX_deg = PIXEL_TO_ANGLE_deg * X_RESOLUTION
fieldOfViewY_deg = PIXEL_TO_ANGLE_deg * Y_RESOLUTION



class MOVEMENTSTATE(Enum):
    MISSION_TO_TARGET = 1
    TRACKING_WITH_CAMERA = 2
    MANUAL_CONTROL = 3

compensateEarthrotation: bool = True 

raToMotor = 0 # Right Ascension in degrees
decToMotor = 0 # Declination in degrees

#-----------------------------------------------------------#

XY_TO_ALTAZ = np.array([[-np.sin(np.radians(camAngle)),np.cos(np.radians(camAngle))],
                    [np.cos(np.radians(camAngle)),np.sin(np.radians(camAngle))  ]])
X = 1
Y = 2

[alt,az]= XY_TO_ALTAZ @ np.array([X,Y]) # Vecteur pointant vers la droite de la caméra (1,0) transformé en coordonnées Alt/Az


#-----------------------------------------------------------#
#Functions to implement

def raTracking():
 
    timeUpdate = 2 # seconds
    global lastTime
    currentTime = timeScale.now()
    timePassed = (currentTime.tt - lastTime.tt)*86400
    #print("test",(timeScale.now().tt - currentTime.tt) * 86400)
    if  timePassed > timeUpdate: # Update every 2 seconds
        raChange = timePassed * EARTH_ROTATION_SPEED
        
        print(f"Time passed: {timePassed:.2f} seconds, RA change: {raChange:.2f} degrees")
        lastTime = currentTime
    try:
        return raChange
    except UnboundLocalError:
        return 0
    

def rotateXYtoAltAz(X,Y):
    [alt, az] = PIXEL_TO_ANGLE_deg * XY_TO_ALTAZ @ np.array([X, Y]) # Convert joystick movement to Alt/Az changes
    print(f"Vecteur pointant vers la droite de la caméra en coordonnées Alt/Az : {alt:.2f}° Alt, {az:.2f}° Az")
    return alt, az

def setupCameraAngle():
    # Move in X,Y and compare with X,Y mesure from the camera
    # 
    #define center x,y of an objet
    #move in X,Y and compare with X,Y mesure from the camera

    pass

def recenterWithCamera():
    # Code pour recentrer la caméra sur la cible spécifiée par targetAlt et targetAz
    # Cela pourrait impliquer de calculer les mouvements nécessaires en fonction de l'angle actuel de la caméra
    pass

def moveManually(inputX, inputY):
    # Security check for joystick input
    if inputX > MAX_INPUT_FROM_JOYSTICK[1]:
        inputX = MAX_INPUT_FROM_JOYSTICK[1]
    elif inputX < MAX_INPUT_FROM_JOYSTICK[0]:
        inputX = MAX_INPUT_FROM_JOYSTICK[0]
    if inputY > MAX_INPUT_FROM_JOYSTICK[1]:
        inputY = MAX_INPUT_FROM_JOYSTICK[1]
    elif inputY < MAX_INPUT_FROM_JOYSTICK[0]:
        inputY = MAX_INPUT_FROM_JOYSTICK[0]

    # Convert the joystick inputs to degrees of movement
    moveX = (inputX / (MAX_INPUT_FROM_JOYSTICK[1] - MAX_INPUT_FROM_JOYSTICK[0])) * MANUAL_SPEED_MAX
    moveY = (inputY / (MAX_INPUT_FROM_JOYSTICK[1] - MAX_INPUT_FROM_JOYSTICK[0])) * MANUAL_SPEED_MAX

    [alt, az] = rotateXYtoAltAz(moveX, moveY)
    [ha,dec] = astro_fct.ALTAZtoHADEC(alt, az) # Convert joystick movement to Alt/Az changes
    return ha, dec

def moveToTarget(stateOfTelescope = 0):
    # code for moving the camera each 2 sec
    # case 1 : go to new taget
    # case 2 : camera recentering
    # case 3 : manual control
    # case 4 : do nothing

    global raToMotor, decToMotor
    
    match stateOfTelescope:
        case MOVEMENTSTATE.MISSION_TO_TARGET:
            #redefine raToMotor and decToMotor to move to the new target
            targetCelestialBody = None #---francis---# replace with actual target celestial body
            raToMotor, decToMotor = astro_fct.print_HaDec(targetCelestialBody)
        case MOVEMENTSTATE.TRACKING_WITH_CAMERA:
            recenterWithCamera()
        case MOVEMENTSTATE.MANUAL_CONTROL:
            # get input from joystick
            inputX = 0 #---francis---# replace with actual input
            inputY = 0 #---francis---# replace with actual input
            manualInput = moveManually(inputX, inputY)
            raToMotor += manualInput[0]
            decToMotor += manualInput[1]
        case _:
            pass
    if compensateEarthrotation:
        raToMotor += raTracking() # add earth rotation to ra
    print(f"Moving to RA: {raToMotor:.10f}°, Dec: {decToMotor:.10f}°")
    # adding or ignoring the rotation of the earth in the calculations with trackingTarget
    
    
    return raToMotor, decToMotor


# -------------------------------------------------------
# MAIN
# -------------------------------------------------------
if __name__ == "__main__":
    
    while True:
        moveToTarget()
        
       
        
        time.sleep(1) # Sleep for a short time to prevent excessive CPU usage
        if cv2.waitKey(1) == 27:  # ESC
            break
        

# -------------------------------------------------------#
#setup order
    # 1. jp with arduino code for motor
    # 1. francis setup camera and server ()

    #2. simon with limit swich setup the motors
    #2. francis and antoine update the catalog with 

    #3. emile setup the focus looking at the night sky

    #(4.) antoine setup the angle of the camera

    #(5-.) rotate the telescope to the side
    #5. manually move the telescope to the polar star
    #(5+.) replace the telescope on [0,0]

    #6. start compensate earth rotataion

#loop:
    #verify if the angle are more the the max

        #looking for change in the parameter
            #exposition time, gain, type of picture (planetary, deep sky, auto) -> change the camera setting
            #compensate eart rotation on/off -> change the compensateEarthrotation variable
        #looking for command for the type of movement
            #MISSION_TO_TARGET

            #TRACKING_WITH_CAMERA

            #MANUAL_CONTROL
        
        #looking for change in the target, if change -> MISSION_TO_TARGET
        #if TRACKING_WITH_CAMERA -> recenterWithCamera()
        #looking for joystick input, if input -> MANUAL_CONTROL
        
        #looking if taking picture
            #if planetary picture -> take picture with planetary setting
            #if deep sky picture -> take picture with deep sky setting

            #store picture with name and date in the adequate folder
