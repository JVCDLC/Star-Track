import numpy as np
import time
import catalog; from catalog import CATALOG
from skyfield.api import load, Star, wgs84
from skyfield.data import hipparcos

from enum import Enum

#console input without blocking the main loop
try:
    import msvcrt  # Windows-specific; provides getch/ kbhit
except ImportError:
    msvcrt = None



#load the star catalog from hipparcos
with load.open(hipparcos.URL) as f:
    stars = hipparcos.load_dataframe(f)

#-------------------------------------------------------------------------------------------#
#Global variables and initializations


# from skyfield.api import Loader
#on the pi : scp -r ~/.skyfield pi@raspberrypi.local:/home/pi/
# load = Loader('/home/pi/skyfield-data', expire=False) OFFLINE
# timeScale = load.timescale()
# currentTime = timeScale.now()
# solarSystem = load('de421.bsp')

timeScale = load.timescale()
currentTime = timeScale.now()
solarSystem = load('de421.bsp') #Planet data including the SUN and the MOON
earth = solarSystem['earth']
polaris= Star.from_dataframe(stars.loc[11767]) #refrence star Polaris
visibleCatalog = CATALOG



#-------------------------------------------------------------------------------------------#
#position of the observer
    #Sherbrooke, QC : 45.365434 , -71.939477
latitude = 45.365434 #  N 45° 21' 55.563''
longitude = -71.939477 # W 71° 56' 22.115''
observer = earth + wgs84.latlon(latitude, longitude)
temperature = 15.0 #degrees Celsius
pressure = 1005.0 #mbar
#-------------------------------------------------------------------------------------------#

celestialBody = {}



for i in CATALOG["tab-solar"]:
    celestialBody[i['name']] = solarSystem[i['id']]

for i in CATALOG["tab-stars"]:
    celestialBody[i['name']] = Star.from_dataframe(stars.loc[int(i['id'])])

for i in CATALOG["tab-deep"]:
    celestialBody[i['name']] = Star(ra_hours=i['ra_hours'], dec_degrees=i['dec_degrees'])
    

#-------------------------------------------------------------------------------------------#

class MOVEMENTSTATE(Enum):
    MISSION_TO_TARGET = 1
    TRACKING_WITH_CAMERA = 2
    MANUAL_CONTROL = 3

compensateEarthrotation: bool = True 

raToMotor = 0 # Right Ascension in degrees
decToMotor = 0 # Declination in degrees
lastRaToMotor = 0
lastDecToMotor = 0

#Variables for movement
INTERVALS = {
    'variable_update': 0.01,
    'tracking': 10,
    'manual': 0.1,
    'earth_rotation': 2
}

next_updates = {key: time.monotonic() + interval for key, interval in INTERVALS.items()}


EARTH_ROTATION_SPEED = 360/24/3600 # degrees per sec : 0.0041667 min_motor_speed : 0,00833
MANUAL_SPEED_MAX = 5*EARTH_ROTATION_SPEED # degees per second 
MAX_INPUT_FROM_JOYSTICK = [-1,1]
RA_MINIMUM = 0.00833 ###########TO CHANGE ONE NEW GEAR# minimum change in RA to send a new command to the motors in degrees
DEC_MINIMUM = 0.00833 # minimum change in Dec to send a new command to
#-----------------------------------------------------------#
#camera spec
FOCAL_LENGHT_mm = 400
CAMERA_PIXEL_SIZE_um = 5.2
PIXEL_TO_ANGLE_deg = ( (CAMERA_PIXEL_SIZE_um /1000 )/ FOCAL_LENGHT_mm) * (180/np.pi) # Angle subtended by one pixel in degrees
X_RESOLUTION = 1304
Y_RESOLUTION = 976
fieldOfViewX_deg = PIXEL_TO_ANGLE_deg * X_RESOLUTION
fieldOfViewY_deg = PIXEL_TO_ANGLE_deg * Y_RESOLUTION

camAngle = 0 # angle of the camera in degrees, 0 -> top , 90 -> right
#---------------------------END OF GLOBAL VARIABLES--------------------------------{




    #-----------------------------------------------------------#
    #                  Functions to implement
    #-----------------------------------------------------------#


#----------------------ATRONOMICAL CALCULATION FUNCTION-------------------------------------#
def updateLocation(lat,lon):
    """
        Update the observer location
        Inputs: lat : Latitude in decimal degrees
                lon : Longitude in decimal degrees
        Outputs: None
    """
    global observer
    observer = earth + wgs84.latlon(lat, lon)
    return



def isVisible(CB):
    """
        Check if a celestial body is visible from the observer location
        Inputs: CB : name of Celestial Body object from skyfield
        Outputs: True if the celestial body is above the horizon, False otherwise
    """
    global currentTime
    currentTime = timeScale.now()
    observerInfo = observer.at(currentTime).observe(CB).apparent().altaz(temperature_C=temperature, pressure_mbar=pressure)
    altitude = observerInfo[0].degrees
    if altitude > 0: # we consider the object visible if its altitude is greater than 10 degrees to avoid tracking objects too close to the horizon
        # print("the object is visible, altitude :", altitude)
        return True
    else:
        # print("the object is not visible, altitude :", altitude)
        return False

def updateVisibleCatalog():
    """
        Update the visible catalog by checking the visibility of each celestial body in the catalog
        Order the catalog by tab and by higher altitude
        Inputs: None
        Outputs: number of objet in the visibleCatalog
    """
    print('Applying visibility filter to the catalog...')
    numberOfObjets = 0
    global visibleCatalog
    visibleCatalog = {key: [] for key in CATALOG.keys()} # reset the visible catalog
    print("test", visibleCatalog)
    for key in CATALOG.keys():
        for Body in CATALOG[key]:
            try: 
                if isVisible(celestialBody[Body['name']]):
                    visibleCatalog[key].append(Body)
                    numberOfObjets += 1
                    print_HaDec(celestialBody[Body['name']], Body['name'])
                    print_AltAz(celestialBody[Body['name']], Body['name'])
            except KeyError:
                print("Error: Celestial body", Body['name'], "not found in celestialBody dictionary.")

    # Sort each category by altitude (highest first)
    for key in visibleCatalog.keys():
        visibleCatalog[key].sort(
            key=lambda body: observer.at(currentTime).observe(celestialBody[body['name']]).apparent().altaz(temperature_C=temperature, pressure_mbar=pressure)[0].degrees, 
            reverse=True
        )

    return numberOfObjets


def print_HaDec(CB,name=''):
    """
        Coordonate printing function for celestial bodies
        The coordonate are given in Right Ascension, Hour Angle and Declination
        Hour angle (HA) and Declination (dec) are directly related to RA_motor and Dec_motor
        Inputs: CB : Celestial Body object from skyfield
                name : Name of the celestial body (string)
        Outputs: None
    """
    global currentTime
    currentTime = timeScale.now()
    observerInfo = observer.at(currentTime).observe(CB).apparent().hadec() 
    ha = observerInfo[0].degrees
    dec = observerInfo[1].degrees
    print('\nCurrent position of', name)
    #print('  HA :', ha)
    if abs(ha) > 90:
        print("Warning: The object HA is greater than 90°.............................")
    #print('  Dec:', dec)
    #print('Distance:', observerInfo[2])
    
    new_ha, new_dec = new_ha_dec(ha,dec)

    print(f" new_ha : {new_ha:.3f} °")
    if abs(new_ha) > 90:
        print("Warning: The object HA is greater than 90°.............................")
    print(f"  new_dec: {new_dec:.3f} °")
    return  new_ha, new_dec 

def print_AltAz(CB,name=''):
    """
        Coordonate printing function for celestial bodies
        The coordonate are given in Altitude and Azimuth
        Inputs: CB : Celestial Body object from skyfield
                name : Name of the celestial body (string)
        Outputs: None
    """
    global currentTime
    currentTime = timeScale.now()
    observerInfo = observer.at(currentTime).observe(CB).apparent().altaz(temperature_C=temperature, pressure_mbar=pressure) 
    print('\nCurrent position of', name)
    print('  Altitude :', observerInfo[0])
    print('  Azimuth:', observerInfo[1])
    print('  Distance:', observerInfo[2])
    
    return  observerInfo[0].degrees, observerInfo[1].degrees #Alt,Az

def new_ha_dec(ha,dec):
    """
        Convert ha [-180;180] and dec [-90;90]
        to
        ha [-90;90] and dec [-180;180]
        This is usefull because the ha motor need to be limite in it's motion to protect the telescope
    """
    new_ha = (ha % 360 -90) % 180  -90 #-90 à 90 if abs(ha) > 90 -> be the opposit
    if ha == -90 : 
        new_ha = 90
    new_dec=dec 
    if abs(ha) >= 90:
        new_dec = (180- dec % 360) % 360 #if new_ha is the opposit -> new_dec need to be mirror
    return new_ha, new_dec

def print_Visible_Catalog():
  
    updateVisibleCatalog()
    print("\nVisible catalog :")
    for key in visibleCatalog.keys():
        print("\n", key, ":")
        for Body in visibleCatalog[key]:
            print("  -", Body['name'], " (", Body['icon'], ")")
            print(observer.at(currentTime).observe(celestialBody[Body['name']]).apparent().altaz(temperature_C=temperature, pressure_mbar=pressure)[0].degrees)
    print("\nTotal number of visible objects :", sum(len(visibleCatalog[key]) for key in visibleCatalog.keys()))


#--------------------------------TELESCOPE CONTROLE FUNCTION--------------------------------------------------#
def rotateXYtoHADEC(X,Y):
    """
        Rotate the movement in X,Y to HA/Dec changes based on the camera angle
        Inputs: X, Y : movement in X and Y in degrees
        Outputs: haChange, decChange : movement in HA/Dec based on the camera angle in degrees
    """
    # Rotate the movement in X,Y to HA/Dec changes based on the camera angle
    global raToMotor
    global camAngle
    X = 0
    Y = 1
    XYcam_TO_XYtelescope = np.array([[   np.cos(np.radians(camAngle)), np.sin(np.radians(camAngle))  ],
                                     [  -np.sin(np.radians(camAngle)), np.cos(np.radians(camAngle))  ]])
    XYtelescope = XYcam_TO_XYtelescope @ np.array([X, Y])

    dec = XYtelescope[X]+ XYtelescope[Y]*(-np.sign(raToMotor)) #to verify
    ha = XYtelescope[Y]*(-np.sign(raToMotor))*(-np.sign(XYtelescope[Y])) # to verify
    #same sign if y is negative and opposite sign if y is positive, to verify
  

    return ha, dec

def setupCameraAngle():
    """
        Setup the camera angle by moving the telescope in a known pattern and measuring the movement of the stars in the camera frame to calculate the angle of the camera
        Inputs: None
        Outputs: None (sets the global variable camAngle)
    """
    #not useful if we are unable to identify the movement of the stars
    global decToMotor
    global camAngle

    angleArray = []
    for i in range(10):
        CenterOfObjectXY = np.array([0,0]) # to replace with actual center of the object in the camera frame
        ra,dec = moveManually(-1**i,0) # move in X
        decToMotor += dec
        #wait for the movement to be done and the camera to be stable
        CenterOfObjectXY_afterMoveX = np.array([0,0]) # to replace with actual center of the object in the camera frame after moving in X
        VectorMove = CenterOfObjectXY_afterMoveX - CenterOfObjectXY
        try:
            VectorAngle = np.degrees(np.arctan(VectorMove[1]/ VectorMove[0]))
            angleArray.append((VectorAngle))
        except ZeroDivisionError:
            print("Error: Division by zero occurred while calculating vector angle.")
    
    meanAngle = np.mean(angleArray)
    deviation = max(np.abs(angleArray-meanAngle))
    if deviation > 5 or deviation == 0: # if the deviation is too high, we consider that the angle is not well defined
        print("Angle measurement is not reliable, please check the setup")
        return 0 # Return 0 to indicate that the angle is not reliable
    camAngle = meanAngle
    print(f"Camera angle set to {camAngle} degrees")
    pass

def recenterWithCamera(BodyCenterX, BodyCenterY):
    """
        Recenter the camera on the celestial body by calculating the movement needed in HA/Dec based on the position of the body in the camera frame
        Inputs: BodyCenterX, BodyCenterY : coordinates of the center of the celestial body in the camera frame in pixels
        Outputs: haChange, decChange : movement needed in HA/Dec to recenter the camera in degrees
    """
    CenteredZoneX = X_RESOLUTION*0.1 # Define a zone around the center of the camera where we consider the object to be centered
    CenteredZoneY = Y_RESOLUTION*0.1
    if abs(BodyCenterX-X_RESOLUTION/2) < CenteredZoneX and abs(BodyCenterY-Y_RESOLUTION/2) < CenteredZoneY:#to verify i##################
        print("Object is already centered.")
        return
    else:
        # Calculate the movement needed to recenter the object
        moveX = PIXEL_TO_ANGLE_deg*(BodyCenterX - X_RESOLUTION/2)
        moveY = PIXEL_TO_ANGLE_deg*(BodyCenterY - Y_RESOLUTION/2)
        print(f"Moving camera to recenter object: moveX={moveX}, moveY={moveY}")
        # Convert the movement in pixels to HA/Dec changes and update raToMotor and decToMotor accordingly
        haChange, decChange = rotateXYtoHADEC(moveX, moveY)
        return haChange, decChange

def moveManually(inputX, inputY):
    """
        Move the telescope manually based on joystick input
        Inputs: inputX, inputY : joystick inputs for X and Y movement (range should be defined, e.g. -1 to 1)
        Outputs: haChange, decChange : movement in HA/Dec based on joystick input in degrees
    """
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

    [ha,dec] = rotateXYtoHADEC(moveX, moveY)
    return ha, dec

def moveToTarget(stateOfTelescope = 0):
    # code for moving the camera each 2 sec
    # case 1 : go to new taget
    # case 2 : camera recentering
    # case 3 : manual control
    # case 0 : do nothing
    now = time.monotonic()
    global raToMotor, decToMotor
    targetCelestialBody = None #---francis---# replace with actual target celestial body
    match stateOfTelescope:
        case MOVEMENTSTATE.MISSION_TO_TARGET:
            #redefine raToMotor and decToMotor to move to the new target
            
            tab, bodyInfo = catalog.findObjetByNameOrId(targetCelestialBody)
            if bodyInfo is not None:
                targetCelestialBody = celestialBody[bodyInfo['name']]
                raToMotor, decToMotor = print_HaDec(targetCelestialBody)

                if tab == 'tab-solar':
                    stateOfTelescope = MOVEMENTSTATE.TRACKING_WITH_CAMERA
                else:
                    stateOfTelescope = 0

            if targetCelestialBody is None:
                print("Error: No target celestial body defined for MISSION_TO_TARGET state.")
            
            
            stateOfTelescope = 0
        case MOVEMENTSTATE.TRACKING_WITH_CAMERA:
            if now >= next_updates['tracking']:
                next_updates['tracking'] += INTERVALS['tracking']
                BodyCenterX = 0 #---JP---# replace with actual X coordinate of the celestial body in the camera frame
                BodyCenterY = 0 #---JP---# replace with actual Y coordinate of the celestial body in the camera frame
                haChange, decChange = recenterWithCamera(BodyCenterX, BodyCenterY) * INTERVALS['tracking'] # multiply by the time interval to get the actual movement in degrees
                raToMotor += haChange
                decToMotor += decChange
        case MOVEMENTSTATE.MANUAL_CONTROL:
            if now >= next_updates['manual']:
                next_updates['manual'] += INTERVALS['manual']
                # get input from joystick
                inputX = 0 #---francis---# replace with actual input
                inputY = 0 #---francis---# replace with actual input
                if abs(inputX) > 0.1 and abs(inputY) > 0.1: # out of the Dead zone for joystick input
                    manualInput = moveManually(inputX, inputY) * INTERVALS['manual'] # multiply by the time interval to get the actual movement in degrees
                    raToMotor += manualInput[0]
                    decToMotor += manualInput[1]
                
            tab, bodyInfo = catalog.findObjetByNameOrId(targetCelestialBody)
            if bodyInfo is not None:
                targetCelestialBody = celestialBody[bodyInfo['name']]
                

                if tab == 'tab-solar':
                    stateOfTelescope = MOVEMENTSTATE.TRACKING_WITH_CAMERA
                else:
                    stateOfTelescope = 0

            if targetCelestialBody is None:
                print("Error: No target celestial body defined for MISSION_TO_TARGET state.")
        case _:
            pass

    # adding or ignoring the rotation of the earth in the calculations with trackingTarget
    if compensateEarthrotation:
        if now >= next_updates['earth_rotation']:
            next_updates['earth_rotation'] += INTERVALS['earth_rotation']
            raToMotor += EARTH_ROTATION_SPEED * INTERVALS['earth_rotation'] # add earth rotation to ra


    print(f"Moving to RA: {raToMotor:.4f}°, Dec: {decToMotor:.4f}°")
    return raToMotor, decToMotor

def main():
    
    #setup
    print('setup...')
    stateOfTelescope = 0
    print('Setting up camera angle...')
    setupCameraAngle()
    print('Camera angle setup complete.')

    print('Main loop...')

    
    i=0######
    
    while True:
        # example operation prior to waiting, could call moveToTarget()
        
        now = time.monotonic()
        
        if now >= next_updates['variable_update']:
            next_updates['variable_update'] += INTERVALS['variable_update']

            #----------------------------UPDATE camera VARIABLES----------------------------#

            #----------------------------UPDATE movement VARIABLES----------------------------#
            """
            if joystick_input_available: #---francis---# replace with actual check for joystick input availability
                inputX = 0 #---francis---# replace with actual input
                inputY = 0 #---francis---# replace with actual input
                if abs(inputX) > 0.1 and abs(inputY) > 0.1: # out of the Dead zone for joystick input
                    stateOfTelescope = MOVEMENTSTATE.MANUAL_CONTROL
            if new_target_available: #---francis---# replace with actual check for new target availability
                stateOfTelescope = MOVEMENTSTATE.MISSION_TO_TARGET

            if tracking_with_camera==False and stateOfTelescope == MOVEMENTSTATE.TRACKING_WITH_CAMERA: #---francis---# replace with actual check for tracking with camera mode
                stateOfTelescope = 0
            if compensateEarthrotation == False:
                #that's it, lol
            """
            #----------------------------UPDATE MOVEMENT-----------------------------#
            #calculate ra and dec
            moveToTarget(stateOfTelescope)

            #verify ra and dec
            if abs(raToMotor) > 100 or abs(decToMotor) > 160:
                print("Warning: Movement exceeds telescope limits.")
                #exit
                break
            """
            if limit_switch_triggered(): #---simon---# replace with actual check for limit switch triggered
                print("Warning: Limit switch triggered. Movement stopped to prevent damage.")
                #exit
                break
            """
            #send info to jp if > min ra or > min dec avaliable
            if abs(raToMotor - lastRaToMotor) > RA_MINIMUM:
                lastRaToMotor = raToMotor
                #send raToMotor to JP for motor control
                pass
            
            if abs(decToMotor - lastDecToMotor) > DEC_MINIMUM:
                lastDecToMotor = decToMotor
                #send decToMotor to JP for motor control
                pass
            """
               hello jp
            """
            
            #------------------------------EXIT CONDITION------------------------------#
            #escape key to exit the loop
            key = None
            if msvcrt:
                if msvcrt.kbhit():
                    ch = msvcrt.getch()
                    try:
                        key = ord(ch)
                    except TypeError:
                        # Python3 returns bytes; take first byte
                        key = ch[0]
            
            if key in (27, ord('q')):  # ESC or 'q'
                break
            #------------------------------------test-----------------------------------#
        if now >= next_updates['earth_rotation']:
            print("Moving to target...")
            next_updates['earth_rotation'] += INTERVALS['earth_rotation']
            print(i)
            i+=1
            print_HaDec(celestialBody['JUPITER'], 'Jupiter') # to verify the movement of the telescope
            
            jupi = solarSystem['jupiter barycenter']
            print_HaDec(jupi, 'Jupiter')

    time.sleep(0.001)  # not overload CPU with a tight loop
    pass
    

# -------------------------------------------------------
# MAIN
# -------------------------------------------------------
if __name__ == "__main__":
    main()
    pass

        

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