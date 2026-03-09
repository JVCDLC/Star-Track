#-----------------------------------------------------#
#   Star-Tack project
#   Movement Module
#  
#   Entry point for movement functionalities
#   Created by Antoine Desgranges
#   
#-----------------------------------------------------#

# Import necessary modules

#from movement_controller import MovementController
import time
from skyfield.api import load, Star, wgs84
from skyfield.data import hipparcos
from catalog import CATALOG



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
#Functions
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



#-------------------------------------------------------------------------------------------#
# MAIN
# -------------------------------------------------------
def print_Visible_Catalog():
  
    updateVisibleCatalog()
    print("\nVisible catalog :")
    for key in visibleCatalog.keys():
        print("\n", key, ":")
        for Body in visibleCatalog[key]:
            print("  -", Body['name'], " (", Body['icon'], ")")
            print(observer.at(currentTime).observe(celestialBody[Body['name']]).apparent().altaz(temperature_C=temperature, pressure_mbar=pressure)[0].degrees)
    print("\nTotal number of visible objects :", sum(len(visibleCatalog[key]) for key in visibleCatalog.keys()))
# #-------------------------------------------------------------------------------------------#
if __name__ == "__main__":
    print_HaDec(celestialBody['JUPITER'], 'Jupiter')
    time.sleep(1)
    currentTime = timeScale.now()
    print_HaDec(celestialBody['JUPITER'], 'Jupiter')
    pass
#-------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------------#