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
from datetime import timedelta
import time
from skyfield.api import load, Star, wgs84, Topos
from skyfield.data import hipparcos
from catalog import CATALOG
from skyfield import almanac
from pytz import timezone
from timezonefinder import TimezoneFinder



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
observer = earth + Topos(latitude_degrees=latitude, longitude_degrees=longitude)
temperature = 15.0 #degrees Celsius
pressure = 1005.0 #mbar
timeZone = 'America/Toronto'
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
    global timeZone
    observer = earth + wgs84.latlon(lat, lon)
    timeZone = str(timezone_from_position(lat,lon))
    return

def timezone_from_position(lat, lon):
    tf = TimezoneFinder()
    tz_name = tf.timezone_at(lat=lat, lng=lon)
    if tz_name is None:
        return None
    return timezone(tz_name)

# def isVisible(CB):
#     """
#         Check if a celestial body is visible from the observer location
#         Inputs: CB : name of Celestial Body object from skyfield
#         Outputs: True if the celestial body is above the horizon, False otherwise
#     """
#     global currentTime
#     currentTime = timeScale.now()
#     observerInfo = observer.at(currentTime).observe(CB).apparent().altaz(temperature_C=temperature, pressure_mbar=pressure)
#     altitude = observerInfo[0].degrees
#     if altitude > 0: # we consider the object visible if its altitude is greater than 0 degrees to avoid tracking objects too close to the horizon
#         # print("the object is visible, altitude :", altitude)
#         return True
#     else:
#         # print("the object is not visible, altitude :", altitude)
#         return False

def update_visible_catalog():
    """
        Update the visible catalog by checking the visibility of each celestial body in the catalog
        Order the catalog by tab and by higher altitude
        Inputs: None
        Outputs: none
    """
    print('Applying visibility filter to the catalog...')
    
    global visibleCatalog
    visibleCatalog = {key: [] for key in CATALOG.keys()} # reset the visible catalog
    #print("test", visibleCatalog)
    for key in CATALOG.keys():
        for Body in CATALOG[key]:
            
            try: 
                rising, setting, visible = rising_setting_time(celestialBody[Body['name']])
                Body['rising'] = rising
                Body['setting'] = setting
                rising25,setting25=rising_setting_above_alt(celestialBody[Body['name']],25)
                Body['rising25'] = rising25
                Body['setting25'] = setting25
                rising45,setting45=rising_setting_above_alt(celestialBody[Body['name']],45)
                Body['rising45'] = rising45
                Body['setting45'] = setting45
                visibleCatalog[key].append(Body)
                #print_HaDec(celestialBody[Body['name']], Body['name'])
                    
            except KeyError:
                print("Error: Celestial body", Body['name'], "not found in celestialBody dictionary.")

    return


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
        print("Warning: The object (OLD) HA is greater than 90°")
    #print('  Dec:', dec)
    #print('Distance:', observerInfo[2])
    
    new_ha, new_dec = new_ha_dec(ha,dec)

    print(f" new_ha : {new_ha:.3f} °")
    if abs(new_ha) > 90:
        print("Warning: The object new HA is greater than 90°!!!!!!!!!!!!!!!!!!!!!")
    print(f"  new_dec: {new_dec:.3f} °")
    return  new_ha, new_dec 

# def print_AltAz(CB,name=''):
#     """
#         Coordonate printing function for celestial bodies
#         The coordonate are given in Altitude and Azimuth
#         Inputs: CB : Celestial Body object from skyfield
#                 name : Name of the celestial body (string)
#         Outputs: None
#     """
#     global currentTime
#     currentTime = timeScale.now()
#     observerInfo = observer.at(currentTime).observe(CB).apparent().altaz(temperature_C=temperature, pressure_mbar=pressure) 
#     print('\nCurrent position of', name)
#     print('  Altitude :', observerInfo[0])
#     print('  Azimuth:', observerInfo[1])
#     print('  Distance:', observerInfo[2])
    
    # return  observerInfo[0].degrees, observerInfo[1].degrees #Alt,Az

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







def rising_setting_time(CB):
    """
    Returns the next rise time (UTC) of a solar system body using Skyfield.
    Default location: Sherbrooke, QC.
    """
  
    # 

    # Time window: now → +2 days
    t0 = timeScale.now()
    t1 = timeScale.utc(t0.utc_datetime() + timedelta(days=2))
    localTZ =  timezone(timeZone)

    # Build function for rise/transit/set
    rising, bool_rise = almanac.find_risings(observer, CB, t0, t1)
    setting, bool_set = almanac.find_settings(observer, CB,t0, t1)
    rising  = rising[0].astimezone(localTZ)
    setting = setting[0].astimezone(localTZ)
    #print(f"  Rise : {rising}")
    #print(f"  Set  : {setting}")
    if rising == None or setting == None:
        print('erreur de rise or set')
        return None
    
    if rising>setting:
        #is visible
        return  rising, setting,True

    return rising, setting,False #is not visible

def above_altitude_predicate( CB, altitude_deg):
    def predicate(t):
        alt, az, dist = observer.at(t).observe(CB).apparent().altaz()
        return alt.degrees > altitude_deg
    predicate.step_days = 0.01
    return predicate



def rising_setting_above_alt( CB, altitude_deg ):
    t0 = timeScale.now()
    t1 = timeScale.utc(t0.utc_datetime() + timedelta(days=3))

    predicate = above_altitude_predicate(CB, altitude_deg)
    times, values = almanac.find_discrete(t0, t1, predicate)
    #print(values)
    # We want transitions:
    #   False → True  = rises above altitude
    #   True → False  = drops below altitude
    
    risingAboveAlt = None
    settingAboveAlt = None
    
    for i in range(1, len(values)):
        #print('i:',values[i])
        if not values[i-1] and values[i]:
            risingAboveAlt = times[i].astimezone(timezone(timeZone))
            break
    
    for j in range(1, len(values)):
        #print('j:',values[j])
        if values[j-1] and not values[j]:
            settingAboveAlt = times[j].astimezone(timezone(timeZone))
            break

    return risingAboveAlt, settingAboveAlt

def print_visible_catalog():
  
    update_visible_catalog()
    print("\nVisible catalog :")
    for key in visibleCatalog.keys():
        print("\n", key, ":")
        for Body in visibleCatalog[key]:
            
            print("  -", Body['name'], " (", Body['icon'], ")")
            print(f'rise :{Body['rising']}, 25:{Body['rising25']}, 45:{Body['rising45']}')
            print(f'rise :{Body['setting']}, 25:{Body['setting25']}, 45:{Body['setting45']}')
            print('alt:', observer.at(currentTime).observe(celestialBody[Body['name']]).apparent().altaz(temperature_C=temperature, pressure_mbar=pressure)[0].degrees)
    print("\nTotal number of visible objects :", sum(len(visibleCatalog[key]) for key in visibleCatalog.keys()))
# #-------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------------#
# MAIN
# -------------------------------------------------------
if __name__ == "__main__":
    rise,sett =rising_setting_above_alt(celestialBody['JUPITER'],50)
    print(rise)
    print(sett)
    rising_setting_time(celestialBody['JUPITER'])
    print_visible_catalog()
    pass
#-------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------------#

#update time
#currentTime = timeScale.now()