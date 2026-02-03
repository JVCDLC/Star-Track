#-----------------------------------------------------#
#   Star-Tack project
#   Movement Module
#   main.py
#   Entry point for movement functionalities
#   Created by Antoine Desgranges
#   Last update : 01-25-2026
#-----------------------------------------------------#

# Import necessary modules

#from movement_controller import MovementController
from numpy import angle
from skyfield.api import load, Star, wgs84
from skyfield.data import hipparcos
import csv
from pathlib import Path

# Chemin vers le fichier CSV depuis src/
csv_path = Path(__file__).resolve().parent.parent / "data" / "messier_qc.csv"



with load.open(hipparcos.URL) as f:
    stars = hipparcos.load_dataframe(f)

#-------------------------------------------------------------------------------------------#
#Global variables and initializations
timeScale = load.timescale()
currentTime = timeScale.now()
planets = load('de421.bsp') #Planet data including the SUN and the MOON
earth = planets['earth']
polaris= Star.from_dataframe(stars.loc[11767]) #refrence star Polaris


#-------------------------------------------------------------------------------------------#
#position of the observer
    #Sherbrooke, QC : 45.365434 , -71.939477
latitude = 45.365434 #  N 45° 21' 55.563''
longitude = -71.939477 # W 71° 56' 22.115''
observer = earth + wgs84.latlon(latitude, longitude)
temperature = 15.0 #degrees Celsius
pressure = 1005.0 #mbar
#-------------------------------------------------------------------------------------------#

celestialBodies = {
    'Sun':planets['sun'],
    'Moon':planets['moon'],
    'Mercury':planets['mercury'],
    'Venus':planets['venus'],
    'Mars':planets['mars'],
    'Jupiter':planets['jupiter barycenter'],
    'Saturn':planets['saturn barycenter'],
    'Uranus':planets['uranus barycenter'],
    'Neptune':planets['neptune barycenter'],
    'Polaris':polaris
}


messier = {}

with open(csv_path, newline='', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    for row in reader:
        star = Star(
            ra_hours=float(row['ra_hours']),
            dec_degrees=float(row['dec_degrees'])
        )
        messier[row['name']] = {
            'type': row['type'],
            'mag': float(row['mag']),
            'comment': row['comment'],
            'star': star
        }

m31 = messier['M31']['star']
"""# Exemple : obtenir la position de M31
# Exemple : obtenir la position de M31
earth = load('de421.bsp')['earth']
m31 = messier['M31']['star']
astrometric = earth.at(t).observe(m31)
ra, dec, distance = astrometric.radec()

print("M31 RA:", ra)
print("M31 Dec:", dec)"""
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
        Inputs: CB : Celestial Body object from skyfield
        Outputs: True if the celestial body is above the horizon, False otherwise
    """
    observerInfo = observer.at(currentTime).observe(CB).apparent().altaz(temperature_C=temperature, pressure_mbar=pressure)
    altitude = observerInfo[0].degrees
    if altitude > 0:
        print("the object is visible, altitude :", altitude)
        return True
    else:
        print("the object is not visible, altitude :", altitude)
        return False

def print_HaDec(CB,name=''):
    """
        Coordonate printing function for celestial bodies
        The coordonate are given in Right Ascension, Hour Angle and Declination
        Hour angle (HA) and Declination (dec) are directly related to RA_motor and Dec_motor
        Inputs: CB : Celestial Body object from skyfield
                name : Name of the celestial body (string)
        Outputs: None
    """
    observerInfo = observer.at(currentTime).observe(CB).apparent().hadec() 
    print('\nCurrent position of', name)
    print('  HA :', observerInfo[0])
    print('  Dec:', observerInfo[1])
    print('  Distance:', observerInfo[2])
    
    return  observerInfo[0].degrees, observerInfo[1].degrees #Ha,dec

#-------------------------------------------------------------------------------------------#
#Main function
def main():

    jupiter = planets['jupiter barycenter']

    a=print_HaDec(jupiter,'Jupiter')
    print("angle en degree : \n ha : ",a[0], " \n dec : ",a[1])
    print_HaDec(polaris,'Polaris')
    print_HaDec(m31,'M31')

#---------------------------#
if __name__ == '__main__':
    main()
#-------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------------#