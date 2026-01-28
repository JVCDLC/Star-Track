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
#Classes
        
class CoordinateOfCelestialBody:   
    """
    Class to store celestial body coordinate information
    Attributes:
        ha : Hour Angle of the celestial body
        dec : Declination of the celestial body

    Methods:
        __init__(self, ha=0.0, dec=0.0):
            Initializes the coordinateOfCelestialBody object with hour angle and declination.
        updateCoordinates(self, newHa, newDec):
            Updates the hour angle and declination of the celestial body.
        
    """
    def __init__(self,body, bodyName = 'name', ha=0.0, dec=0.0):
        self.bodyName = bodyName
        self.body = body
        self.updateCoordinates()

    def updateCoordinates(self):
        """
        Updates the hour angle and declination of the celestial body.
        Inputs: newHa : New hour angle in degrees
                newDec : New declination in degrees
        Outputs: None
        """
        ha , dec = print_HaDec(self.body,self.bodyName)
        self.ha = ha
        self.dec = dec
        return
    def convertDegreeToMotorPositions(self): #jp a dit pas nécessaire
        """
            Converts the hour angle and declination in degrees to motor positions.
            Inputs: None
            Outputs: motorPosition : Motor position in steps
        """
        gearRatio = 10 # Gear ratio
        motorEncoderStepsPerRevolution = 2050 # Motor encoder steps per revolution
        motorPosition = self*gearRatio* motorEncoderStepsPerRevolution/360
        return motorPosition


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
   


#---------------------------#
if __name__ == '__main__':
    main()
#-------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------------#