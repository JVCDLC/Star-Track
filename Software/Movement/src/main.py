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
#-------------------------------------------------------------------------------------------#

class motorRotation:
    """
    Class to store motor rotation information
    Attributes:
        currentPosition : Current position of the motor in degrees
        targetPosition : Target position of the motor in degrees

    Methods:
        __init__(self, currentPosition=0.0, targetPosition=0.0):
            Initializes the motorRotation object with current and target positions.
        moveToTarget(self):
            Simulates moving the motor to the target position with a PID.
        updateTarget(self, newTarget):
            Updates the target position of the motor.
        predictTimeToTarget(self, maxSpeed):
            Predicts the time required to reach the target position based on speed.
    """
    def __init__(self, currentPosition=0.0, targetPosition=0.0):
        self.currentPosition = currentPosition
        self.targetPosition = targetPosition

    def moveToTarget(self):
        """
        Simulates moving the motor to the target position with a PID.
        """
        # Placeholder for PID control logic
        self.currentPosition = self.targetPosition
        print(f"Motor moved to target position: {self.targetPosition} degrees")

    def updateTarget(self, newTarget):
        """
        Updates the target position of the motor.
        Inputs: newTarget : New target position in degrees
        Outputs: None
        """
        self.targetPosition = newTarget
        return
        

    def predictTimeToTarget(self, maxSpeed):
        """
        Predicts the time required to reach the target position based on speed.
        Inputs: speed : Speed of the motor in degrees per second
        Outputs: time : Time in seconds to reach the target position
        """
        distance = abs(self.targetPosition - self.currentPosition)
        if maxSpeed <= 0:
            raise ValueError("Speed must be greater than zero")
        time = distance / maxSpeed #in degrees per second
        return time

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

    return observerInfo[0], observerInfo[1] #Ha,dec


def main():

    jupiter = planets['jupiter barycenter']

    a=print_HaDec(jupiter,'Jupiter')
    print("a0",a[0],a[1])
    print("hello\n")
    #print_HaDec(polaris,'Polaris')

    



if __name__ == '__main__':
    main()
