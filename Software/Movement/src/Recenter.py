import numpy as np
import Software.Movement.src.astrononic_function as astrononic_function
#-----------------------------------------------------------#
#Variables for recentering the camera
camAngle = 90 # Angle actuel de la caméra en degrés 0 -> à droite
currentTime = astrononic_function.currentTime
observer = astrononic_function.observer
pixelToDegree = 0.1 # Conversion de pixels à degrés, à ajuster en fonction de la caméra et de l'objectif utilisés
manualSpeedMax = 5 # degees per second 
maxInputFormJoyStick = [-2,2]
#-----------------------------------------------------------#

rotationYXtoALTAZ = np.array([[np.sin(np.radians(camAngle)),np.cos(np.radians(camAngle))],
                    [np.cos(np.radians(camAngle)),-np.sin(np.radians(camAngle))  ]])
Y= 1
X = 0
[alt,az]= rotationYXtoALTAZ @ np.array([X,Y]) # Vecteur pointant vers la droite de la caméra (1,0) transformé en coordonnées Alt/Az
print(f"Vecteur pointant vers la droite de la caméra en coordonnées Alt/Az : {alt:.2f}° Alt, {az:.2f}° Az")


#-----------------------------------------------------------#
#Functions to implement




def setupCameraAngle():
    # Move in X,Y and compare with X,Y mesure from the camera
    # 
    pass

def recenterCamera():
    # Code pour recentrer la caméra sur la cible spécifiée par targetAlt et targetAz
    # Cela pourrait impliquer de calculer les mouvements nécessaires en fonction de l'angle actuel de la caméra
    pass

def moveManually():
    # Code pour permettre un contrôle manuel de la caméra, par exemple via une interface utilisateur ou des commandes clavier
    pass

# -------------------------------------------------------
# MAIN
# -------------------------------------------------------
if __name__ == "__main__":
    pass