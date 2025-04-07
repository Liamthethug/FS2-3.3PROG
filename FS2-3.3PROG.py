# Copyright 1996-2022 Cyberbotics Ltd.
#
# Controle de la voiture TT-02 simulateur CoVAPSy pour Webots 2023b
# Inspiré de vehicle_driver_altino controller
# Kévin Hoarau, Anthony Juton, Bastien Lhopitallier, Martin Raynaud
# août 2023

from vehicle import Driver
from controller import Lidar
from controller import Camera
from controller import Display
import cv2
import numpy as np
import time

driver = Driver()

cv2.startWindowThread()
cv2.namedWindow("preview")

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# Lidar
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud()

Camera = Camera("camera")
Camera.enable(sensorTimeStep)
display = Display("display")

display_width = display.getWidth()
display_height = display.getHeight()
print("display : ", display_width, display_height)

# clavier
keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# vitesse en km/h
speed = 0
maxSpeed = 28  # km/h

# angle de la direction
angle = 0
maxangle_degre = 16

# mise a zéro de la vitesse et de la direction
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)

tableau_lidar_mm = [0] * 360

def set_vitesse_m_s(vitesse_m_s):
    speed = vitesse_m_s * 3.6
    if speed > maxSpeed:
        speed = maxSpeed
    if speed < 0:
        speed = 0
    driver.setCruisingSpeed(speed)

def set_direction_degre(angle_degre):
    if angle_degre > maxangle_degre:
        angle_degre = maxangle_degre
    elif angle_degre < -maxangle_degre:
        angle_degre = -maxangle_degre
    angle = -angle_degre * 3.14 / 180
    driver.setSteeringAngle(angle)

def recule():
    driver.setCruisingSpeed(-2)

# mode auto desactive
modeAuto = False
print("cliquer sur la vue 3D pour commencer")
print("a pour mode auto (pas de mode manuel sur TT02_jaune), n pour stop")

etat = 1
kv = 0.5
kd = 1

def detect_color_in_parcels(image_rgb):
    """
    Détecte les couleurs rouge et verte dans des parcelles spécifiques de l'image.
    Retourne les pixels rouges et verts détectés dans les parcelles gauche et droite.
    """
    image_rgbcv = cv2.cvtColor(image_rgb, cv2.COLOR_RGBA2RGB)
    image_hsv = cv2.cvtColor(image_rgbcv, cv2.COLOR_BGR2HSV)
    
    # Les plages HSV pour le rouge et le vert
    lower_red1 = np.array([-10, 50, 20])
    upper_red1 = np.array([10, 255, 255])
    lower_green = np.array([40, 50, 20])
    upper_green = np.array([80, 255, 255])

    # Masques pour le rouge et le vert
    mask_red = cv2.inRange(image_hsv, lower_red1, upper_red1) #| cv2.inRange(image_hsv, lower_red2, upper_red2)
    mask_green = cv2.inRange(image_hsv, lower_green, upper_green) #| cv2.inRange(image_hsv, lower_red2, upper_red2)
    
    result1 = cv2.bitwise_and(image_rgb, image_rgb, mask = mask_red)
    
    result2 = cv2.bitwise_and(image_rgb, image_rgb, mask = mask_green)


    # Parcelle droite    
    red_parcel_right = result1[275:568, 640:]
    green_parcel_right = result2[275:568, 640:]
    
    # Parcelle gauche
    red_parcel_left = result1[275:568 , 0 : 640]
    green_parcel_left = result2[275:568,  0 : 640]

    # Calculer les pixels rouges et verts dans chaque parcelle
    red_pixels_left = np.count_nonzero(red_parcel_left)
    green_pixels_left = np.count_nonzero(green_parcel_left)
    red_pixels_right = np.count_nonzero(red_parcel_right)
    green_pixels_right = np.count_nonzero(green_parcel_right)

    return red_pixels_left, green_pixels_left, red_pixels_right, green_pixels_right


while driver.step() != -1:
    while True:
        # Acquisition des données du clavier
        currentKey = keyboard.getKey()
        if currentKey == -1:
            break
        elif currentKey == ord('n') or currentKey == ord('N'):
            if modeAuto:
                modeAuto = False
                print("--------Modes Auto TT-02 jaune Désactivé-------")
        elif currentKey == ord('a') or currentKey == ord('A'):
            if not modeAuto:
                modeAuto = True
                print("------------Mode Auto TT-02 jaune Activé-----------------")

    # Acquisition des données de la caméra
    image_camera = Camera.getImage()
    ir = display.imageNew(image_camera, Display.BGRA, display.width, display.height)
    display.imagePaste(ir, 0, 0, False) 
    display.imageDelete(ir)
    image_camera_array = np.frombuffer(image_camera, dtype=np.uint8).reshape((Camera.getHeight(), Camera.getWidth(), 4))
    image_rgb = image_camera_array[:, :, :3]


    # Détection des couleurs dans les parcelles spécifiques
    red_pixels_left, green_pixels_left, red_pixels_right, green_pixels_right = detect_color_in_parcels(image_rgb)

    # Affichage pour le débogage
    print(f"Rouge gauche : {red_pixels_left}, Vert gauche : {green_pixels_left}")
    print(f"Rouge droite : {red_pixels_right}, Vert droite : {green_pixels_right}")

    if not modeAuto:
        set_direction_degre(0)
        set_vitesse_m_s(0)

    if modeAuto:
        # Vérifier si la voiture est bien orientée
        if green_pixels_left < red_pixels_left and red_pixels_right < green_pixels_right:
            print("La voiture est bien orientée.")
            set_direction_degre(0)  # Maintenir la direction
            set_vitesse_m_s(2)
        else:
            print("Correction de l'orientation...")
            
            if red_pixels_left < green_pixels_left:
                set_direction_degre(20)
                recule()
           
            elif green_pixels_right < red_pixels_right:
                set_direction_degre(-20)
                recule()
            recule()   
            set_vitesse_m_s(1)