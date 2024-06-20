import cv2
import cv2.aruco as aruco
import numpy as np

# Cargar una imagen
image = cv2.imread('aruko.jpg')

# Inicializar el diccionario ArUco
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# Inicializar el parámetro de detección
parameters = aruco.DetectorParameters_create()

# Detectar marcadores ArUco en la imagen
corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)

# Dibujar los marcadores detectados si hay alguno
if ids is not None:
    image = aruco.drawDetectedMarkers(image, corners, ids)

# Mostrar la imagen resultante
cv2.imshow('ArUco Markers', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

