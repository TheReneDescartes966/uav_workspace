import cv2
import numpy as np

# Crear el objeto VideoCapture para capturar video en vivo desde la cámara
cap = cv2.VideoCapture(0)

# Verificar si la captura del video se abrió correctamente
if not cap.isOpened():
    print("Error: No se pudo abrir la cámara.")
    exit()

# Definir el diccionario Aruco y los parámetros del detector
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()

# Crear el detector de marcadores Aruco
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# Crear el objeto VideoWriter para guardar el video de salida
output_video_path = '/content/output_video.mp4'  # Ruta de salida del video
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = cap.get(cv2.CAP_PROP_FPS)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
out = cv2.VideoWriter(output_video_path, fourcc, fps, (frame_width, frame_height))

# Procesar cada fotograma del video
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convertir la imagen a escala de grises (opcional)
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detectar marcadores Aruco en la imagen actual
    marker_corners, marker_ids, rejected_candidates = detector.detectMarkers(frame)

    # Dibujar los marcadores detectados en el fotograma actual
    if marker_ids is not None:
        frame = cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

    # Mostrar el fotograma procesado
    cv2.imshow('Aruco Detector', frame)

    # Escribir el fotograma procesado en el video de salida
    out.write(frame)

    # Salir del bucle si se presiona 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar los objetos de captura y escritura de video
cap.release()
out.release()

# Cerrar todas las ventanas abiertas
cv2.destroyAllWindows()

print(f"Video procesado guardado como {output_video_path}")
