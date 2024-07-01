import cv2

# Crear el objeto VideoCapture para capturar video en vivo desde la cámara
cap = cv2.VideoCapture(0)

# Verificar si la captura del video se abrió correctamente
if not cap.isOpened():
    print("Error: No se pudo abrir la cámara.")
    exit()

# Procesar cada fotograma del video
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Mostrar el fotograma capturado
    cv2.imshow('Video en Vivo', frame)

    # Salir del bucle si se presiona 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar el objeto de captura de video
cap.release()

# Cerrar todas las ventanas abiertas
cv2.destroyAllWindows()
