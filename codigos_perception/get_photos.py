import cv2

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

cont = 1

while camera.isOpened():
    ret, frame = camera.read()
    if not ret: break

    cv2.imshow('frame', frame)
    key = cv2.waitKey(1)
    if key == 27: break
    if key & 0xFF == ord('q'):
        print('imagen guardada en ' + f'calibration/normal/img{cont}.jpg')
        cv2.imwrite(f'calibration/normal/img{cont}.jpg', frame)
        cont += 1

camera.release()
cv2.destroyAllWindows()