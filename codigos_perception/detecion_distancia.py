#!/usr/bin/python3
import sys, cv2, os, numpy as np
sys.dont_write_bytecode = True

path = os.getcwd()

class Calibration:
    def __init__(self):
        # Check if the calibration files exist and load them.
        self.exist_normal = os.path.isfile(path+'/calibration/normal.npz')
        if self.exist_normal: self.mtx, self.dist = np.load(path+'/calibration/normal.npz').values()
        
    def __initialization(self, num_images, tipe):
        # This function is the same for both fisheye and normal camera calibration.
        self.pattern_size = (9, 6)
        square_size = 25.0

        self.img_points = []
        self.obj_points = []

        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= square_size
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
        
        detected_imgs = 0
        for i in range(1, num_images+1):
            img = cv2.imread(path+'/calibration/'+tipe+'/img'+str(i)+'.jpg')
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.img_shape = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size)
            if ret == True:
                detected_imgs += 1
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                self.img_points.append(corners)
                self.obj_points.append(objp)
        print(f'Detected images: {detected_imgs}')
            
    def normal_calibration(self, num_images):
        # A normal camera calibration is performed.
        self.__initialization(num_images, 'normal')
        # mtx, dist= cv2.calibrateCamera(self.obj_points, self.img_points, self.img_shape, None, None)[1:3]
        _, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.obj_points, self.img_points, self.img_shape, None, None)
        np.savez(path+'/calibration/normal.npz', mtx=mtx, dist=dist)
        self.mtx, self.dist = mtx, dist
        print('Successful normal calibration')
        
        # Calculate the reprojection error
        reproyection_error = self.get_reproyection_error(rvecs, tvecs, mtx, dist)
        print(f"Repoyection error: {reproyection_error:.5f}")
        print("For normal calibration, the reproyection error is:")
        print("<1.0 excellent, <3.0 good, >3 bad (consider add more images)")    
        
        
    def normal_correction(self, img):
        if self.exist_normal:
            return cv2.undistort(img, self.mtx, self.dist, None, None)
        raise FileNotFoundError('normal.npz not found, please run normal_calibration() first')
    
    def get_reproyection_error(self, rvecs, tvecs, K, D):
        mean_error = 0
        for i in range(len(self.obj_points)):
            imgpoints2, _ = cv2.projectPoints(self.obj_points[i], rvecs[i], tvecs[i], K, D)
            error = cv2.norm(self.img_points[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        return mean_error/len(self.obj_points)

def calibrate(num_images, tipe, resize=1.0, test_image=1):
    node = Calibration()
    node.normal_calibration(num_images)
    undisort = node.normal_correction(cv2.imread('calibration/normal/img'+str(test_image)+'.jpg'))
    undisort = cv2.resize(undisort, (0, 0), fx=resize, fy=resize)
    img = cv2.imread('calibration/normal/img'+str(test_image)+'.jpg')
    cv2.imshow('frame', undisort)
    cv2.waitKey()
    cv2.destroyAllWindows()

def get_image_corrected():
    marker_length = 0.1
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()

    # Crear el detector de marcadores Aruco
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    node = Calibration()
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break
        frame_corregido = node.normal_correction(frame) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray1 = cv2.cvtColor(frame_corregido, cv2.COLOR_BGR2GRAY)
        camera_matrix = node.mtx
        dist_coeffs = node.dist
        # Detectar marcadores Aruco en la imagen actual
        marker_corners, marker_ids, rejected_candidates = detector.detectMarkers(gray)
        marker_corners1, marker_ids1, rejected_candidates1 = detector.detectMarkers(gray1)

        # Dibujar los marcadores detectados en el fotograma actual
        if marker_ids is not None:
            frame = cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, marker_length, camera_matrix, dist_coeffs)

            for rvec, tvec, marker_corner, marker_id in zip(rvecs, tvecs, marker_corners, marker_ids):
                # Dibujar los marcadores detectados y sus ejes de referencia
                cv2.aruco.drawDetectedMarkers(frame, [marker_corner], marker_id)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                # Obtener la posición en x, y, z del marcador
                x, y, z = tvec[0][0], tvec[0][1], tvec[0][2]
                distance = np.sqrt(x**2 + y**2 + z**2)

                # Mostrar la distancia en la imagen
                cv2.putText(frame, f"ID: {marker_id[0]} Dist: {distance:.2f} m", 
                            (int(marker_corner[0][0][0]), int(marker_corner[0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Dibujar los marcadores detectados en el fotograma actual
        if marker_ids1 is not None:
            frame_corregido = cv2.aruco.drawDetectedMarkers(frame_corregido, marker_corners1, marker_ids1)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners1, marker_length, camera_matrix, dist_coeffs)

            for rvec, tvec, marker_corner, marker_id in zip(rvecs, tvecs, marker_corners1, marker_ids1):
                # Dibujar los marcadores detectados y sus ejes de referencia
                cv2.aruco.drawDetectedMarkers(frame_corregido, [marker_corner], marker_id)
                cv2.drawFrameAxes(frame_corregido, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                # Obtener la posición en x, y, z del marcador
                x, y, z = tvec[0][0], tvec[0][1], tvec[0][2]
                distance = np.sqrt(x**2 + y**2 + z**2)

                # Mostrar la distancia en la imagen
                cv2.putText(frame_corregido, f"ID: {marker_id[0]} Dist: {distance:.2f} m", 
                            (int(marker_corner[0][0][0]), int(marker_corner[0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


    # undisort = node.normal_correction(cv2.imread('calibration/normal/img'+str(40)+'.jpg'))
        cv2.imshow("normal", frame)
        cv2.imshow("corregido", frame_corregido)
        # Salir del bucle si se presiona 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # Liberar los objetos de captura y escritura de video
    cap.release()

    # Cerrar todas las ventanas abiertas
    cv2.destroyAllWindows()

#calibrate(47,0)
get_image_corrected()