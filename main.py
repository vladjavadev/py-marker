import cv2
import cv2.aruco as aruco
import numpy as np
import time

start = time.time()

# Завантажуємо фото
frame = cv2.imread("img/image2.png")

# Переносимо кадр у GPU
gpu_frame = cv2.cuda_GpuMat()
gpu_frame.upload(frame)

# Ресайз на GPU
gpu_resized = cv2.cuda.resize(gpu_frame, (320, 240))

# Конвертація у grayscale на GPU
gpu_gray = cv2.cuda.cvtColor(gpu_resized, cv2.COLOR_BGR2GRAY)

# Завантажуємо назад у CPU для ArUco
frame = gpu_resized.download()
gray = gpu_gray.download()

# Словник маркерів
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Детекція маркерів (CPU)
corners, ids, rejected = aruco.detectMarkers(gray, dictionary)

# Параметри камери (приклад)
camera_matrix = np.array([[1000, 0, frame.shape[1]/2],
                          [0, 1000, frame.shape[0]/2],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5,1))

# Оцінка позиції
if ids is not None:
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
    for i in range(len(ids)):
        print(f"Маркер {ids[i]}:")
        print("  rvec:", rvecs[i])
        print("  tvec:", tvecs[i])

        # Малюємо координатні осі
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)

cv2.imwrite("img/pose_result.png", frame)
print("Time: ", time.time() - start)