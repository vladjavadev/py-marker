import cv2
import cv2.aruco as aruco
import numpy as np
import time


start = time.time()
# Завантажуємо фото з маркером
frame = cv2.imread("img/image2.png")
frame = cv2.resize(frame, (640, 480))


# Словник маркерів
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Детекція маркерів
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

corners, ids, rejected = aruco.detectMarkers(gray, dictionary)

# Параметри камери (треба калібрувати для точності!)
# Тут приклад з умовними значеннями
camera_matrix = np.array([[1000, 0, gray.shape[1]/2],
                          [0, 1000, gray.shape[0]/2],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5,1))  # якщо немає калібрування

# Оцінка позиції
if ids is not None:
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
    for i in range(len(ids)):
        print(f"Маркер {ids[i]}:")
        print("  rvec:", rvecs[i])  # вектор обертання
        print("  tvec:", tvecs[i])  # вектор трансляції

        # Малюємо координатні осі
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)

cv2.imwrite("pose_result.png", frame)
print("Time: ",time.time()-start)
cv2.waitKey(0)
cv2.destroyAllWindows()
