import cv2
import numpy as np

# Выбираем словарь 4x4 (например, 50 маркеров)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# ID маркера (от 0 до 49 для DICT_4X4_50)
for i in range(0,20):
    marker_id = i
    marker_size = 200  # размер изображения в пикселях

    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)


    # Сохраняем в файл
    cv2.imwrite(f"marker/aruco_4x4_id{marker_id}.png", marker_image)

    # Показываем на экране
    # cv2.imshow("ArUco Marker", marker_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()