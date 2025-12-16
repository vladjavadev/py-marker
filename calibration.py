import cv2
import numpy as np
import glob

import os

# --- КОНСТАНТИ КАЛІБРУВАННЯ ---

# Кількість внутрішніх кутів шахівниці: (ширина, висота)
# Наприклад, 9x6 має 8x5 внутрішніх кутів.
CHECKERBOARD_SIZE = (8, 5) 

# Точний фізичний розмір одного квадрата на вашій роздруківці (в міліметрах)
# Це критично для отримання точних відстаней!
SQUARE_SIZE_MM = 20.0 

# Критерії завершення для уточнення кутів
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Папка з калібрувальними зображеннями
IMAGE_DIR = 'calibration_images' 
OUTPUT_FILE = 'camera_params.yaml'


def setup_object_points():
    """Створює шаблон 3D-точок (об'єктні точки) для шахівниці (Z=0)."""
    # Генеруємо 2D-координати сітки (x, y) і множимо на розмір квадрата
    objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE_MM
    return objp

def save_camera_parameters(filename, camera_matrix, dist_coeffs, img_size):
    """Зберігає результати калібрування у файл YAML."""
    data = {
        'image_width': img_size[0],
        'image_height': img_size[1],
        'camera_matrix': camera_matrix.tolist(),
        'distortion_coefficients': dist_coeffs.tolist(),
    }

    print("data:\n", data)
    # with open(filename, 'w') as f:
    #     yaml.dump(data, f)
    print(f"\n✅ Параметри камери успішно збережено у файлі: {filename}")


def calibrate_camera():
    """Основна функція калібрування."""
    
    # 1. Підготовка даних
    objp = setup_object_points()
    objpoints = []  # 3D точки (координати в реальному світі)
    imgpoints = []  # 2D точки (координати в пікселях зображення)
    
    # Завантажуємо всі зображення з папки
    images = glob.glob(os.path.join(IMAGE_DIR, '*.jpg'))
    print(f"Знайдено {len(images)} зображень для калібрування.")
    
    if not images:
        print(f"ПОМИЛКА: Не знайдено зображень у папці '{IMAGE_DIR}'.")
        print("Будь ласка, створіть цю папку та додайте до неї фотографії шахівниці.")
        return

    # 2. Обробка зображень
    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"Помилка завантаження зображення: {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Знайти кути шахівниці
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, None)

        # Якщо кути знайдені, додаємо об'єктні та зображальні точки
        if ret == True:
            objpoints.append(objp)
            
            # Уточнення позиції кутів
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
            imgpoints.append(corners2)
            
            # Візуалізація: малюємо знайдені кути
            cv2.drawChessboardCorners(img, CHECKERBOARD_SIZE, corners2, ret)
            cv2.putText(img, 'Found', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(img, 'NOT Found', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        cv2.imshow('Calibration Image', img)
        cv2.waitKey(200) # Показуємо зображення на 200 мс

    cv2.destroyAllWindows()
    
    # 3. Калібрування
    if objpoints:
        print("\nЗапуск калібрування...")
        img_size = gray.shape[::-1] # (width, height)
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, img_size, None, None
        )

        if ret:
            print("Калібрування успішно завершено!")
            print("-----------------------------------------------------")
            print("Матриця Камери (K):\n", camera_matrix)
            print("Коефіцієнти Дисторсії (D):\n", dist_coeffs)
            print("-----------------------------------------------------")
            # Зберігаємо результати
            save_camera_parameters(OUTPUT_FILE, camera_matrix, dist_coeffs, img_size)
            
            # Оцінка помилки (RMSE)
            mean_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
                error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                mean_error += error
            print(f"\nСередня помилка репроєкції (Re-projection Error): {mean_error/len(objpoints):.4f} пікселів.")
            print("Чим менше, тим краще (бажано < 1.0).")

        else:
            print("Калібрування не вдалося. Перевірте зображення.")
    else:
        print("Калібрування скасовано: жодна шахівниця не була знайдена.")


if __name__ == '__main__':
    calibrate_camera()