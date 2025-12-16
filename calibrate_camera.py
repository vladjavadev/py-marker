import numpy as np
import cv2
import glob
import time

# === 1. ВИЗНАЧЕННЯ ПАРАМЕТРІВ ШАХІВНИЦІ ===
# Кількість внутрішніх кутів (перетинів) по горизонталі (width)
CHECKERBOARD_W = 8 
# Кількість внутрішніх кутів (перетинів) по вертикалі (height)
CHECKERBOARD_H = 6 
# Розмір шаблону, тобто загальна кількість внутрішніх кутів
CHECKERBOARD_SIZE = (CHECKERBOARD_W, CHECKERBOARD_H) 

# Умови для термінації ітеративного алгоритму калібрування
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Масиви для зберігання координат об'єкта (3D) та координат зображення (2D)
objpoints = [] # 3D точки реального світу (координати кутів на шаблоні)
imgpoints = [] # 2D точки зображення (координати кутів на кадрі)

# Створення "точок об'єкта" (координат у реальному світі). 
# Ми припускаємо, що шаблон лежить у площині Z=0, а розмір клітинки = 1.
objp = np.zeros((1, CHECKERBOARD_W * CHECKERBOARD_H, 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD_W, 0:CHECKERBOARD_H].T.reshape(-1, 2)


# === 2. ЗБІР ЗОБРАЖЕНЬ З КАМЕРИ ===
def collect_images(num_images=15):
    """Захоплює зображення з камери та знаходить кути шахівниці."""
    
    cap = cv2.VideoCapture(1)  # 0 - індекс вашої камери
    if not cap.isOpened():
        print("Помилка: Не вдалося відкрити камеру.")
        return

    print(f"Початок збору. Натисніть 'S' для збереження кадру (потрібно {num_images} кадрів)...")
    
    count = 0
    while count < num_images:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Перетворюємо кадр у відтінки сірого для обробки
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Знаходимо кути шахівниці
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        cv2.find
        if ret:
            # Якщо кути знайдені, малюємо їх
            cv2.drawChessboardCorners(frame, CHECKERBOARD_SIZE, corners, ret)
            status_text = f"ЗНАЙДЕНО КУТИ: {count}/{num_images} (Натисніть 'S' для збереження)"
        else:
            status_text = f"Шукаю шаблон... {count}/{num_images}"

        # Відображення тексту статусу
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow('Calibration View', frame)
        
        key = cv2.waitKey(10)
        
        if key == ord('s') and ret:
            # Збереження знайдених кутів, якщо вони коректні
            objpoints.append(objp)
            
            # Уточнення позицій кутів у підпіксельному діапазоні
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
            imgpoints.append(corners2)
            
            count += 1
            print(f"Кадр {count} збережено.")
            
        elif key == 27: # Esc
            break
        # time.sleep(2.0)
    cap.release()
    cv2.destroyAllWindows()
    return gray.shape[::-1] # Повертає (ширина, висота) кадру

# === 3. ВИКОНАННЯ КАЛІБРУВАННЯ ===
def calibrate(frame_size):
    """Виконує калібрування на основі зібраних точок."""
    
    if len(objpoints) < 5:
        print("ПОМИЛКА: Недостатньо успішно збережених кадрів для калібрування.")
        return None, None
        
    print(f"\nПочаток калібрування з {len(objpoints)} кадрами...")
    
    # Головна функція калібрування OpenCV
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frame_size, None, None)
    
    if ret:
        print("\nКалібрування УСПІШНО завершено!")
        print("--- МАТРИЦЯ КАМЕРИ (K) ---")
        print(mtx)
        print("\n--- КОЕФІЦІЄНТИ СПОТВОРЕННЯ (D) ---")
        print(dist)
        
        return mtx, dist
    else:
        print("ПОМИЛКА: Калібрування не вдалося.")
        return None, None

# === 4. ЗБЕРЕЖЕННЯ РЕЗУЛЬТАТІВ ===
def save_calibration_data(mtx, dist, filename='camera_params.npz'):
    """Зберігає матрицю камери та коефіцієнти спотворення у файл NPZ."""
    if mtx is not None and dist is not None:
        np.savez(filename, mtx=mtx, dist=dist)
        print(f"\nПараметри калібрування збережено у файл: {filename}")
        print("Тепер ви можете завантажити їх у своєму основному скрипті.")


# === ЗАПУСК ===
if __name__ == '__main__':
    # 1. Збір зображень
    # 
    size = collect_images(num_images=15)
    
    if size:
        # 2. Калібрування
        camera_matrix, dist_coeffs = calibrate(size)
        
        # 3. Збереження
        save_calibration_data(camera_matrix, dist_coeffs)