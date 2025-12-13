import cv2
import cv2.aruco as aruco
import numpy as np
import time




class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        # Интеграл ошибки
        self.integral += error * dt
        # Производная ошибки
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        # PID формула
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # Запоминаем ошибку
        self.prev_error = error
        return output


marker_pid = {}

# Основной код
start = time.time()

# Загрузка изображения
frame = cv2.imread("test-img/image3.png")

# Словарь маркеров
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Детекция маркеров
corners, ids, rejected = aruco.detectMarkers(frame, dictionary)

# Параметры камеры
camera_matrix = np.array([[1000, 0, frame.shape[1]/2],
                          [0, 1000, frame.shape[0]/2],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5,1))

P_list = np.array([[-0.05, 0, 0.2],
         [0.0,0.0,0.2],
         [0.05, 0, 0.2]])
# Оценка позиции



dt = 0.05  # шаг времени (например, 50 мс)

if ids is not None:
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
    
    current_time = time.time()
    
    for i in range(len(ids)):
        print(f"Маркер {ids[i]}:")
        print("  rvec:", rvecs[i])
        print("  tvec:", tvecs[i])
        if i not in marker_pid:
             marker_pid[i] = {"linear":PIDController(Kp=0.5, Ki=0.0, Kd=0.1),
                              "angular":PIDController(Kp=1.0, Ki=0.0, Kd=0.2)}
            
        pid_linear = marker_pid[i]["linear"]
        pid_angular = marker_pid[i]["angular"]

        # Рисуем текущие оси маркера (красные)
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)
        
        tvec = tvecs[i][0]
    
        # Вычисление целевой ориентации
        P_target = P_list[i] # Точка, на которую должен смотреть маркер
        V_up = np.array([0, 1, 0])  # Вектор "вверх"
        
        # Вычисление осей целевой ориентации
        Z_target = P_target - tvec
        Z_target_unit = Z_target / np.linalg.norm(Z_target)
        
        X_target = np.cross(Z_target_unit, V_up)
        X_target_unit = X_target / np.linalg.norm(X_target)
        
        Y_target_unit = np.cross(Z_target_unit, X_target_unit)
        
        # Построение матрицы поворота
        R_target = np.column_stack((X_target_unit, Y_target_unit, Z_target_unit))
        
        # # Преобразование в вектор вращения
        # rvec_target, _ = cv2.Rodrigues(R_target)
        
        tvec_leader = np.array([0.0, 0.0, 0.0])
        rvec_leader = np.array([0.0, 0.0, 0.0])  # Смотрит вдоль оси Z камеры
        
        # Желаемое смещение ведомого относительно мастера
        # [0, 0, -0.3] означает: ведомый должен быть на 30см ПОЗАДИ мастера
        # Но в координатах камеры "позади" = положительная Z!
        offset = np.array([0.0, 0.0, 0.3])  # 30см перед камерой
        

        # Рисуем целевые оси (зеленые)
        # cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_target, tvecs[i], 0.05)
        R_marker, _ = cv2.Rodrigues(rvecs[i])
        R_camera = R_target
        print(f"Rmarker: {R_marker}")
        print(f"Rcamera: {R_camera}")
        R_delta = R_camera @ R_marker.T

        #calc rotation angle theta
        #np trace - sum of diagonal elements matrix
        theta  = np.arccos((np.trace(R_delta)-1)/2)

        y_axis  = R_delta[0,2]-R_delta[2,0]/(2*np.sin(theta))

        print(f"Rdelta: {y_axis}")

        distance = np.linalg.norm(P_target - tvec)

        print(f"distances: {distance}")

        pos_err = np.linalg.norm(P_target - tvec)   # ошибка по позиции
        yaw_err = np.arctan2(Z_target_unit[0], Z_target_unit[2])  # ошибка курса (пример)

        speed = pid_linear.update(pos_err, dt)
        omega = pid_angular.update(yaw_err, dt)

        print(f"Linear speed: {speed:.3f}, Angular speed: {omega:.3f}")



        # # print("Целевая матриця поворота R:\n", R_target)
        # # print("Целевой rvec:", rvec_target.flatten())
        # (speed, omega, target_pos, yaw_err) = fc.compute_follower_control(tvec_leader, rvec_leader, tvecs[i][0],rvecs[i][0], P_target)

        
        # Визуализация целевой позиции для следования (голубая точка)
        target_img_point, _ = cv2.projectPoints(
            P_target.reshape(1, 3), 
            np.zeros((3,1)), 
            np.zeros((3,1)), 
            camera_matrix, 
            dist_coeffs
        )
        
        cv2.circle(frame, 
                  tuple(target_img_point[0][0].astype(int)), 
                  10, (255, 255, 0), -1)  # Голубая точка
        cv2.putText(frame, "FOLLOW TARGET", 
                   tuple(target_img_point[0][0].astype(int) + np.array([15, -10])),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # Визуализация P_target (точка для ориентации - фиолетовая)
        p_target_img, _ = cv2.projectPoints(
            P_target.reshape(1, 3), 
            np.zeros((3,1)), 
            np.zeros((3,1)), 
            camera_matrix, 
            dist_coeffs
        )
        
        cv2.circle(frame, 
                  tuple(p_target_img[0][0].astype(int)), 
                  8, (255, 0, 255), -1)  # Фиолетовая точка
        cv2.putText(frame, "ORIENT TARGET", 
                   tuple(p_target_img[0][0].astype(int) + np.array([15, 10])),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        # Стрелка от ведомого к целевой позиции следования
        follower_img, _ = cv2.projectPoints(
            tvecs[i][0].reshape(1, 3), 
            np.zeros((3,1)), 
            np.zeros((3,1)), 
            camera_matrix, 
            dist_coeffs
        )
        cv2.arrowedLine(frame, 
                       tuple(follower_img[0][0].astype(int)),
                       tuple(target_img_point[0][0].astype(int)),
                       (0, 255, 255), 2, tipLength=0.3)
        
        # Информация на изображении
        cv2.putText(frame, f"ID: {ids[i][0]}", 
                   tuple(follower_img[0][0].astype(int) + np.array([-50, -15])),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

cv2.imshow("result", frame)
print(f"Time: {time.time()-start:.4f} сек")
cv2.waitKey(0)
cv2.destroyAllWindows()