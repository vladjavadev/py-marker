import cv2
import cv2.aruco as aruco
import numpy as np
import time
from collections import deque
import threading
from data.robot_dto import Robot
from data.robot_dto import DeltaPos
from data.follow_point import FollowPoint as fp
from data.marker import Marker



pre_proc_frames = deque(maxlen=3)
post_proc_frames = deque(maxlen=3)
robot_dto_dict = {}

marker_pid = {}
follow_points = [
    fp([-0.05,0.0,0.2]),
    fp([0.0,0.0,0.2]),
    fp([0.05,0.0,0.2]),
]



# Основной код
start = time.time()
dt = 0.05



try:
    # Завантажуємо дані з файлу, створеного скриптом калібрування
    calibration_data = np.load('data/camera_params.npz')
    
    # Витягуємо матрицю камери та коефіцієнти спотворення
    camera_matrix = calibration_data['mtx']
    dist_coeffs = calibration_data['dist']
    
    print("Параметри калібрування успішно завантажено!")
    
except FileNotFoundError:
    print("ПОМИЛКА: Файл 'camera_params.npz' не знайдено.")
    print("Переконайтеся, що ви виконали калібрування та зберегли файл у тій же папці.")
    # Тут можна залишити умовні значення, але оцінка позиції буде неточною
    # camera_matrix = ...
    # dist_coeffs = ...
    exit() # Або завершити програму, якщо калібрування критично важливе

cap = cv2.VideoCapture(1)

def read_img():
    ix = 0
    while True:
        time.sleep(0.2)
        start = time.time()
        if len(pre_proc_frames)==pre_proc_frames.maxlen:
            last = pre_proc_frames.pop()
            pre_proc_frames.clear()
            pre_proc_frames.append(last)
        else:
            _,frame = cap.read()
            pre_proc_frames.append(frame)
            print("reading time: ", time.time()-start)
        ix=(ix+1)%3
        


def prepocess_img():
    while True:
        time.sleep(0.1)
        start = time.time()
        if len(post_proc_frames)==post_proc_frames.maxlen:
            last = post_proc_frames.pop()
            post_proc_frames.clear()
            post_proc_frames.append(last)
        if len(pre_proc_frames)>0:
            frame = pre_proc_frames.pop()
            # frame = cv2.resize(frame, (320, 240))
            # frame = frame[0:120, :]
            post_proc_frames.append(frame)
       
            print("preproc time: ", time.time()-start)


 
    
def init():
    ids = []
    global cap
    while cap.isOpened()==False:
        cap.release()
        time.sleep(1.0)
        cap = cv2.VideoCapture(1)
        
    while len(ids)<3:
        _, frame = cap.read()


        # Словник маркерів
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # Детекція маркерів
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(frame, dictionary)
        if ids is None or len(ids) == 0:
            # No markers detected
            ids = []

        time.sleep(0.1)



    # Оцінка позиції
    if ids is not None:
        _, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        for i in range(len(ids)):
            id = ids[i][0]
            # is_connected = wait_client(id)
            # if not is_connected:
            #     continue

            if id not in robot_dto_dict:
                marker = Marker(id)
                robot_dto_dict[id] = Robot(marker=marker)

            robot = robot_dto_dict[id]

            tvec = tvecs[i][0]
            lst_dist = float("inf")
            target_fp = None
            for fp in follow_points:

                # Вычисление целевой ориентации
                P_target = np.array(fp.pos) # Точка, на которую должен смотреть маркер


                distance = np.linalg.norm(tvec - P_target)
                if fp.available:
                    if distance < lst_dist:
                        lst_dist = distance
                        target_fp = fp

            if target_fp:
                target_fp.available=False
                robot.follow_point = target_fp

    time.sleep(0.15)
    print("RUN THREADS")
    threads = [
        threading.Thread(target=read_img,name="Read Image"),
        threading.Thread(target=prepocess_img,name="preproc Image"),
        threading.Thread(target=detect_markers,name="detect markers")
    ]
    for i in threads:
        i.start()




def detect_markers():

    _lock = threading.Lock()
    while True:
        time.sleep(0.1)
        start = time.time()
        if len(post_proc_frames)==0:
            continue
        _lock.acquire()
        frame = post_proc_frames.pop()
        _lock.release()

        # Словник маркерів
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # Детекція маркерів
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(frame, dictionary)


        # Оцінка позиції
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                if ids[i][0] not in  robot_dto_dict:
                    continue
                    

                # if i not in marker_pid:
                #     marker_pid[i] = {"linear":PIDController(Kp=0.5, Ki=0.0, Kd=0.1),
                #               "angular":PIDController(Kp=1.0, Ki=0.0, Kd=0.2)}

                # pid_linear = marker_pid[i]["linear"]
                # pid_angular = marker_pid[i]["angular"]


                # Рисуем текущие оси маркера (красные)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)
                
                tvec = tvecs[i][0]
                
                # Вычисление целевой ориентации
                robot = robot_dto_dict[ids[i][0]]
                robot.pos = tvec
                robot.angle = rvecs[i][0]

                P_target =  np.array(robot.follow_point.pos)
                V_up = np.array([0, 1, 0])  # Вектор "вверх"
                
                # Вычисление осей целевой ориентации
                Z_target = P_target - tvec
                Z_target_unit = Z_target / np.linalg.norm(Z_target)
                
                X_target = np.cross(Z_target_unit, V_up)
                X_target_unit = X_target / np.linalg.norm(X_target)
                
                Y_target_unit = np.cross(Z_target_unit, X_target_unit)
                
                # Построение матрицы поворота
                R_target = np.column_stack((X_target_unit, Y_target_unit, Z_target_unit))


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
                robot.deltaPos = DeltaPos()
                robot.deltaPos.linear = distance
                robot.deltaPos.angular = y_axis

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
                
                dist_mm = distance*1000
                cv2.putText(
                    frame,
                    f"dist: {dist_mm:.2f}",   # обрезаем до 2 знаков после запятой
                    tuple(p_target_img[0][0].astype(int) + np.array([15, 100])),     # позиция текста: слева внизу
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 255),
                    2
                )


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
        if cv2.waitKey(1) == ord('q'):
            break


if __name__ == "__main__":
    # threads = [
    #     threading.Thread(target=read_img,name="Read Image"),
    #     threading.Thread(target=prepocess_img,name="preproc Image"),
    #     threading.Thread(target=detect_markers,name="detect markers")
    # ]
    # for i in threads:
    #     i.start()
    init()