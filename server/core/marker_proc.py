from turtle import title
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
import data.server_state  as s_state



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

cap = cv2.VideoCapture(0)

def read_img():
    _lock = threading.Lock()
    ix = 0
    while True:
        time.sleep(0.1)
        start = time.time()
        if len(pre_proc_frames)==pre_proc_frames.maxlen:
            _lock.acquire()
            last = pre_proc_frames.pop()
            pre_proc_frames.clear()
            pre_proc_frames.append(last)
            _lock.release()
        else:
            _,frame = cap.read()
            _lock.acquire()

            pre_proc_frames.append(frame)
            _lock.release()
            # print("reading time: ", time.time()-start)
        ix=(ix+1)%3
        


def prepocess_img():
    _lock = threading.Lock()
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
        cap = cv2.VideoCapture(0)
    s_state.camera_is_on = True
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
        
        cv2.imshow("result", frame)
        print(f"Time: {time.time()-start:.4f} сек")
        cv2.waitKey(1)
    cv2.destroyWindow("result")



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
    s_state.robot_new_values = robot_dto_dict
    # threading.Thread(target=read_img,name="Read Image").start()





def detect_markers():
    s_state.status="update-pos"
    print_counter = 0
    # _lock = threading.Lock()
    while True:
        time.sleep(0.1)
        start = time.time()
        _ , frame = cap.read()
        # if len(pre_proc_frames)==0:
        #     continue
        # _lock.acquire()
        # frame = pre_proc_frames.pop()
        # _lock.release()


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
                    
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)
                
                tvec = tvecs[i][0]
                
                # Вычисление целевой ориентации
                robot = robot_dto_dict[ids[i][0]]
                pos = tvec
                

                R_marker, _ = cv2.Rodrigues(rvecs[i])
                P_target = np.array(robot.follow_point.pos)

                # Направление к целевой точке
                direction_to_target = P_target - tvec
                direction_to_target[1] = 0  # Проецируем на горизонтальную плоскость
                direction_to_target = direction_to_target / np.linalg.norm(direction_to_target)

                # Текущее направление маркера (ось Z в системе маркера)
                current_direction = R_marker[:, 2]  # Третий столбец - ось Z
                current_direction[1] = 0  # Проецируем на горизонтальную плоскость
                current_direction = current_direction / np.linalg.norm(current_direction)
                # Convert inputs to numpy arrays (минимизируем создание массивов)
                theta = np.arctan2(
                    np.cross(current_direction, direction_to_target)[1],
                    np.dot(current_direction, direction_to_target)
                )
                # Угол между направлениями

                if print_counter==0:
                    print(f"Rmarker: {R_marker}")

                dx = P_target[0] - pos[0]
                dz = P_target[2] - pos[2]
                distance = np.sqrt(dx*dx + dz*dz) * 1000  # mm

                robot.pos = pos
                robot.dir = current_direction
                robot.target_dir = direction_to_target

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
                

                cv2.putText(
                    frame,
                    f"dist: {distance:.2f}",   # обрезаем до 2 знаков после запятой
                    tuple(p_target_img[0][0].astype(int) + np.array([15, 100])),     # позиция текста: слева внизу
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 255),
                    2
                )


                cv2.putText(
                    frame,
                    f"Rdelta: {theta:.2f}",   # обрезаем до 2 знаков после запятой
                    tuple(p_target_img[0][0].astype(int) + np.array([15, -20])),     # позиция текста: слева внизу
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
                
                print_counter=(print_counter+1)%5

        if print_counter==0:
            cv2.imshow("result", frame)
            cv2.setWindowProperty("result", cv2.WND_PROP_TOPMOST, 1)

            print(f"Time: {time.time()-start:.4f} сек")
        if cv2.waitKey(1) == ord('q'):
            break


def run():
    while s_state.status=="start-server":
        time.sleep(0.5)
    init()
    detect_markers()
