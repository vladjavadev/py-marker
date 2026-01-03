from turtle import title
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from collections import deque
import threading
from server.data.robot_dto import Robot
from server.data.follow_point import FollowPoint 
from server.data.marker import Marker
import server.data.server_state  as s_state
from typing import List


COUNT_FP = 2
pre_proc_frames = deque(maxlen=3)
post_proc_frames = deque(maxlen=3)
robot_dto_dict: dict[int, Robot] = {}

fp_tpos_dict = {}



# Основной код
start = time.time()
dt = 0.05



try:
    # Завантажуємо дані з файлу, створеного скриптом калібрування
    calibration_data = np.load('server/data/camera_params.npz')
    
    # Витягуємо матрицю камери та коефіцієнти спотворення
    camera_matrix = calibration_data['mtx']
    dist_coeffs = calibration_data['dist']
    
    print("Calibration camera parameters loaded successfully!")
    
except FileNotFoundError:
    print("ERROR: File 'camera_params.npz' not found.")
    print("Ensure you have performed calibration and saved the file in the same directory.")
    # Тут можна залишити умовні значення, але оцінка позиції буде неточною
    # camera_matrix = ...
    # dist_coeffs = ...
    exit() # Або завершити програму, якщо калібрування критично важливе

cap = cv2.VideoCapture(0)
_lock = threading.Lock()
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)


def read_img():
    ix = 0
    global _lock
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

    #normalize pixel target points to world coordinates    
    _, frame = cap.read()
    fh, fw = frame.shape[:2]
    padding = fw//3.5
    cx_pixel_list = np.linspace(padding, fw-padding, COUNT_FP)
    cy_pixel = fh / 2

    K_inv = np.linalg.inv(camera_matrix)
    for cx in cx_pixel_list:
        pixel_homog = np.array([cx, cy_pixel, 1.0])
        ray_camera = K_inv @ pixel_homog
        scale = 0.2 / ray_camera[2] 
        world_point = ray_camera * scale
        new_fp = FollowPoint(world_point.tolist())

        fp_tpos_dict[new_fp] = (cx, cy_pixel)


    s_state.camera_is_on = True

    #wait until at least 3 markers detected
    while len(ids)<3:
        _, frame = cap.read()

        corners, ids, rejected = aruco.detectMarkers(frame, ARUCO_DICT)
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

            #set closest follow point to robot
            for fp in fp_tpos_dict:

                P_target = np.array(fp.pos) 


                distance = np.linalg.norm(tvec - P_target)
                if fp.available:
                    if distance < lst_dist:
                        lst_dist = distance
                        target_fp = fp

            if target_fp:
                target_fp.available=False
                robot.follow_point_world = target_fp
                robot.target_pos_px = fp_tpos_dict[target_fp]

    time.sleep(0.15)
    #added detected robots to server state
    s_state.robot_new_values = robot_dto_dict
    # threading.Thread(target=read_img,name="Read Image").start()



def calculate_angle_and_distance(R_marker, tvec, P_target):
    """
    Calculates the angle of rotation and distance to the target point
    
    Fixes:
    1. Using copies of vectors instead of modifying originals
    2. Checking for zero vectors before normalization
    3. More robust calculation
    
    Args:
        R_marker: rotation matrix of the marker (3x3)
        tvec: marker position [x, y, z]
        P_target: target point [x, y, z]

    Returns:
        theta: angle in radians
        distance: distance in mm
        current_direction: normalized direction of the marker (XZ)
        target_direction: normalized direction to the target (XZ)
    """
    
    # direction vector to the target point
    direction_to_target = P_target - tvec
    direction_to_target_xz = np.array([direction_to_target[0], 0.0, direction_to_target[2]])
    
    # current forward direction of the marker
    marker_forward = R_marker[:, 2]  # Третий столбец - ось Z
    current_direction_xz = np.array([marker_forward[0], 0.0, marker_forward[2]])
    
    #check for zero vectors and normalize
    norm_current = np.linalg.norm(current_direction_xz)
    norm_target = np.linalg.norm(direction_to_target_xz)
    
    if norm_current < 1e-6 or norm_target < 1e-6:
        # if vector is too small, return zero angle and distance
        return 0.0, 0.0, np.array([0, 0, 1]), np.array([0, 0, 1])
    
    current_direction_xz = current_direction_xz / norm_current
    direction_to_target_xz = direction_to_target_xz / norm_target
    
    # calculate angle using arctan2 for better stability
    theta = np.arctan2(
        np.cross(current_direction_xz, direction_to_target_xz)[1],
        np.dot(current_direction_xz, direction_to_target_xz)
    )
    
    # distance to target in xz plane
    dx = P_target[0] - tvec[0]
    dz = P_target[2] - tvec[2]
    distance = np.sqrt(dx*dx + dz*dz) * 1000  # mm
    
    return theta, distance, current_direction_xz, direction_to_target_xz



def detect_markers():
    s_state.status="update-pos"
    print_counter = 0
    #run detection loop
    while True:
        time.sleep(0.1)
        start = time.time()
        ret , frame = cap.read()
        if not ret:
            continue

        corners, ids, rejected = aruco.detectMarkers(frame, ARUCO_DICT)


        # estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                if ids[i][0] not in  robot_dto_dict:
                    continue
                    
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)
                
                tvec = tvecs[i][0]
                marker_corners = corners[i][0]

                # calculation current position in pixels
                cX = int(sum([corner[0] for corner in marker_corners]) / 4)
                cY = int(sum([corner[1] for corner in marker_corners]) / 4)
                current_robot_pos_px = (cX, cY)

                robot = robot_dto_dict[ids[i][0]]
                if ids[i][0]==3:
                    print(">> id#3_pos:",tvec)
                # Rotation matrix from rotation vector
                R_marker, _ = cv2.Rodrigues(rvecs[i])
                

                if robot.follow_point_world is None:
                    continue

                P_target = np.array(robot.follow_point_world.pos) # Точка, на которую должен смотреть маркер

                target_pos_px = robot.target_pos_px
                error_x = target_pos_px[0] - current_robot_pos_px[0]
                # direction vector to the target point
                theta, distance, current_direction, direction_to_target = \
                    calculate_angle_and_distance(R_marker, tvec, P_target)
                
                # Update robot DTO
                robot.pos_world = tvec.tolist()
                robot.pos_px = current_robot_pos_px
                robot.dir = current_direction.tolist()
                robot.target_dir = direction_to_target.tolist()
                
                #output visualization
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
                    f"error_x: {error_x:.2f}",   # обрезаем до 2 знаков после запятой
                    tuple(p_target_img[0][0].astype(int) + np.array([15, -50])),     # позиция текста: слева внизу
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
