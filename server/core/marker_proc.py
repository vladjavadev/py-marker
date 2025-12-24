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
import data.server_state as s_state


pre_proc_frames = deque(maxlen=3)
post_proc_frames = deque(maxlen=3)
robot_dto_dict = {}

marker_pid = {}
follow_points = [
    fp([-0.05,0.0,0.2]),
    fp([0.0,0.0,0.2]),
    fp([0.05,0.0,0.2]),
]



start = time.time()
dt = 0.05



try:
    # load camera params from file
    calibration_data = np.load('server/data/camera_params.npz')
    
    # parse data
    camera_matrix = calibration_data['mtx']
    dist_coeffs = calibration_data['dist']
    
    print("calibrate params loads successfully")
    
except FileNotFoundError:
    print("Error: calibrate params file not found")

    exit() 

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
            _, frame = cap.read()
            # frame = cv2.resize(frame, (320, 240))
            # frame = frame[0:120, :]
            pre_proc_frames.append(frame)
            print("reading time: ", time.time()-start)
        ix=(ix+1)%3
        


# def prepocess_img():
#     while True:
#         time.sleep(0.1)
#         start = time.time()
#         if len(post_proc_frames)==post_proc_frames.maxlen:
#             last = post_proc_frames.pop()
#             post_proc_frames.clear()
#             post_proc_frames.append(last)
#         if len(pre_proc_frames)>0:
#             frame = pre_proc_frames.pop()

#             post_proc_frames.append(frame)
       
#             print("preproc time: ", time.time()-start)


def wait_client(id):
    s_state.status="wait-clients"
    count = 0
    max_iteration = 20
    while count<max_iteration:
        if id in s_state.connected_clients:
            return True
        time.sleep(1)
        count+=1

    return False
    
    
def init():
    global cap
    if not cap.isOpened():
        print("Camera is not opened!")
        cap.release()
        cap = cv2.VideoCapture(0)
        return
    
    s_state.camera_is_on = True
    ids = []
    corners = None
    while ids is None or len(ids)==0:
        _, frame = cap.read()


        # marker dictionary
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(frame, dictionary)



    # Estimate pos
    if ids is not None:
        _, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        for i in range(len(ids)):
            id = ids[i][0]
            is_connected = wait_client(id)
            if not is_connected:
                continue

            if id not in robot_dto_dict:
                marker = Marker(id)
                robot_dto_dict[id] = Robot(marker=marker)

            robot = robot_dto_dict[id]

            tvec = tvecs[i][0]
            lst_dist = float("inf")
            target_fp = None
            for fp in follow_points:

                # Current pos
                P_target = np.array(fp.pos) 


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
            # threading.Thread(target=prepocess_img,name="preproc Image"),
            threading.Thread(target=detect_markers,name="detect markers")
        ]
        for i in threads:
            i.start()




def detect_markers():
    s_state.status="update-pos"
    _lock = threading.Lock()
    while True:
        time.sleep(0.1)
        start = time.time()
        if len(post_proc_frames)==0:
            continue
        _lock.acquire()
        frame = post_proc_frames.pop()
        _lock.release()


        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)


        corners, ids, rejected = aruco.detectMarkers(frame, dictionary)


        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                if ids[i][0] not in  robot_dto_dict:
                    continue
                    

                # draw marker axis
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)
                
                tvec = tvecs[i][0]
                
                # calculate current orientation
                robot = robot_dto_dict[ids[i][0]]
                robot.pos = tvec
                robot.angle = rvecs[i][0]

                P_target =  np.array(robot.follow_point.pos)
                V_up = np.array([0, 1, 0]) #up vector
                
                #calc current orientation axis
                Z_target = P_target - tvec
                Z_target_unit = Z_target / np.linalg.norm(Z_target)
                
                X_target = np.cross(Z_target_unit, V_up)
                X_target_unit = X_target / np.linalg.norm(X_target)
                
                Y_target_unit = np.cross(Z_target_unit, X_target_unit)
                
                # build rotate matrix
                R_target = np.column_stack((X_target_unit, Y_target_unit, Z_target_unit))



                R_marker, _ = cv2.Rodrigues(rvecs[i])
                R_camera = R_target
                # print(f"Rmarker: {R_marker}")
                # print(f"Rcamera: {R_camera}")
                R_delta = R_camera @ R_marker.T

                #calc rotation angle theta
                #np trace - sum of diagonal elements matrix
                theta  = np.arccos((np.trace(R_delta)-1)/2)

                y_axis  = R_delta[0,2]-R_delta[2,0]/(2*np.sin(theta))

                # print(f"Rdelta: {y_axis}")

                distance = np.linalg.norm(P_target - tvec)

                # print(f"distances: {distance}")
                robot.deltaPos = DeltaPos()
                robot.deltaPos.linear = distance
                robot.deltaPos.angular = y_axis

                _lock.acquire()
                s_state.robot_new_values[marker_id] = robot
                _lock.release()

        #         target_img_point, _ = cv2.projectPoints(
        #             P_target.reshape(1, 3), 
        #             np.zeros((3,1)), 
        #             np.zeros((3,1)), 
        #             camera_matrix, 
        #             dist_coeffs
        #         )
                
        #         cv2.circle(frame, 
        #                 tuple(target_img_point[0][0].astype(int)), 
        #                 10, (255, 255, 0), -1)  
        #         cv2.putText(frame, "FOLLOW TARGET", 
        #                 tuple(target_img_point[0][0].astype(int) + np.array([15, -10])),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                

        #         p_target_img, _ = cv2.projectPoints(
        #             P_target.reshape(1, 3), 
        #             np.zeros((3,1)), 
        #             np.zeros((3,1)), 
        #             camera_matrix, 
        #             dist_coeffs
        #         )
                
        #         cv2.circle(frame, 
        #                 tuple(p_target_img[0][0].astype(int)), 
        #                 8, (255, 0, 255), -1)  # Фиолетовая точка
        #         cv2.putText(frame, "ORIENT TARGET", 
        #                 tuple(p_target_img[0][0].astype(int) + np.array([15, 10])),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                
        #         dist_mm = distance*1000
        #         cv2.putText(
        #             frame,
        #             f"dist: {dist_mm:.2f}",   # обрезаем до 2 знаков после запятой
        #             tuple(p_target_img[0][0].astype(int) + np.array([15, 100])),     # позиция текста: слева внизу
        #             cv2.FONT_HERSHEY_SIMPLEX,
        #             0.7,
        #             (255, 0, 255),
        #             2
        #         )


        #         # arrow slave robot to follow point
        #         follower_img, _ = cv2.projectPoints(
        #             tvecs[i][0].reshape(1, 3), 
        #             np.zeros((3,1)), 
        #             np.zeros((3,1)), 
        #             camera_matrix, 
        #             dist_coeffs
        #         )
        #         cv2.arrowedLine(frame, 
        #                     tuple(follower_img[0][0].astype(int)),
        #                     tuple(target_img_point[0][0].astype(int)),
        #                     (0, 255, 255), 2, tipLength=0.3)
                
        #         # show id on image
        #         cv2.putText(frame, f"ID: {ids[i][0]}", 
        #                 tuple(follower_img[0][0].astype(int) + np.array([-50, -15])),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # cv2.imshow("result", frame)
        print(f"Time: {time.time()-start:.4f} сек")
        if cv2.waitKey(1) == ord('q'):
            break


if __name__ == "__main__":

    init()