import cv2
import cv2.aruco as aruco
import numpy as np
import time
from collections import deque
import threading


pre_proc_frames = deque(maxlen=3)
post_proc_frames = deque(maxlen=3)

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
            frame = cv2.imread(f"img/image{ix+1}.png")
            pre_proc_frames.append(frame)
            print("reading time: ", time.time()-start)
        ix=(ix+1)%4
        


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
            frame = cv2.resize(frame, (320, 240))
            frame = frame[0:120, :]
            post_proc_frames.append(frame)
            print("preproc time: ", time.time()-start)

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

        # Параметри камери (треба калібрувати для точності!)
        # Тут приклад з умовними значеннями
        camera_matrix = np.array([[1000, 0, frame.shape[1]/2],
                                [0, 1000, frame.shape[0]/2],
                                [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((5,1))  # якщо немає калібрування

        # Оцінка позиції
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
            for i in range(len(ids)):
                # print(f"Маркер {ids[i]}:")
                # print("  rvec:", rvecs[i])  # вектор обертання
                # print("  tvec:", tvecs[i])  # вектор трансляції

                # Малюємо координатні осі
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)
            for rvec in rvecs:
                R, _ = cv2.Rodrigues(rvec)

                # print("Rotation matrix:\n", R)

                # Применим к точке в системе маркера
                point_marker = np.array([0, 0, 1])  # точка на оси Z маркера
                point_camera = R @ point_marker
                # print("Point in camera coords:", point_camera)


        cv2.imshow("Marker_result", frame)
        print("Detect marker Time: ",time.time()-start)


if __name__ == "__main__":
    threads = [
        threading.Thread(target=read_img,name="Read Image"),
        threading.Thread(target=prepocess_img,name="preproc Image"),
        threading.Thread(target=detect_markers,name="detect markers")
    ]
    for i in threads:
        i.start()