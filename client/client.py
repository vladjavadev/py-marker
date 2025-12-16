from pid_controller import PIDController
import client_websocket as cw

marker_pid = {}
def update_pos():

    for robot in cw.servant_dict:
        id = robot.marker.id
        if id not in marker_pid:
            marker_pid[id] = {"linear":PIDController(Kp=0.5, Ki=0.0, Kd=0.1),
                      "angular":PIDController(Kp=1.0, Ki=0.0, Kd=0.2)}

        pid_linear = marker_pid[id]["linear"]
        pid_angular = marker_pid[id]["angular"]
        
        linear_error = pid_linear.update(robot.deltaPos.linear)
        angular_error = pid_angular.update(robot.deltaPos.angular)
