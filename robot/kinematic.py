import numpy as np
import math



rps = [0.7,0.93,0.99,1.03,1.06]
wheelRadius = 30 #mm
speeds =[2*3.14*wheelRadius*i for i in rps ]
LwheelBase = 150 #mm

# kdc = 0.8



def get_deltaT(vL, vR, deg):
    omega = (vR - vL)/LwheelBase
    turnRad = math.radians(deg)
    absOmega = abs(omega)
    deltaT = turnRad/absOmega
    return deltaT

def get_robot_turn(vL, vR, deltaT):
    irc= (vL+vR)*LwheelBase/(2*(vR - vL))
    omega = (vR - vL)/LwheelBase
    dOmega = omega*deltaT
    return dOmega, irc


def calcRobotPos(x, y,curTeta, dOmega, irc):
    x0 = x - irc*math.sin(curTeta)
    y0 = y + irc*math.cos(curTeta)

    irc_v = np.array([[int(x0)],[int(y0)]])
    P_old = np.array([[int(x)],[int(y)]])
    P_offset =  P_old - irc_v

    cos_dt = math.cos(dOmega)
    sin_dt = math.sin(dOmega)
    R_dt = np.array([[cos_dt, -sin_dt], [sin_dt, cos_dt]])
    P_new = np.dot(R_dt, P_offset) + irc_v

    x1 = P_new[0][0]
    y1 = P_new[1][0]
    return x1, y1

