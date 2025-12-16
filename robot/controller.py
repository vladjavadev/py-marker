from robot import kinematic as rk
from robot import motor_driver as rd
from data.grid_dto import GridDto
import time
import math


class Controller:
    speed_mode_max = 5
    speed_mode_min = 1
    unit = 200#mm
    

    def __init__(self, dto:GridDto):
        self.totalDistance = 0
        self.totalTime = 0
        self.dto=dto
        self.unit = dto.get_unit()
        rd.init()

    def get_speed(self, ix: int):
        return rk.speeds[ix-1]
    
    def get_turn_deltaT(self, vL, vR, deg):
        return rk.get_deltaT(vL, vR, deg)

    def clamp_speed(self, value, default=0):
        if value < self.speed_mode_min or value > self.speed_mode_max:
            return default
        return value-1
        

    def calc_fwd_time(self,smode):
        cM = self.clamp_speed(smode)
        return self.unit/rk.speeds[cM]

    def copmute_arc_distance(self,vl,vr,deltaT):
        dOmega, irc = rk.get_robot_turn(vl, vr, deltaT)
        x1, y1 =rk.calcRobotPos(0,0,0,dOmega,irc)
        dist=math.sqrt(x1**2 + y1**2) 
        return dist
    

    def turnLeft(self, speed_mode=1,step=1):
        self.dto._lock_dist.acquire()
        cM = self.clamp_speed(speed_mode)
        vl = rk.speeds[cM]
        turnTime = rk.get_deltaT(vl, 0, 45*step)

        dist = self.copmute_arc_distance(vl,0,turnTime)

        rd.turnLeft(turnTime,cM)
        rd.stop()
        self.totalDistance+=dist
        self.totalTime+=turnTime
        self.dto._lock_dist.release()



    def turnRight(self, speed_mode=1,step=1):
        self.dto._lock_dist.acquire()
        
        cM = self.clamp_speed(speed_mode)
        vr = rk.speeds[cM]
        turnTime = rk.get_deltaT(0, vr, 45*step)

        dist = self.copmute_arc_distance(0,vr,turnTime)
        
        rd.turnRight(turnTime, cM)
        rd.stop()
        self.totalTime+=turnTime
        self.totalDistance+=dist
        self.dto._lock_dist.release()


    def forward(self, speed_mode=1):
        self.dto._lock_dist.acquire()
        cM = self.clamp_speed(speed_mode)
        timeSleep = self.calc_fwd_time(speed_mode)
        rd.forward(cM)
        time.sleep(timeSleep)
        self.totalTime+=timeSleep
        self.totalDistance+=self.unit
        self.dto._lock_dist.release()



    def reverse(self,speed_mode=1):
        self.dto._lock_dist.acquire()
        cM = self.clamp_speed(speed_mode)
        timeSleep = self.calc_fwd_time(speed_mode)
        rd.reverse(cM)
        time.sleep(timeSleep)
        self.totalTime+=timeSleep
        self.totalDistance+=self.unit
        self.dto._lock_dist.release()



    def stop(self):
        rd.stop()
        print("RD stop")