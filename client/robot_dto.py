from marker import Marker

class DeltaPos:
    def __init__(self):
        self.linear = []
        self.angular = []


class Robot:
    def __init__(self,marker:Marker):
        self.marker = marker
        self.pos=[]
        self.follow_point = (0,0,0.2)
        self.angle=[]
        self.deltaPos=DeltaPos()

    def update_fp(self, follow_point):
        self.follow_point = follow_point

    def update_pos(self, pos, angle):
        self.pos = pos
        self.angle = angle

    def update_delta(self, linear, angular):
        self.deltaPos.linear = linear
        self.deltaPos.angular = angular