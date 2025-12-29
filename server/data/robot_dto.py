from data.marker import Marker
from data.follow_point import FollowPoint as fp

class DeltaPos:
    def __init__(self):
        self.linear = []
        self.angular = []


class Robot:
    def __init__(self,marker:Marker):
        self.marker = marker
        self.pos=[]
        self.follow_point = fp((0,0,0.2))
        self.dir=[]
        self.target_dir=[]


