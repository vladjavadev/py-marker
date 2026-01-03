from server.data.marker import Marker
from server.data.follow_point import FollowPoint as fp


class Robot:
    def __init__(self,marker:Marker):
        self.marker = marker
        self.pos_px=[]
        self.target_pos_px=[]

        self.dir=[]
        self.target_dir=[]

        self.pos_world=[]
        self.follow_point_world = None

