from server.data.marker import Marker
from server.data.follow_point import FollowPoint as fp
import json


class Robot:
    def __init__(self,marker:Marker):
        self.marker = marker
        self.pos_px=[]
        self.target_pos_px=[]

        self.dir=[]
        self.target_dir=[]

        self.pos_world=[]
        self.follow_point_world = None

    def __repr__(self):
        return (f"Robot("
                f"marker={self.marker.id}, "
                f"pos_px={self.pos_px}, "
                f"target_pos_px={self.target_pos_px}, "
                f"dir={self.dir}, "
                f"target_dir={self.target_dir}, "
                f"pos_world={self.pos_world}, "
                f"follow_point_world={self.follow_point_world.pos: if self.follow_point_world else None}"
                f")")
    
    def to_dict(self):
        return {
            "marker_id": str(self.marker.id),  # or marker.id if it has an ID
            "pos_px": self.pos_px,
            "target_pos_px": self.target_pos_px,
            "dir": self.dir,
            "target_dir": self.target_dir,
            "pos_world": self.pos_world,
            "follow_point_world": str(self.follow_point_world.pos) if self.follow_point_world else None
        }

    def to_json(self):
        return json.dumps(self.to_dict(), indent=4)
