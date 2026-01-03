from client.data.marker import Marker


class Robot:
    def __init__(self,marker:Marker):
        self.marker = marker
        self.pos_px=[]
        self.target_pos_px=[]

        self.dir=[]
        self.target_dir=[]

        self.pos_world=[]
        self.follow_point_world = (0,0,0.2)


    def update_pos(self, pos_px, pos_world, dir, target_dir, target_pos_px):
            self.pos_world = pos_world
            self.dir = dir
            self.target_dir = target_dir
            self.pos_px = pos_px
            self.target_pos_px = target_pos_px

    def update_fp(self, fp_world):
        self.follow_point_world = fp_world
            