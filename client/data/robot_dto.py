from data.marker import Marker



class Robot:
    def __init__(self,marker:Marker):
        self.marker = marker
        self.pos=[]
        self.dir=[]
        self.target_dir=[]
        self.follow_point = (0,0,0.2)


    def update_pos(self, pos, dir,target_dir):
            self.pos = pos
            self.dir = dir
            self.target_dir = target_dir

    def update_fp(self, fp):
        self.follow_point = fp        
            