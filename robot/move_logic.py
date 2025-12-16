# from robot import mock_controller as control
from robot import controller as control
from data.grid_dto import GridDto
import time


DIRECTIONS = [
    (0, 1),    # 0: N
    (1, 1),    # 1: NE
    (1, 0),    # 2: E
    (1, -1),   # 3: SE
    (0, -1),   # 4: S
    (-1, -1),  # 5: SW
    (-1, 0),   # 6: W
    (-1, 1)    # 7: NW
]






class Logic:
    def __init__(self, pos=(0, 0), dir=(0, 1), vMode = 1):
        self.dir = dir
        self.pos = pos
        self.vMode = vMode
        self.moveStep=0
        self.rotateStep=0

    def __init_controller(self):
        self.mk_control = control.Controller(self.dto)

    def init_dto(self,dto:GridDto):
        self.dto = dto
        self.__init_controller()

    def get_dir(self, pos, new_pos):
        delta_x = new_pos[0]-pos[0]
        delta_y = new_pos[1] - pos[1]
        norm_dir = self.normalize_dir((delta_x, delta_y))
        # print(f"   get_dir: delta=({norm_dir[0]}, {norm_dir[1]})")
        return norm_dir
    
    def normalize_dir(self, vector):
        norm_x = 0 if vector[0] == 0 else (1 if vector[0] > 0 else -1)
        norm_y = 0 if vector[1] == 0 else (1 if vector[1] > 0 else -1)
        return (norm_x,norm_y)

    def get_dir_ix(self, vector):
        for i, dir_vec in enumerate(DIRECTIONS):
            if vector == dir_vec:
                return i
            
    def turns_needed(self, start_vec, target_vec):
        if target_vec==start_vec or target_vec==(0,0) or start_vec==(0,0):
            return (0,"straight")
        start_idx = self.get_dir_ix(start_vec)
        target_idx = self.get_dir_ix(target_vec)

        spinL = (target_idx - start_idx) % 8
        spinR = (start_idx - target_idx) % 8
        turns = (spinR, "right") if spinR <= spinL else (spinL, "left")

        return turns

    def update_dir_pos(self, new_pos, new_dir): 
        self.pos = new_pos 
        self.dir = new_dir
    
    def move_robot(self, turns, new_pos):
            if turns[0] > 0:
                self.rotateStep+=1
                if turns[1] == "right":
                    self.mk_control.stop()
                    # print("Turn right: ",turns[0])
                    self.mk_control.turnRight(self.vMode,turns[0])
                elif turns[1] == "left":
                    self.mk_control.stop()
                    # print("Turn left: ",turns[0])
                    self.mk_control.turnLeft(self.vMode,turns[0])
            self.moveStep+=1
            self.mk_control.forward(self.vMode)
            # print(f"@@@@ MOveSteps:{self.moveStep} ... RotateStep:{self.rotateStep}")

    def stop(self):
        self.mk_control.stop()  

    def predict_time_distance(self, path):
        total_time = 0
        total_distance = 0
        current_dir = self.dir
        current_pos = self.pos
        speed = self.mk_control.get_speed(self.vMode)
        if not path:
            return (0, 0)
        for next_pos in path[1:]:
            next_dir = self.get_dir(current_pos, next_pos)
            turns = self.turns_needed(current_dir, next_dir)

            # Estimate time for turns
            if turns[0] > 0:
                turn_time = self.mk_control.get_turn_deltaT(0, speed, 45 * turns[0])
                turn_dist = self.mk_control.copmute_arc_distance(0, speed, turn_time)
                total_time += turn_time
                total_distance += turn_dist

            # Estimate time for forward movement
            forward_time = self.mk_control.calc_fwd_time(self.vMode)
            total_time += forward_time
            total_distance += self.mk_control.unit

            # Update current position and direction
            current_pos = next_pos
            current_dir = next_dir

        return (total_time, total_distance)

    def build_route(self,new_pos):
        start_route_procc = time.time()
        pos = self.dto.get_position()
        if pos == new_pos:
            return
        new_dir = self.get_dir(pos,new_pos)
        turns = self.turns_needed(self.dir,new_dir)
        time_route_procc = time.time() - start_route_procc 
        start_move = time.time()
        self.move_robot(turns,new_pos)
        time_move = time.time() - start_move 
        self.dto.set_distance(self.mk_control.totalDistance)
        self.update_dir_pos(new_pos,new_dir)
        return abs(time_move-time_route_procc)
        



    



    





