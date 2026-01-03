from server.core.motor_driver import MotorDriver
from typing import List
import time



class RouteNode:
    def __init__(self, lm, rm, delay):
        self.left_motor = lm
        self.right_motor = rm
        self.delay = delay

class MotorController:
    def __init__(self,route:List[RouteNode]):
        self.driver = MotorDriver(v_max=100)  # Example max velocity
        self.route = route
        self.node_ix = 0
        self.paused = False
        self.stopped = False

    def run(self):
        while self.node_ix < len(self.route):

            if self.paused:
                self.driver.stop() 
                while self.paused:
                    print("Paused...")
                    time.sleep(0.1)

            if self.stopped:
                self.driver.stop()
                return False

            self.next_node()
        return True
    

    def next_node(self):

        node = self.route[self.node_ix]
        self.driver.set_velocity(node.left_motor, node.right_motor)
        time.sleep(node.delay)
        self.node_ix += 1

    def stop(self):
        self.driver.stop()



if __name__ == "__main__":
    route_list = [
    RouteNode(50,50,10),
    RouteNode(-50,-50,2),
    RouteNode(-30,50,10),
    RouteNode(0,0,2),
    RouteNode(60,-50,1),
    RouteNode(80,80,10),

]
    controller = MotorController(route_list)
    controller.run()