#!/usr/bin/env python
"""Integrated WebSocket client with robot motor control."""
import asyncio
import websockets
import json
from client.data.robot_dto import Robot
from client.data.marker import Marker
import time
import client.data.client_state as cs
from client.core.servant_controller import SlaveController
import math
import client.core.kinematic as rk
import numpy as np


# WebSocket URI
# uri = "ws://192.168.0.80:8765"
# uri = "ws://192.168.239.178:8765"
uri = "ws://localhost:8765"

last_theta =0

class RobotClient:
    """Integrated robot client with WebSocket communication and motor control."""
    
    def __init__(self):
        self.controller = SlaveController(kp=0.08, ki=0.0001, kd=1.2, vMode=3)
        self.marker_id = None
        self.target_position = None
        self.target_angle = None
        
    async def init_marker(self, marker: Marker):
        """Initialize marker on server."""
        msg_body = {"robot": {"marker_id": marker.id}}
        await self._send_message("init-marker", msg_body)
        self.marker_id = marker.id
        
    async def update_pos(self):
        """Fetch robot position from server and update control."""
        try:
            async with websockets.connect(uri) as websocket:
                event = {
                    "type": "update-pos",
                    "robot": {
                        "marker_id": cs.marker_id
                    }
                }
                
                await websocket.send(json.dumps(event))
                # print(f"✓ Event send: {event}")
                
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=8.0)
                    msg = json.loads(response)
                    
                    if msg["type"] == "update-pos":
                        if "robot" in msg:
                            robot_state = msg["robot"]
                            pos_world = robot_state["pos_world"]
                            pos_px = robot_state["pos_px"]
                            dir =  robot_state["dir"]
                            target_dir = robot_state["target_dir"]
                            target_pos_px = robot_state.get("target_pos_px")
                            fp = robot_state.get("follow_point_world")
                            
                            # Update robot state
                            cs.robot.update_pos(pos_px, pos_world, dir,target_dir, target_pos_px)
                            if fp:
                                cs.robot.update_fp(fp)
                            
                            # Calculate and apply motor control
                            await self._calculate_and_apply_control(cs.robot)

                            
                except asyncio.TimeoutError:
                    print("Server not respond")
                    
        except ConnectionRefusedError:
            print("Connection refused")
        except Exception as e:
            print("Error: {}".format(e))
    

    async def _calculate_and_apply_control(self, robot: Robot):
        """Calculate motor velocities based on position and target using rotation vectors."""
        if robot.follow_point_world is None:
            self.controller.set_target_velocity(0.0, 0.0)
            self.controller.update()
            return
        
        # Convert inputs to numpy arrays (минимизируем создание массивов)
        theta = np.arctan2(
            np.cross(robot.dir, robot.target_dir)[1],
            np.dot(robot.dir, robot.target_dir)
        )

        # Vector in horizontal plane from robot to follow point
        dx = robot.follow_point_world[0] - robot.pos_world[0]
        dz = robot.follow_point_world[2] - robot.pos_world[2]
        distance = np.sqrt(dx*dx + dz*dz) * 1000  # mm


        errorX = robot.pos_px[0] - robot.target_pos_px[0]
        
        # Check for stopping condition
        if distance < 50.0:  # STOP_THRESHOLD_MMs
            self.controller.set_target_duty(0.0, 0.0)
            self.controller.update()
            return
        
        # Adjust base duty based on distance
        self.controller.base_duty = 30
        if distance > 500.0:
            self.controller.base_duty = 60
        
        if theta < 1e-6:
            current_angle = 0.0


        TURN_THRESHOLD = 0.8  # radians, approx 45 degrees
        
        # Slow down for sharp turns
        if abs(theta) > TURN_THRESHOLD:
            errorX *=0.8
            print("Sharp turn detected, reducing speed")


        self.controller.set_delta_error(errorX)
        self.controller.update()
        
        # debug output
        if distance > 100:  
            print("duty_l={:.0f} duty_r={:.0f} d={:.0f}mm".format(self.controller.duty_l,self.controller.duty_r, distance))

    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    async def get_status(self):
        """Fetch connection status from server."""
        try:
            async with websockets.connect(uri) as websocket:
                event = {"type": "get-status"}
                
                await websocket.send(json.dumps(event))
                print("Event send: {event}".format(event=event))
                
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                    event = json.loads(response)
                    if event["type"] == "get-status":
                        if "value" in event:
                            cs.status = event["value"]
                    print("Status: {cs.status}".format(cs=cs))
                except asyncio.TimeoutError:
                    print("Server not respond")
                    
        except ConnectionRefusedError:
            print("Connection refused")
        except Exception as e:
            print("Error: {}".format(e))
    
    async def _send_message(self, msg_type: str, message: dict):
        """Send message to WebSocket server."""
        try:
            async with websockets.connect(uri) as websocket:
                event = {
                    "type": msg_type,
                    **message
                }
                
                await websocket.send(json.dumps(event))
                print("Event send: {event}".format(event=event))    
                
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                    msg = json.loads(response)
                    if msg["type"] == "init-marker":
                        if "robot" in msg:
                            robot_state = msg["robot"]
                            marker_id = robot_state["marker_id"]
                            cs.robot = Robot(Marker(marker_id))
                            print("Robot initialized withmarker_id={marker_id}".format(marker_id=marker_id))
                            
                except asyncio.TimeoutError:
                    print("Server not respond")
                    
        except ConnectionRefusedError:
            print("Connection refused")
        except Exception as e:
            print("Error : {}".format(e))


async def run_client():
    """Main control loop - calls functions from other modules."""
    print("=== Start client robot ===")
    
    # Initialize robot client with max velocity from kinematic module
    robot_client = RobotClient()
    while cs.status=="start-client":
        await robot_client.get_status()
        await asyncio.sleep(4.0)
    
    # Get marker ID from user
    marker_id = cs.marker_id
    
    # Initialize marker on server (calls Marker class from marker.py)
    while cs.robot is None:
        marker = Marker(marker_id)
        await robot_client.init_marker(marker)
        await asyncio.sleep(2.0)
    
    print("init...")
    time.sleep(6)
    
    # Wait for other clients
    await robot_client.get_status()
    while cs.status == "wait-clients":
        print("wait clients...")
        await robot_client.get_status()
        await asyncio.sleep(2)
    
    
    print("Start movement")
    while True:
        # Calls update_pos which internally:
        # - Updates cs.robot (Robot class from robot_dto.py)
        # - Calls SlaveController methods (from servant_controller.py)
        await robot_client.update_pos()
        await asyncio.sleep(0.2)  # 10 Hz update rate
    
    # Stop motors when done (calls SlaveController methods)
    robot_client.controller.set_target_velocity(0.0, 0.0)
    robot_client.controller.update()
    print("Move end")


