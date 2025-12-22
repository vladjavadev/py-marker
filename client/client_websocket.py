#!/usr/bin/env python
"""Integrated WebSocket client with robot motor control."""
import asyncio
import websockets
import json
from data.robot_dto import Robot
from data.marker import Marker
import time
import data.client_state as cs
from core.servant_controller import SlaveController
import math
import core.kinematic as rk
import numpy as np

# Import kinematic functions from robot.kinematic module


# WebSocket URI
uri = "ws://localhost:8765"


class RobotClient:
    """Integrated robot client with WebSocket communication and motor control."""
    
    def __init__(self, v_max: float = 200):
        self.controller = SlaveController(v_max=v_max)
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
                            pos = robot_state["pos"]
                            angle = robot_state["angle"]
                            fp = robot_state.get("follow_point")
                            
                            # Update robot state
                            cs.robot.update_pos(pos, angle)
                            if fp:
                                cs.robot.update_fp(fp)
                            
                            # Calculate and apply motor control
                            await self._calculate_and_apply_control(pos, angle, fp)
                            
                            # print(f"New pos: {pos}, angle: {angle}")
                            
                except asyncio.TimeoutError:
                    print("Server not respond")
                    
        except ConnectionRefusedError:
            print("Connection refused")
        except Exception as e:
            print("Error: {}".format(e))
    

    async def _calculate_and_apply_control(self, pos, angle, follow_point):
        """Calculate motor velocities based on position and target using rotation vectors."""
        if follow_point is None:
            self.controller.set_target_velocity(0.0, 0.0)
            self.controller.update()
            return
        
        # Convert inputs to numpy arrays (минимизируем создание массивов)
        pos = np.asarray(pos, dtype=np.float32)
        follow_point = np.asarray(follow_point, dtype=np.float32)
        rvec = np.asarray(angle, dtype=np.float32)
        
        # --- УПРОЩЕННЫЙ ПОДХОД ДЛЯ 2D РОБОТА ---
        # Вместо полных 3D матриц используем 2D геометрию
        
        # Вектор к цели в горизонтальной плоскости
        dx = follow_point[0] - pos[0]
        dz = follow_point[2] - pos[2]
        distance = np.sqrt(dx*dx + dz*dz) * 1000  # mm
        
        # Проверка близости к цели
        if distance < 20.0:  # STOP_THRESHOLD_MM
            self.controller.set_target_velocity(0.0, 0.0)
            self.controller.update()
            return
        
        # Целевой угол (atan2 быстрее матричных операций)
        target_angle = np.arctan2(dx, dz)
        
        # Текущий угол робота из rotation vector
        # Для 2D достаточно Y-компоненты rvec
        theta = np.linalg.norm(rvec)
        
        if theta < 1e-6:
            current_angle = 0.0
        else:
            # Упрощенное извлечение угла поворота вокруг Y
            # Для малых углов: угол ≈ rvec[1]
            current_angle = rvec[1] if theta < 0.1 else np.arctan2(rvec[0], rvec[2])
        
        # Разница углов (нормализованная)
        angle_diff = target_angle - current_angle
        # Нормализация в [-π, π]
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        
        # --- УПРАВЛЕНИЕ ---
        LINEAR_GAIN = 2.0
        ANGULAR_GAIN = 2.0
        TURN_THRESHOLD = 0.5  # ~30 градусов
        
        # Угловая скорость
        angular_omega = angle_diff * ANGULAR_GAIN
        
        # Линейная скорость с ограничением
        linear_v = min(distance * LINEAR_GAIN, self.controller.v_max)
        
        # Замедление при резких поворотах
        if abs(angle_diff) > TURN_THRESHOLD:
            linear_v *= 0.5
        
        # Применение управления
        self.controller.set_target_velocity(float(linear_v), float(angular_omega))
        self.controller.update()
        
        # Упрощенный вывод (print может быть узким местом)
        if distance > 50:  # Печатаем только если далеко от цели
            print("v={:.0f} ω={:.2f} d={:.0f}mm".format(linear_v, angular_omega, distance))

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
                            print("Робот инициализирован с marker_id={marker_id}".format(marker_id=marker_id))
                            
                except asyncio.TimeoutError:
                    print("Server not respond")
                    
        except ConnectionRefusedError:
            print("Connection refused")
        except Exception as e:
            print("Error : {}".format(e))


async def main():
    """Main control loop - calls functions from other modules."""
    print("=== Start client robot ===")
    
    # Initialize robot client with max velocity from kinematic module
    robot_client = RobotClient(v_max=rk.speeds[-1])
    while cs.status=="start-client":
        await robot_client.get_status()
        await asyncio.sleep(0.1)
    
    # Get marker ID from user
    marker_id = cs.marker_id
    
    # Initialize marker on server (calls Marker class from marker.py)
    marker = Marker(marker_id)
    await robot_client.init_marker(marker)
    
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
        await asyncio.sleep(0.1)  # 10 Hz update rate
    
    # Stop motors when done (calls SlaveController methods)
    robot_client.controller.set_target_velocity(0.0, 0.0)
    robot_client.controller.update()
    print("Move end")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n Program interrupted by user")
    except Exception as e:
        print("\n\nCritical error: {}".format(e))