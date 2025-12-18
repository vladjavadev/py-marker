#!/usr/bin/env python
"""Integrated WebSocket client with robot motor control."""
import asyncio
from websockets.asyncio.client import connect
import json
from data.robot_dto import Robot
from data.marker import Marker
import time
import data.client_state as cs
from core.servant_controller import SlaveController
import math
import core.kinematic as rk

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
            async with connect(uri) as websocket:
                event = {
                    "type": "update-pos",
                    "robot": {
                        "marker_id": cs.marker_id
                    }
                }
                
                await websocket.send(json.dumps(event))
                print(f"✓ Событие отправлено: {event}")
                
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
                            
                            print(f"Позиция обновлена: {pos}, угол: {angle}")
                            
                except asyncio.TimeoutError:
                    print("Сервер не ответил в течение 8 секунд")
                    
        except ConnectionRefusedError:
            print("❌ Не удалось подключиться к серверу")
        except Exception as e:
            print(f"❌ Ошибка: {e}")
    
    async def _calculate_and_apply_control(self, pos, angle, follow_point):
        """Calculate motor velocities based on position and target."""
        if follow_point is None:
            # Stop if no target
            self.controller.set_target_velocity(0.0, 0.0)
            self.controller.update()
            return
        angle_y = angle[1]
        # Calculate distance and angle to target
        dx = follow_point[0] - pos[0]
        dz = follow_point[2] - pos[2]
        distance = math.sqrt(dx**2 + dz**2)*1000
        target_angle = math.atan2(dx,dz)
        
        # Calculate angle error
        angle_error = self._normalize_angle(target_angle - angle_y)
        
        # Simple control law
        if distance < 20:  # Stop threshold (20mm)
            linear_v = 0.0
            angular_omega = 0.0
        else:
            # Linear velocity proportional to distance
            linear_v = min(distance * 2.0, self.controller.v_max)
            
            # Angular velocity proportional to angle error
            angular_omega = angle_error * 2.0  # gain factor
            
            # Reduce linear velocity when turning
            if abs(angle_error) > 0.5:  # ~30 degrees
                linear_v *= 0.5
        
        # Apply velocities
        self.controller.set_target_velocity(linear_v, angular_omega)
        self.controller.update()
        
        print(f"Управление: v={linear_v:.1f} mm/s, ω={angular_omega:.2f} rad/s")
    
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
            async with connect(uri) as websocket:
                event = {"type": "get-status"}
                
                await websocket.send(json.dumps(event))
                print(f"✓ Событие отправлено: {event}")
                
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                    event = json.loads(response)
                    if event["type"] == "get-status":
                        if "value" in event:
                            cs.status = event["value"]
                    print(f"Статус: {cs.status}")
                except asyncio.TimeoutError:
                    print("Сервер не ответил в течение 5 секунд")
                    
        except ConnectionRefusedError:
            print("❌ Не удалось подключиться к серверу")
        except Exception as e:
            print(f"❌ Ошибка: {e}")
    
    async def _send_message(self, msg_type: str, message: dict):
        """Send message to WebSocket server."""
        try:
            async with connect(uri) as websocket:
                event = {
                    "type": msg_type,
                    **message
                }
                
                await websocket.send(json.dumps(event))
                print(f"✓ Событие отправлено: {event}")
                
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                    msg = json.loads(response)
                    if msg["type"] == "init-marker":
                        if "robot" in msg:
                            robot_state = msg["robot"]
                            marker_id = robot_state["marker_id"]
                            cs.robot = Robot(Marker(marker_id))
                            print(f"Робот инициализирован с marker_id={marker_id}")
                            
                except asyncio.TimeoutError:
                    print("Сервер не ответил в течение 5 секунд")
                    
        except ConnectionRefusedError:
            print("❌ Не удалось подключиться к серверу")
        except Exception as e:
            print(f"❌ Ошибка: {e}")


async def main():
    """Main control loop - calls functions from other modules."""
    print("=== Запуск интегрированного клиента робота ===")
    
    # Initialize robot client with max velocity from kinematic module
    robot_client = RobotClient(v_max=rk.speeds[-1])
    while cs.status=="start-client":
        await robot_client.get_status()
        time.sleep(0.1)
    
    # Get marker ID from user
    marker_id = cs.marker_id
    
    # Initialize marker on server (calls Marker class from marker.py)
    marker = Marker(marker_id)
    await robot_client.init_marker(marker)
    
    print("Ожидание инициализации...")
    time.sleep(6)
    
    # Wait for other clients
    await robot_client.get_status()
    while cs.status == "wait-clients":
        print("Ожидание других клиентов...")
        await robot_client.get_status()
        await asyncio.sleep(2)
    
    
    print("Все клиенты подключены! Начинаем управление.")
    
    # Main control loop
    while cs.status == "update-pos":
        # Calls update_pos which internally:
        # - Updates cs.robot (Robot class from robot_dto.py)
        # - Calls SlaveController methods (from servant_controller.py)
        await robot_client.update_pos()
        await asyncio.sleep(0.1)  # 10 Hz update rate
    
    # Stop motors when done (calls SlaveController methods)
    robot_client.controller.set_target_velocity(0.0, 0.0)
    robot_client.controller.update()
    print("Управление завершено.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n⚠️ Программа прервана пользователем")
    except Exception as e:
        print(f"\n\n❌ Критическая ошибка: {e}")