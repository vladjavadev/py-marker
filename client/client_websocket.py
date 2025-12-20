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
        """Calculate motor velocities based on position and target using rotation vectors."""
        if follow_point is None:
            # Stop if no target
            self.controller.set_target_velocity(0.0, 0.0)
            self.controller.update()
            return
        
        # Convert inputs to numpy arrays
        pos = np.array(pos)
        follow_point = np.array(follow_point)
        rvec = np.array(angle)  # angle is the rotation vector (rvec)
        
        # Calculate target direction vector
        P_target = follow_point
        V_up = np.array([0, 1, 0])  # Up vector
        
        # Calculate target orientation axes
        Z_target = P_target - pos
        Z_target_norm = np.linalg.norm(Z_target)
        
        # Check if target is too close (avoid division by zero)
        if Z_target_norm < 1e-6:
            self.controller.set_target_velocity(0.0, 0.0)
            self.controller.update()
            return
        
        Z_target_unit = Z_target / Z_target_norm
        
        # Calculate perpendicular axes
        X_target = np.cross(Z_target_unit, V_up)
        X_target_norm = np.linalg.norm(X_target)
        
        # Handle case where Z_target is parallel to V_up
        if X_target_norm < 1e-6:
            X_target_unit = np.array([1, 0, 0])
        else:
            X_target_unit = X_target / X_target_norm
        
        Y_target_unit = np.cross(Z_target_unit, X_target_unit)
        
        # Build target rotation matrix
        R_target = np.column_stack((X_target_unit, Y_target_unit, Z_target_unit))
        
        # Convert current rotation vector to rotation matrix (Rodrigues formula)
        # R = I + sin(θ) * K + (1 - cos(θ)) * K²
        # where K is the skew-symmetric matrix of the normalized axis
        
        theta = np.linalg.norm(rvec)
        
        if theta < 1e-6:
            # No rotation, use identity matrix
            R_marker = np.eye(3)
        else:
            # Normalize rotation axis
            k = rvec / theta
            
            # Create skew-symmetric matrix K
            K = np.array([
                [0, -k[2], k[1]],
                [k[2], 0, -k[0]],
                [-k[1], k[0], 0]
            ])
            
            # Rodrigues' rotation formula
            R_marker = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
        
        # Calculate rotation difference
        R_delta = R_target @ R_marker.T
        
        # Calculate rotation angle theta from R_delta
        trace_R = np.trace(R_delta)
        theta_delta = np.arccos(np.clip((trace_R - 1) / 2, -1.0, 1.0))
        
        # Extract rotation axis from R_delta
        if np.abs(theta_delta) < 1e-6:
            # No rotation needed
            angular_omega = 0.0
        else:
            # Rotation axis from skew-symmetric part of R_delta
            # For small angles or to get the y-component of rotation axis:
            rotation_axis = np.array([
                R_delta[2, 1] - R_delta[1, 2],
                R_delta[0, 2] - R_delta[2, 0],
                R_delta[1, 0] - R_delta[0, 1]
            ])
            
            axis_norm = np.linalg.norm(rotation_axis)
            if axis_norm > 1e-6:
                rotation_axis = rotation_axis / axis_norm
            
            # Project rotation onto y-axis (vertical axis rotation)
            y_component = rotation_axis[1] * theta_delta
            
            # For 2D robot control, use the y-axis rotation component
            angular_omega = y_component * 2.0  # ANGULAR_GAIN
        
        # Calculate distance in horizontal plane
        distance = np.sqrt((follow_point[0] - pos[0])**2 + (follow_point[2] - pos[2])**2) * 1000  # mm
        
        # Control law with tunable gains
        STOP_THRESHOLD_MM = 20.0
        LINEAR_GAIN = 2.0
        ANGULAR_GAIN = 2.0
        TURN_THRESHOLD_RAD = 0.5  # ~30 degrees
        
        if distance < STOP_THRESHOLD_MM:
            # Stop if close enough
            linear_v = 0.0
            angular_omega = 0.0
        else:
            # Linear velocity proportional to distance
            linear_v = np.minimum(distance * LINEAR_GAIN, self.controller.v_max)
            
            # Reduce linear velocity when turning sharply
            if np.abs(angular_omega) > TURN_THRESHOLD_RAD:
                linear_v *= 0.5
        
        # Apply velocities
        self.controller.set_target_velocity(float(linear_v), float(angular_omega))
        self.controller.update()
        
        print(f"Управление: v={linear_v:.1f} mm/s, ω={angular_omega:.2f} rad/s, "
            f"distance={distance:.1f} mm, theta_delta={np.degrees(theta_delta):.1f}°")
        

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