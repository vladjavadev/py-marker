import asyncio
import threading
import time
import numpy as np
from client.servant_controller import SlaveController
import client_websocket as cw
from client.marker import Marker


# Configuration
ROBOT_ID = 4
LINEAR_KP = 200.0   # Convert distance error (meters) to linear velocity (mm/s)
ANGULAR_KP = 2.0    # Convert angular error (radians) to angular velocity (rad/s)

MAX_LINEAR_SPEED = 150.0  # mm/s
MAX_ANGULAR_SPEED = 2.0   # rad/s

# Global controller
controller = SlaveController(kp=0.6, ki=0.1, kd=0.01, v_max=MAX_LINEAR_SPEED)
running = True


def control_loop():
    """Main control loop that sends velocity commands to robot."""
    print("Control loop started...")
    
    while running:
        time.sleep(0.05)  # 20 Hz control loop
        
        # Check if robot data exists
        if cw.robot is None:
            continue
        
        robot = cw.robot
        
        # Check if we have position delta
        if robot.deltaPos is None:
            # No tracking data, stop the robot
            controller.set_target_velocity(0.0, 0.0)
            controller.update()
            continue
        
        # Extract linear and angular errors
        distance_error = robot.deltaPos.linear  # meters
        angular_error = robot.deltaPos.angular  # radians (y-axis rotation)
        
        # Dead zone to prevent oscillation when close to target
        if abs(distance_error) < 0.01:  # 10mm threshold
            distance_error = 0.0
        
        if abs(angular_error) < 0.05:  # ~3 degrees threshold
            angular_error = 0.0
        
        # Proportional control: convert errors to velocities
        linear_velocity = LINEAR_KP * distance_error  # mm/s
        angular_velocity = ANGULAR_KP * angular_error  # rad/s
        
        # Clamp velocities to safe limits
        linear_velocity = np.clip(linear_velocity, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
        angular_velocity = np.clip(angular_velocity, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        
        # Send commands to motor controller
        controller.set_target_velocity(linear_velocity, angular_velocity)
        duty_l, duty_r = controller.update()
        
        # Debug output (optional, comment out in production)
        if abs(distance_error) > 0.01 or abs(angular_error) > 0.05:
            print(f"Robot {ROBOT_ID}: dist={distance_error:.3f}m, "
                  f"ang={angular_error:.3f}rad, "
                  f"v={linear_velocity:.1f}mm/s, "
                  f"ω={angular_velocity:.2f}rad/s, "
                  f"duty=({duty_l:.1f}, {duty_r:.1f})")


async def main():
    """Main async function to coordinate websocket and control loop."""
    global running
    
    try:
        print(f"Initializing robot with ID: {ROBOT_ID}")
        
        while cw.status.status is not "ready":
            await cw.get_status()
            time.sleep(1.0)
        # Initialize marker on server
        marker = Marker(ROBOT_ID)
        await cw.init_marker(marker)
        
        print("Waiting for initial robot data...")
        await asyncio.sleep(1.0)  # Give time for initial data
        
        # Start control loop in separate thread
        control_thread = threading.Thread(target=control_loop, daemon=True)
        control_thread.start()
        print("Control thread started")
        
        # Start position update handler
        print("Starting position update listener...")
        await cw.position_handler()
        
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        running = False
        controller.set_target_velocity(0.0, 0.0)
        controller.update()
    except Exception as e:
        print(f"Error in main: {e}")
        running = False
        controller.set_target_velocity(0.0, 0.0)
        controller.update()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        running = False
        controller.set_target_velocity(0.0, 0.0)
        controller.update()