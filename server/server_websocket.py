#!/usr/bin/env python
"""Optimized WebSocket server with proper connection handling."""

import asyncio
from websockets.asyncio.server import serve, ServerConnection
from websockets.exceptions import ConnectionClosed
import json
import threading
import core.marker_proc as m
import data.server_state as s_state
from typing import Dict


ip = "0.0.0.0"


class ConnectionManager:
    """Manage WebSocket connections."""
    
    def __init__(self):
        self.connections: Dict[int, ServerConnection] = {}
        self.lock = asyncio.Lock()
    
    async def add(self, marker_id: int, websocket: ServerConnection):
        """Add a new connection."""
        async with self.lock:
            if marker_id in self.connections:
                old_ws = self.connections[marker_id]
                try:
                    await old_ws.close()
                except:
                    pass
            
            self.connections[marker_id] = websocket
    
    async def remove(self, marker_id: int):
        """Remove a connection."""
        async with self.lock:
            if marker_id in self.connections:
                del self.connections[marker_id]
    
    async def send_to(self, marker_id: int, message: dict) -> bool:
        """Send message to a specific client."""
        async with self.lock:
            if marker_id not in self.connections:
                return False
            
            websocket = self.connections[marker_id]
            try:
                await websocket.send(json.dumps(message))
                return True
            except ConnectionClosed:
                await self.remove(marker_id)
                return False
            except:
                return False


# Global connection manager
conn_manager = ConnectionManager()


def send_stop_robot(robot_dto):
    """Thread-safe method to send stop command."""
    if s_state.loop and s_state.loop.is_running():
        asyncio.run_coroutine_threadsafe(
            _send_stop_robot_async(robot_dto),
            s_state.loop
        )


async def _send_stop_robot_async(robot_dto):
    """Asynchronous stop robot command."""
    event = {
        "type": "stop-robot",
        "robot": {
            "marker_id": robot_dto.marker.id
        }
    }
    
    await conn_manager.send_to(robot_dto.marker.id, event)


async def handle_get_status(websocket: ServerConnection):
    """Handle status request."""
    status_msg = {
        "type": "get-status",
        "value": s_state.status
    }
    await websocket.send(json.dumps(status_msg))


async def handle_update_pos(websocket: ServerConnection, event: dict):
    """Handle position update request."""
    marker_id = event["robot"]["marker_id"]
    updated_robot = s_state.robot_new_values[marker_id]
    
    msg = {
        "type": "update-pos",
        "robot": {
            "marker_id": int(updated_robot.marker.id),
            "pos": updated_robot.pos.tolist(),
            "angle": updated_robot.angle.tolist(),
            "follow_point": updated_robot.follow_point.pos if updated_robot.follow_point else None,
            "deltaPos": {
                "linear": updated_robot.deltaPos.linear,
                "angular": updated_robot.deltaPos.angular
            }
        }
    }
    
    await websocket.send(json.dumps(msg))


async def handle_init_marker(websocket: ServerConnection, event: dict):
    """Handle marker initialization."""
    marker_id = event["robot"]["marker_id"]
    
    await conn_manager.add(marker_id, websocket)
    
    init_marker_msg = {
        "type": "init-marker",
        "robot": {
            "marker_id": marker_id
        }
    }
    
    await websocket.send(json.dumps(init_marker_msg))
    
    if s_state.init_thread is None:
        s_state.init_thread = threading.Thread(
            target=m.init, 
            name="init-marker",
            daemon=True
        )
        s_state.init_thread.start()


async def handle_client(websocket: ServerConnection):
    """Handle client connection."""
    client_marker_id = None
    
    try:
        async for message in websocket:
            event = json.loads(message)
            event_type = event.get("type")
            
            if event_type == "get-status":
                await handle_get_status(websocket)
            
            elif event_type == "update-pos":
                await handle_update_pos(websocket, event)
            
            elif event_type == "init-marker":
                await handle_init_marker(websocket, event)
                client_marker_id = event.get("robot", {}).get("marker_id")
                
    except ConnectionClosed:
        pass
    finally:
        if client_marker_id is not None:
            await conn_manager.remove(client_marker_id)


async def main():
    """Main server function."""
    s_state.loop = asyncio.get_running_loop()
    
    async with serve(handle_client, ip, 8765) as server:
        await server.serve_forever()


def run_server():
    """Start the server."""
    asyncio.run(main())


if __name__ == "__main__":
    run_server()