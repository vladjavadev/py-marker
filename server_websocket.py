#!/usr/bin/env python

"""Echo server using the asyncio API."""

import asyncio
from websockets.asyncio.server import serve, ServerConnection
import json
import threading
import time
import functools
from typing import List
from follow_point import FollowPoint as fp
from client.robot_dto import Robot
import websockets
import main as m



ip="0.0.0.0"
# ip="localhost"
connected_clients = {}
def send_update(robot_dto):
    if robot_dto.marker.id in connected_clients:
        asyncio.run(_send_update_msg(robot_dto))


def send_stop_robot(robot_dto):
    if robot_dto.marker.id in connected_clients:
        asyncio.run(_send_stop_robot_msg(robot_dto))



async def _send_update_msg(robot_dto):
    client = connected_clients[robot_dto.marker.id]
    async with websockets.connect(client) as websocket:
            event = {
                "type": "update-pos",
                "robot": robot_dto
            }
            
            await websocket.send(json.dumps(event))

async def _send_stop_robot_msg(robot_dto):
    client = connected_clients[robot_dto.marker.id]
    async with websockets.connect(client) as websocket:
            event = {
                "type": "stop-robot",
                "robot": robot_dto
            }
            
            await websocket.send(json.dumps(event))



async def echo(websocket:ServerConnection):
    message = await websocket.recv()
    event = json.loads(message)
   
    # assert event["type"] == "start"
    if event["type"] == "init-marker":
        if "marker" in event:
            marker_id = event["id"]
            connected_clients[marker_id] = websocket.remote_address
            m.init(marker_id)
            
    else:
        KeyError("NO route finded")



        

async def main():
    print("<!!!! Run SERVER !!!!>")
    bound_handler = functools.partial(echo)
    async with serve(bound_handler, ip, 8765) as server:
        await server.serve_forever()


def run_server():
    asyncio.run(main())

