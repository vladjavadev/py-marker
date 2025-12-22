#!/usr/bin/env python

"""Echo server using the asyncio API."""

import asyncio
from websockets.asyncio.server import serve, ServerConnection
import json
import threading
import functools
import core.marker_proc as m
import data.server_state as s_state



ip="0.0.0.0"

def send_stop_robot(robot_dto):
    if robot_dto.marker.id in s_state.connected_clients:
        asyncio.run_coroutine_threadsafe(
            _send_stop_robot_msg(robot_dto), 
            s_state.loop  # You need to store the main loop reference
        )



async def _send_stop_robot_msg(robot_dto):
    client = s_state.connected_clients[robot_dto.marker.id]
    async with client:
            event = {
                "type": "stop-robot",
                "robot": {
                    "marker_id":robot_dto.marker.id
                }
            }
            
            await client.send(json.dumps(event))



async def echo(websocket:ServerConnection):
    message = await websocket.recv()
    event = json.loads(message)

            

    if event["type"] == "get-status":
        status_msg = {
                "type": "get-status",
                "value": s_state.status
        }
        await websocket.send(json.dumps(status_msg))

    if event["type"] == "update-pos":
        if "robot" in event:
            id = event["robot"]["marker_id"]
            if id in s_state.robot_new_values: 
                updated_robot = s_state.robot_new_values[id]
                msg = {
                    "type": "update-pos",
                    "robot": {
                        "marker_id":int(updated_robot.marker.id),
                        "pos":updated_robot.pos.tolist(),
                        "angle":updated_robot.angle.tolist(),
                        "follow_point":updated_robot.follow_point.pos,
                        "deltaPos":{
                            "linear":updated_robot.deltaPos.linear,
                            "angular":updated_robot.deltaPos.angular
                        }
                    }
                }

                await websocket.send(json.dumps(msg))    
   
    # assert event["type"] == "start"
    if event["type"] == "init-marker":

        if "robot" in event:
            init_marker_msg = {
                "type": "init-marker",
                "robot": {
                    "marker_id":event["robot"]["marker_id"]
                }
            }
            marker_id = event["robot"]["marker_id"]
            s_state.connected_clients[marker_id] = websocket
 
            if s_state.init_thread == None:
                s_state.init_thread = threading.Thread(target=m.init,name="init-marker")
                s_state.init_thread.start()
            await asyncio.sleep(2.0)
            if s_state.camera_is_on:
                await websocket.send(json.dumps(init_marker_msg))
            else:
                s_state.init_thread = None

                

                



    else:
        KeyError("NO route finded")



        

async def main():
    print("<!!!! Run SERVER !!!!>")
    s_state.loop = asyncio.get_event_loop()
    bound_handler = functools.partial(echo)
    async with serve(bound_handler, ip, 8765) as server:
        await server.serve_forever()
    



def run_server():
    asyncio.run(main())

if __name__ == "__main__":
    run_server()



