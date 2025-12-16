#!/usr/bin/env python
"""Client using the asyncio API."""
import asyncio
from websockets.asyncio.client import connect
import json
from robot_dto import Servant
from typing import List
from marker import Marker
from status import Status




# uri = "ws://localhost:8765"
# uri = "ws://192.168.110.58:8765"
# uri = "ws://192.168.178.55:8765"
# uri = "ws://192.168.7.2:8765"
# uri = "ws://localhost:8765"
# uri = "ws://192.168.178.58:8765"
uri = "ws://192.168.0.45:8765"

m_types = ["init-marker","update-pos","get-status"]

servant_dict= {}

status = Status("initial")

async def init_marker(marker):
    msg_body = {"marker": marker}
    call_func = send_message("init-marker", msg_body)
    asyncio.run(call_func)

async def update_pos(marker):
    msg_body = {"marker": marker}
    call_func = fetch_pos("update-pos", msg_body)
    asyncio.run(call_func)

async def position_handler():
    try:
        async with connect(uri) as websocket:

            async with websocket.connect(uri) as ws:
                while True:
                    message = await ws.recv()
                    print("Получено:", message)
                    if message["type"] == "update-pos":
                        if "robot" in message:
                            id = message["id"]
                            if id in servant_dict:
                                linear = message["linear"]
                                angular = message["angular"]         
                                robot = servant_dict[id]
                                robot.update_delta(linear, angular)
                            
    except ConnectionRefusedError:
        print("❌ Не удалось подключиться к серверу. Проверьте, что сервер запущен на ws://localhost:8765")
    except Exception as e:
        print(f"❌ Ошибка: {e}")




async def send_message(type, message):

    try:
        async with connect(uri) as websocket:
            event = {
                "type": type,
                **message
            }
            
            await websocket.send(json.dumps(event))
            print(f"✓ Событие отправлено: {event}")
            
            # Опционально: ожидание ответа от сервера
            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                init_marker = json.loads(response)
                if init_marker["type"] == "init-marker":
                    if "marker" in init_marker:
                        id = init_marker["id"]
                        pos = init_marker["pos"]
                        angle = init_marker["angle"]
                        fp = init_marker["follow_point"]
                        servant_robot = Servant(Marker(id))
                        servant_robot.update_pos(pos,angle)
                        servant_robot.update_fp(fp)

                        servant_dict[id] = servant_robot
                print(f"Ответ сервера: {response}")
            except asyncio.TimeoutError:
                print("Сервер не ответил в течение 5 секунд")
            
    except ConnectionRefusedError:
        print("❌ Не удалось подключиться к серверу. Проверьте, что сервер запущен на ws://localhost:8765")
    except Exception as e:
        print(f"❌ Ошибка: {e}")


   
async def fetch_connection_status():

    try:
        async with connect(uri) as websocket:
            event = {
                "type": "get-status"
            }
            
            await websocket.send(json.dumps(event))
            print(f"✓ Событие отправлено: {event}")
            
            # Опционально: ожидание ответа от сервера
            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=40.0)
                event = json.loads(response)
                if event["type"] == "get-status":
                    if "status" in event:
                        status.update(event["status"])
                print(f"Ответ сервера: {response}")
            except asyncio.TimeoutError:
                print("Сервер не ответил в течение 5 секунд")
            
    except ConnectionRefusedError:
        print("❌ Не удалось подключиться к серверу. Проверьте, что сервер запущен на ws://localhost:8765")
    except Exception as e:
        print(f"❌ Ошибка: {e}")



if __name__ == "__main__":
    try:
        asyncio.run(send_message())
    except KeyboardInterrupt:
        print("\n\nПрограмма прервана пользователем")