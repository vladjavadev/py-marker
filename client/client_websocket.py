#!/usr/bin/env python
"""Client using the asyncio API."""
import asyncio
from websockets.asyncio.client import connect
import json
from robot_dto import Robot
from typing import List
from marker import Marker
import time
import client_state as cs




# uri = "ws://localhost:8765"
# uri = "ws://192.168.110.58:8765"
# uri = "ws://192.168.178.55:8765"
# uri = "ws://192.168.7.2:8765"
# uri = "ws://localhost:8765"
# uri = "ws://192.168.178.58:8765"
uri = "ws://localhost:8765"

m_types = ["init-marker","update-pos","get-status"]



async def get_status():
    status = await _fetch_connection_status()
    return status



async def init_marker(marker):
    msg_body = {"robot":{"marker_id": marker.id}}
    await _send_message("init-marker", msg_body)

async def update_pos():
    await _update_pos()






async def _update_pos():

    try:
        async with connect(uri) as websocket:
            event = {
                "type": "update-pos",
                "robot":{
                    "marker_id": cs.marker_id
                }
            }
            
            await websocket.send(json.dumps(event))
            print(f"✓ Событие отправлено: {event}")
            
            # Опционально: ожидание ответа от сервера
            try:
                response = await asyncio.wait_for(websocket.recv(),timeout=8.0)
                msg = json.loads(response)
                if msg["type"] == "update-pos":
                    if "robot" in msg:
                        robot_state  = msg["robot"]
                        id = robot_state["marker_id"]
                        pos = robot_state["pos"]
                        angle = robot_state["angle"]
                        fp = robot_state["follow_point"]
                        cs.robot.update_pos(pos,angle)
                        cs.robot.update_fp(fp)

                    print(f"Ответ сервера: {response}")
            except asyncio.TimeoutError:
                print("Сервер не ответил в течение 5 секунд")
            
    except ConnectionRefusedError:
        print("❌ Не удалось подключиться к серверу. Проверьте, что сервер запущен на ws://localhost:8765")
    except Exception as e:
        print(f"❌ Ошибка: {e}")


   


async def _send_message(type, message):

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
                response = await asyncio.wait_for(websocket.recv(),timeout=5.0)
                msg = json.loads(response)
                if msg["type"] == "init-marker":
                        if "robot" in msg:
                            robot_state = msg["robot"]
                            id = robot_state["marker_id"]
                            cs.robot = Robot(Marker(id))

            except asyncio.TimeoutError:
                print("Сервер не ответил в течение 5 секунд")
            
    except ConnectionRefusedError:
        print("❌ Не удалось подключиться к серверу. Проверьте, что сервер запущен на ws://localhost:8765")
    except Exception as e:
        print(f"❌ Ошибка: {e}")


   
async def _fetch_connection_status():
    try:
        async with connect(uri) as websocket:
            event = {
                "type": "get-status"
            }
            
            await websocket.send(json.dumps(event))
            print(f"✓ Событие отправлено: {event}")
            
            # Опционально: ожидание ответа от сервера
            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                event = json.loads(response)
                if event["type"] == "get-status":
                    if "value" in event:
                        cs.status = event["value"]
                print(f"Ответ сервера: {response}")
            except asyncio.TimeoutError:
                print("Сервер не ответил в течение 5 секунд")
            
    except ConnectionRefusedError:
        print("❌ Не удалось подключиться к серверу. Проверьте, что сервер запущен на ws://localhost:8765")
    except Exception as e:
        print(f"❌ Ошибка: {e}")



async def main():
    print("start client")
    count = 0
    max_count = 10
    while True:

        marker_id = int(input("Marker_ID: "))
        cs.marker_id = marker_id
        
        await init_marker(Marker(marker_id))

        while count<max_count:
            await get_status()
            if cs.status=="wait-clients":
                break
            time.sleep(1)


        while cs.status == "wait-clients":
            await get_status()

            print("Wait other clients ...")
            time.sleep(2)  # keep the loop alive long enough
        break

        # time.sleep(10)
    while cs.status == "update-pos":
        await get_status()
        time.sleep(0.1)
        await update_pos()

    


if __name__ == "__main__":
    try:
        asyncio.run(main())

    
    except KeyboardInterrupt:
        print("\n\nПрограмма прервана пользователем")