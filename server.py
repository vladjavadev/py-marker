#!/usr/bin/env python

"""Echo server using the asyncio API."""

import asyncio
from websockets.asyncio.server import serve, ServerConnection
from data.grid_dto import GridDto
from robot.move_logic import Logic
import core.algorithm as agm
from path_observation import PathObservation
import json
import threading
import time
import functools
from typing import List

#change value for 2 mode 
mode=3
g_dt = GridDto()

logic = Logic(vMode=mode)
trajectory_times = []
pred_times = []
path_build_time_list = []
path_build_time = 0
ip="0.0.0.0"
# ip="localhost"

async def echo(dto:GridDto, websocket:ServerConnection):
    message = await websocket.recv()
    event = json.loads(message)
   
    # assert event["type"] == "start"
    if event["type"] == "set-obs":
        
        if "obs" in event:
            # print("<!------",event["obs"][0],"------!>")
            dto.set_obs(event["obs"][0])
        elif "no-obs" in event:
            dto.rem_obs(event["no-obs"][0])
    elif event["type"] == "get-location":
        pos = dto.get_position()
        goal= dto.get_goal()
        path = dto.get_path()
        totalDistance = dto.get_total_distance()
        pred_time, pred_distance = dto.get_predict_time_distance()
        event_location = {
            "type":"location",
            "current_pos":pos,
            "goal":goal,
            "path":path,
            "distance":totalDistance,
            "pred_time": pred_time,
            "pred_distance": pred_distance
        }
        await websocket.send(json.dumps(event_location))

    elif event["type"] == "get-status":
        event_connected = {
            "type":"get-status",
            "status":"connected"
        }
        await websocket.send(json.dumps(event_connected))
    elif event["type"] == "init-dim":

        dto.set_dim(event["grid_dim"])
        dto.set_unit(event["cell_unit"])
        logic.init_dto(dto)

    elif event["type"] == "init-points":
        dto.set_start(event["start"])
        dto.set_goal(event["goal"])
    

        m_threads = [DoWork(shared=g_dt, task_func=agm.run_algorithm, name='a'), 
        DoWork(shared=logic, task_func=moving_robot, name='b')]
        for i in m_threads:
            i.start()
        # for i in m_threads:
        #     i.join()
    else:
        KeyError("NO route finded")


async def get_pos(dto: GridDto, websocket:ServerConnection):
    pos = dto.get_position()
    websocket.send(pos)

        

async def main():
    print("<!!!! Run SERVER !!!!>")
    bound_handler = functools.partial(echo, g_dt)
    async with serve(bound_handler, ip, 8765) as server:
        await server.serve_forever()


def moving_robot(logic: Logic):
    time.sleep(5.0)
    last_path = []
    next_pos = None
    last_pred_time = 0
    last_pbuild_time = 0
    _lock = threading.Lock()

    p_obs = PathObservation() 
    threading.Thread(target=move_process, args=(p_obs, logic,trajectory_times)).start()
    start_time = time.time()
    while True:
        try:
            time.sleep(0.1)
            path = logic.dto.get_path()
            if path is not None and path!=last_path:
                if len(path)>1:

                    if not p_obs.is_updated:
                        _lock.acquire()
                        next_pos = path[1]
                        p_obs.update(next_pos)
                        ptime, pdistance = logic.predict_time_distance(path)
                        if(last_pred_time!=ptime or len(pred_times)==0):
                            pred_times.append(ptime)
                            path_build_time = logic.dto.get_time_build_path()
                            path_build_time_list.append(path_build_time)
                        logic.dto.set_predict_time_distance(ptime,pdistance)

                        last_pred_time = pred_times[-1]
                        _lock.release()


            if logic.dto.get_position() == tuple(logic.dto.get_goal()):
                print(f"MOVE ROBOT POS:{logic.dto.get_position()}")
                p_obs.is_done = True
                print("Client: Reached Goal!")
                jsonData = {
                    "build_trajectory_times": trajectory_times,
                    "predict_times": pred_times,
                    "path_build_times": path_build_time_list
                }
                print("Total time: ", time.time()-start_time)
                print(json.dumps(jsonData, indent=4))
                break
        except Exception as e:
            print("MOVING ROBOT EXCEPTION:", e)
            p_obs.is_done
            logic.stop()


class DoWork(threading.Thread):
    def __init__(self, shared, task_func, *args, **kwargs):
        super(DoWork, self).__init__(*args, **kwargs)
        self.shared = shared
        self.task_func = task_func  # передаём функцию

    def run(self):
        print(threading.current_thread(), 'start')
        time.sleep(1)
        self.task_func(self.shared)  # вызываем переданную функцию
        print(threading.current_thread(), 'done')


def move_process(p_obs: PathObservation, logic: Logic, route_times: List[int]):
    counter = 0
    update_time_start = time.time()
    while True:

        time.sleep(0.02)
        if p_obs.is_updated:
            build_route_time = logic.build_route(p_obs.next_pos)
            logic.dto.set_position(p_obs.next_pos)
            if(build_route_time):
                route_times.append(build_route_time)
            p_obs.is_updated = False
            counter = 0
        elif counter ==5:
            print("No path update, stop robot ", time.time()-update_time_start)
            update_time_start = time.time()
            logic.stop()
        elif p_obs.is_done:
            route_times.append(0)
            logic.stop()
            break

        counter  = counter%5+1


def run_server(dto):
    asyncio.run(main())


if __name__ == "__main__":
 
    threads = [ DoWork(shared=g_dt, task_func=run_server, name='c')]
    for t in threads:
        t.start()

    for t in threads:
        t.join()
   

