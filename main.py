import server.server_websocket as sw
import server.core.marker_proc as mp
from threading import Thread 


if __name__ == "__main__":
    Thread(target=sw.run_server, name="server").start()
    mp.run()

    
