
import asyncio
import client.client_websocket as cw

if __name__ == "__main__":
    try:
        asyncio.run(cw.run_client())
    except KeyboardInterrupt:
        print("\n\n Program interrupted by user")
    except Exception as e:
        print("\n\nCritical error: {}".format(e))