import asyncio

client_events = {}

async def wait_client(id, timeout=5):
    event = client_events.setdefault(id, asyncio.Event())
    try:
        await asyncio.wait_for(event.wait(), timeout)
        print(f"Клиент {id} подключился!")
        return True
    except asyncio.TimeoutError:
        print(f"Клиент {id} не подключился за {timeout} секунд.")
        return False

async def connect_client(id, delay=2):
    await asyncio.sleep(delay)
    event = client_events.setdefault(id, asyncio.Event())
    event.set()
    print(f"Событие для клиента {id} установлено.")

async def main():
    # создаём задачи
    task1 = asyncio.create_task(wait_client(1, timeout=5))
    task2 = asyncio.create_task(wait_client(2, timeout=3))

    # подключаем клиента 1 через 2 секунды
    asyncio.create_task(connect_client(1, delay=2))

    # ждём обе задачи
    results = await asyncio.gather(task1, task2)
    print("Результаты:", results)

asyncio.run(main())