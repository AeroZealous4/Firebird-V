import asyncio
from bleak import discover


async def run():
    devices = await discover()
    for d in devices:
        print(d)


# if python version >= 3.7 
# asyncio.run(main())
# else
loop = asyncio.get_event_loop()
loop.run_until_complete(run())
