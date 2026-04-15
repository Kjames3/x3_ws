import asyncio, websockets

async def t():
    async with websockets.connect('ws://localhost:8765', subprotocols=['foxglove.sdk.v1']) as ws:
        msg = await ws.recv()
        print('OK:', str(msg)[:200])

asyncio.run(t())
