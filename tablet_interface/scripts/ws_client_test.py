#!/usr/bin/env python3
import argparse
import asyncio
import json
import time

import websockets


def _now_ms() -> int:
    return int(time.time() * 1000)


async def _send_cmds(ws, rate_hz: float) -> None:
    seq = 0
    period = 1.0 / rate_hz
    linear = {"x": 0.2, "y": 0.0, "z": 0.0}
    angular = {"x": 0.0, "y": 0.0, "z": 0.2}
    while True:
        payload = {
            "type": "teleop_cmd",
            "seq": seq,
            "mode": 0,
            "linear": linear,
            "angular": angular,
        }
        await ws.send(json.dumps(payload))
        seq += 1
        await asyncio.sleep(period)


async def _recv_state(ws) -> None:
    while True:
        msg = await ws.recv()
        print(msg)


async def main() -> None:
    parser = argparse.ArgumentParser(description="Tablet interface WS client test")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--path", default="/ws/control")
    parser.add_argument("--rate", type=float, default=50.0)
    args = parser.parse_args()

    uri = f"ws://{args.host}:{args.port}{args.path}"
    async with websockets.connect(uri) as ws:
        send_task = asyncio.create_task(_send_cmds(ws, args.rate))
        recv_task = asyncio.create_task(_recv_state(ws))
        done, pending = await asyncio.wait(
            [send_task, recv_task], return_when=asyncio.FIRST_EXCEPTION
        )
        for task in pending:
            task.cancel()
        for task in done:
            if task.exception():
                raise task.exception()


if __name__ == "__main__":
    asyncio.run(main())
