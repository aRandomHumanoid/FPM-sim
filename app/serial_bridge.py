from __future__ import annotations

import asyncio
import os
import tty
from typing import Awaitable, Callable


SerialSink = Callable[[str], Awaitable[None]]
RegisterSink = Callable[[SerialSink], None]
UnregisterSink = Callable[[SerialSink], None]
EnqueueLine = Callable[[str], Awaitable[None]]
RxLineCallback = Callable[[str], Awaitable[None]]


class VirtualSerialBridge:
    def __init__(self) -> None:
        self.master_fd: int | None = None
        self.slave_fd: int | None = None
        self.device_path: str | None = None
        self.error: str | None = None

        self._rx_task: asyncio.Task[None] | None = None
        self._sink: SerialSink | None = None
        self._rx_buffer = ""
        self._enqueue_line: EnqueueLine | None = None
        self._unregister_sink: UnregisterSink | None = None
        self._on_rx_line: RxLineCallback | None = None

    async def start(
        self,
        enqueue_line: EnqueueLine,
        register_sink: RegisterSink,
        unregister_sink: UnregisterSink,
        on_rx_line: RxLineCallback | None = None,
    ) -> None:
        self._enqueue_line = enqueue_line
        self._unregister_sink = unregister_sink
        self._on_rx_line = on_rx_line

        try:
            self.master_fd, self.slave_fd = os.openpty()
            self.device_path = os.ttyname(self.slave_fd)
            tty.setraw(self.slave_fd)
            os.set_blocking(self.master_fd, True)
            os.set_blocking(self.slave_fd, True)
        except OSError as exc:
            self.error = str(exc)
            return

        self._sink = self._write_to_host
        register_sink(self._sink)
        self._rx_task = asyncio.create_task(self._read_loop(), name="virtual-serial-rx")

    async def stop(self) -> None:
        if self._rx_task is not None:
            self._rx_task.cancel()
            try:
                await self._rx_task
            except asyncio.CancelledError:
                pass
            self._rx_task = None

        if self._sink is not None and self._unregister_sink is not None:
            self._unregister_sink(self._sink)
            self._sink = None

        if self.master_fd is not None:
            os.close(self.master_fd)
            self.master_fd = None

        if self.slave_fd is not None:
            os.close(self.slave_fd)
            self.slave_fd = None

    def info(self) -> dict:
        return {
            "enabled": self.device_path is not None,
            "path": self.device_path,
            "error": self.error,
        }

    async def _write_to_host(self, message: str) -> None:
        if self.master_fd is None:
            return

        payload = (message + "\r\n").encode("utf-8", errors="replace")
        await asyncio.to_thread(os.write, self.master_fd, payload)

    async def _read_loop(self) -> None:
        while True:
            if self.master_fd is None:
                return

            chunk = await asyncio.to_thread(os.read, self.master_fd, 1024)
            if not chunk:
                await asyncio.sleep(0.01)
                continue

            self._rx_buffer += chunk.decode("utf-8", errors="ignore")
            self._rx_buffer = self._rx_buffer.replace("\r", "\n")

            while "\n" in self._rx_buffer:
                line, self._rx_buffer = self._rx_buffer.split("\n", 1)
                stripped = line.strip()
                if not stripped:
                    continue
                if self._on_rx_line is not None:
                    await self._on_rx_line(stripped)
                if self._enqueue_line is not None:
                    await self._enqueue_line(stripped)
