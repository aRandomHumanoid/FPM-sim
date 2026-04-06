from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

from fastapi import FastAPI, File, HTTPException, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .gcode_engine import MarlinEngine
from .machine import PrinterState
from .probing import ProbeWorld
from .serial_bridge import SerialSink, VirtualSerialBridge


ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / "static"
MESH_DIR = ROOT / "meshes"
MESH_DIR.mkdir(parents=True, exist_ok=True)


class GcodePayload(BaseModel):
    gcode: str


class MeshTransformPayload(BaseModel):
    tx: float | None = None
    ty: float | None = None
    tz: float | None = None
    rx: float | None = None
    ry: float | None = None
    rz: float | None = None


class SimulatorRuntime:
    def __init__(self) -> None:
        self.state = PrinterState()
        self.probe_world = ProbeWorld()
        self.connections: set[WebSocket] = set()
        self.queue: asyncio.Queue[str] = asyncio.Queue()
        self.worker_task: asyncio.Task[Any] | None = None
        self.serial_sinks: set[SerialSink] = set()
        self.engine = MarlinEngine(
            state=self.state,
            probe_world=self.probe_world,
            publish_state=self.broadcast_state,
            publish_message=self.broadcast_serial,
            publish_motion=self.broadcast_motion,
        )

    def _make_engine(self) -> MarlinEngine:
        return MarlinEngine(
            state=self.state,
            probe_world=self.probe_world,
            publish_state=self.broadcast_state,
            publish_message=self.broadcast_serial,
            publish_motion=self.broadcast_motion,
        )

    async def start(self) -> None:
        if self.worker_task is None or self.worker_task.done():
            self.worker_task = asyncio.create_task(self._worker(), name="gcode-worker")

    async def stop(self) -> None:
        if self.worker_task is None:
            return
        self.worker_task.cancel()
        try:
            await self.worker_task
        except asyncio.CancelledError:
            pass

    async def _worker(self) -> None:
        while True:
            line = await self.queue.get()
            await self.engine.execute_line(line)

    async def enqueue(self, gcode_block: str) -> int:
        lines = [ln.strip() for ln in gcode_block.splitlines() if ln.strip()]
        for ln in lines:
            await self.enqueue_line(ln)
        return len(lines)

    async def enqueue_line(self, line: str) -> None:
        await self.queue.put(line)

    async def reset_printer(self) -> int:
        cleared_queue = self.queue.qsize()
        self.queue = asyncio.Queue()

        if self.worker_task is not None:
            self.worker_task.cancel()
            try:
                await self.worker_task
            except asyncio.CancelledError:
                pass
            self.worker_task = None

        self.state = PrinterState()
        self.engine = self._make_engine()
        await self.start()
        await self.broadcast_serial(f"echo:printer reset ({cleared_queue} queued command(s) cleared)")
        await self.broadcast_state(self.state.snapshot())
        return cleared_queue

    def add_serial_sink(self, sink: SerialSink) -> None:
        self.serial_sinks.add(sink)

    def remove_serial_sink(self, sink: SerialSink) -> None:
        self.serial_sinks.discard(sink)

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self.connections.add(ws)
        await ws.send_json({"type": "state", "state": self.state.snapshot()})

    def disconnect(self, ws: WebSocket) -> None:
        self.connections.discard(ws)

    async def broadcast_serial(self, message: str) -> None:
        await self._broadcast({"type": "serial", "message": message})
        dead: list[SerialSink] = []
        for sink in list(self.serial_sinks):
            try:
                await sink(message)
            except Exception:
                dead.append(sink)
        for sink in dead:
            self.serial_sinks.discard(sink)

    async def broadcast_serial_monitor(self, message: str) -> None:
        await self._broadcast({"type": "serial", "message": message})

    async def broadcast_state(self, state: dict) -> None:
        await self._broadcast({"type": "state", "state": state})

    async def broadcast_motion(self, motion: dict) -> None:
        await self._broadcast({"type": "motion", "motion": motion})

    async def _broadcast(self, payload: dict) -> None:
        dead: list[WebSocket] = []
        for ws in list(self.connections):
            try:
                await ws.send_json(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.connections.discard(ws)


runtime = SimulatorRuntime()
serial_bridge = VirtualSerialBridge()
app = FastAPI(title="FPM Sim", version="0.1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
app.mount("/meshes", StaticFiles(directory=MESH_DIR), name="meshes")


@app.on_event("startup")
async def startup_event() -> None:
    await runtime.start()
    await serial_bridge.start(
        enqueue_line=runtime.enqueue_line,
        register_sink=runtime.add_serial_sink,
        unregister_sink=runtime.remove_serial_sink,
        on_rx_line=lambda line: runtime.broadcast_serial_monitor(f"> serial:{line}"),
    )
    info = serial_bridge.info()
    if info["enabled"] and info["path"]:
        print(f"[startup] Virtual serial port: {info['path']}", flush=True)
        await runtime.broadcast_serial(f"echo:virtual serial ready {info['path']}")
    elif info.get("error"):
        print(f"[startup] Virtual serial unavailable: {info['error']}", flush=True)


@app.on_event("shutdown")
async def shutdown_event() -> None:
    await serial_bridge.stop()
    await runtime.stop()


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/state")
async def get_state() -> dict:
    return runtime.state.snapshot()


@app.get("/api/serial")
async def get_serial_info() -> dict:
    return serial_bridge.info()


@app.post("/api/gcode")
async def post_gcode(payload: GcodePayload) -> dict:
    count = await runtime.enqueue(payload.gcode)
    return {"queued": count}


@app.post("/api/mesh")
async def upload_mesh(file: UploadFile = File(...)) -> dict:
    if not file.filename:
        raise HTTPException(status_code=400, detail="Missing filename")

    suffix = Path(file.filename).suffix.lower()
    if suffix not in {".stl", ".obj", ".ply"}:
        raise HTTPException(status_code=400, detail="Supported mesh formats: STL, OBJ, PLY")

    target = MESH_DIR / file.filename
    data = await file.read()
    target.write_bytes(data)

    try:
        details = runtime.probe_world.load_mesh(target)
    except Exception as exc:
        target.unlink(missing_ok=True)
        raise HTTPException(status_code=400, detail=f"Failed to load mesh: {exc}") from exc

    await runtime.broadcast_serial(f"echo:mesh loaded {details['name']}")
    await runtime.broadcast_state(runtime.state.snapshot())

    return details


@app.get("/api/meshes")
async def list_meshes() -> list[dict]:
    return runtime.probe_world.list_meshes()


@app.patch("/api/mesh/{mesh_name}")
async def patch_mesh(mesh_name: str, payload: MeshTransformPayload) -> dict:
    try:
        details = runtime.probe_world.update_mesh_transform(mesh_name, payload.model_dump())
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Mesh not found") from exc

    await runtime.broadcast_serial(f"echo:mesh transformed {mesh_name}")
    return details


@app.delete("/api/mesh/{mesh_name}")
async def delete_mesh(mesh_name: str) -> dict:
    removed = runtime.probe_world.delete_mesh(mesh_name)
    if not removed:
        raise HTTPException(status_code=404, detail="Mesh not found")

    (MESH_DIR / mesh_name).unlink(missing_ok=True)
    await runtime.broadcast_serial(f"echo:mesh deleted {mesh_name}")
    return {"ok": True, "name": mesh_name}


@app.post("/api/path/clear")
async def clear_path() -> dict:
    runtime.state.path.clear()
    await runtime.broadcast_state(runtime.state.snapshot())
    return {"ok": True}


@app.post("/api/printer/reset")
async def reset_printer() -> dict:
    cleared_queue = await runtime.reset_printer()
    return {
        "ok": True,
        "cleared_queue": cleared_queue,
        "state": runtime.state.snapshot(),
    }


@app.websocket("/ws")
async def ws_handler(ws: WebSocket) -> None:
    await runtime.connect(ws)
    try:
        while True:
            msg = await ws.receive_text()
            if msg.strip():
                await runtime.enqueue(msg)
    except WebSocketDisconnect:
        runtime.disconnect(ws)
