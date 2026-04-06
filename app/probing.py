from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import trimesh


@dataclass(slots=True)
class ProbeHit:
    point: np.ndarray
    distance: float
    mesh_name: str


class ProbeWorld:
    def __init__(self) -> None:
        self.meshes: Dict[str, trimesh.Trimesh] = {}
        self.transforms: Dict[str, Dict[str, float]] = {}

    @staticmethod
    def _default_transform() -> Dict[str, float]:
        return {"tx": 0.0, "ty": 0.0, "tz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}

    @staticmethod
    def _compose_transform(transform: Dict[str, float]) -> np.ndarray:
        rx = np.deg2rad(transform.get("rx", 0.0))
        ry = np.deg2rad(transform.get("ry", 0.0))
        rz = np.deg2rad(transform.get("rz", 0.0))
        tx = transform.get("tx", 0.0)
        ty = transform.get("ty", 0.0)
        tz = transform.get("tz", 0.0)

        rotation = trimesh.transformations.euler_matrix(rx, ry, rz, "sxyz")
        translation = trimesh.transformations.translation_matrix([tx, ty, tz])
        return translation @ rotation

    def _transformed_mesh(self, name: str) -> trimesh.Trimesh:
        base = self.meshes[name]
        transform = self.transforms.get(name, self._default_transform())

        mesh = base.copy()
        if any(abs(transform[k]) > 1e-12 for k in ("tx", "ty", "tz", "rx", "ry", "rz")):
            mesh.apply_transform(self._compose_transform(transform))
        return mesh

    def _mesh_details(self, name: str) -> dict:
        mesh = self._transformed_mesh(name)
        bounds = mesh.bounds
        return {
            "name": name,
            "vertices": int(mesh.vertices.shape[0]),
            "faces": int(mesh.faces.shape[0]),
            "bounds": {
                "min": bounds[0].tolist(),
                "max": bounds[1].tolist(),
            },
            "transform": dict(self.transforms.get(name, self._default_transform())),
        }

    def load_mesh(self, path: Path) -> dict:
        loaded = trimesh.load(path, force="mesh")

        if isinstance(loaded, trimesh.Scene):
            if not loaded.geometry:
                raise ValueError("Scene has no mesh geometry")
            mesh = trimesh.util.concatenate(tuple(loaded.geometry.values()))
        else:
            mesh = loaded

        if mesh.is_empty:
            raise ValueError("Mesh is empty")

        self.meshes[path.name] = mesh
        self.transforms[path.name] = self._default_transform()
        return self._mesh_details(path.name)

    def list_meshes(self) -> list[dict]:
        out: list[dict] = []
        for name in self.meshes:
            out.append(self._mesh_details(name))
        return out

    def update_mesh_transform(self, name: str, values: Dict[str, float]) -> dict:
        if name not in self.meshes:
            raise KeyError(name)

        transform = self.transforms.get(name, self._default_transform())
        for key in ("tx", "ty", "tz", "rx", "ry", "rz"):
            if key in values and values[key] is not None:
                transform[key] = float(values[key])
        self.transforms[name] = transform
        return self._mesh_details(name)

    def delete_mesh(self, name: str) -> bool:
        if name not in self.meshes:
            return False
        self.meshes.pop(name, None)
        self.transforms.pop(name, None)
        return True

    def probe_segment(self, start: np.ndarray, end: np.ndarray) -> Optional[ProbeHit]:
        direction = end - start
        length = float(np.linalg.norm(direction))
        if length <= 1e-8:
            return None

        unit = direction / length

        closest: Optional[ProbeHit] = None

        for name in self.meshes:
            mesh = self._transformed_mesh(name)
            locations, _, _ = mesh.ray.intersects_location(
                ray_origins=np.array([start]),
                ray_directions=np.array([unit]),
                multiple_hits=True,
            )

            if locations.size == 0:
                continue

            rel = locations - start
            dists = rel @ unit
            valid = dists[(dists >= 0.0) & (dists <= length + 1e-6)]

            if valid.size == 0:
                continue

            dist = float(np.min(valid))
            hit_point = start + unit * dist
            hit = ProbeHit(point=hit_point, distance=dist, mesh_name=name)

            if closest is None or hit.distance < closest.distance:
                closest = hit

        return closest

    def vertical_probe(self, x: float, y: float, z_start: float, z_min: float) -> Optional[ProbeHit]:
        start = np.array([x, y, z_start], dtype=np.float64)
        end = np.array([x, y, z_min], dtype=np.float64)
        return self.probe_segment(start, end)

    def in_contact(self, point: np.ndarray, tolerance: float = 0.05) -> bool:
        for name in self.meshes:
            mesh = self._transformed_mesh(name)
            try:
                _, _, dist = mesh.nearest.on_surface(np.array([point], dtype=np.float64))
            except Exception:
                continue

            if float(dist[0]) <= tolerance:
                return True

            if mesh.is_watertight:
                try:
                    inside = bool(mesh.contains(np.array([point], dtype=np.float64))[0])
                except Exception:
                    inside = False
                if inside:
                    return True
        return False

    def probe_contact_break_segment(
        self,
        start: np.ndarray,
        end: np.ndarray,
        tolerance: float = 0.05,
        steps: int = 200,
    ) -> Optional[ProbeHit]:
        direction = end - start
        length = float(np.linalg.norm(direction))
        if length <= 1e-8:
            return None

        if not self.in_contact(start, tolerance=tolerance):
            return None

        prev_t = 0.0
        for i in range(1, steps + 1):
            t = i / steps
            p = start + direction * t
            if not self.in_contact(p, tolerance=tolerance):
                lo = prev_t
                hi = t
                for _ in range(16):
                    mid = 0.5 * (lo + hi)
                    pm = start + direction * mid
                    if self.in_contact(pm, tolerance=tolerance):
                        lo = mid
                    else:
                        hi = mid

                hit_t = hi
                hit_point = start + direction * hit_t
                return ProbeHit(point=hit_point, distance=float(length * hit_t), mesh_name="contact-break")

            prev_t = t

        return None
