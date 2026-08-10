"""Reusable triangle-mesh primitives for vehicle proxy geometry."""

from __future__ import annotations

from dataclasses import dataclass, field

__all__ = ["Mesh"]


@dataclass
class Mesh:
    """
    A watertight triangle mesh in scene metres.

    ``faces`` are 0-based index triples, wound counter-clockwise seen from
    outside the solid.
    """

    verts: list[tuple[float, float, float]] = field(default_factory=list)
    faces: list[tuple[int, int, int]] = field(default_factory=list)
    name: str = "mesh"
    metadata: dict = field(default_factory=dict)
    uvs: list[tuple[float, float]] = field(default_factory=list)

    @property
    def vertex_count(self) -> int:
        return len(self.verts)

    @property
    def face_count(self) -> int:
        return len(self.faces)

    def bounds(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Axis-aligned bounds as ``(min, max)`` corners."""
        if not self.verts:
            raise ValueError("empty mesh has no bounds")
        xs, ys, zs = zip(*self.verts)
        return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))

    def volume(self) -> float:
        """Signed volume via the divergence theorem."""
        total = 0.0
        for i, j, k in self.faces:
            ax, ay, az = self.verts[i]
            bx, by, bz = self.verts[j]
            cx, cy, cz = self.verts[k]
            total += (
                ax * (by * cz - bz * cy)
                - ay * (bx * cz - bz * cx)
                + az * (bx * cy - by * cx)
            )
        return total / 6.0
