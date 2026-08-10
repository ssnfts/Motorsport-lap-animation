from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "server"))

from geometry import Mesh  # noqa: E402
import cars  # noqa: E402


def test_mesh_keeps_car_proxy_contract():
    mesh = Mesh(verts=[(0.0, 0.0, 0.0)], name="probe")
    assert mesh.name == "probe"
    assert mesh.vertex_count == 1
    assert cars.car_mesh("car").vertex_count > 0


def test_empty_mesh_has_no_bounds():
    with pytest.raises(ValueError, match="empty mesh"):
        Mesh().bounds()


def test_mesh_reports_face_count_and_bounds():
    mesh = Mesh(
        verts=[(-2.0, 4.0, 1.0), (3.0, -1.0, 5.0), (0.0, 2.0, -4.0)],
        faces=[(0, 1, 2)],
    )

    assert mesh.face_count == 1
    assert mesh.bounds() == ((-2.0, -1.0, -4.0), (3.0, 4.0, 5.0))


def test_mesh_volume_preserves_signed_winding():
    mesh = Mesh(
        verts=[
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ],
        faces=[(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)],
    )

    assert mesh.volume() == pytest.approx(1.0 / 6.0)
