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
