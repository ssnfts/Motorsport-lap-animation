"""Generic 2021-style open-wheel geometry tests.

These tests constrain dimensions and topology without borrowing a team's car,
livery, or tyre-maker markings. The requested Yas shot is dated December 2021,
so the earlier small-rim/high-sidewall wheel package is the relevant reference,
not the 18-inch geometry introduced later.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "server"))

import cars  # noqa: E402


def _signed_volume(*meshes) -> float:
    """Signed volume of one closed assembly from its disjoint triangle meshes."""
    volume = 0.0
    for mesh in meshes:
        for a, b, c in mesh.faces:
            ax, ay, az = mesh.verts[a]
            bx, by, bz = mesh.verts[b]
            cx, cy, cz = mesh.verts[c]
            volume += (
                ax * (by * cz - bz * cy)
                + ay * (bz * cx - bx * cz)
                + az * (bx * cy - by * cx)
            ) / 6.0
    return volume


def test_2021_wheel_spec_matches_fia_dimension_limits():
    spec = cars.F1_2021
    assert spec.tyre_diameter == pytest.approx(0.670)
    assert spec.tyre_width_front == pytest.approx(0.380)
    assert spec.tyre_width_rear == pytest.approx(0.465)
    assert spec.rim_diameter == pytest.approx(0.358)


def test_wheel_meshes_have_separate_brakes_and_positive_closed_assembly():
    tyres, rims, brakes = cars.wheel_meshes("probe", cars.F1_2021, segments=32)

    assert tyres.name == "probe_tyres"
    assert rims.name == "probe_rims"
    assert brakes.name == "probe_brakes"
    assert tyres.metadata["compound"] == "generic_dry_slick"
    assert rims.metadata["rim_lip_diameter_m"] == pytest.approx(0.358)
    assert brakes.metadata["kind"] == "brake_discs"
    # A tyre/rim assembly can look normal with reversed faces. Its signed volume
    # cannot: negative means inward normals and a dark, hollow wheel in V-Ray.
    assert _signed_volume(tyres, rims) > 0.0
    assert len(brakes.verts) > 4 * 32


def test_grid_builds_four_material_parts_for_each_generic_car():
    meshes = cars.cars_on_grid([(0.0, 0.0, 0.0)], cars.F1_2021)
    assert [mesh.name for mesh in meshes] == [
        "car_01", "car_01_tyres", "car_01_rims", "car_01_brakes",
    ]
