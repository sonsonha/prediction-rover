"""Unit tests for pure Prediction visualization conversions."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest
from visualization_msgs.msg import Marker

from prediction_visualization.marker_builders import (
    build_collision_markers,
    build_object_markers,
    build_rollover_markers,
    build_terrain_normal_markers,
    build_zmp_markers,
    clear_prediction_overlays,
    lookup_step,
    trajectory_steps_by_id,
    zmp_local_to_map,
)


def _step(step_id: int, x: float, y: float, yaw: float = 0.0):
    return SimpleNamespace(step_id=step_id, x=x, y=y, yaw=yaw)


def _header():
    return SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="map")


def test_trajectory_step_lookup_by_step_id_not_index():
    steps = [_step(10, 1.0, 2.0), _step(20, 3.0, 4.0), _step(5, 9.0, 8.0)]
    by_id = trajectory_steps_by_id(steps)
    assert lookup_step(by_id, 20).x == 3.0
    assert lookup_step(by_id, 0) is None
    assert lookup_step(by_id, 5).y == 8.0


def test_empty_tracked_objects_no_exception():
    tracked = SimpleNamespace(header=_header(), objects=[])
    arr = build_object_markers(tracked, "map", label_enabled=True)
    assert any(m.action == Marker.DELETEALL for m in arr.markers)
    assert not any(m.action == Marker.ADD for m in arr.markers)


def test_tracked_object_footprint_closed_line_strip():
    tracked = SimpleNamespace(
        header=_header(),
        objects=[
            SimpleNamespace(
                track_id=7,
                class_name="pipe",
                footprint_polygon_xy=[
                    SimpleNamespace(x=0.0, y=0.0),
                    SimpleNamespace(x=1.0, y=0.0),
                    SimpleNamespace(x=1.0, y=1.0),
                ],
            )
        ],
    )
    arr = build_object_markers(tracked, "map", label_enabled=True)
    strips = [m for m in arr.markers if m.type == Marker.LINE_STRIP]
    assert len(strips) == 1
    assert len(strips[0].points) == 4  # closed
    assert strips[0].points[0].x == strips[0].points[-1].x
    assert strips[0].points[0].y == strips[0].points[-1].y


def test_terrain_valid_normal_arrow():
    geom = SimpleNamespace(
        header=_header(),
        steps=[
            SimpleNamespace(
                step_id=1,
                normal=SimpleNamespace(x=0.0, y=0.0, z=1.0),
            )
        ],
    )
    by_id = trajectory_steps_by_id([_step(1, 2.0, 3.0)])
    arr = build_terrain_normal_markers(
        geom, by_id, "map", stride=1, arrow_scale=0.5
    )
    arrows = [m for m in arr.markers if m.type == Marker.ARROW and m.action == Marker.ADD]
    assert len(arrows) == 1
    assert arrows[0].points[0].x == pytest.approx(2.0)
    assert arrows[0].points[1].z == pytest.approx(0.05 + 0.5)


def test_terrain_invalid_normal_no_fabrication():
    geom = SimpleNamespace(
        header=_header(),
        steps=[
            SimpleNamespace(
                step_id=1,
                normal=SimpleNamespace(x=float("nan"), y=0.0, z=1.0),
            ),
            SimpleNamespace(
                step_id=2,
                normal=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            ),
        ],
    )
    by_id = trajectory_steps_by_id([_step(1, 0.0, 0.0), _step(2, 1.0, 0.0)])
    arr = build_terrain_normal_markers(
        geom, by_id, "map", stride=1, arrow_scale=0.5
    )
    assert not any(m.type == Marker.ARROW and m.action == Marker.ADD for m in arr.markers)


def test_collision_step_joins_by_step_id():
    by_id = trajectory_steps_by_id([_step(1, 0.0, 0.0), _step(5, 10.0, 20.0)])
    pred = SimpleNamespace(
        header=_header(),
        collision_steps=[
            SimpleNamespace(
                step_id=5,
                collision_objects=[
                    SimpleNamespace(min_distance_m=0.1, track_id=1, object_class="pipe")
                ],
            )
        ],
    )
    arr = build_collision_markers(pred, by_id, "map")
    spheres = [m for m in arr.markers if m.type == Marker.SPHERE]
    assert len(spheres) == 1
    assert spheres[0].pose.position.x == pytest.approx(10.0)
    assert spheres[0].pose.position.y == pytest.approx(20.0)


def test_rollover_step_joins_by_step_id():
    by_id = trajectory_steps_by_id([_step(3, 4.0, 5.0)])
    pred = SimpleNamespace(
        header=_header(),
        rollover_steps=[
            SimpleNamespace(
                step_id=3,
                static_stability_margin_m=0.2,
                stability_moment=SimpleNamespace(
                    valid=True, minimum_stability_moment_nm=100.0
                ),
            )
        ],
    )
    arr = build_rollover_markers(pred, by_id, "map")
    cyl = [m for m in arr.markers if m.type == Marker.CYLINDER]
    assert len(cyl) == 1
    assert cyl[0].pose.position.x == pytest.approx(4.0)


def test_zmp_local_to_map_transform():
    mx, my = zmp_local_to_map(10.0, 20.0, math.pi / 2, 1.0, 0.0)
    assert mx == pytest.approx(10.0)
    assert my == pytest.approx(21.0)
    mx2, my2 = zmp_local_to_map(0.0, 0.0, 0.0, 0.3, -0.1)
    assert mx2 == pytest.approx(0.3)
    assert my2 == pytest.approx(-0.1)


def test_zmp_valid_marker_uses_map_transform():
    by_id = trajectory_steps_by_id([_step(1, 10.0, 20.0, math.pi / 2)])
    pred = SimpleNamespace(
        header=_header(),
        rollover_steps=[
            SimpleNamespace(
                step_id=1,
                zmp=SimpleNamespace(valid=True, x=1.0, y=0.0, margin_m=0.1),
            )
        ],
    )
    arr = build_zmp_markers(pred, by_id, "map")
    spheres = [m for m in arr.markers if m.type == Marker.SPHERE]
    assert len(spheres) == 1
    assert spheres[0].pose.position.x == pytest.approx(10.0)
    assert spheres[0].pose.position.y == pytest.approx(21.0)


def test_invalid_zmp_no_marker():
    by_id = trajectory_steps_by_id([_step(1, 0.0, 0.0)])
    pred = SimpleNamespace(
        header=_header(),
        rollover_steps=[
            SimpleNamespace(
                step_id=1,
                zmp=SimpleNamespace(valid=False, x=0.0, y=0.0, margin_m=0.0),
            )
        ],
    )
    arr = build_zmp_markers(pred, by_id, "map")
    assert not any(m.action == Marker.ADD for m in arr.markers)


def test_old_prediction_does_not_match_new_trajectory_lookup():
    by_id_new = trajectory_steps_by_id([_step(100, 1.0, 1.0)])
    pred_old = SimpleNamespace(
        header=_header(),
        source_trajectory_id=1,
        collision_steps=[
            SimpleNamespace(
                step_id=0,
                collision_objects=[SimpleNamespace(min_distance_m=0.0)],
            )
        ],
    )
    arr = build_collision_markers(pred_old, by_id_new, "map")
    assert not any(m.type == Marker.SPHERE for m in arr.markers)


def test_stale_marker_deletion_namespaces():
    arr = clear_prediction_overlays("map")
    nss = {m.ns for m in arr.markers}
    assert "collision" in nss
    assert "rollover" in nss
    assert "zmp" in nss
    assert "status" in nss
    assert all(m.action == Marker.DELETEALL for m in arr.markers)


def test_finite_value_handling_skips_nan_steps():
    by_id = trajectory_steps_by_id(
        [_step(1, float("nan"), 0.0), _step(2, 1.0, 2.0)]
    )
    assert 1 not in by_id
    assert 2 in by_id
