"""Unit tests for partial GeometryArray build semantics (no ROS spin)."""

from __future__ import annotations

from types import SimpleNamespace

from lr_prediction_bridge.geometry_partial import build_geometry_steps


def _steps(*ids: int):
    return [SimpleNamespace(step_id=i, x=float(i), y=0.0) for i in ids]


def _lookup_from_set(valid: set[int]):
    def lookup(x: float, y: float):
        step_id = int(round(x))
        if step_id not in valid:
            return None
        return (0.0, 0.0, 1.0, 0.9, True, f"plane-{step_id}")

    return lookup


def test_all_normals_valid_publishes_all_steps():
    result = build_geometry_steps(_steps(0, 1, 2), _lookup_from_set({0, 1, 2}))
    assert result.should_publish
    assert [s.step_id for s in result.steps] == [0, 1, 2]
    assert result.missing_step_ids == ()
    assert result.coverage_ratio == 1.0


def test_one_normal_missing_in_middle_publishes_remaining():
    result = build_geometry_steps(_steps(0, 1, 2, 3), _lookup_from_set({0, 1, 3}))
    assert [s.step_id for s in result.steps] == [0, 1, 3]
    assert result.missing_step_ids == (2,)
    assert result.coverage_ratio == 0.75


def test_first_normal_missing_later_steps_still_published():
    result = build_geometry_steps(_steps(0, 1, 2), _lookup_from_set({1, 2}))
    assert [s.step_id for s in result.steps] == [1, 2]
    assert result.missing_step_ids == (0,)


def test_last_normal_missing_earlier_steps_published():
    result = build_geometry_steps(_steps(0, 1, 2), _lookup_from_set({0, 1}))
    assert [s.step_id for s in result.steps] == [0, 1]
    assert result.missing_step_ids == (2,)


def test_multiple_missing_only_valid_step_ids_published():
    result = build_geometry_steps(
        _steps(0, 1, 2, 3, 4, 5), _lookup_from_set({0, 1, 3, 5})
    )
    assert [s.step_id for s in result.steps] == [0, 1, 3, 5]
    assert result.missing_step_ids == (2, 4)


def test_zero_valid_normals_should_not_publish():
    result = build_geometry_steps(_steps(0, 1, 2), _lookup_from_set(set()))
    assert not result.should_publish
    assert result.steps == ()
    assert result.missing_step_ids == (0, 1, 2)
    assert result.coverage_ratio == 0.0


def test_step_ids_preserved_not_renumbered():
    # Trajectory step_ids are not dense array indices.
    traj = [
        SimpleNamespace(step_id=10, x=10.0, y=0.0),
        SimpleNamespace(step_id=11, x=11.0, y=0.0),
        SimpleNamespace(step_id=12, x=12.0, y=0.0),
    ]
    result = build_geometry_steps(traj, _lookup_from_set({10, 12}))
    assert [s.step_id for s in result.steps] == [10, 12]


def test_normals_and_plane_ids_copied_from_lookup():
    def lookup(x, y):
        return (0.1, 0.2, 0.9798, 0.55, True, "terrain-r1-c2")

    result = build_geometry_steps(_steps(7), lookup)
    assert result.steps[0].plane_id == "terrain-r1-c2"
    assert result.steps[0].normal_x == 0.1
    assert result.steps[0].confidence == 0.55


def test_no_flat_fallback_when_disabled():
    result = build_geometry_steps(
        _steps(0, 1),
        _lookup_from_set({0}),
        allow_flat_fallback=False,
    )
    assert [s.step_id for s in result.steps] == [0]
    assert result.used_flat_fallback_count == 0
    assert all(not s.plane_id.startswith("flat-fallback") for s in result.steps)


def test_flat_fallback_unchanged_when_enabled():
    result = build_geometry_steps(
        _steps(0, 1),
        _lookup_from_set({0}),
        allow_flat_fallback=True,
        flat_fallback_confidence=0.25,
    )
    assert [s.step_id for s in result.steps] == [0, 1]
    assert result.used_flat_fallback_count == 1
    flat = result.steps[1]
    assert flat.plane_id == "flat-fallback-1"
    assert flat.normal_x == 0.0
    assert flat.normal_y == 0.0
    assert flat.normal_z == 1.0
    assert flat.confidence == 0.25
    assert flat.confidence_valid is True
    assert result.should_publish
