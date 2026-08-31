"""Unit tests for Detection3D → TrackedObject conversion."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose

from lr_prediction_bridge.detection3d_conversion import (
    convert_detection3d,
    convert_detection3d_array,
    frame_local_track_id,
    oriented_footprint_polygon_xy,
)


def _detection(
    *,
    detection_id: str = "1000:0",
    class_id: str = "pipe",
    score: float = 0.8,
    center=(10.0, 5.0, 0.0),
    size=(2.0, 4.0, 1.0),
    orientation=(0.0, 0.0, 0.0, 1.0),
) -> Detection3D:
    message = Detection3D()
    message.id = detection_id
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = class_id
    hypothesis.hypothesis.score = score
    message.results = [hypothesis]
    message.bbox.center.position.x = float(center[0])
    message.bbox.center.position.y = float(center[1])
    message.bbox.center.position.z = float(center[2])
    ox, oy, oz, ow = orientation
    message.bbox.center.orientation.x = ox
    message.bbox.center.orientation.y = oy
    message.bbox.center.orientation.z = oz
    message.bbox.center.orientation.w = ow
    message.bbox.size.x = float(size[0])
    message.bbox.size.y = float(size[1])
    message.bbox.size.z = float(size[2])
    return message


def test_axis_aligned_footprint_corners():
    polygon = oriented_footprint_polygon_xy(
        (10.0, 5.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        2.0,
        4.0,
    )
    assert polygon is not None
    assert len(polygon) == 4
    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    assert min(xs) == pytest.approx(9.0)
    assert max(xs) == pytest.approx(11.0)
    assert min(ys) == pytest.approx(3.0)
    assert max(ys) == pytest.approx(7.0)


def test_yaw_90_degree_footprint():
    half = math.pi / 4.0
    polygon = oriented_footprint_polygon_xy(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, math.sin(half), math.cos(half)),
        2.0,
        4.0,
    )
    assert polygon is not None
    xs = sorted(point[0] for point in polygon)
    ys = sorted(point[1] for point in polygon)
    assert xs[0] == pytest.approx(-2.0, abs=1e-6)
    assert xs[-1] == pytest.approx(2.0, abs=1e-6)
    assert ys[0] == pytest.approx(-1.0, abs=1e-6)
    assert ys[-1] == pytest.approx(1.0, abs=1e-6)


def test_convert_detection_already_map():
    converted = convert_detection3d(_detection(), 0, 1_000_000_000)
    assert converted is not None
    assert converted.class_name == "pipe"
    assert converted.confidence_valid is True
    assert converted.velocity_valid is False
    assert len(converted.footprint_polygon_xy) == 4


def test_empty_detection_array():
    converted, stats = convert_detection3d_array([], 123)
    assert converted == ()
    assert stats.received_detections == 0
    assert stats.converted_objects == 0


def test_velocity_unknown_semantics():
    converted = convert_detection3d(_detection(), 0, 123)
    assert converted is not None
    assert converted.velocity_valid is False


def test_multiple_detections_unique_ids():
    detections = [
        _detection(detection_id="1000:0", class_id="pipe"),
        _detection(detection_id="1000:1", class_id="person"),
    ]
    converted, stats = convert_detection3d_array(detections, 1_000)
    assert stats.converted_objects == 2
    ids = {item.track_id for item in converted}
    assert len(ids) == 2


def test_malformed_detection_skipped():
    bad = _detection(size=(0.0, 4.0, 1.0))
    converted, stats = convert_detection3d_array([bad], 1_000)
    assert converted == ()
    assert stats.skipped_invalid == 1


def test_frame_local_track_id_from_detection_id():
    track_id = frame_local_track_id("1783700699279984000:0", 0, 999)
    assert isinstance(track_id, int)
    assert track_id >= 0
    other = frame_local_track_id("1783700699279984000:1", 1, 999)
    assert track_id != other


def test_transform_to_map():
    polygon = oriented_footprint_polygon_xy(
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        2.0,
        2.0,
        transform_rotation=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        transform_translation=(10.0, 5.0, 0.0),
    )
    assert polygon is not None
    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    # center (1,0,0) + size 2x2 → x∈[0,2], y∈[-1,1]; then +translation (10,5)
    assert min(xs) == pytest.approx(10.0)
    assert max(xs) == pytest.approx(12.0)
    assert min(ys) == pytest.approx(4.0)
    assert max(ys) == pytest.approx(6.0)


def test_missing_hypothesis_rejected():
    message = Detection3D()
    message.id = "1000:0"
    message.results = []
    message.bbox.center.position.x = 1.0
    message.bbox.center.position.y = 2.0
    message.bbox.center.position.z = 3.0
    message.bbox.center.orientation.w = 1.0
    message.bbox.size.x = 1.0
    message.bbox.size.y = 1.0
    message.bbox.size.z = 1.0
    assert convert_detection3d(message, 0, 1000) is None
