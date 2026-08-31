"""Unit tests for adapter helpers (no ROS spin required)."""

from lr_prediction_bridge.helpers import subsample_indices, yaw_from_quaternion
import math


def test_yaw_identity():
    assert abs(yaw_from_quaternion(0.0, 0.0, 0.0, 1.0)) < 1e-12


def test_yaw_90deg():
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    assert abs(yaw_from_quaternion(0.0, 0.0, s, c) - math.pi / 2) < 1e-9


def test_subsample_stride():
    assert subsample_indices(10, horizon_steps=20, stride=2) == [0, 2, 4, 6, 8, 9]


def test_subsample_horizon_cap():
    idxs = subsample_indices(100, horizon_steps=5, stride=1)
    assert idxs[0] == 0
    assert idxs[-1] == 99
    assert len(idxs) <= 5
