from pathlib import Path

import pytest

from prediction_core.config import RoverConfig, load_config


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def mock_config() -> RoverConfig:
    return load_config(ROOT / "config" / "rover.mock.yaml")

