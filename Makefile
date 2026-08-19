# Landfill Rover Prediction Core — short developer commands
#
# Usage (from this directory):
#   make test
#   make mocks
#   make scenario FILE=mock/scenarios/side_slope_15deg.json
#   make install

PY      := .venv/bin/python
CONFIG  := config/rover.mock.yaml
OUTDIR  := outputs/mock_validation
CACHE   := MPLCONFIGDIR=.cache/matplotlib XDG_CACHE_HOME=.cache

.PHONY: help install test ros-test ros-build mocks scenario

help:
	@echo "Targets:"
	@echo "  make install   # create .venv and install package + pytest"
	@echo "  make test      # run core + ROS wrapper pytest"
	@echo "  make ros-test  # run ROS wrapper pytest only"
	@echo "  make ros-build # build safety_perception_msgs + prediction_ros with colcon"
	@echo "  make mocks     # run all mock scenarios"
	@echo "  make scenario FILE=mock/scenarios/<name>.json"

install:
	python3 -m venv .venv
	$(PY) -m pip install -e '.[test]'

test:
	$(PY) -m pytest -q

ros-test:
	$(PY) -m pytest ros2/prediction_ros/test -q

ros-build:
	@test -n "$$ROS_DISTRO" || (echo "Source ROS 2 first: source /opt/ros/<distro>/setup.bash" && exit 1)
	cd ros2 && colcon build --packages-select safety_perception_msgs prediction_ros

mocks:
	$(CACHE) $(PY) -m prediction_core.cli run-all-mocks \
		--config $(CONFIG) \
		--output $(OUTDIR)

scenario:
	@test -n "$(FILE)" || (echo "Usage: make scenario FILE=mock/scenarios/<name>.json" && exit 1)
	$(CACHE) $(PY) -m prediction_core.cli run-scenario $(FILE) \
		--config $(CONFIG) \
		--output outputs/$(notdir $(basename $(FILE)))
