# Landfill Rover Prediction Core — short developer commands
#
# Usage (from this directory):
#   make help
#   make test
#   make demo-viz          # all canonical demos → outputs/runtime_demos/<name>/
#   make demo-flat         # one static flat demo + plots
#   make demo-dynamic      # dynamic-profile demos + plots
#   make runtime SCENARIO=mock/runtime_scenarios/demo_side_slope_15deg.json PROFILE=static
#   make mocks
#   make scenario FILE=mock/scenarios/side_slope_15deg.json

PY         := .venv/bin/python
CONFIG     := config/rover.mock.yaml
OUTDIR     := outputs/mock_validation
DEMO_OUT   := outputs/runtime_demos
RUNTIME_DIR := mock/runtime_scenarios
CACHE      := MPLCONFIGDIR=.cache/matplotlib XDG_CACHE_HOME=.cache
PROFILE    ?= static
SCENARIO   ?=

.PHONY: help install test ros-test ros-build mocks scenario \
	demo demo-viz demo-flat demo-dynamic demo-slope list-demos runtime open-demos

help:
	@echo "Prediction Python V1 — make targets"
	@echo ""
	@echo "  make install              # create .venv and install package + pytest"
	@echo "  make test                 # run pytest"
	@echo ""
	@echo "  Runtime demos (event streams + PNG/JSON artifacts):"
	@echo "  make demo-viz             # all demo_*.json with default profiles"
	@echo "  make demo-flat            # flat static + plots"
	@echo "  make demo-slope           # side slope 15° + plots"
	@echo "  make demo-dynamic         # dynamic profile demos + plots"
	@echo "  make runtime SCENARIO=... PROFILE=static|dynamic"
	@echo "  make list-demos           # list runtime scenario files"
	@echo "  make open-demos           # print artifact paths under $(DEMO_OUT)"
	@echo ""
	@echo "  Algorithm mocks (PredictionCore, no runtime profile):"
	@echo "  make mocks                # all mock/scenarios/*.json"
	@echo "  make scenario FILE=mock/scenarios/<name>.json"
	@echo ""
	@echo "  ROS (optional — requires sourced ROS 2 + colcon):"
	@echo "  make ros-test / ros-build"
	@echo "  ros2 launch prediction_ros prediction.launch.py config_path:=... prediction_profile:=dynamic"

install:
	python3 -m venv .venv
	$(PY) -m pip install -e '.[test]'

test:
	$(PY) -m pytest -q

list-demos:
	@ls -1 $(RUNTIME_DIR)/*.json

# Quiet text-only smoke (no plots). Prefer demo-viz for readable outputs.
demo:
	$(PY) -m prediction_core.replay --version
	$(PY) -m prediction_core.replay --profile static --config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_flat_static.json --quiet
	$(PY) -m prediction_core.replay --profile static --config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_side_slope_15deg.json --quiet
	$(PY) -m prediction_core.replay --profile static --config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_dynamic_lateral_acceleration.json --quiet
	$(PY) -m prediction_core.replay --profile static --config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_external_force_height.json --quiet
	$(PY) -m prediction_core.replay --profile static --config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_missing_acceleration.json --quiet

# Canonical demos with visualization artifacts under outputs/runtime_demos/
demo-viz: demo-flat demo-slope demo-dynamic
	@echo ""
	@echo "All demo artifacts under: $(DEMO_OUT)/"
	@find $(DEMO_OUT) -type f \( -name '*.png' -o -name '*.txt' -o -name '*.json' \) | sort

demo-flat:
	$(CACHE) $(PY) -m prediction_core.replay \
		--profile static \
		--config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_flat_static.json \
		--quiet \
		--viz-dir $(DEMO_OUT)/demo_flat_static

demo-slope:
	$(CACHE) $(PY) -m prediction_core.replay \
		--profile static \
		--config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_side_slope_15deg.json \
		--quiet \
		--viz-dir $(DEMO_OUT)/demo_side_slope_15deg

demo-dynamic:
	$(CACHE) $(PY) -m prediction_core.replay \
		--profile dynamic \
		--config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_dynamic_waits_for_state.json \
		--quiet \
		--viz-dir $(DEMO_OUT)/demo_dynamic_waits_for_state
	$(CACHE) $(PY) -m prediction_core.replay \
		--profile dynamic \
		--config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_dynamic_zero_acceleration.json \
		--quiet \
		--viz-dir $(DEMO_OUT)/demo_dynamic_zero_acceleration
	$(CACHE) $(PY) -m prediction_core.replay \
		--profile static \
		--config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_dynamic_lateral_acceleration.json \
		--quiet \
		--viz-dir $(DEMO_OUT)/demo_dynamic_lateral_acceleration
	$(CACHE) $(PY) -m prediction_core.replay \
		--profile static \
		--config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_external_force_height.json \
		--quiet \
		--viz-dir $(DEMO_OUT)/demo_external_force_height
	$(CACHE) $(PY) -m prediction_core.replay \
		--profile static \
		--config $(CONFIG) \
		--scenario $(RUNTIME_DIR)/demo_missing_acceleration.json \
		--quiet \
		--viz-dir $(DEMO_OUT)/demo_missing_acceleration

# One-off: make runtime SCENARIO=mock/runtime_scenarios/foo.json PROFILE=dynamic
runtime:
	@test -n "$(SCENARIO)" || (echo "Usage: make runtime SCENARIO=$(RUNTIME_DIR)/<name>.json PROFILE=static|dynamic" && exit 1)
	$(CACHE) $(PY) -m prediction_core.replay \
		--profile $(PROFILE) \
		--config $(CONFIG) \
		--scenario $(SCENARIO) \
		--quiet \
		--viz-dir $(DEMO_OUT)/$(notdir $(basename $(SCENARIO)))_$(PROFILE)
	@echo "Open: $(DEMO_OUT)/$(notdir $(basename $(SCENARIO)))_$(PROFILE)/rollover_profile.png"

open-demos:
	@echo "Artifact roots:"
	@ls -1d $(DEMO_OUT)/*/ 2>/dev/null || echo "(none yet — run make demo-viz)"

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
