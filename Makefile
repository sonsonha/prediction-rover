# Landfill Rover Prediction Core — short developer commands
#
# Usage (from this directory):
#   make help
#   make static-node / static-echo / static-mock   # 3-terminal ROS static
#   make dynamic-node / dynamic-echo / dynamic-mock
#   make test
#   make demo-viz
#   make runtime SCENARIO=mock/runtime_scenarios/demo_side_slope_15deg.json PROFILE=static

PY         := .venv/bin/python
CONFIG     := config/rover.mock.yaml
OUTDIR     := outputs/mock_validation
DEMO_OUT   := outputs/runtime_demos
RUNTIME_DIR := mock/runtime_scenarios
SCRIPTS    := scripts
CACHE      := MPLCONFIGDIR=.cache/matplotlib XDG_CACHE_HOME=.cache
PROFILE    ?= static
SCENARIO   ?=

HUMBLE_COMPOSE := docker/humble/docker-compose.yml
HUMBLE_RUN := docker compose -f $(HUMBLE_COMPOSE) run --rm prediction-humble bash -lc

.PHONY: help install test ros-test ros-build ros-status mocks scenario \
	demo demo-viz demo-flat demo-dynamic demo-slope list-demos runtime open-demos \
	static-node static-echo static-mock \
	dynamic-node dynamic-echo dynamic-mock dynamic-zero-mock dynamic-wrench-mock \
	humble-image humble-shell humble-install humble-test humble-build \
	humble-static humble-dynamic humble-bag humble-status

help:
	@echo "Prediction Python V1 — make targets"
	@echo ""
	@echo "STATIC  (3 terminals)"
	@echo "  make static-node           Terminal 1: Prediction static"
	@echo "  make static-echo           Terminal 2: Echo /predict_output"
	@echo "  make static-mock           Terminal 3: Mock static"
	@echo ""
	@echo "DYNAMIC  (3 terminals)"
	@echo "  make dynamic-node          Terminal 1: Prediction dynamic"
	@echo "  make dynamic-echo          Terminal 2: Echo /predict_output"
	@echo "  make dynamic-mock          Terminal 3: Mock acceleration case"
	@echo "  make dynamic-zero-mock     Terminal 3: Mock zero acceleration"
	@echo "  make dynamic-wrench-mock   Terminal 3: Mock external wrench"
	@echo ""
	@echo "ROS tooling (Jazzy host)"
	@echo "  make ros-status            Env snapshot (distro / python / pkg prefix)"
	@echo "  make ros-test / ros-build  Pytest for prediction_ros / colcon build"
	@echo ""
	@echo "Humble Docker (Ubuntu 22.04 integration)"
	@echo "  make humble-image          Build prediction-humble-dev image"
	@echo "  make humble-shell          Interactive shell in Humble container"
	@echo "  make humble-install        Python .venv-humble + pip install"
	@echo "  make humble-test           Pure Python pytest in container"
	@echo "  make humble-build          colcon -> ros2/install_humble"
	@echo "  make humble-static         Static mock smoke test"
	@echo "  make humble-dynamic        Dynamic mock smoke test"
	@echo "  make humble-bag            Short rosbag regression"
	@echo "  make humble-status         Env snapshot inside container"
	@echo ""
	@echo "Python package"
	@echo "  make install               # create .venv and install package + pytest"
	@echo "  make test                  # run pytest"
	@echo ""
	@echo "  Runtime demos (event streams + PNG/JSON artifacts):"
	@echo "  make demo-viz              # all demo_*.json with default profiles"
	@echo "  make demo-flat             # flat static + plots"
	@echo "  make demo-slope            # side slope 15° + plots"
	@echo "  make demo-dynamic          # dynamic profile demos + plots"
	@echo "  make runtime SCENARIO=... PROFILE=static|dynamic"
	@echo "  make list-demos            # list runtime scenario files"
	@echo "  make open-demos            # print artifact paths under $(DEMO_OUT)"
	@echo ""
	@echo "  Algorithm mocks (PredictionCore, no runtime profile):"
	@echo "  make mocks                 # all mock/scenarios/*.json"
	@echo "  make scenario FILE=mock/scenarios/<name>.json"

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

# --- ROS 3-terminal manual test helpers (no auto-rebuild; no background fan-out) ---
static-node:
	@$(SCRIPTS)/run_prediction.sh static

static-echo:
	@$(SCRIPTS)/run_echo.sh

static-mock:
	@$(SCRIPTS)/run_mock.sh static

dynamic-node:
	@$(SCRIPTS)/run_prediction.sh dynamic

dynamic-echo:
	@$(SCRIPTS)/run_echo.sh

dynamic-mock:
	@$(SCRIPTS)/run_mock.sh dynamic

dynamic-zero-mock:
	@$(SCRIPTS)/run_mock.sh dynamic_zero

dynamic-wrench-mock:
	@$(SCRIPTS)/run_mock.sh dynamic_wrench

ros-status:
	@$(SCRIPTS)/ros_status.sh

# --- Humble Docker workflow (does not touch Jazzy .venv / ros2/install) ---
humble-image:
	docker compose -f $(HUMBLE_COMPOSE) build

humble-shell:
	docker compose -f $(HUMBLE_COMPOSE) run --rm prediction-humble bash

humble-install:
	$(HUMBLE_RUN) "chmod +x scripts/*.sh && scripts/humble_install_python.sh"

humble-test: humble-install humble-build
	$(HUMBLE_RUN) "scripts/humble_test_python.sh"

humble-build:
	$(HUMBLE_RUN) "chmod +x scripts/*.sh && scripts/humble_build.sh"

humble-static: humble-build
	$(HUMBLE_RUN) "scripts/humble_smoke_mock.sh static"

humble-dynamic: humble-build
	$(HUMBLE_RUN) "scripts/humble_smoke_mock.sh dynamic"

humble-bag: humble-build
	$(HUMBLE_RUN) "scripts/humble_smoke_bag.sh /workspace/bags/manual_rollover_20260818_080815 8"

humble-status:
	$(HUMBLE_RUN) "source scripts/humble_env.sh && echo PROJECT_ROOT=\$$PROJECT_ROOT && echo ROS_DISTRO=\$$ROS_DISTRO && which python && python --version && ros2 pkg prefix prediction_ros && ros2 pkg prefix safety_perception_msgs"

mocks:
	$(CACHE) $(PY) -m prediction_core.cli run-all-mocks \
		--config $(CONFIG) \
		--output $(OUTDIR)

scenario:
	@test -n "$(FILE)" || (echo "Usage: make scenario FILE=mock/scenarios/<name>.json" && exit 1)
	$(CACHE) $(PY) -m prediction_core.cli run-scenario $(FILE) \
		--config $(CONFIG) \
		--output outputs/$(notdir $(basename $(FILE)))
