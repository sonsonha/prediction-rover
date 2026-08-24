# Landfill Rover Prediction — Python V1

ROS-independent **safety evidence** package for a downstream Decision Node.

**Prediction Python V1** provides:

- exact 2D footprint collision candidates
- terrain-normal → predicted roll / pitch
- Static SSM + Normalized Static SSM (primary baseline)
- Stability Moment (primary dynamic, when acceleration is available)
- optional point-mass ZMP + secondary tip / effective-SSM diagnostics

It does **not** assign severity, risk, thresholds, or Stop/Go.

> `config/rover.mock.yaml` is TEST / MOCK ONLY — not measured rover parameters.

## Install and test

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python -m pytest -q
```

## Run the canonical Python demo

```bash
# Easy: Makefile targets (recommended)
make demo-viz          # all demos → outputs/runtime_demos/<name>/{*.png,*.json,*.txt}
make demo-flat         # flat static + plots
make demo-dynamic      # dynamic-profile demos + plots
make runtime SCENARIO=mock/runtime_scenarios/demo_side_slope_15deg.json PROFILE=static

# Or call the CLI directly
.venv/bin/python -m prediction_core.replay \
  --profile static \
  --config config/rover.mock.yaml \
  --scenario mock/runtime_scenarios/demo_flat_static.json \
  --viz-dir outputs/runtime_demos/demo_flat_static
```

Each `--viz-dir` / `make demo-*` folder contains:

- `rollover_profile.png` — attitude, SSM, Stability Moment, support snapshot
- `collision_topdown.png` — trajectory + footprints
- `prediction_output.json` — full evidence
- `run_summary.txt` — short readable snapshot

Default `--profile` is **static** (backward compatible).

**Profiles:** static requires trajectory + objects + matching geometry.
Dynamic additionally requires `RoverState` with valid `acceleration_xyz`
(`None` ≠ zero). External wrench stays optional (`None` ≠ `[]`). Prediction
still runs at most once per trajectory cycle.

Other demos: `demo_side_slope_15deg`, `demo_dynamic_lateral_acceleration`,
`demo_external_force_height`, `demo_missing_acceleration`,
`demo_dynamic_waits_for_state`, `demo_dynamic_zero_acceleration` under
`mock/runtime_scenarios/`.
Algorithm regression mocks with plots:

```bash
.venv/bin/python -m prediction_core.cli run-all-mocks \
  --config config/rover.mock.yaml \
  --output outputs/mock_validation
```

## Library API

```python
from prediction_core import (
    PredictionCore,
    PredictionProfile,
    PredictionRuntime,
    load_config,
)

config = load_config("config/rover.mock.yaml")
runtime = PredictionRuntime(config, profile=PredictionProfile.STATIC)  # default
core = PredictionCore(config)
output = core.predict(trajectory, tracked_objects, geometry, state=state)
# external_wrenches=None | [] | [...]
```
## Package layout

```text
prediction_core/     contracts, algorithms, runtime, canonical CLI
mock/scenarios/      low-level algorithm regression JSON
mock/runtime_scenarios/  full event-stream demos
visualization/       development plots (not Decision)
tests/               unit / invariant tests
documents/           technical docs
analysis/            method-comparison runner
ros2/                future adapter (not required for Python V1)
```

## Documentation

| Doc | Contents |
|---|---|
| [`documents/PREDICTION_PYTHON_V1.md`](documents/PREDICTION_PYTHON_V1.md) | **Canonical** V1 architecture & hierarchy |
| [`documents/DYNAMIC_ROLLOVER_METRICS.md`](documents/DYNAMIC_ROLLOVER_METRICS.md) | Metric formulas & availability semantics |
| [`ROLLOVER_METHOD_COMPARISON.md`](ROLLOVER_METHOD_COMPARISON.md) | Engineering comparison / retention decisions |
| [`documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md`](documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md) | Future ROS interface |

## Rollover hierarchy (summary)

```text
PRIMARY BASELINE     roll/pitch, Static SSM, Normalized SSM
PRIMARY DYNAMIC      Stability Moment
OPTIONAL DIAGNOSTIC  Point-mass ZMP
SECONDARY            Critical tip angle, Effective SSM
```

Effective SSM ≈ Point-mass ZMP under gravity + translational accel with empty wrenches.

## Known V1 limitations

- Discrete collision poses (no swept path); object velocity unused
- Point-mass dynamic model (no inertia tensor / full ZMP / LTR / FASM)
- Mock physical parameters
- ROS not validated in this phase

**Next step:** on a ROS 2 host, build `safety_perception_msgs` + `prediction_ros`,
launch with `prediction_profile:=static|dynamic`, and validate with mock upstream
or real bags. See [`documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md`](documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md).
