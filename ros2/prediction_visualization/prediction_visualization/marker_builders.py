"""Pure marker / path builders for Prediction RViz visualization.

No ROS subscriptions. No Decision / invented thresholds. No physics changes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Sequence

from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


def finite(v: float) -> bool:
    return math.isfinite(v)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = 0.5 * yaw
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def local_xy_to_map(
    origin_x: float, origin_y: float, yaw: float, local_x: float, local_y: float
) -> tuple[float, float]:
    """Support-plane / body-local XY → map (same as rover_rectangle corners)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (
        origin_x + c * local_x - s * local_y,
        origin_y + s * local_x + c * local_y,
    )


def zmp_local_to_map(
    step_x: float, step_y: float, step_yaw: float, zmp_x: float, zmp_y: float
) -> tuple[float, float]:
    """ZmpEvidence (rover +X forward, +Y left) → map."""
    return local_xy_to_map(step_x, step_y, step_yaw, zmp_x, zmp_y)


@dataclass(frozen=True)
class StepPose:
    step_id: int
    x: float
    y: float
    yaw: float


def trajectory_steps_by_id(steps: Sequence[Any]) -> dict[int, StepPose]:
    """Index trajectory steps by step_id (not array index)."""
    out: dict[int, StepPose] = {}
    for step in steps:
        sid = int(step.step_id)
        x, y, yaw = float(step.x), float(step.y), float(step.yaw)
        if not (finite(x) and finite(y) and finite(yaw)):
            continue
        out[sid] = StepPose(step_id=sid, x=x, y=y, yaw=yaw)
    return out


def lookup_step(by_id: Mapping[int, StepPose], step_id: int) -> StepPose | None:
    return by_id.get(int(step_id))


def _as_time(stamp: Any | None) -> Time | None:
    if stamp is None:
        return None
    if isinstance(stamp, Time):
        return stamp
    if hasattr(stamp, "sec") and hasattr(stamp, "nanosec"):
        t = Time()
        t.sec = int(stamp.sec)
        t.nanosec = int(stamp.nanosec)
        return t
    return None


def _header(frame_id: str, stamp: Any | None = None) -> Header:
    h = Header()
    h.frame_id = frame_id
    t = _as_time(stamp)
    if t is not None:
        h.stamp = t
    return h


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _point(x: float, y: float, z: float = 0.0) -> Point:
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def delete_all_marker(ns: str, frame_id: str, stamp: Any | None = None) -> Marker:
    m = Marker()
    m.header = _header(frame_id, stamp)
    m.ns = ns
    m.id = 0
    m.action = Marker.DELETEALL
    return m


def clear_namespaces(frame_id: str, namespaces: Iterable[str], stamp: Any | None = None) -> MarkerArray:
    arr = MarkerArray()
    for ns in namespaces:
        arr.markers.append(delete_all_marker(ns, frame_id, stamp))
    return arr


def build_trajectory_path(
    trajectory: Any, frame_id: str, stamp: Any | None = None
) -> Path:
    path = Path()
    path.header = _header(frame_id, stamp if stamp is not None else trajectory.header.stamp)
    for step in trajectory.steps:
        x, y, yaw = float(step.x), float(step.y), float(step.yaw)
        if not (finite(x) and finite(y) and finite(yaw)):
            continue
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position = _point(x, y, 0.0)
        ps.pose.orientation = yaw_to_quaternion(yaw)
        path.poses.append(ps)
    return path


def build_trajectory_step_markers(
    trajectory: Any,
    frame_id: str,
    *,
    enabled: bool,
    z_lift_m: float = 0.05,
    stamp: Any | None = None,
) -> MarkerArray:
    arr = MarkerArray()
    stamp = stamp if stamp is not None else trajectory.header.stamp
    arr.markers.append(delete_all_marker("trajectory_steps", frame_id, stamp))
    if not enabled:
        return arr
    mid = 1
    for step in trajectory.steps:
        x, y = float(step.x), float(step.y)
        if not (finite(x) and finite(y)):
            continue
        m = Marker()
        m.header = _header(frame_id, stamp)
        m.ns = "trajectory_steps"
        m.id = mid
        mid += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = _point(x, y, z_lift_m)
        m.pose.orientation.w = 1.0
        m.scale = Vector3(x=0.08, y=0.08, z=0.08)
        m.color = _color(0.15, 0.75, 0.95, 0.85)
        arr.markers.append(m)
    return arr


def build_object_markers(
    tracked: Any,
    frame_id: str,
    *,
    label_enabled: bool,
    z_lift_m: float = 0.02,
    stamp: Any | None = None,
) -> MarkerArray:
    """Empty objects → DELETEALL only (valid, no exception)."""
    arr = MarkerArray()
    stamp = stamp if stamp is not None else tracked.header.stamp
    arr.markers.append(delete_all_marker("objects", frame_id, stamp))
    arr.markers.append(delete_all_marker("object_labels", frame_id, stamp))
    objects = list(tracked.objects)
    mid = 1
    lid = 1
    for obj in objects:
        pts = []
        for p in obj.footprint_polygon_xy:
            x, y = float(p.x), float(p.y)
            if not (finite(x) and finite(y)):
                pts = []
                break
            pts.append((x, y))
        if len(pts) < 3:
            continue
        # Closed line strip
        m = Marker()
        m.header = _header(frame_id, stamp)
        m.ns = "objects"
        m.id = mid
        mid += 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.04
        m.color = _color(0.95, 0.55, 0.1, 0.95)
        for x, y in pts:
            m.points.append(_point(x, y, z_lift_m))
        # close polygon
        m.points.append(_point(pts[0][0], pts[0][1], z_lift_m))
        arr.markers.append(m)

        if label_enabled:
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            t = Marker()
            t.header = _header(frame_id, stamp)
            t.ns = "object_labels"
            t.id = lid
            lid += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position = _point(cx, cy, z_lift_m + 0.25)
            t.pose.orientation.w = 1.0
            t.scale.z = 0.18
            t.color = _color(1.0, 0.9, 0.7, 0.95)
            t.text = f"{obj.class_name}:{int(obj.track_id)}"
            arr.markers.append(t)
    return arr


def _normal_is_displayable(nx: float, ny: float, nz: float) -> bool:
    if not (finite(nx) and finite(ny) and finite(nz)):
        return False
    return math.sqrt(nx * nx + ny * ny + nz * nz) > 1e-12


def build_terrain_normal_markers(
    geometry: Any,
    steps_by_id: Mapping[int, StepPose],
    frame_id: str,
    *,
    stride: int,
    arrow_scale: float,
    stamp: Any | None = None,
) -> MarkerArray:
    """Render provided normals faithfully; never fabricate or flip."""
    arr = MarkerArray()
    stamp = stamp if stamp is not None else geometry.header.stamp
    arr.markers.append(delete_all_marker("terrain_normals", frame_id, stamp))
    stride = max(1, int(stride))
    mid = 1
    for i, gs in enumerate(geometry.steps):
        if i % stride != 0:
            continue
        pose = lookup_step(steps_by_id, int(gs.step_id))
        if pose is None:
            continue
        nx = float(gs.normal.x)
        ny = float(gs.normal.y)
        nz = float(gs.normal.z)
        if not _normal_is_displayable(nx, ny, nz):
            continue
        # Faithful direction (no upward flip)
        mag = math.sqrt(nx * nx + ny * ny + nz * nz)
        ux, uy, uz = nx / mag, ny / mag, nz / mag
        m = Marker()
        m.header = _header(frame_id, stamp)
        m.ns = "terrain_normals"
        m.id = mid
        mid += 1
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.04
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color = _color(0.35, 0.85, 0.45, 0.9)
        start = _point(pose.x, pose.y, 0.05)
        end = _point(
            pose.x + ux * arrow_scale,
            pose.y + uy * arrow_scale,
            0.05 + uz * arrow_scale,
        )
        m.points = [start, end]
        arr.markers.append(m)
    return arr


def build_rover_markers(
    rover_state: Any,
    frame_id: str,
    *,
    body_length_m: float,
    body_width_m: float,
    body_height_m: float,
    heading_arrow_length_m: float,
    z_lift_m: float = 0.05,
    stamp: Any | None = None,
) -> MarkerArray:
    arr = MarkerArray()
    stamp = stamp if stamp is not None else rover_state.header.stamp
    for ns in ("rover_body", "rover_heading", "rover_text"):
        arr.markers.append(delete_all_marker(ns, frame_id, stamp))
    if not bool(rover_state.pose_valid):
        return arr
    p = rover_state.pose.position
    o = rover_state.pose.orientation
    if not all(finite(float(v)) for v in (p.x, p.y, p.z, o.x, o.y, o.z, o.w)):
        return arr

    body = Marker()
    body.header = _header(frame_id, stamp)
    body.ns = "rover_body"
    body.id = 1
    body.type = Marker.CUBE
    body.action = Marker.ADD
    body.pose = rover_state.pose
    body.pose.position.z = float(p.z) + z_lift_m + body_height_m * 0.5
    body.scale = Vector3(x=body_length_m, y=body_width_m, z=body_height_m)
    body.color = _color(0.2, 0.45, 0.95, 0.75)
    arr.markers.append(body)

    # Heading from yaw of orientation
    yaw = math.atan2(
        2.0 * (o.w * o.z + o.x * o.y),
        1.0 - 2.0 * (o.y * o.y + o.z * o.z),
    )
    arrow = Marker()
    arrow.header = _header(frame_id, stamp)
    arrow.ns = "rover_heading"
    arrow.id = 1
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    arrow.pose.orientation.w = 1.0
    arrow.scale.x = 0.06
    arrow.scale.y = 0.12
    arrow.scale.z = 0.12
    arrow.color = _color(0.95, 0.95, 0.2, 0.95)
    z = float(p.z) + z_lift_m + body_height_m * 0.5
    tip_x = float(p.x) + math.cos(yaw) * heading_arrow_length_m
    tip_y = float(p.y) + math.sin(yaw) * heading_arrow_length_m
    arrow.points = [_point(p.x, p.y, z), _point(tip_x, tip_y, z)]
    arr.markers.append(arrow)

    lines = []
    if bool(rover_state.twist_valid):
        vx = float(rover_state.twist.linear.x)
        vy = float(rover_state.twist.linear.y)
        if finite(vx) and finite(vy):
            lines.append(f"v={math.hypot(vx, vy):.2f} m/s")
    if bool(rover_state.acceleration_valid):
        ax = float(rover_state.acceleration.linear.x)
        ay = float(rover_state.acceleration.linear.y)
        az = float(rover_state.acceleration.linear.z)
        if finite(ax) and finite(ay) and finite(az):
            lines.append(f"|a|={math.sqrt(ax*ax+ay*ay+az*az):.2f} m/s²")
    if lines:
        text = Marker()
        text.header = _header(frame_id, stamp)
        text.ns = "rover_text"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = _point(p.x, p.y, z + 0.35)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.16
        text.color = _color(0.9, 0.95, 1.0, 0.95)
        text.text = "\n".join(lines)
        arr.markers.append(text)
    return arr


def build_collision_markers(
    prediction: Any,
    steps_by_id: Mapping[int, StepPose],
    frame_id: str,
    *,
    z_lift_m: float = 0.12,
    stamp: Any | None = None,
) -> MarkerArray:
    arr = MarkerArray()
    stamp = stamp if stamp is not None else prediction.header.stamp
    arr.markers.append(delete_all_marker("collision", frame_id, stamp))
    arr.markers.append(delete_all_marker("collision_text", frame_id, stamp))
    mid = 1
    tid = 1
    for cs in prediction.collision_steps:
        pose = lookup_step(steps_by_id, int(cs.step_id))
        if pose is None:
            continue
        # Evidence exists iff collision_objects present (contract).
        if not list(cs.collision_objects):
            continue
        m = Marker()
        m.header = _header(frame_id, stamp)
        m.ns = "collision"
        m.id = mid
        mid += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = _point(pose.x, pose.y, z_lift_m)
        m.pose.orientation.w = 1.0
        m.scale = Vector3(x=0.28, y=0.28, z=0.28)
        m.color = _color(0.95, 0.15, 0.12, 0.9)
        arr.markers.append(m)

        n = len(cs.collision_objects)
        dmin = min(float(o.min_distance_m) for o in cs.collision_objects)
        if not finite(dmin):
            continue
        t = Marker()
        t.header = _header(frame_id, stamp)
        t.ns = "collision_text"
        t.id = tid
        tid += 1
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position = _point(pose.x, pose.y, z_lift_m + 0.25)
        t.pose.orientation.w = 1.0
        t.scale.z = 0.15
        t.color = _color(1.0, 0.6, 0.55, 0.95)
        t.text = f"col n={n} d={dmin:.2f}m"
        arr.markers.append(t)
    return arr


def build_rollover_markers(
    prediction: Any,
    steps_by_id: Mapping[int, StepPose],
    frame_id: str,
    *,
    z_lift_m: float = 0.1,
    stamp: Any | None = None,
) -> MarkerArray:
    """Show reported rollover diagnostics only — no invented unsafe thresholds."""
    arr = MarkerArray()
    stamp = stamp if stamp is not None else prediction.header.stamp
    arr.markers.append(delete_all_marker("rollover", frame_id, stamp))
    arr.markers.append(delete_all_marker("rollover_text", frame_id, stamp))
    mid = 1
    tid = 1
    for rs in prediction.rollover_steps:
        pose = lookup_step(steps_by_id, int(rs.step_id))
        if pose is None:
            continue
        ssm = float(rs.static_stability_margin_m)
        if not finite(ssm):
            continue
        m = Marker()
        m.header = _header(frame_id, stamp)
        m.ns = "rollover"
        m.id = mid
        mid += 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position = _point(pose.x, pose.y, z_lift_m)
        m.pose.orientation.w = 1.0
        m.scale = Vector3(x=0.18, y=0.18, z=0.06)
        m.color = _color(0.85, 0.35, 0.9, 0.75)
        arr.markers.append(m)

        sm = rs.stability_moment
        if bool(sm.valid) and finite(float(sm.minimum_stability_moment_nm)):
            sm_txt = f"SM={float(sm.minimum_stability_moment_nm):.0f}Nm"
        else:
            sm_txt = "SM=N/A"
        t = Marker()
        t.header = _header(frame_id, stamp)
        t.ns = "rollover_text"
        t.id = tid
        tid += 1
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position = _point(pose.x, pose.y, z_lift_m + 0.22)
        t.pose.orientation.w = 1.0
        t.scale.z = 0.12
        t.color = _color(0.95, 0.8, 1.0, 0.9)
        t.text = f"ssm={ssm:.2f} {sm_txt}"
        arr.markers.append(t)
    return arr


def build_zmp_markers(
    prediction: Any,
    steps_by_id: Mapping[int, StepPose],
    frame_id: str,
    *,
    z_lift_m: float = 0.08,
    stamp: Any | None = None,
) -> MarkerArray:
    arr = MarkerArray()
    stamp = stamp if stamp is not None else prediction.header.stamp
    arr.markers.append(delete_all_marker("zmp", frame_id, stamp))
    mid = 1
    for rs in prediction.rollover_steps:
        zmp = rs.zmp
        if not bool(zmp.valid):
            continue
        zx, zy = float(zmp.x), float(zmp.y)
        if not (finite(zx) and finite(zy)):
            continue
        pose = lookup_step(steps_by_id, int(rs.step_id))
        if pose is None:
            continue
        mx, my = zmp_local_to_map(pose.x, pose.y, pose.yaw, zx, zy)
        if not (finite(mx) and finite(my)):
            continue
        sphere = Marker()
        sphere.header = _header(frame_id, stamp)
        sphere.ns = "zmp"
        sphere.id = mid
        mid += 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = _point(mx, my, z_lift_m)
        sphere.pose.orientation.w = 1.0
        sphere.scale = Vector3(x=0.12, y=0.12, z=0.12)
        sphere.color = _color(0.15, 0.95, 0.95, 0.95)
        arr.markers.append(sphere)

        stem = Marker()
        stem.header = _header(frame_id, stamp)
        stem.ns = "zmp"
        stem.id = mid
        mid += 1
        stem.type = Marker.ARROW
        stem.action = Marker.ADD
        stem.pose.orientation.w = 1.0
        stem.scale.x = 0.03
        stem.scale.y = 0.05
        stem.scale.z = 0.05
        stem.color = _color(0.1, 0.8, 0.85, 0.85)
        stem.points = [_point(mx, my, 0.0), _point(mx, my, z_lift_m)]
        arr.markers.append(stem)
    return arr


def build_status_markers(
    prediction: Any | None,
    matched_trajectory_id: int | None,
    frame_id: str,
    *,
    enabled: bool,
    anchor_xy: tuple[float, float] | None,
    stamp: Any | None = None,
    decision_evidence: Any | None = None,
    decision_output: Any | None = None,
) -> MarkerArray:
    arr = MarkerArray()
    arr.markers.append(delete_all_marker("status", frame_id, stamp))
    if not enabled or matched_trajectory_id is None or anchor_xy is None:
        return arr
    ax, ay = anchor_xy
    lines: list[str] = [f"traj {matched_trajectory_id}"]
    pred_matches = (
        prediction is not None
        and int(prediction.source_trajectory_id) == int(matched_trajectory_id)
    )
    if pred_matches:
        n_col = len(prediction.collision_steps)
        n_roll = len(prediction.rollover_steps)
        sm_txt = "N/A"
        zmp_txt = "N/A"
        if prediction.rollover_steps:
            rs0 = prediction.rollover_steps[0]
            sm = rs0.stability_moment
            if bool(sm.valid) and finite(float(sm.minimum_stability_moment_nm)):
                sm_txt = f"{float(sm.minimum_stability_moment_nm):.1f} Nm"
            z = rs0.zmp
            if bool(z.valid) and finite(float(z.margin_m)):
                zmp_txt = f"m={float(z.margin_m):.3f}"
        lines.append(f"collision_steps={n_col} rollover_steps={n_roll}")
        lines.append(f"SM {sm_txt}  ZMP {zmp_txt}")
    if decision_evidence is not None:
        state_map = {0: "NONE", 1: "STALE", 2: "CURRENT"}
        ev_txt = state_map.get(int(decision_evidence.evidence_state), "?")
        if int(decision_evidence.evidence_state) == 2:
            col_txt = "yes" if bool(decision_evidence.collision_candidates_present) else "no"
            sm_ev = "valid" if bool(decision_evidence.dynamic_stability_moment_valid) else "N/A"
            zmp_ev = "valid" if bool(decision_evidence.zmp_valid) else "N/A"
        else:
            col_txt = "N/A"
            sm_ev = "N/A"
            zmp_ev = "N/A"
        lines.append(f"Prediction Evidence: {ev_txt}")
        lines.append(f"Collision candidates: {col_txt}")
        lines.append(f"Dynamic SM: {sm_ev}  ZMP: {zmp_ev}")
    if decision_output is not None:
        decision_txt = "GO" if int(decision_output.decision) == 0 else "STOP"
        reason_map = {
            0: "CURRENT_CLEAR",
            1: "NO_CURRENT_PREDICTION",
            2: "PREDICTION_STALE",
            3: "COLLISION_CANDIDATE",
            4: "ROLLOVER_EVIDENCE_INVALID",
            5: "ROLLOVER_POLICY_TRIGGERED",
        }
        reason_txt = reason_map.get(int(decision_output.reason), "?")
        lines.append("PROTOTYPE POLICY")
        lines.append(f"Decision: {decision_txt}")
        lines.append(f"Reason: {reason_txt}")
    if len(lines) == 1 and decision_evidence is None and decision_output is None:
        return arr
    text = Marker()
    text.header = _header(frame_id, stamp)
    text.ns = "status"
    text.id = 1
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose.position = _point(ax, ay, 1.2)
    text.pose.orientation.w = 1.0
    text.scale.z = 0.22
    if decision_output is not None and int(decision_output.decision) == 1:
        text.color = _color(1.0, 0.15, 0.15, 0.98)
        text.scale.z = 0.28
    elif decision_output is not None and int(decision_output.decision) == 0:
        text.color = _color(0.2, 0.95, 0.35, 0.95)
    else:
        text.color = _color(1.0, 1.0, 1.0, 0.95)
    text.text = "\n".join(lines)
    arr.markers.append(text)
    return arr


PREDICTION_MARKER_NAMESPACES = (
    "collision",
    "collision_text",
    "rollover",
    "rollover_text",
    "zmp",
    "status",
)


def clear_prediction_overlays(frame_id: str, stamp: Any | None = None) -> MarkerArray:
    return clear_namespaces(frame_id, PREDICTION_MARKER_NAMESPACES, stamp)
