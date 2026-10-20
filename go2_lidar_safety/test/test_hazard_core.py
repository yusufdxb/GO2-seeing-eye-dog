"""Pure tests for the LiDAR hazard classifier (no ROS)."""
import math

import numpy as np
import pytest
from go2_lidar_safety.hazard_core import (
    CLEAR,
    DROP_DETECTED,
    EMERGENCY_STOP,
    SLOWDOWN,
    HazardParams,
    evaluate,
    quaternion_matrix,
    restrict,
    severity,
    transform_points,
)

P = HazardParams()
NOSE = P.body_x_max


def wall_ahead(distance_from_nose, n=20, z=0.1):
    """A small cluster of returns straight ahead at a given clearance."""
    ys = np.linspace(-0.1, 0.1, n)
    return np.column_stack([np.full(n, NOSE + distance_from_nose), ys, np.full(n, z)])


def floor(n_x=30, n_y=9, x0=0.0, x1=3.0):
    xs, ys = np.meshgrid(np.linspace(x0, x1, n_x), np.linspace(-1.0, 1.0, n_y))
    return np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, P.floor_z)])


def test_empty_cloud_is_clear():
    result = evaluate(np.zeros((0, 3)), P)
    assert result.decision == CLEAR
    assert math.isinf(result.nearest_ahead_m)
    assert result.points_evaluated == 0


@pytest.mark.parametrize(
    "distance, expected",
    [(0.2, "RESTRICT:F"), (0.6, SLOWDOWN), (2.0, CLEAR)],
)
def test_obstacle_ahead_thresholds(distance, expected):
    result = evaluate(wall_ahead(distance), P)
    assert result.decision == expected
    assert result.nearest_ahead_m == pytest.approx(distance)


def test_obstacle_beside_corridor_is_ignored_ahead():
    # 0.2 m ahead of the nose but 0.8 m to the side: outside the corridor and
    # beyond the surround radius.
    pts = wall_ahead(0.2)
    pts[:, 1] += 0.8
    result = evaluate(pts, P)
    assert math.isinf(result.nearest_ahead_m)
    assert result.decision == CLEAR


def test_corridor_edge_is_inclusive_of_margin():
    pts = wall_ahead(0.2, n=5)
    pts[:, 1] = P.corridor_half_width - 0.01
    assert "F" in evaluate(pts, P).decision


def test_floor_only_is_clear():
    result = evaluate(floor(), P)
    assert result.decision == CLEAR
    assert result.points_evaluated == 30 * 9


def test_ceiling_is_ignored():
    pts = wall_ahead(0.2, z=P.z_max + 0.2)
    assert evaluate(pts, P).decision == CLEAR


def test_self_returns_inside_body_box_are_ignored():
    rng = np.random.default_rng(0)
    pts = np.column_stack(
        [
            rng.uniform(P.body_x_min, P.body_x_max, 500),
            rng.uniform(-P.body_half_width, P.body_half_width, 500),
            rng.uniform(P.z_min, P.z_max, 500),
        ]
    )
    result = evaluate(pts, P)
    assert result.decision == CLEAR
    assert math.isinf(result.nearest_surround_m)


def test_single_noise_point_does_not_stop():
    pts = np.array([[NOSE + 0.1, 0.0, 0.1]])
    assert evaluate(pts, P).decision == CLEAR


def test_min_obstacle_points_is_respected_exactly():
    pts = wall_ahead(0.1, n=P.min_obstacle_points)
    # 0.1 m ahead of the nose (0.50 m from base_link) is outside the 0.48 m
    # rotation sweep of the physical footprint: forward only.
    assert evaluate(pts, P).decision == "RESTRICT:F"
    pts = wall_ahead(0.1, n=P.min_obstacle_points - 1)
    assert evaluate(pts, P).decision == CLEAR


def test_nans_and_infs_are_dropped():
    pts = np.vstack([wall_ahead(0.6), [[np.nan, 0, 0], [np.inf, 0, 0], [0, np.nan, 1]]])
    result = evaluate(pts, P)
    assert result.decision == SLOWDOWN
    assert result.points_evaluated == 20


def test_all_nan_cloud_is_clear_with_zero_points():
    pts = np.full((10, 3), np.nan)
    result = evaluate(pts, P)
    assert result.decision == CLEAR
    assert result.points_evaluated == 0


def test_obstacle_hugging_the_side_forbids_rotation_only():
    # A wall 0.1 m from the flank, alongside the body, not ahead: a turn would
    # swing a corner into it, but driving straight past is safe.
    xs = np.linspace(P.body_x_min, P.body_x_max, 10)
    pts = np.column_stack([xs, np.full(10, P.body_half_width + 0.1), np.full(10, 0.1)])
    result = evaluate(pts, P)
    assert result.decision == "RESTRICT:W"
    assert result.nearest_surround_m == pytest.approx(0.1)
    assert result.distance == pytest.approx(0.1)


def test_obstacle_behind_does_not_count_ahead():
    pts = wall_ahead(0.1)
    pts[:, 0] = P.body_x_min - 0.5
    result = evaluate(pts, P)
    assert math.isinf(result.nearest_ahead_m)
    assert result.decision == CLEAR


def test_drop_check_disabled_by_default():
    assert evaluate(np.zeros((0, 3)), P).floor_points_ahead is None


def test_drop_check_flags_missing_floor():
    params = HazardParams(drop_check_enabled=True)
    assert evaluate(floor(), params).decision == CLEAR
    # Floor present only near the robot, missing beyond: a drop-off.
    near_only = floor(x0=0.0, x1=NOSE + 0.2)
    result = evaluate(near_only, params)
    assert result.decision == DROP_DETECTED
    assert result.floor_points_ahead == 0


def test_drop_outranks_a_directional_restriction():
    params = HazardParams(drop_check_enabled=True)
    assert evaluate(wall_ahead(0.1), params).decision == DROP_DETECTED


def test_boxed_in_is_an_emergency_stop_and_outranks_drop():
    params = HazardParams(drop_check_enabled=True)
    behind = wall_ahead(0.1)
    behind[:, 0] = P.body_x_min - 0.1
    side = np.column_stack([np.linspace(-0.2, 0.2, 5), np.full(5, 0.30), np.full(5, 0.1)])
    boxed = np.vstack([wall_ahead(0.1), behind, side])
    assert evaluate(boxed, params).decision == EMERGENCY_STOP
    # Front and rear blocked but free to turn: pivot, not a stop.
    assert evaluate(np.vstack([wall_ahead(0.1), behind]), P).decision == "RESTRICT:FB"


def test_obstacle_close_behind_forbids_reverse():
    pts = wall_ahead(0.1)
    pts[:, 0] = P.body_x_min - 0.2  # 0.2 m behind the tail, beyond the sweep radius
    assert evaluate(pts, P).decision == "RESTRICT:B"


def _doorway(width):
    xs = np.linspace(-0.2, 0.2, 8)
    half = width / 2
    left = np.column_stack([xs, np.full(8, half), np.full(8, 0.2)])
    right = np.column_stack([xs, np.full(8, -half), np.full(8, 0.2)])
    return np.vstack([left, right])


def test_centred_in_a_1m_doorway_heading_corrections_are_allowed():
    # Footprint corners reach 0.43 m; frames at 0.50 m. Regression: a sweep
    # built from the padded self-return box (0.55 m) stranded the robot here.
    assert "W" not in evaluate(_doorway(1.0), P).decision


def test_narrow_doorway_forbids_rotation_but_not_straight_travel():
    decision = evaluate(_doorway(0.9), P).decision
    assert decision == "RESTRICT:W"


def test_restriction_strings_and_severity_order():
    assert restrict("WF") == "RESTRICT:FW"
    assert restrict("BWF") == EMERGENCY_STOP
    assert severity(EMERGENCY_STOP) > severity(DROP_DETECTED) > severity("RESTRICT:FB")
    assert severity("RESTRICT:FB") > severity("RESTRICT:F") > severity(SLOWDOWN) > severity(CLEAR)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"stop_distance_m": 1.0, "slowdown_distance_m": 0.5},
        {"z_min": 0.5, "z_max": 0.1},
        {"min_obstacle_points": 0},
        {"stop_distance_m": float("nan")},
        {"body_x_min": 1.0, "body_x_max": 0.0},
    ],
)
def test_invalid_params_are_rejected(kwargs):
    with pytest.raises(ValueError):
        HazardParams(**kwargs)


def test_transform_rotates_and_translates():
    # 90 deg yaw: sensor x maps to base y.
    s = math.sqrt(0.5)
    rot = quaternion_matrix(0.0, 0.0, s, s)
    out = transform_points(np.array([[1.0, 0.0, 0.0]]), rot, (0.3, 0.0, 0.1))
    assert out[0] == pytest.approx([0.3, 1.0, 0.1])


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError):
        quaternion_matrix(0.0, 0.0, 0.0, 0.0)


def test_deployed_config_stops_only_inside_ten_centimetres():
    """lidar_safety.yaml as deployed: the robot slows from 0.90 m ahead of the nose and stops
    only for what is within 0.10 m (owner, 2026-10-20)."""
    import pathlib

    import yaml
    cfg = yaml.safe_load((pathlib.Path(__file__).resolve().parents[1] / "config"
                          / "lidar_safety.yaml").read_text())["lidar_hazard_node"]["ros__parameters"]
    fields = HazardParams.__dataclass_fields__
    params = HazardParams(**{k: v for k, v in cfg.items() if k in fields})
    assert params.stop_distance_m == pytest.approx(0.10)
    assert evaluate(wall_ahead(0.09), params).decision.startswith("RESTRICT:")
    assert "F" in evaluate(wall_ahead(0.09), params).decision
    assert evaluate(wall_ahead(0.12), params).decision == SLOWDOWN
    assert evaluate(wall_ahead(0.85), params).decision == SLOWDOWN
    assert evaluate(wall_ahead(1.0), params).decision == CLEAR
