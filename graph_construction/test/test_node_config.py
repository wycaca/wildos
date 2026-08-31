from collections import deque
from pathlib import Path

import pytest

from graph_construction.graph_builder import GraphBuilderConfig
from graph_construction.node import (
    GraphConstructionNode,
    InputFreshnessGate,
    _builder_config,
    _graph_diagnostic_metrics,
    _load_config,
    _resolve_config,
)
from graph_construction.performance_stats import TimingSummary


def test_resolve_config_uses_graph_builder_defaults_once():
    resolved = _resolve_config({})
    builder_config = _builder_config(resolved)

    assert builder_config == GraphBuilderConfig()


@pytest.mark.parametrize(
    "name",
    [
        "robot_namespace",
        "sample_stride",
        "min_node_separation",
        "max_edge_neighbors",
        "current_node_max_edge_neighbors",
        "max_edge_candidates_per_node",
        "low_degree_retry_threshold",
        "random_seed",
        "robot_blind_zone_initial_only",
        "viz_publish_rate_hz",
        "viz_show_full_edges",
        "viz_show_radius_markers",
        "viz_topic",
    ],
)
def test_resolve_config_rejects_removed_or_misspelled_keys(name):
    with pytest.raises(ValueError, match=name):
        _resolve_config({name: 1})


@pytest.mark.parametrize(
    "config",
    [
        {"grid_input_type": "image"},
        {"publish_rate_hz": 0.0},
        {"diagnostics_period_sec": 0.0},
        {"max_grid_odom_time_delta_sec": 0.0},
        {"grid_map_free_threshold": 0.1, "grid_map_obstacle_threshold": 0.2},
    ],
)
def test_resolve_config_rejects_invalid_values(config):
    with pytest.raises(ValueError):
        _resolve_config(config)


@pytest.mark.parametrize(
    "name",
    [
        "diagnostics_log_period_sec",
        "slow_cycle_warning_ms",
        "grid_map_traversability_layer",
        "grid_map_elevation_layer",
        "grid_map_normalize_traversability",
        "grid_map_normalize_low_quantile",
        "grid_map_normalize_high_quantile",
        "grid_map_z_offset",
        "grid_map_majority_fill_iterations",
        "grid_map_majority_fill_min_neighbors",
    ],
)
def test_resolve_config_rejects_internal_constants(name):
    with pytest.raises(ValueError, match=name):
        _resolve_config({name: 1})


def test_load_config_rejects_missing_file(tmp_path: Path):
    with pytest.raises(FileNotFoundError):
        _load_config(str(tmp_path / "missing.yaml"))


def test_load_config_rejects_non_mapping_yaml(tmp_path: Path):
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text("- invalid\n- config\n", encoding="utf-8")

    with pytest.raises(ValueError, match="must be a mapping"):
        _load_config(str(config_path))


def test_deployment_config_limits_go2_blind_zone():
    config_path = (
        Path(__file__).parents[1]
        / "configs"
        / "graph_construction_elevation.yaml"
    )

    builder_config = _builder_config(_load_config(str(config_path)))

    assert builder_config.robot_blind_zone_radius == 0.4
    assert builder_config.robot_blind_zone_elevation_search_radius == 4.0
    assert builder_config.robot_ground_height_offset == 0.70


def test_deployment_config_enables_low_rate_diagnostics():
    config_path = (
        Path(__file__).parents[1]
        / "configs"
        / "graph_construction_elevation.yaml"
    )

    config = _load_config(str(config_path))

    assert config["diagnostics_enabled"] is True
    assert config["diagnostics_period_sec"] == 30.0


def test_graph_diagnostics_keep_stage_timing_machine_readable():
    total = TimingSummary(4, 20.0, 25.0, 30.0)
    stats = type(
        "Stats",
        (),
        {
            "local_node_count": 10,
            "total_node_count": 20,
            "dirty_cell_count": 30,
            "local_pair_count": 40,
            "edge_clearance_check_count": 50,
            "edge_add_count": 2,
            "edge_remove_count": 1,
            "total_edge_count": 60,
            "frontier_candidate_count": 7,
            "active_frontier_owner_count": 3,
        },
    )()

    metrics = _graph_diagnostic_metrics(
        {"total": total},
        {"frontier": total},
        2.0,
        stats,
    )

    assert metrics["publish.rate_hz"] == 2.0
    assert metrics["cycle.total.p95_ms"] == 25.0
    assert metrics["graph.frontier.maximum_ms"] == 30.0
    assert metrics["workload.total_node_count"] == 20
    assert metrics["workload.total_edge_count"] == 60


def test_take_latest_inputs_processes_each_grid_once():
    node = GraphConstructionNode.__new__(GraphConstructionNode)
    node.config = {"max_grid_odom_time_delta_sec": 0.5}
    node._odom_cache = deque()
    node._input_freshness = InputFreshnessGate()
    node._input_freshness.accept_grid(1_000_000_000)
    node._input_freshness.accept_odom(1_100_000_000)
    node.latest_grid = object()
    node.latest_odom = object()
    node._odom_cache.append((1_100_000_000, node.latest_odom))
    node._latest_grid_sequence = 1
    node._processed_grid_sequence = 0

    assert node._take_latest_inputs() == (node.latest_grid, node.latest_odom)
    assert node._take_latest_inputs() is None

    node.latest_grid = object()
    node._latest_grid_sequence += 1
    node._input_freshness.accept_grid(1_200_000_000)

    assert node._take_latest_inputs() == (node.latest_grid, node.latest_odom)


def test_input_freshness_gate_rejects_duplicate_and_older_stamps():
    gate = InputFreshnessGate()

    assert gate.accept_grid(2_000_000_000)
    assert not gate.accept_grid(2_000_000_000)
    assert not gate.accept_grid(1_900_000_000)
    assert gate.accept_grid(2_100_000_000)

    assert gate.accept_odom(2_000_000_000)
    assert not gate.accept_odom(1_000_000_000)
    assert gate.accept_odom(2_200_000_000)
    assert gate.time_delta_seconds() == pytest.approx(0.1)


def test_take_latest_inputs_waits_for_matching_odom_stamp():
    node = GraphConstructionNode.__new__(GraphConstructionNode)
    node.config = {"max_grid_odom_time_delta_sec": 0.5}
    node._odom_cache = deque()
    node._input_freshness = InputFreshnessGate()
    node._input_freshness.accept_grid(10_000_000_000)
    node._input_freshness.accept_odom(8_000_000_000)
    node.latest_grid = object()
    node.latest_odom = object()
    node._odom_cache.append((8_000_000_000, node.latest_odom))
    node._latest_grid_sequence = 1
    node._processed_grid_sequence = 0
    node._warn_input_freshness = lambda *args: None

    assert node._take_latest_inputs() is None
    assert node._processed_grid_sequence == 0

    matching_odom = object()
    node._input_freshness.accept_odom(9_800_000_000)
    node._odom_cache.append((9_800_000_000, matching_odom))

    assert node._take_latest_inputs() == (node.latest_grid, matching_odom)
