from pathlib import Path

import pytest

from graph_construction.graph_builder import GraphBuilderConfig
from graph_construction.node import _builder_config, _load_config, _resolve_config


def test_resolve_config_uses_graph_builder_defaults_once():
    resolved = _resolve_config({})
    builder_config = _builder_config(resolved)

    assert builder_config == GraphBuilderConfig()


def test_resolve_config_rejects_removed_or_misspelled_keys():
    with pytest.raises(ValueError, match="robot_namespace"):
        _resolve_config({"robot_namespace": "spot1"})


@pytest.mark.parametrize(
    "config",
    [
        {"grid_input_type": "image"},
        {"publish_rate_hz": 0.0},
        {"free_threshold": 70, "obstacle_threshold": 65},
        {"grid_map_free_threshold": 0.1, "grid_map_obstacle_threshold": 0.2},
        {"grid_map_normalize_low_quantile": 0.9, "grid_map_normalize_high_quantile": 0.1},
    ],
)
def test_resolve_config_rejects_invalid_values(config):
    with pytest.raises(ValueError):
        _resolve_config(config)


def test_load_config_rejects_missing_file(tmp_path: Path):
    with pytest.raises(FileNotFoundError):
        _load_config(str(tmp_path / "missing.yaml"))


def test_load_config_rejects_non_mapping_yaml(tmp_path: Path):
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text("- invalid\n- config\n", encoding="utf-8")

    with pytest.raises(ValueError, match="must be a mapping"):
        _load_config(str(config_path))
