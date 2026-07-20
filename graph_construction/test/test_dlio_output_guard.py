from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_removed_input_filter_does_not_reappear():
    source = (
        REPO_ROOT
        / "graph_construction"
        / "graph_construction"
        / "dlio_output_guard.py"
    ).read_text(encoding="utf-8")

    assert "last_pointcloud_stamp" not in source
    assert "last_imu_stamp" not in source
    assert "is_strictly_new_stamp" not in source
