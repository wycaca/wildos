from types import SimpleNamespace

from radio import utils


def test_single_process_fallback_without_distributed_api(monkeypatch):
    monkeypatch.setattr(utils, "dist", SimpleNamespace())

    assert utils.get_rank() == 0
    assert utils.get_world_size() == 1
    assert utils.barrier() is None


def test_initialized_distributed_api_is_forwarded(monkeypatch):
    calls = []
    distributed = SimpleNamespace(
        is_initialized=lambda: True,
        get_rank=lambda group: 2,
        get_world_size=lambda group: 4,
        barrier=lambda group: calls.append(group),
    )
    monkeypatch.setattr(utils, "dist", distributed)

    assert utils.get_rank("group") == 2
    assert utils.get_world_size("group") == 4
    utils.barrier("group")

    assert calls == ["group"]
