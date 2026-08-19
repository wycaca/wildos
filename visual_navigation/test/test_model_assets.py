import hashlib
import json

import pytest
import torch

from explorfm import explorfm_model
from nvidia_radio.model_assets import verify_model_asset, verify_model_manifest


def _write_manifest(root, asset, content):
    asset.parent.mkdir(parents=True)
    asset.write_bytes(content)
    manifest = {
        "version": 1,
        "assets": [
            {
                "path": asset.relative_to(root).as_posix(),
                "version": "test",
                "source": "test",
                "size": len(content),
                "sha256": hashlib.sha256(content).hexdigest(),
            }
        ],
    }
    manifest_path = root / "ckpts" / "manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")


def test_manifest_accepts_only_matching_asset(tmp_path):
    asset = tmp_path / "ckpts" / "model.bin"
    _write_manifest(tmp_path, asset, b"trusted weights")

    assert verify_model_asset(asset, tmp_path) == asset
    assert verify_model_manifest(tmp_path) == 1


def test_manifest_rejects_modified_asset(tmp_path):
    asset = tmp_path / "ckpts" / "model.bin"
    _write_manifest(tmp_path, asset, b"trusted weights")
    asset.write_bytes(b"modified weights")

    with pytest.raises(RuntimeError, match="大小不匹配|SHA256 不匹配"):
        verify_model_asset(asset, tmp_path)


def test_manifest_rejects_unlisted_and_external_paths(tmp_path):
    asset = tmp_path / "ckpts" / "model.bin"
    _write_manifest(tmp_path, asset, b"trusted weights")
    unlisted = tmp_path / "ckpts" / "other.bin"
    unlisted.write_bytes(b"other")

    with pytest.raises(RuntimeError, match="不在授权清单"):
        verify_model_asset(unlisted, tmp_path)
    with pytest.raises(RuntimeError, match="越出仓库"):
        verify_model_asset(tmp_path.parent / "outside.bin", tmp_path)


def test_head_loader_uses_weights_only_and_validates_tensors(monkeypatch, tmp_path):
    checkpoint = tmp_path / "head.ckpt"
    checkpoint.touch()
    load_options = {}

    monkeypatch.setattr(
        explorfm_model,
        "verify_model_asset",
        lambda path: checkpoint,
    )

    def fake_load(path, **options):
        assert path == checkpoint
        load_options.update(options)
        return {"state_dict": {"head.weight": torch.ones(1)}}

    monkeypatch.setattr(torch, "load", fake_load)

    state_dict = explorfm_model._load_head_state_dict(str(checkpoint))

    assert load_options["weights_only"] is True
    assert torch.equal(state_dict["head.weight"], torch.ones(1))
