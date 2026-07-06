import os
from pathlib import Path

import cv2
import numpy as np
import torch

from explorfm.explorfm_model import ExploRFMInference


INPUT_DIR = Path("/home/ks-server3/han/wildos_ws/src/nebula2-wildos/test/imgs/road_test")
OUT_DIR = Path("test/outputs/road_test_seg")

TRAVERSABILITY_THRESHOLD = 0.5
FRONTIER_THRESHOLD = 0.5
OVERLAY_ALPHA = 0.45

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}

os.environ["HF_HUB_OFFLINE"] = "1"
os.environ["TRANSFORMERS_OFFLINE"] = "1"

TEXT_QUERIES = [
    "car", "building", "streetlamp", "sky", "tree", "road", "flag", "other"
    # , "sidewalk"
]


def list_image_files(input_dir: Path):
    image_files = []
    for file_path in sorted(input_dir.rglob("*")):
        if file_path.is_file() and file_path.suffix.lower() in IMAGE_SUFFIXES:
            image_files.append(file_path)
    return image_files


def apply_thresholded_colormap(
    image_bgr: np.ndarray,
    confidence: np.ndarray,
    threshold: float,
    colormap: int,
    inverse: bool = False,
):
    confidence_for_color = 1.0 - confidence if inverse else confidence
    confidence_u8 = np.clip(confidence_for_color * 255, 0, 255).astype(np.uint8)
    color_map = cv2.applyColorMap(confidence_u8, colormap)

    mask = confidence >= threshold

    output = image_bgr.copy()
    output[mask] = (
        (1.0 - OVERLAY_ALPHA) * image_bgr[mask]
        + OVERLAY_ALPHA * color_map[mask]
    ).astype(np.uint8)

    return output


def normalize_tensor(tensor: torch.Tensor, dim: int, eps: float = 1e-6):
    return tensor / (tensor.norm(dim=dim, keepdim=True) + eps)


def resize_logits_to_image(logits: torch.Tensor, image_shape):
    image_height, image_width = image_shape[:2]

    resized_logits = torch.nn.functional.interpolate(
        logits,
        size=(image_height, image_width),
        mode="bilinear",
        align_corners=False,
    )

    return resized_logits


def compute_text_similarity_logits(spatial_features, text_features):
    """
    spatial_features: [B, C, H, W] or [C, H, W]
    text_features: [K, C] or [1, K, C]
    return: [B, K, H, W]
    """

    if spatial_features.dim() == 3:
        spatial_features = spatial_features.unsqueeze(0)

    if text_features.dim() == 3:
        text_features = text_features.squeeze(0)

    spatial_features = spatial_features.float()
    text_features = text_features.float()

    spatial_features = normalize_tensor(spatial_features, dim=1)
    text_features = normalize_tensor(text_features, dim=1)

    logits = torch.einsum("bchw,kc->bkhw", spatial_features, text_features)
    return logits


def make_semantic_color_table(num_classes: int):
    base_colors = np.array(
        [
            [0, 0, 255],       # car, red in BGR
            [255, 0, 0],       # building, blue
            [0, 255, 255],     # streetlamp, yellow
            [255, 255, 0],     # sky, cyan
            [0, 255, 0],       # tree, green
            [128, 128, 128],   # road, gray
            [255, 0, 255],     # sidewalk, magenta
            [0, 128, 255],     # curb, orange
            [0, 128, 0],       # grass, dark green
            [64, 64, 64],      # other, dark gray
        ],
        dtype=np.uint8,
    )

    if num_classes <= len(base_colors):
        return base_colors[:num_classes]

    extra_count = num_classes - len(base_colors)
    extra_colors = np.random.default_rng(0).integers(
        0,
        255,
        size=(extra_count, 3),
        dtype=np.uint8,
    )

    return np.concatenate([base_colors, extra_colors], axis=0)


def save_open_vocab_results(
    image_path: Path,
    image_bgr: np.ndarray,
    class_logits: torch.Tensor,
    text_queries,
):
    relative_path = image_path.relative_to(INPUT_DIR)
    output_subdir = OUT_DIR / relative_path.parent
    output_subdir.mkdir(parents=True, exist_ok=True)

    stem = image_path.stem

    resized_logits = resize_logits_to_image(class_logits, image_bgr.shape)
    probability = torch.softmax(resized_logits, dim=1)

    class_confidence, class_index = torch.max(probability, dim=1)

    class_index_np = class_index.squeeze(0).detach().cpu().numpy().astype(np.uint8)
    class_confidence_np = class_confidence.squeeze(0).detach().cpu().numpy()

    color_table = make_semantic_color_table(len(text_queries))
    semantic_color = color_table[class_index_np]

    semantic_overlay = (
        (1.0 - OVERLAY_ALPHA) * image_bgr
        + OVERLAY_ALPHA * semantic_color
    ).astype(np.uint8)

    # cv2.imwrite(
    #     str(output_subdir / f"{stem}_open_vocab_semantic.png"),
    #     semantic_color,
    # )

    cv2.imwrite(
        str(output_subdir / f"{stem}_open_vocab_overlay.png"),
        semantic_overlay,
    )

    # cv2.imwrite(
    #     str(output_subdir / f"{stem}_open_vocab_confidence.png"),
    #     np.clip(class_confidence_np * 255, 0, 255).astype(np.uint8),
    # )

    for class_id, class_name in enumerate(text_queries):
        class_mask = class_index_np == class_id
        class_mask_u8 = (class_mask.astype(np.uint8) * 255)

        cv2.imwrite(
            str(output_subdir / f"{stem}_mask_{class_id:02d}_{class_name}.png"),
            class_mask_u8,
        )

    class_stats = {}
    total_pixels = class_index_np.size

    for class_id, class_name in enumerate(text_queries):
        pixel_count = int((class_index_np == class_id).sum())
        class_stats[class_name] = pixel_count / total_pixels

    return class_stats

def save_results(image_path: Path, image_bgr, traversability, frontiers):
    relative_path = image_path.relative_to(INPUT_DIR)
    output_subdir = OUT_DIR / relative_path.parent
    output_subdir.mkdir(parents=True, exist_ok=True)

    stem = image_path.stem

    traversability_map = traversability.squeeze().detach().float().cpu().numpy()
    frontier_map = frontiers.squeeze().detach().float().cpu().numpy()

    # Project style:
    # Visual Frontiers: Jet, red means higher frontier confidence.
    frontier_overlay = apply_thresholded_colormap(
        image_bgr=image_bgr,
        confidence=frontier_map,
        threshold=FRONTIER_THRESHOLD,
        colormap=cv2.COLORMAP_JET,
    )

    # Project style:
    # Visual Traversability: inverse Jet, blue means higher traversability confidence.
    traversability_overlay = apply_thresholded_colormap(
        image_bgr=image_bgr,
        confidence=traversability_map,
        threshold=TRAVERSABILITY_THRESHOLD,
        colormap=cv2.COLORMAP_JET,
        inverse=True,
    )

    cv2.imwrite(
        str(output_subdir / f"{stem}_visual_frontiers_jet.png"),
        frontier_overlay,
    )
    cv2.imwrite(
        str(output_subdir / f"{stem}_visual_traversability_inverse_jet.png"),
        traversability_overlay,
    )

    return {
        "traversability_min": traversability_map.min(),
        "traversability_max": traversability_map.max(),
        "traversability_mean": traversability_map.mean(),
        "frontier_min": frontier_map.min(),
        "frontier_max": frontier_map.max(),
        "frontier_mean": frontier_map.mean(),
    }


def main():
    if not INPUT_DIR.exists():
        raise FileNotFoundError(f"Input folder not found: {INPUT_DIR}")

    image_files = list_image_files(INPUT_DIR)
    if not image_files:
        raise RuntimeError(f"No image files found in: {INPUT_DIR}")

    model = ExploRFMInference(
        frontier_ckpt="ckpts/frontier_ckpt_new.ckpt",
        traversability_ckpt="ckpts/traversability_ckpt.ckpt",
        model_version="ckpts/c-radio_v3-b_half.pth.tar",
        adaptor_version="siglip2",
        adaptor_ckpt_path="ckpts/siglip2",
        use_naclip=True,
        use_summary_for_spatial=True,
        radio_dim=768,
        static_scale_factor=0.5,
        model_precision="FP16",
    )

    print(f"Found {len(image_files)} images.")
    print(f"Saving results to: {OUT_DIR}")
    print(f"Traversability threshold: {TRAVERSABILITY_THRESHOLD}")
    print(f"Frontier threshold: {FRONTIER_THRESHOLD}")

    for index, image_path in enumerate(image_files, start=1):
        image_bgr = cv2.imread(str(image_path))
        if image_bgr is None:
            print(f"[SKIP] failed to read: {image_path}")
            continue

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        with torch.no_grad():
            traversability, frontiers, spatial_features = model.forward_on_numpy(image_rgb.copy())
            text_features = model.forward_on_text(TEXT_QUERIES)
            class_logits = compute_text_similarity_logits(spatial_features, text_features)

        stats = save_results(
            image_path=image_path,
            image_bgr=image_bgr,
            traversability=traversability,
            frontiers=frontiers,
        )

        class_stats = save_open_vocab_results(
            image_path=image_path,
            image_bgr=image_bgr,
            class_logits=class_logits,
            text_queries=TEXT_QUERIES,
        )

        print(
            f"[{index}/{len(image_files)}] {image_path} "
            f"trav_max={stats['traversability_max']:.4f}, "
            f"trav_mean={stats['traversability_mean']:.4f}, "
            f"frontier_max={stats['frontier_max']:.4f}, "
            f"frontier_mean={stats['frontier_mean']:.4f}, "
            f"road={class_stats.get('road', 0.0):.3f}, "
            f"sidewalk={class_stats.get('sidewalk', 0.0):.3f}, "
            f"curb={class_stats.get('curb', 0.0):.3f}"
        )


if __name__ == "__main__":
    main()
