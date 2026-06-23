import argparse
import os
from pathlib import Path

import cv2
import numpy as np
import torch

from explorfm.explorfm_model import ExploRFMInference


os.environ["HF_HUB_OFFLINE"] = "1"
os.environ["TRANSFORMERS_OFFLINE"] = "1"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input video path")
    parser.add_argument("--output-dir", default="test/outputs/explorfm_video")
    parser.add_argument("--trav-threshold", type=float, default=0.5)
    parser.add_argument("--frontier-threshold", type=float, default=0.5)
    parser.add_argument("--overlay-alpha", type=float, default=0.45)
    parser.add_argument("--max-frames", type=int, default=-1)
    parser.add_argument("--precision", default="FP16", choices=["FP16", "FP32"])
    return parser.parse_args()


def apply_thresholded_colormap(
    image_bgr: np.ndarray,
    confidence: np.ndarray,
    threshold: float,
    overlay_alpha: float,
    inverse: bool = False,
):
    confidence_for_color = 1.0 - confidence if inverse else confidence
    confidence_u8 = np.clip(confidence_for_color * 255, 0, 255).astype(np.uint8)
    color_map = cv2.applyColorMap(confidence_u8, cv2.COLORMAP_JET)

    mask = confidence >= threshold

    output = image_bgr.copy()
    output[mask] = (
        (1.0 - overlay_alpha) * image_bgr[mask]
        + overlay_alpha * color_map[mask]
    ).astype(np.uint8)

    return output


def build_model(precision: str):
    return ExploRFMInference(
        frontier_ckpt="ckpts/frontier_head.ckpt",
        traversability_ckpt="ckpts/trav_head.ckpt",
        model_version="ckpts/c-radio_v3-b_half.pth.tar",
        adaptor_version="siglip2",
        adaptor_ckpt_path="ckpts/siglip2",
        use_naclip=True,
        use_summary_for_spatial=True,
        radio_dim=768,
        static_scale_factor=0.5,
        model_precision=precision,
    )


def main():
    args = parse_args()

    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if not input_path.exists():
        raise FileNotFoundError(f"Input video not found: {input_path}")

    cap = cv2.VideoCapture(str(input_path))
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video: {input_path}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        fps = 30.0

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    stem = input_path.stem

    frontier_output_path = output_dir / f"{stem}_visual_frontiers_jet.mp4"
    traversability_output_path = output_dir / f"{stem}_visual_traversability_inverse_jet.mp4"

    frontier_writer = cv2.VideoWriter(
        str(frontier_output_path),
        fourcc,
        fps,
        (width, height),
    )
    traversability_writer = cv2.VideoWriter(
        str(traversability_output_path),
        fourcc,
        fps,
        (width, height),
    )

    model = build_model(args.precision)
    model.model.eval()

    print(f"Input video: {input_path}")
    print(f"FPS: {fps:.2f}, size: {width}x{height}, frames: {total_frames}")
    print(f"Frontier output: {frontier_output_path}")
    print(f"Traversability output: {traversability_output_path}")
    print(f"Frontier threshold: {args.frontier_threshold}")
    print(f"Traversability threshold: {args.trav_threshold}")

    frame_index = 0

    with torch.no_grad():
        while True:
            ok, frame_bgr = cap.read()
            if not ok:
                break

            if args.max_frames > 0 and frame_index >= args.max_frames:
                break

            image_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            traversability, frontiers, _ = model.forward_on_numpy(image_rgb.copy())

            traversability_map = traversability.squeeze().detach().float().cpu().numpy()
            frontier_map = frontiers.squeeze().detach().float().cpu().numpy()

            frontier_overlay = apply_thresholded_colormap(
                image_bgr=frame_bgr,
                confidence=frontier_map,
                threshold=args.frontier_threshold,
                overlay_alpha=args.overlay_alpha,
                inverse=False,
            )

            traversability_overlay = apply_thresholded_colormap(
                image_bgr=frame_bgr,
                confidence=traversability_map,
                threshold=args.trav_threshold,
                overlay_alpha=args.overlay_alpha,
                inverse=True,
            )

            frontier_writer.write(frontier_overlay)
            traversability_writer.write(traversability_overlay)

            frame_index += 1

            if frame_index % 10 == 0:
                print(
                    f"Processed {frame_index}/{total_frames} frames, "
                    f"trav_max={traversability_map.max():.4f}, "
                    f"frontier_max={frontier_map.max():.4f}"
                )

    cap.release()
    frontier_writer.release()
    traversability_writer.release()

    print(f"Done. Processed {frame_index} frames.")


if __name__ == "__main__":
    main()
