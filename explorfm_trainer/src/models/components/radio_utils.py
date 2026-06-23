import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import torch

RADIO_MODEL_VERSIONS = [
    "radio_v2.5-g", # for RADIOv2.5-g model (ViT-H/14)
    "radio_v2.5-h", # for RADIOv2.5-H model (ViT-H/16)
    "radio_v2.5-l", # for RADIOv2.5-L model (ViT-L/16)
    "radio_v2.5-b", # for RADIOv2.5-B model (ViT-B/16)
    "c-radio_v3-b", # for C_RADIOv3-B model (ViT-B/16)
    "c-radio_v3-l", # for C_RADIOv3-L model (ViT-L/16)
    "e-radio_v2", # for E-RADIO
]
RADIO_ADAPTOR_VERSIONS = {
    "none": 768,  # No adaptor
    "clip": 1280,  # CLIP adaptor
    "siglip": None,  # SigLIP adaptor
    "siglip2": 1152,  # SigLIP2 adaptor
    "dino_v2": 1536,  # DINO adaptor
    "sam": 1280,  # SAM adaptor
}

def gen_logging_image(
        batch_data: dict,
        seg_colormap: dict,
        num_log_imgs: int,
        cmap: str = "inferno",
        vmax: float = 1,
    ) -> np.ndarray:
    """
    Visualize the predictions. 
    Displays the original image, ground truth, the heatmap of text similarity, and the binary mask.
    Also display the legend for the segmentation categories and the heatmap.
    """
    B = batch_data["viz_img"].shape[0]
    idxs = np.random.choice(
        range(B), min(num_log_imgs, B), replace=False
    )  # randomly select indices for logging

    log_imgs = []
    for i in idxs:
        # 处理可视化图像（0-1 范围的未归一化图像）
        viz_img_tensor = batch_data["viz_img"][i].cpu().numpy()
        if viz_img_tensor.ndim == 3:
            img_rgb = viz_img_tensor.transpose(1, 2, 0)
        else:
            img_rgb = viz_img_tensor
        img_rgb = (np.clip(img_rgb, 0, 1) * 255).astype(np.uint8)
        
        # 处理分割标签（CHW 格式）- 转为 HWC
        gt_seg_chw = batch_data["gt_segmentation"][i].cpu().numpy()
        if gt_seg_chw.ndim == 3:
            gt_seg = gt_seg_chw.transpose(1, 2, 0).squeeze()
        else:
            gt_seg = gt_seg_chw.squeeze()
        
        # 处理安全遍历标签 - 确保是 2D
        gt_traversability_raw = batch_data["gt_traversability"][i].cpu().numpy()
        if gt_traversability_raw.ndim == 3:
            gt_traversability = gt_traversability_raw[0]
        else:
            gt_traversability = gt_traversability_raw
        
        # 处理预测 - 确保是 2D
        preds_raw = batch_data["preds"][i].cpu().numpy()
        if preds_raw.ndim == 3:
            preds = preds_raw[0]
        else:
            preds = preds_raw
        
        # 处理概率 - 确保是 2D
        probs_raw = batch_data["probs"][i].cpu().numpy()
        if probs_raw.ndim == 3:
            probs = probs_raw[0]
        else:
            probs = probs_raw

        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        axes[0, 2].axis('off')
        
        # 1. Original RGB Image
        axes[0, 0].imshow(img_rgb)
        axes[0, 0].set_title(f"Original RGB: {batch_data['img_path'][i]}")
        axes[0, 0].axis('off')
        
        # 2. Ground Truth Traversability Mask with Color Overlay
        axes[0, 1].imshow(img_rgb)
        gt_overlay = np.zeros_like(img_rgb)
        gt_overlay[gt_traversability > 0.5] = [244, 35, 232]  # 紫色 (Cityscapes Sidewalk Color)
        axes[0, 1].imshow(gt_overlay, alpha=0.4)
        axes[0, 1].set_title('Ground Truth Traversability')
        axes[0, 1].axis('off')

        # 3. Heatmap (text similarity)
        axes[1, 0].imshow(img_rgb)
        heatmap = axes[1, 0].imshow(probs, cmap=cmap, vmin=0, vmax=vmax, alpha=0.5)
        axes[1, 0].set_title('Predicted Probabilities')
        axes[1, 0].axis('off')
        plt.colorbar(heatmap, ax=axes[1,0], fraction=0.046, pad=0.04)

        # 4. Binary mask (thresholded)
        axes[1, 1].imshow(img_rgb)
        axes[1, 1].imshow(preds, cmap='gray', alpha=0.5)
        axes[1, 1].set_title(f'Predicted Binary Mask')
        axes[1, 1].axis('off')

        # 5. Ground Truth Safe Mask
        axes[1, 2].imshow(img_rgb)
        axes[1, 2].imshow(gt_traversability, cmap='gray', alpha=0.5)
        axes[1, 2].set_title('Ground Truth Safe Mask')
        axes[1, 2].axis('off')

        # Add segmentation legend below all subplots
        handles = [
            mpatches.Patch(color=np.array(color)/255.0, label=label)
            for label, color in seg_colormap.items()
        ]
        fig.legend(handles=handles, loc='upper right', ncol=4, fontsize='small', frameon=False)

        plt.tight_layout(rect=[0, 0.1, 1, 1])  # Leave space for the legend
        
        # Convert plot to image
        fig.canvas.draw()
        data = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8)
        data = data.reshape(fig.canvas.get_width_height()[::-1] + (4,))[:,:,1:]

        plt.close(fig)  # Close the figure to free memory
        log_imgs.append(data)

    return log_imgs