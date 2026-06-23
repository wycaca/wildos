import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import torch

# def gen_logging_image(
#         batch_data: dict,
#         num_log_imgs: int,
#         cmap: str = "inferno",
#         vmax: float = 1,
#     ) -> np.ndarray:
#     """
#     Visualize the predictions. 
#     Displays the original image with bbox annotations, SAM boundaries, overlaid gt mask, predicted prob heatmap and the predicted binary mask.
#     """
#     B = batch_data["image"].shape[0]
#     idxs = np.random.choice(
#         range(B), min(num_log_imgs, B), replace=False
#     )  # randomly select indices for logging

#     log_imgs = []
#     for i in idxs:
#         img_name = batch_data["image_name"][i]
#         img_rgb = batch_data["image"][i].cpu().numpy().transpose(1, 2, 0)
#         img_rgb = (img_rgb * 255).astype(np.uint8)  # convert to uint8
#         gt_seg = batch_data["mask"][i][0].cpu().numpy()
#         gt_bound = batch_data["boundary"][i][0].cpu().numpy()
#         viz_ann = batch_data["bbox_annotations"][i].cpu().numpy().transpose(1, 2, 0)
#         preds = batch_data["preds"][i][0].cpu().numpy().astype(np.uint8)
#         probs = batch_data["probs"][i][0].cpu().numpy()

#         img_rgb = img_rgb.astype(np.uint8)
#         fig, axes = plt.subplots(2, 3, figsize=(15, 8))
#         axes[1, 2].axis('off')

#         # Visualize the raw image with BBox labels
#         img1 = img_rgb.copy()
#         img1[np.any(viz_ann, axis=-1)] = viz_ann[np.any(viz_ann, axis=-1)]  # Apply bbox annotations
#         axes[0, 0].imshow(img1)
#         axes[0, 0].set_title(f"Image with BBox Labels: {img_name}")
#         axes[0, 0].axis('off')

#         # Visualize the ground truth boundary overlaid on the image
#         boundary_overlay = img_rgb.copy()
#         boundary_overlay[gt_bound > 0] = [255, 0, 0]  # Red overlay for boundary
#         axes[0, 1].imshow(boundary_overlay)
#         axes[0, 1].set_title(f"Ground Truth Boundary")
#         axes[0, 1].axis('off')

#         # Visualize the ground truth mask overlaid on the image
#         mask_overlay = img_rgb.copy()
#         axes[0, 2].imshow(mask_overlay)
#         axes[0, 2].imshow(gt_seg, cmap='gray', alpha=0.5)
#         axes[0, 2].set_title(f"Ground Truth Mask")
#         axes[0, 2].axis('off')

#         # 3. Heatmap
#         axes[1, 0].imshow(img_rgb)
#         heatmap = axes[1, 0].imshow(probs, cmap=cmap, vmin=0, vmax=vmax)
#         axes[1, 0].set_title('Predicted Probabilities')
#         axes[1, 0].axis('off')
#         plt.colorbar(heatmap, ax=axes[1,0], fraction=0.046, pad=0.04)

#         # 4. Binary mask (thresholded)
#         predmask_overlay = img_rgb.copy()
#         axes[1, 1].imshow(predmask_overlay)
#         axes[1, 1].imshow(preds, cmap='gray', alpha=0.5)
#         axes[1, 1].set_title(f'Predicted Binary Mask')
#         axes[1, 1].axis('off')

#         plt.tight_layout()  # Leave space for the legend
        
#         # Convert plot to image
#         fig.canvas.draw()
#         data = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8)
#         data = data.reshape(fig.canvas.get_width_height()[::-1] + (4,))[:,:,1:]

#         plt.close(fig)  # Close the figure to free memory
#         log_imgs.append(data)

#     return log_imgs

def gen_logging_image(
    batch_data: dict,
    num_log_imgs: int,
    cmap: str = "inferno",
    vmax: float = 1,
) -> list:
    # 1. 自动适配键名：优先取 gt_segmentation (即我们在 dataset 定义的 gt_img)
    images = batch_data.get("gt_segmentation")
    masks = batch_data["gt_traversability"]
    preds = batch_data["preds"]
    probs = batch_data["probs"]
    img_paths = batch_data["img_path"]

    B = images.shape[0]
    num_log_imgs = min(num_log_imgs, B)
    log_imgs = []

    for i in range(num_log_imgs):
        # 处理底图
        img_np = images[i].cpu().numpy()
        # 处理通道维度 (CHW -> HWC)
        if img_np.ndim == 3 and img_np.shape[0] in [3, 4]:
            img_np = img_np.transpose(1, 2, 0)
        
        # 根据值域自动转换为 uint8
        if img_np.max() <= 1.0:
            # 0-1 范围的浮点数
            img_viz = (img_np * 255).astype(np.uint8)
        else:
            # 已经是 0-255 范围
            img_viz = np.clip(img_np, 0, 255).astype(np.uint8)

        gt_mask = masks[i].cpu().numpy().squeeze()
        prob_map = probs[i].cpu().numpy().squeeze()
        pred_mask = preds[i].cpu().numpy().squeeze()

        # 创建 1行3列 的对比图
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        
        # 第一张：原图 + Ground Truth (紫色叠加)
        axes[0].imshow(img_viz)
        gt_overlay = np.zeros_like(img_viz)
        gt_overlay[gt_mask > 0] = [244, 35, 232] # Cityscapes Sidewalk Color
        axes[0].imshow(gt_overlay, alpha=0.4)
        axes[0].set_title(f"GT Mask: {img_paths[i]}")
        axes[0].axis('off')

        # 第二张：预测概率热力图 (Heatmap)
        axes[1].imshow(img_viz)
        im = axes[1].imshow(prob_map, cmap=cmap, vmin=0, vmax=vmax, alpha=0.6)
        axes[1].set_title("Predicted Probabilities")
        plt.colorbar(im, ax=axes[1], fraction=0.046, pad=0.04)
        axes[1].axis('off')

        # 第三张：最终二值化预测 (紫色叠加)
        axes[2].imshow(img_viz)
        pred_overlay = np.zeros_like(img_viz)
        pred_overlay[pred_mask > 0] = [244, 35, 232]
        axes[2].imshow(pred_overlay, alpha=0.4)
        axes[2].set_title("Binary Prediction")
        axes[2].axis('off')

        plt.tight_layout()

        # Canvas 转 Numpy 给 Wandb
        fig.canvas.draw()
        data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        plt.close(fig)
        log_imgs.append(data)

    return log_imgs