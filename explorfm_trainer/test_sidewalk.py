import os
import torch
import cv2
import numpy as np
import typing
from pathlib import Path
from PIL import Image
from src.models.components.radio_cnn import RADIO_CNN
from torchvision import transforms

# 1. 加载模型
device = "cuda" if torch.cuda.is_available() else "cpu"
model = RADIO_CNN(model_version="c-radio_v3-b", use_naclip=True).to(device)

# 加载训练好的权重 (假设 checkpoint 路径如下)
# ckpt_path = "/mnt/hhd/han/train_log/explorfm_trainer/logs/train/runs/2026-06-02_16-22-47/checkpoints/last.ckpt"
ckpt_path = "/mnt/hhd/han/train_log/explorfm_trainer/logs/train/runs/2026-06-03_13-54-15/checkpoints/last.ckpt"
checkpoint = torch.load(ckpt_path, map_location=device, weights_only=False)
state_dict = checkpoint['state_dict']
# 过滤掉 backbone 权重，只加载头 (如果使用了 ModelCheckpointWithoutBackbone)
new_state_dict = {k.replace('net.', ''): v for k, v in state_dict.items()}
model.load_state_dict(new_state_dict, strict=False)
model.eval()

# 2. 准备图像
def predict_sidewalk(img_path):
    # 1. 图像加载与尺寸记录 (PIL 格式)
    img_orig = Image.open(img_path).convert('RGB')
    w_orig, h_orig = img_orig.size # 记录原始尺寸，用于最后还原

    # 2. 预处理 (必须包含 Normalize，否则 RADIO 特征会偏移)
    transform = transforms.Compose([
        transforms.Resize((512, 1024)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    input_tensor = transform(img_orig).unsqueeze(0).to(device)

    # 3. 模型推理
    with torch.no_grad():
        output = model(input_tensor)
        # 核心：获取 0.0 到 1.0 的概率分布
        probs = torch.sigmoid(output).squeeze().cpu().numpy()

    # 1. 将概率图 (0-1) 转换为 8位灰度图 (0-255)
    # 0.0 会变成黑色，1.0 会变成白色
    heatmap_gray = (probs * 255).astype(np.uint8)

    # 2. 应用热力图颜色映射 (JET: 蓝色代表低概率，红色代表高概率)
    # 注意：输入必须是 512x1024
    heatmap_color = cv2.applyColorMap(heatmap_gray, cv2.COLORMAP_JET)

    # 3. 将热力图缩放回原图尺寸
    img_orig = Image.open(img_path).convert('RGB')
    w_orig, h_orig = img_orig.size
    heatmap_resized = cv2.resize(heatmap_color, (w_orig, h_orig))

    # 4. 准备原图 (BGR)
    original_bgr = cv2.cvtColor(np.array(img_orig), cv2.COLOR_RGB2BGR)

    # 5. 融合叠加
    # 热力图和原图 5:5 叠加，这样既能看清路面，也能看清概率
    fusion = cv2.addWeighted(original_bgr, 0.5, heatmap_resized, 0.5, 0)

    # 保存热力图结果 (确保目录存在)
    repo_root = Path(__file__).resolve().parents[1]
    output_path = repo_root / "test/outputs/sidewalk_test_seg/sidewalk_02.png"
    output_dir = os.path.dirname(output_path)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    cv2.imwrite(output_path, fusion)
    # 打印最大概率值，方便调试模型信心
    print(f"热力图已生成！最大概率: {probs.max():.4f}, 最小概率: {probs.min():.4f}")

# 测试机器狗视角图
predict_sidewalk(Path(__file__).resolve().parents[1] / "test/imgs/road_test/road_10.jpg")
# predict_sidewalk("/mnt/hhd/han/dataset/cityscapes/leftImg8bit/train/aachen/aachen_000000_000019_leftImg8bit.png")
