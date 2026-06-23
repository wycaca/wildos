import os
import glob
import torch
import numpy as np
from PIL import Image
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
import torchvision.transforms.functional as F
from lightning import LightningDataModule

class CityscapesSidewalkDataset(Dataset):
    def __init__(self, data_dir, split='train', transform=None, colormap_path=None):
        self.data_dir = data_dir
        self.split = split
        # 针对机器狗视角的特殊增强方案
        self.transform = transform
        self.colormap_path = colormap_path # 需要在 DataModule 中传入

        # 1. 初始化 Colormap (核心修复)
        # 如果提供了路径则读取，否则提供一个硬编码的默认值（适配人行道任务）
        if self.colormap_path and os.path.exists(self.colormap_path):
            self.seg_colormap = self.load_annotations()
        else:
            # 手动定义：让日志系统知道 sidewalk 应该涂成什么颜色 (RGB)
            # 这里的键名 'sidewalk' 必须对应你 config 里的 labels
            self.seg_colormap = {"sidewalk": (244, 35, 232)} # Cityscapes 官方人行道颜色
        
        # 使用递归通配符匹配所有城市子文件夹中的图像
        # 路径示例: data_dir/leftImg8bit/train/aachen/aachen_000000_000019_leftImg8bit.png
        search_path = os.path.join(data_dir, 'leftImg8bit', split, '*', '*_leftImg8bit.png')
        self.images = sorted(glob.glob(search_path))
        
        if not self.images:
            raise FileNotFoundError(f"未在 {search_path} 找到数据，请检查路径。")

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img_path = self.images[idx]
        mask_path = img_path.replace('_leftImg8bit.png', '_gtFine_labelIds.png').replace('leftImg8bit', 'gtFine')
        
        img_pil = Image.open(img_path).convert('RGB')
        mask_pil = Image.open(mask_path)

        # 1. raw_img: 给模型用 (带 Normalize)
        raw_img = self.transform(img_pil)

        # 2. gt_traversability: 训练标签 (0/1)
        mask_np = np.array(mask_pil)
        binary_mask = (mask_np == 8).astype(np.float32)
        gt_tensor = torch.from_numpy(binary_mask).unsqueeze(0) # (1, H, W)
        
        # 标签已经裁剪过了，直接插值到模型输出大小
        gt_traversability = torch.nn.functional.interpolate(
            gt_tensor.unsqueeze(0), 
            size=(raw_img.shape[1], raw_img.shape[2]), 
            mode='nearest'
        ).squeeze(0)

        # 3. gt_segmentation: 给日志用, 原始分割图 (CHW 格式，与模型输出一致)
        gt_segmentation = transforms.Compose([
            transforms.Resize((raw_img.shape[1], raw_img.shape[2])),
            transforms.ToTensor()
        ])(img_pil)

        # 4. viz_img: 给日志用, 未归一化的图像 (CHW 格式，0-1 范围)
        viz_img = gt_segmentation.clone()

        return {
            "raw_img": raw_img,
            "gt_traversability": gt_traversability, 
            "gt_segmentation": gt_segmentation,
            "viz_img": viz_img,
            "img_path": "/".join(img_path.split("/")[-2:]), 
        }

    def load_annotations(self):
        """读取 colormap 文件，格式要求：index label r g b"""
        colormap = {}
        with open(self.colormap_path, 'r') as file:
            for line in file:
                parts = line.strip().split()
                if len(parts) == 5:
                    _, label, r, g, b = parts
                    colormap[label] = (int(r), int(g), int(b))
                else:
                    # 也可以选择 skip 掉不符合格式的行
                    continue 
        return colormap

class CityscapesDataModule(LightningDataModule):
    def __init__(self, data_dir, batch_size=4, num_workers=4, pin_memory: bool = True, labels: list = None):
        super().__init__()
        self.save_hyperparameters()
        # Cityscapes 原图 2048x1024，建议缩放到 512x1024 以匹配 RADIO 的特征感受野
        # 增强训练集以改善类别不平衡问题
        self.train_transform = transforms.Compose([
            transforms.Resize((512, 1024)),
            transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2),  # 颜色扰动
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        # 验证集只做基础转换
        self.val_transform = transforms.Compose([
            transforms.Resize((512, 1024)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def setup(self, stage=None):
        self.train_ds = CityscapesSidewalkDataset(self.hparams.data_dir, split='train', transform=self.train_transform, colormap_path=self.hparams.get("colormap_path", None))
        self.val_ds = CityscapesSidewalkDataset(self.hparams.data_dir, split='val', transform=self.val_transform)

    def train_dataloader(self):
        return DataLoader(
            self.train_ds, 
            batch_size=self.hparams.batch_size, 
            shuffle=True, 
            num_workers=self.hparams.num_workers, 
            pin_memory=self.hparams.pin_memory
        )

    def val_dataloader(self):
        return DataLoader(
            self.val_ds, 
            batch_size=self.hparams.batch_size, 
            num_workers=self.hparams.num_workers,
            pin_memory=self.hparams.pin_memory
        )