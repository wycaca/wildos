from typing import Optional

import torch
from torch import nn
from torch.nn import functional as F
from typing import Optional

from src.models.components.radio_utils import RADIO_MODEL_VERSIONS, RADIO_ADAPTOR_VERSIONS
from nvidia_radio.hubconf import radio_model

class RADIO_UPSAMPLE(nn.Module):
    """A CNN decoder head on top of a RADIO model using Bilinear Upsampling."""

    def __init__(
        self,
        model_version: str = "c-radio_v3-b",
        adaptor_version: Optional[str] = None,
        use_naclip: bool = False,
        use_summary_for_spatial: bool = False,
        sigmoid_out: bool = False,
    ) -> None:
        """Initialize a `RADIO_UPSAMPLE` module.

        :param model_version: The version of the RADIO model to use.
        :param adaptor_version: The version of the adaptor to use.
        :param use_naclip: Whether to use the NA-CLIP changes.
        :param use_summary_for_spatial: Whether to use the summary adaptor for spatial features.
        :param sigmoid_out: Whether to apply Sigmoid to output. Set to False when using BCEWithLogitsLoss.
        """
        super().__init__()

        if model_version not in RADIO_MODEL_VERSIONS:
            raise ValueError(f"Invalid model version: {model_version}. Available versions: {RADIO_MODEL_VERSIONS}")
        self.model_version = model_version
        if adaptor_version is not None and adaptor_version not in RADIO_ADAPTOR_VERSIONS:
            raise ValueError(f"Invalid adaptor version: {adaptor_version}. Available versions: {RADIO_ADAPTOR_VERSIONS.keys()}")
        self.adaptor_version = adaptor_version
        
        if self.adaptor_version is None:
            self.dim = RADIO_ADAPTOR_VERSIONS["none"]
        else:
            self.dim = RADIO_ADAPTOR_VERSIONS[self.adaptor_version]
        self.use_naclip = use_naclip
        self.use_summary_for_spatial = use_summary_for_spatial        

        self.radio_model, chk = radio_model(
            version=self.model_version,
            progress=True,
            skip_validation=True,
            adaptor_names=self.adaptor_version,
            return_checkpoint=True, 
            use_naclip=self.use_naclip,
            naclip_strategy="kkonly",
            naclip_gaussian_std=5.0,
            fixed_patch_dim=(40,40),
            gaussian_device='cuda',
            use_summary_for_spatial=self.use_summary_for_spatial,
        )
        self.radio_model.eval()
        self.radio_model.requires_grad_(False)  # Disable gradients
        
        print(f"Loaded model: {self.model_version} with adaptor: {self.adaptor_version}")

        # ==========================================
        # 双线性插值 + 卷积 + 批归一化
        # ==========================================
        self.head = nn.Sequential(
            # Stage 1: 2x 放大尺寸
            nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True),
            nn.Conv2d(self.dim, self.dim // 2, kernel_size=3, padding=1),
            nn.BatchNorm2d(self.dim // 2),
            nn.ReLU(inplace=True),
            
            # Stage 2: 4x 放大尺寸
            nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True),
            nn.Conv2d(self.dim // 2, self.dim // 4, kernel_size=3, padding=1),
            nn.BatchNorm2d(self.dim // 4),
            nn.ReLU(inplace=True),
            
            # Stage 3: 8x 放大尺寸
            nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True),
            nn.Conv2d(self.dim // 4, self.dim // 8, kernel_size=3, padding=1),
            nn.BatchNorm2d(self.dim // 8),
            nn.ReLU(inplace=True),
            
            # Stage 4: 16x 放大尺寸，平滑特征通道过渡
            nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True),
            nn.Conv2d(self.dim // 8, self.dim // 16, kernel_size=3, padding=1),
            nn.BatchNorm2d(self.dim // 16),
            nn.ReLU(inplace=True),
            
            # 最终投影层：从多通道平滑缩减至 1 个二分类通道
            nn.Conv2d(self.dim // 16, 1, kernel_size=1)
        )
        self.sigmoid_out = sigmoid_out
        
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Perform a single forward pass through the network.

        :param x: The input tensor.
        :return: A tensor of predictions.
        """
        nearest_res = self.radio_model.get_nearest_supported_resolution(*x.shape[-2:])
        x_resized = F.interpolate(x, nearest_res, mode='bilinear', align_corners=False)

        # forward pass
        if self.adaptor_version is not None:
            summary, spatial_features = self.radio_model(x_resized, feature_fmt='NCHW')[self.adaptor_version]
        else:
            summary, spatial_features = self.radio_model(x_resized, feature_fmt='NCHW')
            
        out = self.head(spatial_features)

        # 确保输出分辨率完美契合原始输入的尺寸
        out = F.interpolate(out, size=x.shape[-2:], mode='bilinear', align_corners=True)

        if self.sigmoid_out:
            out = torch.sigmoid(out)
        
        return out