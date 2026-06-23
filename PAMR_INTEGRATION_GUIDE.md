# Pixel-Adaptive Mask Refinement (PAMR) 集成指南

## 概述

PAMR 已集成到 `BinarySegmentationLitModule` 中，用于优化二值分割任务中的像素级边缘。PAMR 通过迭代精细化过程根据图像局部像素相似度自适应地调整掩膜边界。

## 核心特性

### 1. **自适应边缘精细化**
   - 根据输入图像的像素相似度自动调整预测掩膜的边界
   - 保留高置信度区域，精细化边界模糊区域
   - 多尺度感受野设计捕捉不同尺度的边界特征

### 2. **灵活的配置参数**
   - `use_pamr`: 是否启用 PAMR（默认：True）
   - `pamr_num_iter`: 迭代精细化次数，推荐 5-10 次（默认：5）
   - `pamr_dilations`: 感受野空洞率列表，用于捕捉多尺度特征（默认：[1, 2, 4]）

### 3. **三阶段集成**
   - **训练阶段**：在计算预测时应用 PAMR，帮助模型学习更清晰的边界
   - **验证阶段**：可选性地应用 PAMR 进行评估
   - **测试阶段**：在推理时应用 PAMR 获得精细化的预测

## 使用示例

### 配置文件示例（YAML）

```yaml
# configs/model/sidewalk_segmentation.yaml
_target_: src.models.sidewalk_binary_segmentation_module.BinarySegmentationLitModule
net: ???
optimizer: ???
scheduler: ???
compile: false

# PAMR 相关参数
use_pamr: true                    # 启用 PAMR
pamr_num_iter: 5                  # 迭代次数（5-10 推荐）
pamr_dilations: [1, 2, 4]         # 多尺度感受野

# 其他参数
pred_threshold: 0.6
num_log_imgs: 4
validation_img_log_idx: 0
vmax: 1
pos_weight: 8.0
```

### Python 实例化示例

```python
from src.models.sidewalk_binary_segmentation_module import BinarySegmentationLitModule
from src.models.components.radio_cnn import RADIO_CNN

# 创建模型实例
model = BinarySegmentationLitModule(
    net=RADIO_CNN(
        model_version="c-radio_v3-b",
        adaptor_version=None,
    ),
    optimizer=torch.optim.AdamW,
    scheduler=torch.optim.lr_scheduler.CosineAnnealingLR,
    compile=False,
    
    # PAMR 参数
    use_pamr=True,           # 启用 PAMR
    pamr_num_iter=5,         # 迭代精细化 5 次
    pamr_dilations=[1, 2, 4],  # 多尺度感受野
    
    # 其他参数
    pred_threshold=0.6,
    pos_weight=8.0,
)
```

## 参数详解

### `use_pamr` (bool, 默认: True)
- **作用**：控制是否使用 PAMR 进行边缘精细化
- **推荐值**：True（启用时边界更清晰）
- **用途**：在资源受限的场景可设为 False 以加速推理

### `pamr_num_iter` (int, 默认: 5)
- **作用**：PAMR 迭代次数，控制边缘精细化程度
- **推荐范围**：5-10
  - 5-6：轻量级精细化，速度快
  - 7-8：平衡精细化和速度
  - 9-10：重度精细化，边界最清晰但速度最慢
- **用途**：根据任务需求和计算资源平衡选择

### `pamr_dilations` (list, 默认: [1, 2, 4])
- **作用**：感受野空洞率，捕捉多尺度像素亲和力
- **推荐配置**：
  - `[1, 2, 4]`：标准配置，计算效率高
  - `[1, 2, 4, 8]`：更大的感受野，边界效果更好但速度略慢
  - `[1, 2]`：轻量级，用于快速推理
- **含义**：
  - `1`：原始邻域
  - `2`：间隔为 2 的邻域
  - `4`：间隔为 4 的邻域
  - `8`：间隔为 8 的邻域（捕捉全局信息）

## 算法原理

PAMR 基于以下关键思想：

1. **像素亲和力（Pixel Affinity）**：计算相邻像素之间的相似度
2. **迭代传播**：通过多次迭代传播掩膜标签
3. **自适应调整**：根据像素相似度自动调整边界位置

数学框架：
$$\text{refined\_mask} = \text{PAMR}(\text{image}, \text{original\_mask})$$

其中 PAMR 通过计算像素间的亲和力矩阵并迭代应用来精细化掩膜。

## 性能影响

| 配置 | 精细化效果 | 速度 | 内存 |
|------|---------|------|------|
| `use_pamr=False` | 无 | ✓✓✓ | ✓✓✓ |
| `num_iter=5, dilations=[1,2]` | 轻 | ✓✓ | ✓✓ |
| `num_iter=5, dilations=[1,2,4]` | 中 | ✓ | ✓ |
| `num_iter=10, dilations=[1,2,4,8]` | 强 | ✗ | ✗ |

## 最佳实践

### 训练阶段
```python
# 推荐配置：平衡精细化和训练速度
use_pamr=True
pamr_num_iter=5
pamr_dilations=[1, 2, 4]
```

### 评估阶段
```python
# 可以使用更强的配置以获得最佳评估结果
use_pamr=True
pamr_num_iter=8
pamr_dilations=[1, 2, 4, 8]
```

### 快速推理
```python
# 资源受限场景
use_pamr=False  # 或使用轻量级配置
```

## 集成到训练流程

### 方法 1：在 Hydra 配置中指定

```yaml
# configs/train.yaml
model:
  _target_: src.models.sidewalk_binary_segmentation_module.BinarySegmentationLitModule
  use_pamr: true
  pamr_num_iter: 5
  pamr_dilations: [1, 2, 4]
```

### 方法 2：通过命令行覆盖

```bash
python src/train.py \
  +model.use_pamr=true \
  +model.pamr_num_iter=5 \
  +model.pamr_dilations=[1,2,4]
```

### 方法 3：在代码中直接设置

```python
from hydra.utils import instantiate

model_cfg = {
    '_target_': 'src.models.sidewalk_binary_segmentation_module.BinarySegmentationLitModule',
    'use_pamr': True,
    'pamr_num_iter': 5,
    'pamr_dilations': [1, 2, 4],
    # ... 其他参数
}

model = instantiate(model_cfg)
```

## 故障排除

### 问题 1：PAMR 应用失败
```
Warning: PAMR failed with error ..., using original probs
```
**解决方案**：
- 检查 `radio` 库是否正确安装
- 确保输入图像和概率图的维度正确：[B, C, H, W]
- 尝试减少 `pamr_num_iter` 或使用较小的 `dilations`

### 问题 2：推理速度过慢
**解决方案**：
- 减少 `pamr_num_iter`（从 10 降到 5）
- 简化 `pamr_dilations`（[1, 2] 而非 [1, 2, 4, 8]）
- 或设置 `use_pamr=False` 在快速推理模式

### 问题 3：边界效果不理想
**解决方案**：
- 增加 `pamr_num_iter`（从 5 增到 8-10）
- 扩展 `pamr_dilations`（添加 8）
- 调整 `pred_threshold` 以改变预测的敏感度

## 实验建议

为了找到最优配置，建议进行以下实验：

1. **基准测试**：用 `use_pamr=False` 建立基准
2. **参数扫描**：
   ```bash
   # 扫描 pamr_num_iter
   for iter in 3 5 8 10; do
     python train.py model.pamr_num_iter=$iter
   done
   ```
3. **监控指标**：关注 IoU、F1 分数和推理速度
4. **选择最优配置**：根据精度-速度权衡选择

## 参考资源

- PAMR 原始论文：[Pixel-Adaptive Refinement for Instance Segmentation](https://arxiv.org/abs/2006.02713)
- 实现源码：`radio.pamr.PAMR`

## 修改日志

- **2026-06-03**：初始集成，支持训练/验证/测试阶段的 PAMR
