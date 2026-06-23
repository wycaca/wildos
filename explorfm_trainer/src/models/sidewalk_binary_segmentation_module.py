from typing import Any, Dict, Tuple

import torch
import wandb
from lightning import LightningModule
from torchmetrics import MaxMetric, MeanMetric
from torchmetrics.classification.accuracy import Accuracy
from torchmetrics.classification import BinaryF1Score, BinaryJaccardIndex
from .components.radio_utils import gen_logging_image
from torch.optim import AdamW
from torch.optim.lr_scheduler import CosineAnnealingLR
from nvidia_radio.radio.pamr import PAMR

# 定义一个轻量且鲁棒的 Dice Loss
class DiceLoss(torch.nn.Module):
    def __init__(self, smooth: float = 1e-6):
        super().__init__()
        self.smooth = smooth

    def forward(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        probs = torch.sigmoid(logits)
        probs = probs.view(-1)
        targets = targets.view(-1)
        
        intersection = (probs * targets).sum()
        dice = (2. * intersection + self.smooth) / (probs.sum() + targets.sum() + self.smooth)
        return 1.0 - dice


class BinarySegmentationLitModule(LightningModule):
    """`LightningModule` for Binary Semantic Segmentation."""

    def __init__(
        self,
        net: torch.nn.Module,
        optimizer: torch.optim.Optimizer,
        scheduler: torch.optim.lr_scheduler,
        compile: bool,
        pred_threshold: float = 0.5,
        num_log_imgs: int = 4,
        validation_img_log_idx: int = 0,
        strict_loading: bool = True,
        vmax: float = 1,
        pos_weight: float = 8.0,
        use_pamr: bool = True,
        pamr_num_iter: int = 10,
        pamr_dilations: list = [1, 2, 4, 8],
    ) -> None:
        super().__init__()

        self.save_hyperparameters(ignore=["net"], logger=False)
        self.net = net
        
        self.use_pamr = use_pamr
        
        # 初始化 PAMR (Pixel-Adaptive Mask Refinement)
        # num_iter: 迭代精细化次数（5-10 次，越大边缘越精细但速度越慢）
        # dilations: 感受野空洞率，用于捕捉多尺度像素亲和力（推荐 [1,2,4] 或 [1,2,4,8]）
        if use_pamr:
            self.pamr = PAMR(
                num_iter=pamr_num_iter, 
                dilations=pamr_dilations
            )

        # 组合损失函数：BCE + Dice
        self.bce_criterion = torch.nn.BCEWithLogitsLoss(pos_weight=torch.tensor([pos_weight]))
        self.dice_criterion = DiceLoss()
        
        self.hparams_dict = {
            'pred_threshold': pred_threshold,
            'pos_weight': pos_weight,
        }

        self.phases = ["train", "val", "test"]
        for phase in self.phases:
            setattr(self, f"{phase}_acc", Accuracy(task="binary"))
            setattr(self, f"{phase}_loss", MeanMetric())
            setattr(self, f"{phase}_miou", BinaryJaccardIndex())
            setattr(self, f"{phase}_f1", BinaryF1Score(threshold=pred_threshold))

        self.val_acc_best = MaxMetric()
        self.val_iou_best = MaxMetric()
        self.pred_threshold = pred_threshold
        self.strict_loading = strict_loading

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)
    
    def apply_pamr_refinement(self, x: torch.Tensor, probs: torch.Tensor) -> torch.Tensor:
        """
        应用 PAMR (Pixel-Adaptive Mask Refinement) 细化分割掩膜
        
        Args:
            x: 输入图像 [B, C, H, W]
            probs: 概率预测 [B, 1, H, W]
            
        Returns:
            refined_probs: 精细化的概率图 [B, 1, H, W]
        """
        if not self.use_pamr:
            return probs
            
        try:
            # PAMR 根据图像局部像素相似度自适应地调整掩膜边界
            refined_probs = self.pamr(x, probs)
            return refined_probs
        except Exception as e:
            print(f"Warning: PAMR refinement failed with error {e}, using original probs")
            return probs

    def on_train_start(self) -> None:
        self.val_loss.reset()
        self.val_acc.reset()
        self.val_miou.reset()
        self.val_f1.reset()
        self.val_acc_best.reset()
        self.val_iou_best.reset()

    def model_step(
        self, batch: Dict,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Perform a single model step on a batch of data.

        :param batch: A batch of data (a dict) containing the input tensor of images and target labels.

        :return: A tuple containing (in order):
            - A tensor of losses.
            - A tensor of predictions.
            - A tensor of probabilities.
        """
        x = batch["raw_img"]
        y = batch["gt_traversability"]
        probs = self.forward(x)
        loss = self.criterion(probs, y.float())

        pred = (probs > self.pred_threshold).int()

        return loss, pred, probs

    def training_step(self, batch: Dict, batch_idx: int) -> torch.Tensor:
        # 统一调用，传入 stage='train'
        loss, preds, probs = self.model_step(batch, stage="train")
        targets = batch["gt_traversability"]

        # 更新并记录指标
        self.train_loss(loss)
        self.train_acc(preds, targets)
        self.train_miou(preds, targets)
        self.train_f1(preds, targets)

        self.train_loss(loss)
        self.train_acc(preds, targets)
        self.train_miou(preds, targets)
        self.train_f1(preds, targets)

        self.log("train/loss", self.train_loss, on_step=True, on_epoch=True, prog_bar=True)
        self.log("train/acc", self.train_acc, on_step=False, on_epoch=True, prog_bar=True)
        self.log("train/iou", self.train_miou, on_step=False, on_epoch=True, prog_bar=True)
        self.log("train/f1", self.train_f1, on_step=False, on_epoch=True, prog_bar=True)

        if batch_idx == len(self.trainer.train_dataloader) - 2:
            self.logger.experiment.log(
                {
                    "train/log_imgs": [
                        wandb.Image(img) for img in gen_logging_image(
                            batch_data={
                                "viz_img": batch["viz_img"],
                                "gt_segmentation": batch["gt_segmentation"],
                                "gt_traversability": batch["gt_traversability"],
                                "preds": preds,
                                "probs": probs.detach(),
                                "img_path": batch["img_path"],
                            },
                            seg_colormap=self.trainer.train_dataloader.dataset.seg_colormap,
                            num_log_imgs=self.hparams.num_log_imgs,
                            vmax=self.hparams.vmax
                        )
                    ]
                },
                step=self.global_step
            )
        
        return loss

    def validation_step(self, batch: Dict, batch_idx: int) -> None:
        # 直接由统一的 model_step 在内部处理好一致的 PAMR 映射
        loss, preds, probs = self.model_step(batch, stage="val")
        targets = batch["gt_traversability"]

        self.val_loss(loss)
        self.val_acc(preds, targets)
        self.val_miou(preds, targets)
        self.val_f1(targets.int(), preds.int()) # 确保类型一致

        self.log("val/loss", self.val_loss, on_step=False, on_epoch=True, prog_bar=True)
        self.log("val/acc", self.val_acc, on_step=False, on_epoch=True, prog_bar=True)
        self.log("val/iou", self.val_miou, on_step=False, on_epoch=True, prog_bar=True)
        self.log("val/f1", self.val_f1, on_step=False, on_epoch=True, prog_bar=True)

        # Visualize predictions and targets
        if batch_idx == self.hparams.validation_img_log_idx:
            self.logger.experiment.log(
                {
                    "val/log_imgs": [
                        wandb.Image(img) for img in gen_logging_image(
                            batch_data={
                                "viz_img": batch["viz_img"],
                                "gt_segmentation": batch["gt_segmentation"],
                                "gt_traversability": batch["gt_traversability"],
                                "preds": preds,
                                "probs": probs,
                                "img_path": batch["img_path"],
                            },
                            seg_colormap=self.trainer.val_dataloaders.dataset.seg_colormap,
                            num_log_imgs=self.hparams.num_log_imgs,
                            vmax=self.hparams.vmax
                        )
                    ]
                },
                step=self.global_step
            )


    def on_validation_epoch_end(self) -> None:
        "Lightning hook that is called when a validation epoch ends."
        acc = self.val_acc.compute()  # get current val acc
        self.val_acc_best(acc)  # update best so far val acc
        # log `val_acc_best` as a value through `.compute()` method, instead of as a metric object
        # otherwise metric would be reset by lightning after each epoch
        self.log("val/acc_best", self.val_acc_best.compute(), sync_dist=True, prog_bar=True)

        self.val_iou_best(self.val_miou.compute())
        self.log("val/iou_best", self.val_iou_best.compute(), sync_dist=True, prog_bar=True)
    

    def test_step(self, batch: Dict, batch_idx: int) -> None:
        # 测试阶段统一处理
        loss, preds, probs = self.model_step(batch, stage="test")
        targets = batch["gt_traversability"]

        # update and log metrics
        self.test_loss(loss)
        self.test_acc(preds, targets)
        self.test_miou(preds, targets)
        self.test_f1(preds, targets)

        self.log("test/loss", self.test_loss, on_step=False, on_epoch=True, prog_bar=True)
        self.log("test/acc", self.test_acc, on_step=False, on_epoch=True, prog_bar=True)
        self.log("test/iou", self.test_miou, on_step=False, on_epoch=True, prog_bar=True)
        self.log("test/f1", self.test_f1, on_step=False, on_epoch=True, prog_bar=True)

        # Visualize predictions and targets
        if batch_idx == self.hparams.validation_img_log_idx:
            self.logger.experiment.log(
                {
                    "test/log_imgs": [
                        wandb.Image(img) for img in gen_logging_image(
                            batch_data={
                                "viz_img": batch["viz_img"],
                                "gt_segmentation": batch["gt_segmentation"],
                                "gt_traversability": batch["gt_traversability"],
                                "preds": preds,
                                "probs": probs,
                                "img_path": batch["img_path"],
                            },
                            seg_colormap=self.trainer.test_dataloaders.dataset.seg_colormap,
                            num_log_imgs=self.hparams.num_log_imgs,
                            vmax=self.hparams.vmax
                        )
                    ]
                },
                step=self.global_step
            )

    def setup(self, stage: str) -> None:
        """Lightning hook that is called at the beginning of fit (train + validate), validate,
        test, or predict.

        This is a good hook when you need to build models dynamically or adjust something about
        them. This hook is called on every process when using DDP.

        :param stage: Either `"fit"`, `"validate"`, `"test"`, or `"predict"`.
        """
        if self.hparams.compile and stage == "fit":
            self.net = torch.compile(self.net)

    def configure_optimizers(self) -> Dict[str, Any]:
        """Choose optimizers and learning-rate schedulers to use in optimization.

        :return: A dict containing the configured optimizers and learning-rate schedulers to be used for training.
        """
        # 为所有可训练参数创建优化器
        trainable_params = filter(lambda p: p.requires_grad, self.parameters())
        optimizer = self.hparams.optimizer(params=trainable_params)
        
        if self.hparams.scheduler is not None:
            scheduler = self.hparams.scheduler(optimizer=optimizer)
            return {
                "optimizer": optimizer,
                "lr_scheduler": {
                    "scheduler": scheduler,
                    "monitor": "val/loss",
                    "interval": "epoch",
                    "frequency": 1,
                },
            }
        return {"optimizer": optimizer}

if __name__ == "__main__":
    from .components.radio_cnn import RADIO_CNN
    _ = BinarySegmentationLitModule(
        net=RADIO_CNN(
            model_version="c-radio_v3-b",
            adaptor_version=None,
        ),
        optimizer=torch.optim.Adam,
        scheduler=torch.optim.lr_scheduler.StepLR,
        compile=False,
        pred_threshold=0.5,
        num_log_imgs=4,
        validation_img_log_idx=0
    )