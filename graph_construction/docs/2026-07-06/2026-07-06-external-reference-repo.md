# 社区参考仓库本地副本

日期: 2026-07-06

## 背景

后续 object search, triangulation, goal mux 和 planner 对比会频繁参考社区实现

之前临时下载到 `/tmp/wildos_ref` 不方便长期对比, 因此将社区仓库 clone 到项目内的参考目录

## 目录位置

本地参考目录:

```text
external_references/nebula2-wildos-main_ws
```

远端:

```text
https://github.com/TIKTOKDAD/nebula2-wildos-main_ws.git
```

当前记录:

```text
clone 类型: shallow clone, depth=1
当前 commit: 225be74
目录大小: 338M
```

## 版本管理规则

`external_references/` 已加入主项目 `.gitignore`

原因:

- 这是外部参考代码, 不应混入本项目源码提交
- 社区仓库包含 assets, ckpts 等较大目录
- 后续对比时应显式引用路径, 不要把其中代码直接复制到主项目

## 使用方式

查看参考代码:

```bash
rg -n "initial_goal_mux|obj_mask_triangulation|frontier_scores" external_references/nebula2-wildos-main_ws
```

对比单文件:

```bash
diff -u \
  external_references/nebula2-wildos-main_ws/visual_navigation/visual_navigation/wildos/nav.py \
  visual_navigation/visual_navigation/wildos/nav.py
```

更新参考仓库:

```bash
git -C external_references/nebula2-wildos-main_ws pull --ff-only
```

如果后续需要完整提交历史:

```bash
git -C external_references/nebula2-wildos-main_ws fetch --unshallow
```

## 注意事项

- 不要在参考仓库里做本项目功能开发
- 不要把参考仓库路径加入 `PYTHONPATH`
- 如果需要移植代码, 先写设计说明, 再按本项目 topic profile, 中文日志和注释规则重写
- 引用社区实现时记录源文件路径和 commit, 避免后续远端变化导致语义不一致
