# GraspNet 局部轴到 UR TCP 轴的转换说明

## 结论

原实现已经做了相机坐标到机器人 base 坐标的转换：

```text
T_base_cam_now = T_base_tcp_now @ T_tcp_cam
T_base_grasp = T_base_cam_now @ T_cam_grasp
```

这能把 `best_grasp.translation` 从腕部相机坐标系转换到 UR7e base 坐标系，但没有显式处理 GraspNet 抓取局部轴和现场 UR TCP 轴定义不一致的问题。现在已经补上这一步：

```text
R_base_grasp_as_tcp = R_base_grasp @ grasp_to_tcp_rotation_matrix
```

主配置位置：

```text
configs/eye_in_hand_ur7e_wrist.json/grasp/grasp_to_tcp_rotation_matrix
```

当前默认值：

```json
[
  [0.0, 0.0, 1.0],
  [1.0, 0.0, 0.0],
  [0.0, 1.0, 0.0]
]
```

## 为什么不是直接重排 translation 的 xyz

`best_grasp.translation = [x, y, z]` 表示抓取中心在相机坐标系下的位置点。它的 xyz 是相机坐标轴，不是 GraspNet 夹爪局部轴，所以不能因为 UR TCP 轴定义不同就直接改成 `[y, z, x]`。

正确做法是：

1. 位置点按外参从相机系变到 base：

```text
grasp_center_base = (T_base_cam_now @ [t_cam_grasp, 1])[:3]
```

2. 姿态矩阵先从相机系变到 base：

```text
R_base_grasp = R_base_cam_now @ R_cam_grasp
```

3. 再把 GraspNet 局部轴重解释为 UR TCP 局部轴：

```text
R_base_grasp_as_tcp = R_base_grasp @ grasp_to_tcp_rotation_matrix
```

## 轴定义对应关系

来自 `docs/2_转换的注意事项.md` 的现场约定：

| 语义 | GraspNet 局部轴 | UR TCP 轴 |
| --- | --- | --- |
| 接近 / depth 方向 | X | Z |
| 两指开合 / width 方向 | Y | X |
| 夹爪高度 / height 方向 | Z | Y |

因此默认矩阵表达的是：

```text
UR TCP X = GraspNet Y
UR TCP Y = GraspNet Z
UR TCP Z = GraspNet X
```

这个矩阵是右手系旋转矩阵，满足：

```text
R.T @ R = I
det(R) = 1
```

## 对运行模式的影响

`grasp.tcp_rotation_mode = current` 时，机器人目标姿态仍保持当前 TCP 姿态。这个模式用于第一轮低风险验证位置链路，所以轴重排不会改变 `tcp_goal` 的姿态，但结果 JSON 会输出：

```text
target.R_base_grasp
target.R_base_grasp_as_tcp
target.R_base_tcp_goal
```

可以用它们离线检查姿态转换是否合理。

`grasp.tcp_rotation_mode = graspnet` 时，目标 TCP 姿态使用：

```text
R_base_tcp_goal = R_base_grasp_as_tcp @ tcp_rotation_offset_matrix
```

这里 `tcp_rotation_offset_matrix` 只保留给现场残余修正，不再承担 GraspNet 轴到 UR TCP 轴的基础重排。

## 相关实现

- `src/eye_in_hand_graspnet/config.py`：新增 `grasp_to_tcp_rotation_matrix` 默认值。
- `src/eye_in_hand_graspnet/transforms.py`：在 `compute_tcp_target()` 中计算 `R_base_grasp_as_tcp`。
- `src/eye_in_hand_graspnet/pipeline.py`：把配置传入目标计算函数，并把结果保存到 `outputs/last_wrist_grasp_result.json`。
- `scripts/smoke_test_no_hardware.py`：验证默认矩阵是合法旋转矩阵，并验证 `graspnet` 姿态模式下 `UR TCP X/Y/Z = GraspNet Y/Z/X`。

## 实机建议

首轮仍建议使用：

```json
"tcp_rotation_mode": "current"
```

先验证 `grasp_center_base`、`tcp_pregrasp` 和 `tcp_goal` 的位置变化。确认位置链路、夹爪 TCP 原点偏置和安全高度后，再切换到：

```json
"tcp_rotation_mode": "graspnet"
```

切换后先只开放 `start` 或 `pregrasp`，低速观察 TCP 姿态是否符合现场夹爪安装方向，再逐步开放 `grasp` 和 `close`。
