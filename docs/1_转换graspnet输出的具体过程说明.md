# GraspNet 输出转换为 UR7e base 下 TCP 点位

## 坐标约定

本文使用：

```text
T_A_B 表示把 B 坐标系下的点转换到 A 坐标系
p_A = T_A_B @ p_B
```

核心坐标系：

| 名称 | 含义 |
| --- | --- |
| `base` | UR7e 机器人基座坐标系 |
| `tcp` | 当前夹爪 TCP 坐标系 |
| `cam` | 腕部 RealSense 相机坐标系 |
| `grasp` | GraspNet 输出的抓取坐标系 |

## 已知输入

1. 眼在手上标定矩阵：

```text
configs/eye_in_hand_calibration_tcp.jsonc/T_tcp_cam
```

本轮标定时 RTDE 保存的是夹爪 TCP 位姿，所以这个矩阵按 `T_tcp_cam` 使用。

2. 当前夹爪 TCP：

```text
RTDE getActualTCPPose() -> [x, y, z, rx, ry, rz]
```

代码中由 `src/eye_in_hand_graspnet/robot.py` 读取。

3. GraspNet 输出：

```text
best_grasp.rotation_matrix = R_cam_grasp
best_grasp.translation     = t_cam_grasp
```

也兼容 GraspNet 常见 17 维数组：

```text
[score, width, height, depth, R(9), t(3), object_id]
```

## 转换链路

先把当前 TCP 位姿转成齐次矩阵：

```text
T_base_tcp_now = pose_to_matrix(current_tcp)
```

动态计算腕部相机在 base 下的位姿：

```text
T_base_cam_now = T_base_tcp_now @ T_tcp_cam
```

把 GraspNet 抓取姿态从相机系转换到 base：

```text
T_cam_grasp = [
  R_cam_grasp  t_cam_grasp
  0 0 0        1
]

T_base_grasp = T_base_cam_now @ T_cam_grasp
```

其中：

```text
grasp_center_base = T_base_grasp[:3, 3]
R_base_grasp      = T_base_grasp[:3, :3]
```

这里的 `R_base_grasp` 仍然保留 GraspNet 的局部轴定义：

```text
GraspNet X = 接近/depth 方向
GraspNet Y = 两指开合/width 方向
GraspNet Z = 夹爪高度/height 方向
```

而 UR7e 夹爪 TCP 的现场约定是：

```text
UR TCP X = 平行夹爪开合方向
UR TCP Y = 垂直夹爪开合方向
UR TCP Z = 垂直法兰面/接近方向
```

因此代码会先做一次局部轴重排：

```text
R_base_grasp_as_tcp = R_base_grasp @ grasp_to_tcp_rotation_matrix

grasp_to_tcp_rotation_matrix = [
  [0, 0, 1],
  [1, 0, 0],
  [0, 1, 0],
]
```

等价关系是：

```text
UR TCP X = GraspNet Y
UR TCP Y = GraspNet Z
UR TCP Z = GraspNet X
```

注意：`best_grasp.translation` 是相机坐标系下的三维位置点，不做这种 xyz 重排；它只通过 `T_base_cam_now` 转到机器人 base。轴重排只用于姿态矩阵和按 TCP 坐标系表达的偏移。

## 从抓取中心到 TCP 目标

机器人执行的是 TCP 目标，不是 GraspNet 抓取中心。代码中使用：

```text
t_base_tcp_goal =
  grasp_center_base
  + R_base_tcp_goal @ tcp_translation_offset_m
  + target_base_offset_m
```

`R_base_tcp_goal` 由 `grasp.tcp_rotation_mode` 决定：

| 模式 | 行为 |
| --- | --- |
| `current` | 保持当前 TCP 姿态，适合第一轮验证位置链路 |
| `fixed` | 使用 `fixed_tcp_rotvec` 指定的固定姿态 |
| `graspnet` | 使用 `R_base_grasp_as_tcp`，再右乘 `tcp_rotation_offset_matrix` |

最终输出：

```text
tcp_goal = [x, y, z, rx, ry, rz]
```

代码位置：

```text
src/eye_in_hand_graspnet/transforms.py::compute_tcp_target()
```

## 预抓取点

预抓取点沿 GraspNet 的接近轴退回：

```text
approach_axis = normalize(R_base_grasp[:, approach_axis_index])
t_base_tcp_pregrasp =
  t_base_tcp_goal
  + approach_sign * pregrasp_offset_m * approach_axis
```

当前默认：

```text
approach_axis_index = 0
approach_sign = -1.0
pregrasp_offset_m = 0.08
```

这对应 GraspNet 局部 X 轴作为接近方向。

## 关键实现文件

- `src/eye_in_hand_graspnet/transforms.py`：矩阵、旋转向量、GraspNet 输出解析、TCP 目标计算。
- `src/eye_in_hand_graspnet/pipeline.py`：采集、掩码、推理、RTDE、目标计算、输出和移动编排。
- `scripts/smoke_test_no_hardware.py`：离线验证中心掩码、17 维输出解析和 `T_base_tcp_now @ T_tcp_cam @ T_cam_grasp` 链路。

## 不要混用的东西

- 不要把固定相机的 GraspNet 输出乘 `T_tcp_cam`。
- 不要把 `T_tcp_cam` 当成静态 `T_base_cam`。
- 不要在当前标定前提下额外乘一次法兰盘到 TCP 的变换。
- 不要把 GraspNet 的 `translation` 直接等同于 UR TCP 原点，除非现场确认 TCP 原点就是抓取中心。
