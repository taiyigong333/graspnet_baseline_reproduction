# 类似 graspnet-baseline issue 65 的坐标旋转问题排查与修正

## 结论

本目录当前没有发现 `GraspNet 17 维 grasp` 解析顺序错误，也没有发现 `T_base_cam_now @ T_cam_grasp` 的外参左乘链路写反。问题确实发生在“GraspNet 抓取坐标系到 UR TCP 坐标系”的使用层面，和 issue 65 中“已经把 grasp 位姿变到世界系后，又用世界系旋转去补夹爪局部姿态，导致位置/方向异常”的类型一致。

本次实际风险由三件事叠加触发：

1. `grasp.tcp_rotation_mode` 被切到 `graspnet`，最终 TCP 姿态开始跟随 GraspNet。
2. `motion.enabled_steps` 被改成 `["start", "grasp"]`，跳过了用于低速确认方向的 `pregrasp`。
3. `grasp.target_base_offset_m` 被设置为 `[0.0, 0.0, -0.1]`，这是 10 cm 的 base 坐标系全局位置补偿，远超“毫米级现场修正”的用途。

这会让现场现象看起来像“GraspNet 姿态一修正，位置也不对了”。根因不是 GraspNet 17 维格式本身，而是局部姿态补偿、TCP 局部原点偏置和 base 全局位置补偿没有被强制分开审计。

## 外部依据

外部参考：

- graspnet-baseline issue 65：<https://github.com/graspnet/graspnet-baseline/issues/65>
- GraspNetAPI 6D grasp 格式：<https://graspnetapi.readthedocs.io/en/latest/grasp_format.html#d-grasp>

issue 65 中的问题描述是：用户已经把 GraspNet 位姿转换到世界坐标下，但再按世界坐标做旋转补偿后结果异常。这和本工程需要区分“外部坐标变换左乘”和“夹爪局部轴补偿右乘”的问题一致。

GraspNetAPI 的 6D grasp 格式是：

```text
[score, width, height, depth, rotation_matrix(9), translation(3), object_id]
```

因此本工程解析 `rotation_matrix = data[4:13].reshape(3,3)`、`translation = data[13:16]` 是正确的。

GraspNetAPI 文档还说明 `Grasp/GraspGroup` 可以用 4x4 矩阵整体变换。对应到本工程，外参变换应是左乘：

```text
T_base_grasp = T_base_cam_now @ T_cam_grasp
```

而夹爪局部轴到 UR TCP 局部轴的修正不是世界系外参，不能写成：

```text
T_fix @ T_base_grasp
```

这种写法会连同 translation 一起绕世界原点旋转。正确做法是只右乘姿态部分：

```text
R_base_grasp_as_tcp = R_base_grasp @ grasp_to_tcp_rotation_matrix
```

这正是 issue 65 中容易混淆的点：已经在世界系或机器人 base 系下的位姿，如果要补“夹爪自身坐标系”差异，应右乘局部旋转；左乘是对整个世界/base 位姿再做外部坐标变换。

## 本工程正确链路

坐标约定：

```text
T_A_B 表示把 B 坐标系下的点转换到 A 坐标系
p_A = T_A_B @ p_B
```

眼在手上动态外参：

```text
T_base_tcp_now = pose_to_matrix(current_tcp)
T_base_cam_now = T_base_tcp_now @ T_tcp_cam
```

GraspNet 输出到 base：

```text
T_cam_grasp = [
  R_cam_grasp  t_cam_grasp
  0 0 0        1
]

T_base_grasp = T_base_cam_now @ T_cam_grasp
grasp_center_base = T_base_grasp[:3, 3]
R_base_grasp = T_base_grasp[:3, :3]
```

GraspNet 局部轴到 UR TCP 局部轴：

```text
R_base_grasp_as_tcp = R_base_grasp @ grasp_to_tcp_rotation_matrix
```

当前默认轴语义：

| 语义 | GraspNet 局部轴 | UR TCP 轴 |
| --- | --- | --- |
| 接近 / depth 方向 | X | Z |
| 两指开合 / width 方向 | Y | X |
| 夹爪高度 / height 方向 | Z | Y |

因此：

```text
UR TCP X = GraspNet Y
UR TCP Y = GraspNet Z
UR TCP Z = GraspNet X
```

默认矩阵为：

```json
[
  [0.0, 0.0, 1.0],
  [1.0, 0.0, 0.0],
  [0.0, 1.0, 0.0]
]
```

注意：`best_grasp.translation` 是相机坐标系下的位置点，只通过 `T_base_cam_now` 变换到 base。不要因为夹爪轴定义不同就重排 translation 的 xyz，也不要把局部姿态修正写成 4x4 左乘。

## 已做修正

### 1. 结果 JSON 增加分量诊断

`src/eye_in_hand_graspnet/transforms.py` 的 `TcpTargetResult.to_json_dict()` 现在额外输出：

```text
target.tcp_translation_offset_base
target.target_base_offset
target.approach_axis_base
target.pregrasp_offset_base
```

`src/eye_in_hand_graspnet/pipeline.py` 同步在终端打印这些分量。

以后检查 `outputs/last_wrist_grasp_result.json` 时可以直接判断：

| 字段 | 含义 | 异常时优先检查 |
| --- | --- | --- |
| `grasp_center_base` | GraspNet 抓取中心经 eye-in-hand 外参变到 base 后的位置 | 相机深度、`T_tcp_cam`、当前 TCP 同步 |
| `tcp_translation_offset_base` | `tcp_translation_offset_m` 按最终 TCP 姿态转到 base 后的偏置 | TCP 原点是否等于视觉抓取中心 |
| `target_base_offset` | base 坐标系全局补偿 | 是否误用大补偿掩盖标定或姿态错误 |
| `pregrasp_offset_base` | 预抓取点沿接近轴退让的 base 向量 | `approach_axis_index` 和 `approach_sign` |

### 2. `--execute` 增加安全拦截

`src/eye_in_hand_graspnet/pipeline.py::_validate_execute_config()` 新增两条拒绝运行规则：

1. `grasp.target_base_offset_m` 模长超过 3 cm 时拒绝 `--execute`。
2. `tcp_rotation_mode="graspnet"` 且执行 `grasp`，但 `motion.enabled_steps` 缺少 `pregrasp` 时拒绝 `--execute`。

这两条只限制实机执行，`--dry-run` 仍可用于排查和打印分量。

### 3. 当前主配置与安全建议

`configs/eye_in_hand_ur7e_wrist.jsonc` 当前为：

```json
"tcp_rotation_mode": "current"
"target_base_offset_m": [0.0, 0.0, 0.0]
"enabled_steps": ["start", "open-current", "pregrasp", "grasp", "close"]
```

这样运行完整流程时会按状态顺序执行并打印状态名。首次实机仍建议临时把 `enabled_steps` 缩小到 `start` 或 `pregrasp`，确认位置链路和预抓取退让方向后再恢复完整步骤。

### 4. smoke test 增加回归用例

`scripts/smoke_test_no_hardware.py` 新增检查：

- `grasp_to_tcp_rotation_matrix` 右乘只改变姿态，不移动 `grasp_center_base`。
- `tcp_translation_offset_m` 是 TCP 局部坐标偏置，会通过 `R_base_tcp_goal @ offset` 转成 base 偏置。
- `--execute` 会拒绝 10 cm 级 `target_base_offset_m`。
- `--execute` 会拒绝 `graspnet` 姿态下跳过 `pregrasp` 直接 `grasp`。

## 现场修正流程

1. 先保持：

```json
"tcp_rotation_mode": "current"
"target_base_offset_m": [0.0, 0.0, 0.0]
```

只验证 `grasp_center_base`、`tcp_pregrasp`、`tcp_goal` 的位置是否合理。

2. 如果目标点整体只差几毫米，可以临时用 `target_base_offset_m` 做小补偿；如果需要厘米级补偿，先查标定、深度尺度、TCP 定义和图像/RTDE 时间同步。

3. 如果 UR TCP 原点不在 GraspNet 抓取中心，填 `tcp_translation_offset_m`，不要用 `target_base_offset_m` 替代。这个偏置按最终 TCP 坐标系表达。

4. 确认位置链路后再切到：

```json
"tcp_rotation_mode": "graspnet"
```

切换后首次 `--execute` 必须保留 `pregrasp`，低速确认 TCP 姿态和退让方向，再开放最终 `grasp`。

5. 如果姿态整体固定偏转，只改 `tcp_rotation_offset_matrix`。不要通过左乘 4x4 旋转去补夹爪局部坐标系，也不要重排 `translation`。

## 快速判据

如果看到以下现象，按对应方向查：

| 现象 | 更可能的原因 | 修正 |
| --- | --- | --- |
| 切到 `graspnet` 后位置也大幅变 | 把局部旋转当作世界系变换，或用大 `target_base_offset_m` 掩盖问题 | 保持右乘姿态；把 base 补偿降到毫米级 |
| `grasp_center_base` 明显不在物体附近 | eye-in-hand 外参、深度尺度、相机/RTDE 不同步 | 先不要调姿态，复核 `T_tcp_cam` 与输入深度 |
| `tcp_goal` 相对 `grasp_center_base` 偏得不合理 | `tcp_translation_offset_m` 语义或数值错 | 按最终 TCP 坐标系重测 TCP 原点到视觉抓取中心偏置 |
| `tcp_pregrasp` 朝物体内部移动 | `approach_sign` 反了，或 `approach_axis_index` 不对 | 优先改 `approach_sign`，再检查轴定义 |
| 姿态方向整体差一个固定角度 | 现场夹爪安装相对默认 TCP 轴有固定偏差 | 小范围修改 `tcp_rotation_offset_matrix` |
