# `eye_in_hand_calibration_tcp.json` 配置说明

这个文件是本项目使用的 eye-in-hand 标定矩阵快照。主配置 `eye_in_hand_ur7e_wrist.json` 通过下面两个字段读取它：

```json
"calibration": {
  "path": "configs/eye_in_hand_calibration_tcp.json",
  "matrix_key": "T_tcp_cam"
}
```

读取代码在 `src/eye_in_hand_graspnet/config.py::load_T_tcp_cam()`。真正使用矩阵的地方在 `src/eye_in_hand_graspnet/transforms.py::compute_tcp_target()`。

## 1. 文件用途

本文件把原始手眼标定结果复制成一个独立、可追溯的配置快照，避免运行抓取流程时直接依赖 `calibration/outputs/.../calibration_result.json`。

当前矩阵语义：

```text
T_tcp_cam = 从腕部相机坐标系到 UR 当前 TCP 坐标系的 4x4 齐次变换
```

抓取时的动态转换链路：

```text
T_base_tcp_now = RTDE 当前 TCP 位姿
T_base_cam_now = T_base_tcp_now @ T_tcp_cam
T_base_grasp   = T_base_cam_now @ T_cam_grasp
```

所以 `T_tcp_cam` 不是固定相机外参，也不是 `T_base_cam`。它必须和“当前 UR TCP 定义、腕部相机安装姿态、标定时使用的相机”保持一致。

## 2. 字段说明

| 字段 | 类型 | 当前值/形态 | 是否建议手改 | 含义 |
| --- | --- | --- | --- | --- |
| `source` | `str` | 原始标定结果路径 | 可改 | 记录矩阵来自哪个标定输出文件。只用于追溯，不参与运行计算。 |
| `note` | `str` | 标定语义说明 | 可改 | 说明为什么原标定输出在本工程中按 `T_tcp_cam` 使用。只用于人工阅读。 |
| `mode` | `str` | `"eye_in_hand"` | 不建议手改 | 记录原标定模式。只用于追溯，不参与运行计算。 |
| `num_total_samples` | `int` | `35` | 不建议手改 | 原始采集样本总数。只用于追溯。 |
| `num_valid_samples` | `int` | `34` | 不建议手改 | 原始标定有效样本数。只用于追溯。 |
| `handeye_method` | `str` | `"tsai"` | 不建议手改 | 原标定使用的手眼算法。只用于追溯。 |
| `T_tcp_cam` | `4x4` 数值矩阵 | 见 JSON | 只在替换整套标定结果时改 | 抓取流程真正读取和使用的矩阵。主配置的 `calibration.matrix_key` 当前指向它。 |

## 3. `T_tcp_cam` 矩阵格式

矩阵必须是 4x4 齐次变换：

```text
[
  [r11, r12, r13, tx],
  [r21, r22, r23, ty],
  [r31, r32, r33, tz],
  [0.0, 0.0, 0.0, 1.0]
]
```

其中：

- 左上 3x3 是旋转矩阵，应该接近正交矩阵，行列式接近 1。
- 最后一列前三项 `[tx, ty, tz]` 是相机原点在 TCP 坐标系下的位置，单位米。
- 最后一行必须是 `[0.0, 0.0, 0.0, 1.0]`。

当前平移量约为：

```text
tx = -0.003711826 m
ty = -0.060501883 m
tz =  0.041266688 m
```

也就是相机原点相对 TCP 原点大约是：

```text
x = -3.7 mm
y = -60.5 mm
z =  41.3 mm
```

这个量级可用于人工 sanity check，但不要只凭肉眼量测替换矩阵。

## 4. 什么时候需要改这个文件

需要替换 `T_tcp_cam` 的情况：

- 腕部相机重新安装、松动或更换。
- UR 示教器里的 TCP 定义变了。
- 标定采集时读到的不是同一个 TCP 语义。
- 重新跑了 eye-in-hand 标定，并确认新结果质量更好。
- 当前抓取点出现稳定、明显且不能用小范围 `target_base_offset_m` 解释的系统误差。

不应该改 `T_tcp_cam` 的情况：

- 只是换了固定观察点 `motion.fixed_start_tcp`。
- 只是调整 GraspNet 掩码区域。
- 只是调整夹爪开合值。
- 只是想微调最终抓取点几毫米。小范围现场补偿应优先用主配置里的 `grasp.target_base_offset_m` 或 `grasp.tcp_translation_offset_m`，但仍要避免掩盖标定错误。

## 5. 如何替换标定结果

推荐做法：

1. 在 `calibration/` 工程重新完成 eye-in-hand 标定。
2. 确认标定采集时 RTDE 读取的是夹爪 TCP，而不是法兰盘中心。
3. 从新的标定结果中复制 4x4 矩阵到本文件的 `T_tcp_cam`。
4. 同步更新 `source`、`note`、`num_total_samples`、`num_valid_samples`、`handeye_method` 等追溯字段。
5. 运行无硬件 smoke：

```powershell
python scripts\smoke_test_no_hardware.py
```

6. 连接相机、GraspNet 服务和 RTDE 后先跑 dry-run：

```powershell
python scripts\run_wrist_grasp_cycle.py --config configs\eye_in_hand_ur7e_wrist.json --dry-run
```

7. 检查输出里的 `target.T_base_cam_now`、`target.grasp_center_base`、`target.tcp_pregrasp` 和 `target.tcp_goal` 是否合理。

8. 首次实机执行只开放 `start` 或 `pregrasp`，低速确认方向和高度后再开放 `grasp` 和 `close`。

## 6. 和主配置的关系

主配置中的这两个字段决定读取本文件哪个矩阵：

| 主配置字段 | 当前值 | 作用 |
| --- | --- | --- |
| `calibration.path` | `"configs/eye_in_hand_calibration_tcp.json"` | 指向本文件。 |
| `calibration.matrix_key` | `"T_tcp_cam"` | 从本文件读取 `T_tcp_cam` 字段。 |

如果以后在同一个文件里保留多个矩阵，可以改 `matrix_key` 指向新的字段。但更推荐一个文件只保留当前要运行的矩阵，历史矩阵放到单独命名的快照文件里，减少误用风险。

## 7. 常见错误

| 错误 | 后果 | 修正 |
| --- | --- | --- |
| 把 `T_tcp_cam` 改成固定相机的 `T_base_cam` | 腕部相机动态外参链路失效，抓取点会严重错误。 | 只填 eye-in-hand 标定得到的 TCP 到相机关系。 |
| TCP 定义变了但继续用旧矩阵 | `T_base_cam_now` 系统性偏移。 | 重新标定或恢复标定时的 TCP 定义。 |
| 只改矩阵，不改 `source` 和 `note` | 后续无法判断矩阵来源。 | 替换矩阵时同步更新追溯字段。 |
| 直接手工微调旋转矩阵某个元素 | 可能破坏旋转矩阵正交性。 | 用重新标定或合法旋转补偿处理，不要单点改数。 |
| 用大 `target_base_offset_m` 掩盖标定错误 | 某些姿态看似能抓，换姿态后误差会放大。 | 大偏差先查标定、TCP 定义、深度尺度和时间同步。 |
