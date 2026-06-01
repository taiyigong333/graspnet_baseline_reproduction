# 腕部相机与固定相机 GraspNet 矩阵链路和误差分析

本文把两条 GraspNet 复现链路放在一起说明：

- 腕部相机链路：GraspNet 输入来自夹爪上的 RealSense，外参随当前夹爪 TCP 动态变化。
- 固定相机链路：GraspNet 输入来自机器人外部固定相机，外参是标定后固定的 `T_base_cam_fixed`。

本文使用统一记法：

```text
T_A_B 表示把 B 坐标系下的点变换到 A 坐标系
p_A = T_A_B @ p_B
```

## 1. 前提约定

本轮讨论的关键前提是：

1. `calibration` 采集时，RTDE 读取的是夹爪 TCP 在机器人 `base` 坐标系下的位姿：

```text
T_base_tcp_i = RTDE getActualTCPPose()
```

2. `calibration` 采用双相机联合手眼标定。腕部相机先做 eye-in-hand，固定相机再通过同一块静止标定板反推出 `T_base_cam_fixed`。
3. 因为 RTDE 输入参考系是夹爪 TCP，所以 `calibration_result.json/T_ee_cam_end.matrix_4x4` 在物理语义上应解释为：

```text
T_tcp_cam_wrist
```

也就是“腕部相机坐标系到夹爪 TCP 坐标系”的固定安装外参。

4. 固定相机抓取时应使用：

```text
calibration_result.json/T_base_cam_fixed.matrix_4x4
```

不要把 `T_tcp_cam_wrist` 填成固定相机的 `T_base_cam`。

## 2. 联合手眼标定如何得到两类外参

每个有效标定样本包含：

```text
T_base_tcp_i          # RTDE 当前夹爪 TCP 位姿
T_cam_wrist_board_i  # 腕部相机 PnP 得到的 board -> wrist camera
T_cam_fixed_board_i  # 固定相机 PnP 得到的 board -> fixed camera
```

先由腕部相机 eye-in-hand 求：

```text
T_tcp_cam_wrist = handeye(T_base_tcp_i, T_cam_wrist_board_i)
```

因为标定板在整轮采集中静止，每个样本都应给出同一个标定板 base 位姿：

```text
T_base_board_i =
  T_base_tcp_i
  @ T_tcp_cam_wrist
  @ T_cam_wrist_board_i
```

对所有有效样本求平均后得到：

```text
T_base_board = average(T_base_board_i)
```

固定相机也看到同一块标定板，因此每个样本可反推出固定相机到 base 的候选外参：

```text
T_base_cam_fixed_i =
  T_base_board
  @ inverse(T_cam_fixed_board_i)
```

最终：

```text
T_base_cam_fixed = average(T_base_cam_fixed_i)
```

注意：这里的固定相机外参不是只由固定相机图像独立求出来的，它继承了腕部相机手眼结果、腕部相机 PnP、固定相机 PnP 和 RTDE TCP 的误差。

## 3. 腕部相机 GraspNet 链路

腕部相机运行时的输入是：

```text
T_tcp_cam_wrist      # 标定结果，固定安装外参
T_base_tcp_now       # 运行时 RTDE 当前夹爪 TCP 位姿
T_cam_wrist_grasp    # GraspNet best_grasp，相机坐标系下的抓取位姿
```

其中：

```text
T_cam_wrist_grasp = [
  R_cam_wrist_grasp  t_cam_wrist_grasp
  0 0 0              1
]
```

先根据当前 TCP 动态计算腕部相机在 base 下的位姿：

```text
T_base_cam_wrist_now =
  T_base_tcp_now
  @ T_tcp_cam_wrist
```

再把 GraspNet 输出从腕部相机坐标系变到机器人 base：

```text
T_base_grasp =
  T_base_cam_wrist_now
  @ T_cam_wrist_grasp
```

展开后：

```text
t_base_grasp = (T_base_cam_wrist_now @ [t_cam_wrist_grasp, 1])[:3]
R_base_grasp = R_base_cam_wrist_now @ R_cam_wrist_grasp
```

`t_base_grasp` 是 GraspNet 抓取中心在机器人 base 下的位置。它还不是 UR 要执行的 TCP 原点。

当前本工程还会把 GraspNet 局部轴解释成现场 UR TCP 局部轴：

```text
R_base_grasp_as_tcp =
  R_base_grasp
  @ grasp_to_tcp_rotation_matrix
```

当前默认语义是：

```text
UR TCP X = GraspNet Y
UR TCP Y = GraspNet Z
UR TCP Z = GraspNet X
```

最终 TCP 姿态由配置决定：

```text
R_base_tcp_goal = resolve_tcp_rotation(
  current / fixed / graspnet
)
```

TCP 目标位置为：

```text
t_base_tcp_goal =
  t_base_grasp
  + R_base_tcp_goal @ o_tcp
  + b_base
```

其中：

| 符号 | 对应配置 | 含义 |
| --- | --- | --- |
| `o_tcp` | `grasp.tcp_translation_offset_m` | 从 GraspNet 抓取中心到 UR TCP 原点的偏移，按最终 TCP 坐标系表达 |
| `b_base` | `grasp.target_base_offset_m` | base 坐标系下的小范围整体补偿 |

最终输出给 UR 的抓取点位：

```text
tcp_goal = [t_base_tcp_goal, rotvec(R_base_tcp_goal)]
```

预抓取点沿 GraspNet 接近轴退回：

```text
a_base = normalize(R_base_grasp[:, approach_axis_index])
t_base_tcp_pregrasp =
  t_base_tcp_goal
  + approach_sign * pregrasp_offset_m * a_base
```

对应本工程实现：

```text
src/eye_in_hand_graspnet/transforms.py::compute_tcp_target()
```

## 4. 固定相机复现 GraspNet-baseline 链路

固定相机运行时的输入是：

```text
T_base_cam_fixed   # 联合手眼标定输出，固定相机 -> base
T_cam_fixed_grasp  # GraspNet best_grasp，固定相机坐标系下的抓取位姿
```

固定相机不随机械臂运动，因此相机到 base 的外参不需要每次用 RTDE 动态更新：

```text
T_base_grasp =
  T_base_cam_fixed
  @ T_cam_fixed_grasp
```

展开后：

```text
t_base_grasp = (T_base_cam_fixed @ [t_cam_fixed_grasp, 1])[:3]
R_base_grasp = R_base_cam_fixed @ R_cam_fixed_grasp
```

后续从抓取中心到 TCP 目标的公式与腕部相机链路一致：

```text
R_base_tcp_goal = resolve_tcp_rotation(...)

t_base_tcp_goal =
  t_base_grasp
  + R_base_tcp_goal @ o_tcp
  + b_base

tcp_goal = [t_base_tcp_goal, rotvec(R_base_tcp_goal)]
```

在 `reproduction/graspnet-baseline-in-ur7e` 旧复现链路中，对应配置是：

```text
configs/ur7e_graspnet.local.json
  calibration.T_base_cam = T_base_cam_fixed
```

这条链路可以读取当前 TCP 来做安全跳变量检查、`tcp_rotation_mode=current` 或初始化 XML-RPC 目标，但不应用当前 TCP 去重新计算固定相机外参。

## 5. 两条链路最容易混用的地方

| 场景 | 正确矩阵 | 不要误用 |
| --- | --- | --- |
| 腕部相机 GraspNet | `T_base_cam_wrist_now = T_base_tcp_now @ T_tcp_cam_wrist` | 不要把 `T_tcp_cam_wrist` 当静态 `T_base_cam` |
| 固定相机 GraspNet | `T_base_cam_fixed` | 不要把 `T_tcp_cam_wrist` 或 `T_base_board` 填进固定相机配置 |
| 固定相机联合标定 | `T_base_cam_fixed = average(T_base_board @ inverse(T_cam_fixed_board_i))` | 不要以为它是固定相机单独 eye-to-hand 求解 |
| 姿态轴修正 | `R_base_grasp @ local_rotation_offset` | 不要把局部轴修正写成 4x4 左乘并带着 translation 一起绕 base 原点旋转 |
| 抓取中心到 TCP | `t_base_grasp + R_base_tcp_goal @ o_tcp` | 不要把 GraspNet `translation` 直接等同于 UR TCP 原点 |

## 6. 误差传播的共同公式

两条链路最后都可以抽象为：

```text
t_base_tcp_goal =
  t_base_grasp
  + R_base_tcp_goal @ o_tcp
  + b_base
```

位置误差的一阶近似可以理解为：

```text
delta_t_goal ≈
  delta_t_base_cam
  + R_base_cam @ delta_t_cam_grasp
  + delta_theta_base_cam x (R_base_cam @ t_cam_grasp)
  + delta_theta_tcp_goal x (R_base_tcp_goal @ o_tcp)
  + R_base_tcp_goal @ delta_o_tcp
  + delta_b_base
```

实际排查时不用手算这个式子，但它说明了几个事实：

- 抓取点离相机越远，相机外参旋转误差造成的位置偏差越大。
- `o_tcp` 越大，最终 TCP 姿态误差造成的 TCP 原点偏差越大。
- `target_base_offset_m` 只能补小范围固定偏差，不能掩盖外参方向、深度尺度或 TCP 定义错误。

## 7. 腕部相机链路的主要误差来源

腕部相机链路额外依赖运行时 `T_base_tcp_now`，所以误差主要来自：

1. TCP 定义不一致：标定时使用的是夹爪 TCP，运行时示教器切换了 TCP 名称、TCP 偏置或 TCP 方向，旧的 `T_tcp_cam_wrist` 就不再对应当前 TCP。
2. 拍照和 RTDE 不同步：如果机器人仍在运动，GraspNet 图像和 `T_base_tcp_now` 不是同一姿态，`T_base_cam_wrist_now` 会错。
3. 腕部相机支架松动：`T_tcp_cam_wrist` 假设相机和夹爪 TCP 刚性固定，支架微动会直接变成动态外参误差。
4. 夹爪 TCP 原点不在抓取中心：需要用 `tcp_translation_offset_m` 表达真实偏移，否则目标会稳定偏到 TCP 机械参考点。
5. 姿态轴映射错误：轴映射错通常先表现为 TCP 姿态、预抓取方向和夹爪闭合方向异常；如果没有错误地重排 translation，抓取中心位置本身不应因此绕 base 大幅跳变。

腕部相机优先检查顺序：

1. 确认当前 RTDE 读到的是和标定时同一个夹爪 TCP。
2. 机器人静止后再拍照和读取 TCP。
3. 打印并检查 `T_base_tcp_now`、`T_base_cam_wrist_now`、`grasp_center_base`、`tcp_pregrasp`、`tcp_goal`。
4. 首轮保持 `tcp_rotation_mode=current`，先验证位置链路。
5. 再切到 `tcp_rotation_mode=graspnet`，低速验证姿态和预抓取方向。

## 8. 固定相机链路的主要误差来源

固定相机链路运行时不需要动态 `T_base_tcp_now @ T_tcp_cam`，但它的静态外参来自联合标定，主要误差来自：

1. 腕部相机手眼误差：`T_base_cam_fixed` 依赖 `T_base_board`，而 `T_base_board` 是通过 `T_base_tcp_i @ T_tcp_cam_wrist @ T_cam_wrist_board_i` 得到的。
2. 固定相机 PnP 误差：固定相机图像中 marker 少、角点模糊、内参或畸变不匹配，会直接影响 `T_cam_fixed_board_i`。
3. 标定板静止假设破坏：联合流程要求标定板在整轮采集中固定不动；如果标定板移动，腕部相机和固定相机会共同把错误写进 `T_base_cam_fixed`。
4. 固定相机移动：标定后相机支架、三脚架或线缆牵动都会让旧 `T_base_cam_fixed` 失效。
5. 相机序列号混用：固定相机抓取配置必须使用输出 `T_base_cam_fixed` 时对应的那台相机、分辨率、内参和深度对齐方式。

固定相机优先检查顺序：

1. 确认抓取配置中的 `camera.serial` 是固定相机。
2. 确认 `calibration.T_base_cam` 只来自 `T_base_cam_fixed.matrix_4x4`。
3. 不对 `T_base_cam_fixed` 求逆，不乘 `T_tcp_cam_wrist`。
4. 检查标定日志中的 `T_base_board consistency` 和 `T_base_cam_fixed consistency`。
5. 低速验证 `grasp_center_base` 是否落在机器人真实工作空间和物体附近。

## 9. 结论

两条链路的核心区别只有一个：相机到 base 的外参来源不同。

腕部相机：

```text
T_base_cam = T_base_tcp_now @ T_tcp_cam_wrist
```

固定相机：

```text
T_base_cam = T_base_cam_fixed
```

后面的 GraspNet 抓取中心到 UR TCP 目标计算可以复用同一套公式。真正要严防的是把腕部相机的 `T_tcp_cam_wrist` 当成固定相机的 `T_base_cam`，或在固定相机链路里又乘一次当前 TCP。两种混用都会让抓取点位出现系统性偏差，且通常不能靠 `target_base_offset_m` 稳定补回来。
