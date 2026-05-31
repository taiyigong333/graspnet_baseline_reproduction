# `eye_in_hand_ur7e_wrist.jsonc` 配置说明

这个文件是腕部相机 eye-in-hand GraspNet 抓取流程的主配置。运行入口是：

```powershell
python scripts\run_wrist_grasp_cycle.py --config configs\eye_in_hand_ur7e_wrist.jsonc --dry-run
python scripts\run_wrist_grasp_cycle.py --config configs\eye_in_hand_ur7e_wrist.jsonc --execute
```

配置加载代码在 `src/eye_in_hand_graspnet/config.py`，主流程在 `src/eye_in_hand_graspnet/pipeline.py`。所有相对路径都会按本项目根目录 `reproduction/graspnet_baseline_ur7e_by_eye_in_hand/` 解析。

## 1. 配置前先确认

实机前优先确认这些字段：

| 字段 | 必须确认的原因 |
| --- | --- |
| `camera.serial` | 必须是腕部 RealSense，不能误用固定相机。 |
| `robot.host` | RTDE 和 Dashboard 都会连这个机器人 IP。 |
| `calibration.path` / `calibration.matrix_key` | 决定 `T_tcp_cam`，直接影响相机坐标到机器人 base 的转换。 |
| `graspnet.url` / `graspnet.request_format` | 必须和当前 GraspNet HTTP 服务协议一致。 |
| `grasp.tcp_rotation_mode` | 决定最终 TCP 姿态是否跟随 GraspNet。首轮低风险验证建议用 `current`。 |
| `grasp.tcp_translation_offset_m` | 决定视觉抓取中心和真实 UR TCP 原点之间的偏置。 |
| `motion.fixed_start_tcp` | `--execute` 且启用 `start` 时，机器人会先去这个固定观察位。必须低速确认过。 |
| `motion.enabled_steps` | 决定实机真正执行哪些动作。首次建议只开放到 `start` 或 `pregrasp`。 |
| `motion.xmlrpc_port` | 示教器 URP 的 RPC URL 必须指向 Windows PC 的同一端口。 |
| `motion.grasp_program` | Dashboard 会加载这个示教器程序。 |

## 2. 顶层字段

| 字段 | 类型 | 当前值 | 含义 |
| --- | --- | --- | --- |
| `random_seed` | `int` 或 `null` | `20260531` | 设置 numpy 随机种子，并通过 `graspnet.seed_field` 发送给 GraspNet 服务。若服务端使用该 seed，可提高同一输入下的复现实验一致性。 |
| `camera` | `object` | 见下文 | 腕部 RealSense 采集参数。 |
| `workspace_mask` | `object` | 见下文 | 输入 GraspNet 前的工作区掩码。 |
| `calibration` | `object` | 见下文 | eye-in-hand 标定矩阵文件位置和矩阵字段名。 |
| `robot` | `object` | 见下文 | UR 机器人 RTDE/Dashboard 连接参数。 |
| `graspnet` | `object` | 见下文 | GraspNet HTTP 推理服务请求参数。 |
| `grasp` | `object` | 见下文 | GraspNet 输出到 UR TCP 目标位姿的转换策略。 |
| `motion` | `object` | 见下文 | `--execute` 时的 XML-RPC 目标服务、Dashboard 加载 URP 和动作步骤。 |
| `preview` | `object` | 见下文 | 腕部图像预览和截图保存。 |
| `outputs` | `object` | 见下文 | 结果 JSON 输出路径。 |

## 3. `camera`

代码位置：`src/eye_in_hand_graspnet/camera.py`。

这个部分只启动腕部 RealSense，相机采集后会返回：

- `color_bgr`：BGR 彩色图。
- `depth_raw`：RealSense 原始深度图。
- `depth_scale_m`：每个 depth raw 单位对应的米数。
- `color_intrinsics`：彩色相机内参。
- `timestamp_ms`：采集时间戳。
- `camera_serial`：当前配置的序列号。

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `serial` | `str` | `"419122270341"` | 要启用的 RealSense 序列号。当前链路要求使用腕部相机。换相机时必须同步检查标定矩阵，因为 `T_tcp_cam` 只对标定时那台相机和安装姿态有效。 |
| `color_width` | `int` | `1280` | 彩色图宽度，单位像素。要和 RealSense 支持的分辨率、GraspNet 服务预期输入兼容。 |
| `color_height` | `int` | `720` | 彩色图高度，单位像素。 |
| `depth_width` | `int` | `1280` | 深度图宽度，单位像素。当前建议和彩色图一致。 |
| `depth_height` | `int` | `720` | 深度图高度，单位像素。当前建议和彩色图一致。 |
| `fps` | `int` | `30` | 相机帧率。提高帧率不一定提高抓取效果，先保证曝光和深度稳定。 |
| `warmup_frames` | `int` | `10` | 启动相机后丢弃的预热帧数。相机刚启动时自动曝光和深度可能不稳定，建议保留。 |
| `timeout_ms` | `int` | `3000` | 等待相机帧的超时时间，单位毫秒。相机偶发超时时可适当增大。 |
| `align_depth_to_color` | `bool` | `true` | 是否把深度对齐到彩色图坐标系。当前 GraspNet 输入使用彩色相机内参，建议保持 `true`。 |

常见调整：

- 只换分辨率：同时改 `color_*` 和 `depth_*`，再跑 `--dry-run` 检查 GraspNet 服务是否接受。
- 换腕部相机或重新安装相机：不能只改 `serial`，还要重新生成或替换 `T_tcp_cam`。
- 出现相机 timeout：先检查序列号和 USB，再考虑增大 `timeout_ms` 或降低分辨率。

## 4. `workspace_mask`

代码位置：`src/eye_in_hand_graspnet/mask.py`，主流程调用位置是 `pipeline.py`。

当前实现使用中心矩形掩码：保留图像中心区域，RGB 外部置黑，depth 外部写成 `masked_depth_value`，预览图可选择把外部区域变暗。

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `type` | `str` | `"center_fraction"` | 掩码类型。当前代码按中心比例掩码使用，建议保持该值。 |
| `width_fraction` | `float` | `0.5` | 保留中心区域的宽度比例，范围建议 `(0, 1]`。`0.5` 表示只保留图像中心一半宽度。 |
| `height_fraction` | `float` | `0.5` | 保留中心区域的高度比例。`0.5` 表示只保留图像中心一半高度。 |
| `masked_depth_value` | `int` | `0` | 掩码外深度填充值。通常用 `0` 表示无效深度。 |
| `preview_dim_outside` | `bool` | `true` | 预览图中是否把掩码外区域变暗。只影响显示和保存的预览图，不改变发给 GraspNet 的真实 mask 字段。 |

当前 `width_fraction=0.5` 且 `height_fraction=0.5` 时，理论保留像素约占整图 25%。如果目标经常在掩码外，应先调整相机观察位或固定观察点，再适当放大比例。

## 5. `calibration`

代码位置：`src/eye_in_hand_graspnet/config.py::load_T_tcp_cam()` 和 `src/eye_in_hand_graspnet/transforms.py::compute_tcp_target()`。

转换链路是：

```text
T_base_tcp_now = RTDE 当前 TCP 位姿转 4x4 矩阵
T_base_cam_now = T_base_tcp_now @ T_tcp_cam
T_base_grasp   = T_base_cam_now @ T_cam_grasp
```

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `path` | `str` | `"configs/eye_in_hand_calibration_tcp.jsonc"` | 标定矩阵 JSON 文件路径。相对路径按项目根目录解析。 |
| `matrix_key` | `str` | `"T_tcp_cam"` | 从标定 JSON 中读取哪个字段作为 4x4 外参矩阵。当前必须读取 `T_tcp_cam`。 |

`T_tcp_cam` 的语义是“从相机坐标系到 UR 当前 TCP 坐标系”的齐次变换。它不是固定相机的 `T_base_cam`，不能当作静态相机外参使用。每次抓取都会读取当前 RTDE TCP，再动态计算 `T_base_cam_now`。

## 6. `robot`

代码位置：`src/eye_in_hand_graspnet/robot.py`。

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `host` | `str` | `"192.168.1.88"` | UR 控制器 IP。RTDE 读取当前 TCP、Dashboard 加载/启动 URP 都使用这个地址。 |
| `rtde_enabled` | `bool` | `true` | 是否允许用 RTDE 读取当前 TCP。当前主流程必须读取当前 TCP，实机和 dry-run 都建议保持 `true`。设为 `false` 会在读取 TCP 时直接报错。 |

注意：这里读取的是 `rtde_receive.RTDEReceiveInterface(host).getActualTCPPose()`，返回 `[x, y, z, rx, ry, rz]`，前三项单位是米，后三项是旋转向量，单位弧度。

## 7. `graspnet`

代码位置：`src/eye_in_hand_graspnet/graspnet_client.py`。

这个部分控制如何向 GraspNet-baseline HTTP 服务发送 RGB-D、内参和掩码，并解析返回的 `best_grasp`。

### 7.1 通用字段

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `url` | `str` | `"http://127.0.0.1:18080/infer"` | GraspNet 推理服务地址。服务不在本机时改成实际 IP 和端口。 |
| `timeout_s` | `float` | `30.0` | HTTP 请求超时时间，单位秒。模型推理慢时可增大。 |
| `request_format` | `str` | `"json_npy"` | 请求格式。可选值是 `json_npy` 或 `multipart`。必须和服务端协议一致。 |
| `seed_field` | `str` | `"seed"` | seed 字段名。只有 `random_seed` 不为 `null` 时才发送。 |
| `mask_field` | `str` | `"workspace_mask"` | workspace mask 字段名。`json_npy` 和 `multipart` 都会使用。 |
| `factor_depth_field` | `str` | `"factor_depth"` | 深度尺度字段名。发送值为 `1.0 / depth_scale_m`。 |
| `top_k` | `int` | `50` | 请求服务端返回或筛选的候选数量。是否真正生效取决于服务端实现。 |
| `extra_fields` | `object` | `{"camera_serial": "419122270341"}` | 额外随请求发送的字段。当前用于记录或传递相机序列号。 |

### 7.2 `json_npy` 格式下的字段

当前默认 `request_format="json_npy"`。客户端会发送 JSON，数组字段用 `.npy + base64` 编码。

实际发送字段：

| 发送字段 | 来源 | 说明 |
| --- | --- | --- |
| `color_rgb` | 腕部相机 BGR 转 RGB 后编码 | 字段名在当前代码中固定为 `color_rgb`。 |
| `depth` | 掩码后的 `depth_raw` | 字段名在当前代码中固定为 `depth`。 |
| `intrinsic_matrix` | 彩色相机 3x3 内参矩阵 | 字段名在当前代码中固定为 `intrinsic_matrix`。 |
| `factor_depth` | `1.0 / depth_scale_m` | 字段名由 `factor_depth_field` 控制。 |
| `top_k` | `graspnet.top_k` | 字段名固定为 `top_k`。 |
| `workspace_mask` | 中心矩形掩码 | 字段名由 `mask_field` 控制。 |
| `seed` | `random_seed` | 字段名由 `seed_field` 控制。 |
| `extra_fields` 中每一项 | 配置文件 | 原样加入 JSON payload。 |

重要细节：在 `json_npy` 模式下，`color_field`、`depth_field`、`intrinsics_field` 当前不会影响 `color_rgb`、`depth`、`intrinsic_matrix` 这三个字段名。只有服务端接口改了时，才需要同步改客户端代码或切到 `multipart`。

### 7.3 `multipart` 格式下的字段

如果 `request_format="multipart"`，客户端会用文件上传方式发送：

| 配置字段 | 作用 |
| --- | --- |
| `color_field` | 彩色 PNG 文件字段名。 |
| `depth_field` | 深度 PNG 文件字段名。 |
| `mask_field` | 掩码 PNG 文件字段名。 |
| `intrinsics_field` | 相机内参 JSON 字段名。 |
| `factor_depth_field` | 深度尺度字段名。 |
| `seed_field` | seed 字段名。 |
| `extra_fields` | 额外 form data 字段。 |

只有服务端明确要求 multipart 时才切换。切换后必须先跑 `--dry-run`，确认服务端返回可解析的 `best_grasp`。

### 7.4 GraspNet 返回格式要求

客户端支持两类返回：

- 顶层或 `best_grasp` 字段是 17 维 list：`[score, width, height, depth, R(9), t(3), object_id]`。
- `best_grasp` 是 dict，至少包含旋转和位置：
  - 旋转字段可用 `rotation_matrix` / `R_cam_grasp` / `rotation` / `rotation_3x3`。
  - 平移字段可用 `translation` / `t_cam_grasp` / `center`。

其中 `translation` 是相机坐标系下的抓取中心位置，单位应为米；不要对它做 GraspNet 轴重排。轴重排只用于姿态矩阵。

## 8. `grasp`

代码位置：`src/eye_in_hand_graspnet/transforms.py::compute_tcp_target()`。

这个部分决定如何把 GraspNet 的 `best_grasp` 转成 UR TCP 的 `tcp_goal` 和 `tcp_pregrasp`。

### 8.1 位置计算

位置链路：

```text
grasp_center_base = (T_base_tcp_now @ T_tcp_cam @ T_cam_grasp).translation
tcp_goal_translation =
  grasp_center_base
  + R_base_tcp_goal @ tcp_translation_offset_m
  + target_base_offset_m
```

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `tcp_translation_offset_m` | `[float, float, float]` | `[0.0, 0.0, 0.0]` | 从 GraspNet 抓取中心到 UR TCP 原点的偏置，按最终 TCP 坐标系表达，单位米。不是法兰盘到 TCP 的偏置。夹爪 TCP 原点如果不在视觉抓取中心，需要现场测量后填写。 |
| `target_base_offset_m` | `[float, float, float]` | `[0.0, 0.0, 0.0]` | 对最终目标点的全局补偿，按机器人 base 坐标系表达，单位米。只建议用于小范围现场修正，不要用它掩盖标定错误。 |

举例：

- 如果最终 TCP 的 +Z 轴是接近方向，且 TCP 原点需要比视觉抓取中心沿 TCP +Z 多 2 cm，则可尝试 `tcp_translation_offset_m=[0, 0, 0.02]`。
- 如果所有目标在 base Z 方向都低 5 mm，可临时用 `target_base_offset_m=[0, 0, 0.005]` 验证，但后续应复核标定和深度尺度。

### 8.2 姿态计算

姿态链路：

```text
R_base_grasp = (T_base_tcp_now @ T_tcp_cam @ T_cam_grasp).rotation
R_base_grasp_as_tcp = R_base_grasp @ grasp_to_tcp_rotation_matrix

tcp_rotation_mode == "current"  -> R_base_tcp_goal = 当前 TCP 姿态
tcp_rotation_mode == "graspnet" -> R_base_tcp_goal = R_base_grasp_as_tcp @ tcp_rotation_offset_matrix
tcp_rotation_mode == "fixed"    -> R_base_tcp_goal = fixed_tcp_rotvec 转旋转矩阵
```

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `tcp_rotation_mode` | `str` | `"current"` | 最终 TCP 姿态策略。可选 `current`、`graspnet`、`fixed`。首次实机建议 `current`，先验证位置和高度。 |
| `grasp_to_tcp_rotation_matrix` | `3x3` 矩阵 | `[[0,0,1],[1,0,0],[0,1,0]]` | GraspNet 局部抓取轴到 UR TCP 局部轴的基础映射。默认含义是 `UR TCP X = GraspNet Y`、`UR TCP Y = GraspNet Z`、`UR TCP Z = GraspNet X`。 |
| `tcp_rotation_offset_matrix` | `3x3` 矩阵 | 单位阵 | 仅在 `tcp_rotation_mode="graspnet"` 时使用。用于在基础轴映射后追加现场姿态微调。必须是合法旋转矩阵。 |
| `fixed_tcp_rotvec` | `[float, float, float]` | `[0.0, 0.0, 0.0]` | 仅在 `tcp_rotation_mode="fixed"` 时使用。UR 旋转向量，单位弧度。 |

推荐顺序：

1. 首次低风险验证：`tcp_rotation_mode="current"`。
2. 位置链路确认后，若要让夹爪姿态跟随 GraspNet：切到 `tcp_rotation_mode="graspnet"`。
3. 如果跟随姿态整体有固定偏差：只微调 `tcp_rotation_offset_matrix`。
4. 如果任务要求固定姿态抓取：使用 `tcp_rotation_mode="fixed"` 并填写 `fixed_tcp_rotvec`。

### 8.3 预抓取点

预抓取点计算：

```text
approach_axis = normalize(R_base_grasp[:, approach_axis_index])
tcp_pregrasp_translation =
  tcp_goal_translation + approach_sign * pregrasp_offset_m * approach_axis
```

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `pregrasp_offset_m` | `float` | `0.08` | 预抓取点离最终抓取点的距离，单位米。当前为 8 cm。 |
| `approach_axis_index` | `int` | `0` | 使用 GraspNet 抓取姿态矩阵的第几列作为接近轴。只能是 `0`、`1`、`2`。当前 `0` 表示 GraspNet 局部 X/approach 方向。 |
| `approach_sign` | `float` | `-1.0` | 预抓取点沿接近轴的正反方向。若预抓取点跑到物体后方或更靠近物体，优先检查这个符号。 |

## 9. `motion`

代码位置：`src/eye_in_hand_graspnet/robot.py` 和 `src/eye_in_hand_graspnet/pipeline.py`。

这个部分只在 `--execute` 时真正控制机器人运动。`--dry-run` 会计算并保存目标点，但不会启动 XML-RPC 服务、不会 Dashboard load/play、不会更新示教器目标。

当前执行模式是：

1. Windows 侧启动本机 XML-RPC `get_target()` 服务。
2. Windows 侧通过 Dashboard 对 UR 控制器执行 `stop -> load <grasp_program> -> play`。
3. 示教器 URP 持续轮询 `http://<Windows_PC_IP>:<xmlrpc_port>/RPC2` 的 `get_target()`。
4. Python 按 `enabled_steps` 更新 `[x,y,z,rx,ry,rz,gripper]`。

### 9.1 XML-RPC 和 Dashboard

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `xmlrpc_host` | `str` | `"0.0.0.0"` | Windows 侧 `get_target()` 服务监听地址。通常保持 `0.0.0.0`，让示教器可通过 Windows PC IP 访问。 |
| `xmlrpc_port` | `int` | `50000` | Windows 侧 `get_target()` 服务端口。示教器 URP 中 RPC URL 必须使用同一端口。 |
| `dashboard_port` | `int` | `29999` | UR Dashboard 端口，通常是 `29999`。 |
| `grasp_program` | `str` 或 `null` | `"aaaaaa.urp"` | 示教器里要加载的 URP 文件名。该 URP 必须持续轮询 PC 的 `get_target()`。 |
| `stop_before_load` | `bool` | `true` | load 前是否先 Dashboard `stop`。建议保持 `true`，避免旧程序仍在运行。 |
| `play_after_load` | `bool` | `true` | load 后是否 Dashboard `play`。实机执行通常保持 `true`。 |
| `program_settle_s` | `float` | `2.0` | `load` 后、`play` 前等待时间，单位秒。 |
| `grasp_program_start_wait_s` | `float` | `0.5` | `play` 后等待 URP 启动的时间，单位秒。 |
| `require_grasp_program_running` | `bool` | `true` | `play` 后是否检查 Dashboard `running` 返回 true。建议保持 `true`。 |
| `require_xmlrpc_polling` | `bool` | `true` | 是否要求示教器确实访问了 PC 的 `get_target()`。建议保持 `true`。 |
| `xmlrpc_poll_check_s` | `float` | `2.0` | 检查 XML-RPC 轮询时等待的时间，单位秒。 |

如果 `require_xmlrpc_polling=true` 且示教器没有访问 PC，程序会报错。优先检查：

- 示教器 RPC URL 是否是 `http://<Windows_PC_IP>:50000/RPC2`。
- Windows 防火墙是否放行该端口。
- PC 和 UR 控制器是否在同一网络内互通。
- `xmlrpc_port` 是否和 URP 里写的一致。

### 9.2 起始观察位和执行步骤

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `fixed_start_tcp` | 长度 6 list 或 `null` | `[-0.245099380, -0.254530307, 0.482963999, -0.121633424, -0.947709943, 0.506871697]` | 固定观察位 `[x,y,z,rx,ry,rz]`。前三项米，后三项弧度旋转向量。`enabled_steps` 包含 `start` 且 `--execute` 时必须非空。 |
| `enabled_steps` | `list[str]` | `["start", "open-current", "pregrasp", "grasp", "close"]` | 实机执行步骤。允许值：`start`、`open-current`、`pregrasp`、`grasp`、`close`。 |

各步骤含义：

| step | 实际动作 |
| --- | --- |
| `start` | 把目标设置为 `fixed_start_tcp`，夹爪值为 `open_gripper`。 |
| `open-current` | 把目标设置为当前 TCP，夹爪值为 `open_gripper`。用于先张开夹爪但不移动。 |
| `pregrasp` | 把目标设置为 `tcp_pregrasp`，夹爪值为 `open_gripper`。 |
| `grasp` | 把目标设置为 `tcp_goal`，夹爪值仍为 `open_gripper`。 |
| `close` | 把目标保持在 `tcp_goal`，夹爪值改为 `close_gripper`。 |

每个启用步骤在真正下发目标之前都会先打印：

```text
[motion-state] <step>
```

其中 `<step>` 使用 `enabled_steps` 里的状态名，例如 `start`、`open-current`、`pregrasp`、`grasp`、`close`。

首次实机建议：

```json
"enabled_steps": ["start"]
```

确认安全后再改成：

```json
"enabled_steps": ["start", "open-current", "pregrasp"]
```

确认预抓取方向和高度正确后再加入 `grasp`，最后才加入 `close`。

### 9.3 等待时间和夹爪值

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `settle_s_after_start` | `float` | `0.5` | `motion_waits_s` 没有配置 `start` 时，`start` 步骤默认等待时间。当前因为配置了 `motion_waits_s.start=5.0`，实际用 5 秒。 |
| `motion_wait_s` | `float` | `3.0` | 各动作的默认等待时间，单位秒。 |
| `motion_waits_s` | `object` | 见 JSON | 按动作 label 覆盖等待时间。key 使用 `start`、`open-current`、`move-pregrasp`、`move-grasp`、`close-gripper`。 |
| `open_gripper` | `float` | `100.0` | 发送给 URP 的张开夹爪值。具体含义取决于 URP 中 `apply_gripper_target()` 的实现。 |
| `close_gripper` | `float` | `0.0` | 发送给 URP 的闭合夹爪值。首次实机不要急着启用 `close`。 |

注意：Python 侧只是给示教器 URP 提供目标数组，夹爪值如何映射到真实夹爪动作取决于示教器脚本。

## 10. `preview`

代码位置：`src/eye_in_hand_graspnet/preview.py`。

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `enabled` | `bool` | `true` | 是否显示并保存腕部预览。无显示环境时可设为 `false`。 |
| `window_name` | `str` | `"wrist GraspNet preview"` | OpenCV 窗口名。 |
| `scale` | `float` | `0.75` | 预览显示缩放比例。只影响显示，不影响 GraspNet 输入。 |
| `wait_ms` | `int` | `1` | OpenCV `waitKey` 等待时间，单位毫秒。 |
| `save_path` | `str` 或 `null` | `"outputs/last_wrist_preview.png"` | 预览图保存路径。相对路径按项目根目录解析。设为 `null` 可不保存。 |

预览图用于人工检查：

- 掩码区域是否覆盖目标。
- GraspNet 返回中心是否在物体附近。
- 小夹爪线框投影方向是否明显异常。

## 11. `outputs`

| 字段 | 类型 | 当前值 | 含义与配置方法 |
| --- | --- | --- | --- |
| `last_result_path` | `str` | `"outputs/last_wrist_grasp_result.json"` | 每次运行保存的最后一次结果。包含当前 TCP、相机序列号、掩码统计、GraspNet 原始响应、`T_base_cam_now`、`tcp_pregrasp`、`tcp_goal` 等。 |

实机前重点检查输出 JSON 中：

```text
camera.serial
mask.kept_pixels / mask.total_pixels
current_tcp
target.T_base_cam_now
target.grasp_center_base
target.R_base_grasp_as_tcp
target.tcp_pregrasp
target.tcp_goal
```

## 12. 推荐配置流程

1. 不改 motion，先运行无硬件 smoke：

```powershell
python scripts\smoke_test_no_hardware.py
```

2. 启动 GraspNet 服务，只跑 dry-run：

```powershell
python scripts\run_wrist_grasp_cycle.py --config configs\eye_in_hand_ur7e_wrist.jsonc --dry-run
```

3. 检查 `outputs/last_wrist_grasp_result.json`：

- `camera.serial` 必须是腕部相机。
- `target.T_base_cam_now` 必须存在。
- `target.grasp_center_base` 应在机器人工作空间内。
- `tcp_goal` 相对 `current_tcp` 的跳变量应合理。

4. 首次 `--execute` 前，把 `enabled_steps` 降到只包含 `start` 或只到 `pregrasp`。

5. 低速确认方向、高度、预抓取距离后，再逐步开放 `grasp` 和 `close`。

## 13. 常见错误配置

| 错误 | 后果 | 修正 |
| --- | --- | --- |
| 把固定相机序列号填到 `camera.serial` | 使用了错误视角，`T_tcp_cam` 完全不匹配。 | 改回腕部相机，并重新确认输出 `camera.serial`。 |
| 把 `T_tcp_cam` 当作 `T_base_cam` | 相机外参不会随手腕运动更新，抓取点会严重错误。 | 保持当前动态链路：`T_base_cam_now = T_base_tcp_now @ T_tcp_cam`。 |
| `enabled_steps` 首次就包含 `close` | 方向或高度未确认时可能直接闭合夹爪。 | 首次只跑 `start` 或 `pregrasp`。 |
| `fixed_start_tcp` 未现场确认 | 机器人可能先移动到不安全位置。 | 在示教器低速确认后再填写。 |
| `request_format` 与服务端不一致 | HTTP 请求失败或返回不可解析。 | 先确认服务端协议，再改 `request_format` 和相关字段。 |
| 用 `target_base_offset_m` 修很大的偏差 | 掩盖标定、深度或 TCP 定义错误。 | 大偏差先复核标定矩阵、相机深度尺度和 TCP 定义。 |
