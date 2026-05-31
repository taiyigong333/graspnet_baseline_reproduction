# 腕部相机 Eye-in-Hand GraspNet UR7e 复现

本目录是独立复现工程，不依赖 `reproduction/graspnet-baseline-in-ur7e` 中的代码。

目标链路：

1. UR7e 先移动到固定观察点位。
2. 只启动腕部 RealSense 相机，默认序列号 `419122270341`。
3. 对 RGB-D 输入应用中心工作区掩码：只保留画面中心、宽高各为整图二分之一的矩形。
4. 调用 GraspNet-baseline 推理服务，当前默认使用 `http://127.0.0.1:18080/infer` 的 JSON `.npy_base64` 协议，得到相机坐标系下的 `best_grasp`。
5. 读取当前夹爪 TCP 的 RTDE 位姿，结合眼在手上标定矩阵 `T_tcp_cam`，把抓取中心转换到机器人 `base` 坐标系，并把 GraspNet 局部轴重排为 UR TCP 轴。
6. 输出 `tcp_pregrasp` 和 `tcp_goal`；确认无误后由 Windows 侧 XML-RPC `get_target()` 服务给示教器 URP 轮询执行。

## 目录结构

```text
configs/
  eye_in_hand_ur7e_wrist.jsonc        # 主配置
  eye_in_hand_calibration_tcp.jsonc   # 本项目内的 T_tcp_cam 快照
docs/
  0_项目交接.md
  1_转换graspnet输出的具体过程说明.md
  2_转换的注意事项.md
  3_GraspNet局部轴到UR_TCP轴的转换说明.md
  4_类似issue65的坐标旋转问题排查与修正.md
scripts/
  run_wrist_grasp_cycle.py           # 主运行入口
  smoke_test_no_hardware.py          # 无硬件烟测
src/eye_in_hand_graspnet/
  camera.py                          # 腕部 RealSense RGB-D 采集
  config.py                          # 配置加载与默认值
  graspnet_client.py                 # GraspNet HTTP 客户端
  mask.py                            # 中心矩形工作区掩码
  pipeline.py                        # 抓取流程编排
  preview.py                         # 腕部视角预览
  robot.py                           # RTDE 读取、XML-RPC 目标服务和 Dashboard 控制
  transforms.py                      # 坐标变换、轴系映射与 TCP 目标计算
templates/
  ur/aaaaaa_rpc_template.script      # 示教器侧轮询 get_target() 的 URScript 模板
```

## 最小验证

无硬件环境先运行：

```powershell
python scripts\smoke_test_no_hardware.py
```

只检查配置、坐标变换和本机 `get_target()` XML-RPC 协议，不连接相机、GraspNet 服务或机器人。

## 干跑计算

连接腕部相机、GraspNet 服务和 RTDE 后，先干跑：

```powershell
python scripts\run_wrist_grasp_cycle.py --config configs\eye_in_hand_ur7e_wrist.jsonc --dry-run
```

干跑会拍摄腕部 RGB-D、显示腕部预览、请求 GraspNet、读取当前 TCP，并打印：

- `T_base_tcp_now`
- `T_base_cam_now`
- `grasp_center_base`
- `R_base_grasp_as_tcp`
- `tcp_pregrasp`
- `tcp_goal`

## 实机移动

确认目标点位和运动方向正确后，再显式执行：

```powershell
python scripts\run_wrist_grasp_cycle.py --config configs\eye_in_hand_ur7e_wrist.jsonc --execute
```

执行前必须确认：

- 示教器中已有 `motion.grasp_program` 指定的 URP，默认 `aaaaaa.urp`。
- 该 URP 使用 `templates\ur\aaaaaa_rpc_template.script` 同类逻辑，持续轮询 `http://<Windows_PC_IP>:50000/RPC2` 的 `get_target()`。
- `motion.xmlrpc_host/xmlrpc_port` 与示教器 RPC URL 一致；通常 Windows 侧监听 `0.0.0.0:50000`。
- `motion.fixed_start_tcp` 是现场低速确认过的固定观察点。

`--execute` 会启动 Windows 侧 `get_target()` 服务，通过 Dashboard `stop -> load -> play` 启动示教器 URP，然后按 `motion.enabled_steps` 更新目标。首次实机建议只保留 `start` 或 `pregrasp`，确认方向后再加入 `grasp`，最后才开放 `close`。
