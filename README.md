# 腕部相机 Eye-in-Hand GraspNet UR7e 复现

本目录是独立复现工程，不依赖 `reproduction/graspnet-baseline-in-ur7e` 中的代码。

目标链路：

1. UR7e 先移动到固定观察点位。
2. 只启动腕部 RealSense 相机，默认序列号 `419122270341`。
3. 对 RGB-D 输入应用中心工作区掩码：只保留画面中心、宽高各为整图二分之一的矩形。
4. 调用 GraspNet-baseline 推理服务，当前默认使用 `http://127.0.0.1:18080/infer` 的 JSON `.npy_base64` 协议，得到相机坐标系下的 `best_grasp`。
5. 读取当前夹爪 TCP 的 RTDE 位姿，结合眼在手上标定矩阵 `T_tcp_cam`，把抓取中心转换到机器人 `base` 坐标系。
6. 输出 `tcp_pregrasp` 和 `tcp_goal`；确认无误后可通过 XML-RPC 执行移动。

## 目录结构

```text
configs/
  eye_in_hand_ur7e_wrist.json        # 主配置
  eye_in_hand_calibration_tcp.json   # 本项目内的 T_tcp_cam 快照
docs/
  0_项目交接.md
  1_转换graspnet输出的具体过程说明.md
  2_转换的注意事项.md
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
  robot.py                           # RTDE 读取和 XML-RPC 移动
  transforms.py                      # 坐标变换与 TCP 目标计算
```

## 最小验证

无硬件环境先运行：

```powershell
python scripts\smoke_test_no_hardware.py
```

只检查配置和坐标变换，不连接相机、GraspNet 服务或机器人。

## 干跑计算

连接腕部相机、GraspNet 服务和 RTDE 后，先干跑：

```powershell
python scripts\run_wrist_grasp_cycle.py --config configs\eye_in_hand_ur7e_wrist.json --dry-run
```

干跑会拍摄腕部 RGB-D、显示腕部预览、请求 GraspNet、读取当前 TCP，并打印：

- `T_base_tcp_now`
- `T_base_cam_now`
- `grasp_center_base`
- `tcp_pregrasp`
- `tcp_goal`

## 实机移动

确认目标点位和运动方向正确后，再显式执行：

```powershell
python scripts\run_wrist_grasp_cycle.py --config configs\eye_in_hand_ur7e_wrist.json --execute
```

执行前必须先在 `configs\eye_in_hand_ur7e_wrist.json` 填写现场确认过的 `motion.fixed_start_tcp`。当前配置保留为 `null`，程序会拒绝 `--execute`，避免没有固定观察点位时直接运动。

`--execute` 会按配置调用 XML-RPC 方法移动 TCP。首次实机建议先把 `motion.enabled_steps` 设为只包含 `start` 或 `pregrasp`，确认方向后再加入 `grasp`。
