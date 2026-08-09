# xgc2-b2-link 子产品：APT 正式运行，源码只做合同测试

## 定位

| 项 | 内容 |
|----|------|
| 产品 ID | `xgc2-b2-link` |
| 形态 | **独立 XGC2 子产品**（与 `xgc2-ros-image-rtp-adapter` 同类） |
| 不是 | XGC2 Core/Agent 本体；不是现场 B2/Odin 驱动改写 |
| 包内容 | 共享契约 + 机载 forwarder + LAB ROS2 sim + 合同测试 peer |

生产 G4 已归属 Noetic `xgc_unitree_b2_ros1_adapter`；本包的 Python peer
不作为生产地面入口，避免出现第二个 wire consumer。

## 正式部署边界

```text
        Thor（机载）                    地面 Core 主机
   APT: xgc2-b2-link               APT: B2 ROS1 Adapter
   d0 / 可选 d17 forwarder       唯一 wire consumer + ROS/TF
         │  Zenoh/TCP                       │
         └──────────── 同一冻结合同 ────────────┘
```

| 端 | 推荐安装 | 启动 |
|----|----------|------|
| **Thor 机载** | `sudo apt install ros-jazzy-xgc2-b2-link`（noble, arm64/amd64） | `ros2 run xgc2_b2_link b2_forwarder_d0 -- --robot-id …` |
| **地面生产** | `ros-noetic-xgc2-unitree-b2-adapter` | Adapter Runtime supervisor 启动 |
| **合同测试** | 当前源码树，不进入 Automation/process catalog | `PYTHONPATH=. python3 -m xgc2_b2_link.ground_peer …`，仅验证 key/payload/TCP |

### 为何先只钉 Jazzy APT？

- 机载 Thor = **Jazzy 真源**，forwarder 依赖 ROS2 消息序列化。
- 地面 Core 是 **Focal/Noetic**，只能安装正式 B2 ROS1 Adapter；不得从本包启动 Python ROS 恢复 peer。
- LAB 与 FIELD 的产品运行都只从 APT 和标准 process definition 发现；源码 `PYTHONPATH` 只允许执行合同测试。

## 开发阶段（源码合同测试）

```bash
cd xgc2-devops/products/ros2/driver/xgc2_b2_link
export PYTHONPATH=$PWD
# 契约单测 + TCP 环回
python3 -m pytest test/ -q
```

改 key/话题必须同步冻结 `contract/zenoh_v1.yaml` 与生产 G4 Adapter 测试；不能以本包 Python peer 作为产品闭环。

## APT 发布后（Thor）

```bash
sudo apt update
sudo apt install ros-jazzy-xgc2-b2-link
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix xgc2_b2_link
ros2 run xgc2_b2_link b2_forwarder_d0 -- \
  --config /path/to/forwarder_d0.yaml
```

Zenoh 仅在显式选择时加载；缺失即失败且不回退 TCP。正式 Agent/Thor
禁止运行时 `pip install`，FIELD 启用 Zenoh 前必须由发行列车提供并安装
受控的 Python Zenoh APT 产物。该产物未就位时只允许显式 TCP 验收，不能
把源码 overlay 或容器可写层中的 Python 模块算作产品依赖。

## 与「连接器 / Adapter」的关系

| 层次 | 是否进本 APT |
|------|----------------|
| Zenoh/TCP 契约 + d0/d17 forwarder | **是（本产品）** |
| Python ground peer | **仅源码合同测试**；无安装态 console entry、无 ROS 恢复开关 |
| 仿真 sim_publisher + viz 脚本 | **是（开发/联调）** |
| XGC2 Adapter Runtime → Core projection | **否**；由 Noetic B2 Adapter 产品负责 |
| 现场 b2_ros2_driver / R5 / Odin | **否**（已有现场包） |

## 合规

```bash
.xgc2/scripts/check_package_compliance.sh
```

## 发版列车（对齐 image-rtp-adapter）

1. monorepo 本目录稳定
2. 公开子仓 `lxk36/xgc2-b2-link`（`product.yml` release.repository）
3. CI matrix：noble × amd64/arm64
4. APT 发布 → Thor `apt install`
5. Agent 从 `/opt/ros/jazzy` 运行；地面只安装正式 Noetic B2 Adapter

公开安装门检查本包精确版本，并在没有现场 driver/arm 消息包的干净 Noble
容器中完成真实 APT 安装。LAB 只使用标准 ROS 消息；FIELD 如存在
`ros-jazzy-b2-ros2-driver`，d0 forwarder 会额外消费只读 LowState，d17 则由
`ros-jazzy-arx5-arm-msg` 能力门控制。两者都是可选 source capability，不能
阻止 B2 本体包安装。Zenoh Python 运行库同样必须来自受控发行产物，不能由
Automation 临时下载。

## 产品边界铁律（继承面板）

- 机载 foxglove **不**作跨机主路径
- 无命令入口、无 `cmd_vel`、无 `down/cmd` 消费
- 地面 ROS 恢复只属于生产 B2 Adapter；本包 peer 仅合同测试
