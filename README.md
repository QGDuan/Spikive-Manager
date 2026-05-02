# Spikive Manager (astro_manager)

每架无人机本地运行的节点生命周期管理器。通过 ROS1 Topic 接收前端指令，按依赖顺序启动/停止 SLAM、Planner、MavROS 等节点，并以 1Hz 向前端报告全部子系统状态。

## 部署

```bash
# 在飞机机载计算机上
roslaunch astro_manager AstroManager.launch drone_id:=1
```

| 参数 | 必填 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `drone_id` | 是 | — | 与前端 RobotEntry.droneId 一致 |
| `log_root` | 否 | `$HOME/spikive_logs` | 日志根目录 |
| `config` | 否 | `config/launch_config.yaml` | launch 配置文件 |

## Topic 协议

前端命令和状态 topic 带 `/drone_{id}_` 前缀，与前端 `/drone_{id}_*` 命名约定一致。机载 MAVROS 状态使用无人机内部 topic `/mavros/state`，不带 `drone_id`。

### 命令（前端 → 后端）

**Topic**: `/drone_{id}_command_topic`
**类型**: `astro_manager/Command`

```
Header header
string command_type       # "start_all" | "shutdown_all"
string[] target_launches  # 保留字段，当前 start_all/shutdown_all 忽略
float64[] parameters
string extra_data
```

| 命令 | 说明 | 前置条件 |
| --- | --- | --- |
| `start_all` | 按依赖顺序启动全部 launch | `is_active=false` 且 `armed=false` |
| `shutdown_all` | 逆序停止全部 launch | `armed=false` |

### 状态（后端 → 前端，1Hz）

**Topic**: `/drone_{id}_auto_manager_status`
**类型**: `astro_manager/AutoManager`

```
Header header
string drone_id           # 必须与 /drone_{id}_auto_manager_status 一致
string mode               # "idle" / "starting" / "ready" / "stopping" / "error"
bool is_active            # 全部 launch 就绪 → true
bool starting             # start_all 执行中
bool stopping             # shutdown_all 执行中
bool armed                # 镜像 /mavros/state.armed
string last_error         # 最近拒绝/失败原因
uint32 last_error_seq     # 单调递增（前端用于判断新错误）

uint8 mavros_status       # 0=未执行 1=不完整 2=完整
uint8 lidar_driver_status
uint8 cam_driver_status
uint8 slam_status
uint8 planner_status
uint8 ctrl_status

uint8 odometry_status
Command command
```

**前端按钮逻辑**:
- 添加卡片时先连接 Foxglove，再等待 `/drone_{id}_auto_manager_status`，只有消息 `drone_id` 与手动输入 id 一致才创建卡片
- `is_active=false` 且 `starting=false` 且 `stopping=false` → 显示 **Start**
- `is_active=true` 且 `starting=false` 且 `stopping=false` → 显示 **Stop**
- `starting=true` 或 `mode="starting"` → 按钮禁用 (spinner)
- `stopping=true` 或 `mode="stopping"` → 按钮禁用 (spinner)
- `armed=true` → 按钮红色禁用（飞行中不可操作）
- 状态灯只显示 Drivers: MavROS/Lidar，Tasks: SLAM/Planner
- status 超过 4s 无更新 → 显示 Manager 离线

## 飞行联锁

后端强制订阅机载 `/mavros/state`，缓存 `connected/armed/guided/manual_input/mode/system_status/header.stamp` 和本机接收时间用于日志。这个 topic 不是可配置项；Stop 空中联锁必须始终打开。前端不直接订阅 `/mavros/state`，也不直接用 `AutoManager.armed` 禁用 Start/Stop；点击后由后端用最新 MAVROS 缓存做权威校验。

MAVROS 状态日志在首次收到、关键字段变化、`start_all` 完成、命令被 armed 联锁拒绝时打印。`header.stamp` 每帧都会变化，不作为“变化触发日志”的字段，避免按 `/mavros/state` 频率刷屏。`start_all` 全部 ready 后，日志会明确输出当前 Stop 是否允许：

- `shutdown_all`：必须有 3 秒内新鲜 `/mavros/state` 且 `armed=false` 才执行；missing/stale/armed 都拒绝并写日志
- `start_all`：如果有 3 秒内新鲜 `/mavros/state` 且 `armed=true` 则拒绝；如果 MAVROS 状态 missing/stale，允许启动并写日志说明跳过 armed 预检

## 日志目录

```
~/spikive_logs/drone_{id}/
├── manager.log           # manager 自身事件（不走 rosout）
├── drivers/
│   ├── Lidar_Driver.log
│   └── MavRos.log
└── algo/
    ├── LIO.log
    ├── Planner.log
    └── WaypointRecorder.log
```

manager 使用 Python `logging`（`propagate=False`），不进 rosout，不被 SLAM/Planner 刷屏。

## 启动顺序

```
Lidar_Driver → LIO → Planner
MavRos → WaypointRecorder
```

由 `launch_config.yaml` 中的 `depends_on` 声明，Kahn 拓扑排序。每个 launch 启动后轮询 `rosnode list` 直到声明的节点全部出现，超时则中止。

## 测试示例

```bash
# 启动全部
rostopic pub /drone_1_command_topic astro_manager/Command \
  "{header: auto, command_type: 'start_all', target_launches: [], parameters: [], extra_data: ''}"

# 停止全部
rostopic pub /drone_1_command_topic astro_manager/Command \
  "{header: auto, command_type: 'shutdown_all', target_launches: [], parameters: [], extra_data: ''}"

# 查看状态
rostopic echo /drone_1_auto_manager_status -n 1

# 模拟 armed（测试联锁）
rostopic pub -r 5 /mavros/state mavros_msgs/State '{armed: true}'
```
