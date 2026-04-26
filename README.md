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

所有 topic 带 `/drone_{id}_` 前缀，与前端 `/drone_{id}_*` 命名约定一致。

### 命令（前端 → 后端）

**Topic**: `/drone_{id}_command_topic`
**类型**: `astro_manager/Command`

```
Header header
string command_type       # "start_all" | "shutdown_all"
string[] target_launches  # 仅 start_node/shutdown_node 使用
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
string mode               # "idle" / "starting" / "ready" / "stopping" / "error"
bool is_active            # 全部 launch 就绪 → true
bool starting             # start_all 执行中
bool armed                # 镜像 mavros/state.armed
string last_error         # 最近拒绝/失败原因
uint32 last_error_seq     # 单调递增（前端用于判断新错误）

uint8 mavros_status       # 0=未执行 1=不完整 2=完整
uint8 lidar_driver_status
uint8 cam_driver_status
uint8 slam_status
uint8 planner_status
uint8 ctrl_status

uint8 odometry_status
uint8 restart_status
Command command
```

**前端按钮逻辑**:
- `is_active=false` 且 `starting=false` → 显示 **Start**
- `is_active=true` 且 `starting=false` → 显示 **Stop**
- `starting=true` → 按钮禁用 (spinner)
- `armed=true` → 按钮红色禁用（飞行中不可操作）
- status 超过 4s 无更新 → 显示 Manager 离线

## 飞行联锁

后端订阅 `/drone_{id}_mavros/state`，armed=true 时**拒绝**所有 start/shutdown 命令。前端按钮也做镜像禁用（双保险），但权威判断在后端。

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
rostopic pub -r 5 /drone_1_mavros/state mavros_msgs/State '{armed: true}'
```
