# AutoManager.msg
# 自动化系统的总体状态和控制消息

Header header          # 标准的ROS消息头部，包含时间戳和帧ID
string drone_id        # 当前 astro_manager 管理的无人机编号，必须与 topic /drone_{id}_... 一致
string mode            # 自动化模式名称（"idle" / "starting" / "ready" / "stopping" / "error"）
bool is_active         # 后端权威：所有 launch 全就绪后置 true
bool starting          # 后端权威：start_all 进行中
bool stopping          # 后端权威：shutdown_all 进行中
bool armed             # 镜像 /drone_{id}_mavros/state.armed
string last_error      # 最近一次拒绝/失败原因（前端 toast 直接读这个）
uint32 last_error_seq  # 单调递增；前端用它判断"是不是新错误"，避免重复 toast

uint8 odometry_status  # 里程计状态
                       # 0: UNKNOWN - 未知状态
                       # 1: OK - 状态正常
                       # 2: ERROR - 状态错误

# 监控各模块中的节点执行情况
# 0: 未执行, 1: 不完整（部分节点在跑）, 2: 完整（所有节点都在跑）
uint8 mavros_status
uint8 cam_driver_status
uint8 lidar_driver_status
uint8 slam_status
uint8 planner_status
uint8 ctrl_status

Command command        # 命令消息，最近一次接收到的命令
