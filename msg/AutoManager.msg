# AutoManager.msg
# 自动化系统的总体状态和控制消息

Header header          # 标准的ROS消息头部，包含时间戳和帧ID
string mode            # 自动化模式的名称（例如："exploration", "return_home"）
bool is_active         # 自动化功能是否正在运行

uint8 odometry_status  # 里程计状态
                      # 0: UNKNOWN - 未知状态
                      # 1: OK - 状态正常
                      # 2: ERROR - 状态错误

# 监控各模块中的节点执行情况
uint8 mavros_status    # 0: 未执行, 1: 不完整, 2: 完整
uint8 cam_driver_status # 0: 未执行, 1: 不完整, 2: 完整
uint8 lidar_driver_status # 0: 未执行, 1: 不完整, 2: 完整
uint8 slam_status      # 0: 未执行, 1: 不完整, 2: 完整
uint8 planner_status   # 0: 未执行, 1: 不完整, 2: 完整
uint8 ctrl_status      # 0: 未执行, 1: 不完整, 2: 完整

uint8 restart_status   # 系统重启状态
                      # 0: 无, 1: 重启中

Command command        # 命令消息，用于控制ROS节点的操作
