# Command.msg
# 用于发送给 astro_manager 节点的命令

Header header             # 标准的 ROS 消息头部
string command_type       # 指令类型，可选值：
                          #   "start_all"     - 按依赖顺序启动全部 launch
                          #   "shutdown_all"  - 停止全部 launch
string[] target_launches  # 保留字段，当前 start_all/shutdown_all 忽略
float64[] parameters      # 可选命令参数
string extra_data         # 额外指令数据
