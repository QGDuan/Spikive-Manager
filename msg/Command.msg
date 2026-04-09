# Command.msg
# 用于发送给ROS节点的命令

Header header          # 标准的ROS消息头部，包含时间戳和帧ID
string command_type    # 指令类型，例如："restart_node", "start_node"
                      # 可选值:
                      # "restart_node" - 重启指定的ROS节点
string[] target_launches  # 目标launch文件的名称列表
float64[] parameters   # 可选的命令参数，根据需要定义
string extra_data      # 额外的指令数据，可以根据需要扩展
