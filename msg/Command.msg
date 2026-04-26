# Command.msg
# 用于发送给 astro_manager 节点的命令

Header header             # 标准的 ROS 消息头部
string command_type       # 指令类型，可选值：
                          #   新（推荐）:
                          #     "start_all"     - 按依赖顺序启动全部六个 launch
                          #     "restart_all"   - 先 stop 再 start，全部六个 launch
                          #     "shutdown_all"  - 停止全部六个 launch
                          #   旧（兼容/排错用）:
                          #     "start_node"    - 启动 target_launches 中指定的 launch
                          #     "restart_node"  - 重启指定 launch
                          #     "shutdown_node" - 关闭指定 launch
string[] target_launches  # 目标 launch 文件名列表（仅 *_node 命令使用，*_all 忽略）
float64[] parameters      # 可选命令参数
string extra_data         # 额外指令数据
