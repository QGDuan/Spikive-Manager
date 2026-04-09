start launch文件：
rostopic pub /command_topic astro_manager/Command "{header: auto, command_type: 'start_node', target_launches: ['MavRos'], parameters: [], extra_data: ''}"

shutdown launch 文件
rostopic pub /command_topic astro_manager/Command "{header: auto, command_type: 'shutdown_node', target_launches: ['MavRos'], parameters: [], extra_data: ''}"
