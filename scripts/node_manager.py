#!/usr/bin/env python

import rospy
import subprocess
import yaml
import os
import rospkg
from astro_manager.msg import AutoManager, Command


class NodeManager:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('node_manager', anonymous=True)

        # 获取ROS包路径
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('astro_manager')

        # 加载外部配置文件
        config_path = rospy.get_param('~config_path', 'config/launch_config.yaml')
        full_config_path = os.path.join(package_path, config_path)
        self.load_config(full_config_path)

        # 订阅 Command 话题
        rospy.Subscriber('/command_topic', Command, self.command_callback)

        # 创建发布 AutoManager 消息的发布者
        self.auto_manager_pub = rospy.Publisher('/auto_manager_status', AutoManager, queue_size=10)

        # 启动一个定时器，每秒执行一次 update_status 方法
        self.timer = rospy.Timer(rospy.Duration(1), self.update_status)

        # 打印初始化完成信息
        rospy.loginfo("Node Manager is running...")

        # 初始化AutoManager消息
        self.auto_manager_msg = AutoManager()
        self.auto_manager_msg.mode = "idle"
        self.auto_manager_msg.is_active = False
        self.auto_manager_msg.restart_status = 0

    def load_config(self, config_path):
        # 确保配置文件存在并加载
        if not os.path.exists(config_path):
            rospy.logerr(f"Config file not found: {config_path}")
            rospy.signal_shutdown("No valid config file found.")
            return

        with open(config_path, 'r') as config_file:
            config_data = yaml.safe_load(config_file)
            self.launch_to_nodes_map = config_data.get('launches', {})
            rospy.loginfo(f"Loaded configuration from {config_path}")

    def command_callback(self, msg):
        for launch_name in msg.target_launches:
            if launch_name not in self.launch_to_nodes_map:
                rospy.logerr(f"Launch '{launch_name}' not found in configuration.")
                continue

            if msg.command_type in ["start_node", "restart_node"]:
                rospy.loginfo(f"Starting or restarting launch '{launch_name}'")
                self.start_or_restart_launch(launch_name)
            elif msg.command_type == "shutdown_node":
                if self.is_any_node_running(launch_name):
                    rospy.logwarn(f"Shutting down launch '{launch_name}'")
                    self.shutdown_launch(launch_name)
                else:
                    rospy.logerr(f"Error: Cannot shutdown '{launch_name}' because its nodes are not running.")
            else:
                rospy.loginfo("Received command, but it's not a recognized command.")

    def start_or_restart_launch(self, launch_name):
        # 检查并智能处理节点
        if self.is_any_node_running(launch_name):
            rospy.logwarn(f"Nodes for launch '{launch_name}' already running, shutting down first.")
            self.shutdown_launch(launch_name)

        # 获取启动命令
        launch_cmd = self.launch_to_nodes_map.get(launch_name, {}).get('launch_cmd')

        if not launch_cmd:
            rospy.logerr(f"No launch command found for '{launch_name}'.")
            return

        try:
            # 执行启动命令
            subprocess.Popen(launch_cmd, shell=True)
            rospy.loginfo(f"Launch '{launch_name}' started successfully.")  # 绿色标识
            rospy.sleep(1)  # 启动下一个launch文件前等待3秒
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to start launch '{launch_name}': {e}")

    def shutdown_launch(self, launch_name):
        # 获取需要关闭的节点列表
        nodes_to_kill = self.launch_to_nodes_map.get(launch_name, {}).get('nodes', [])

        if not nodes_to_kill:
            rospy.logwarn(f"No nodes found for launch '{launch_name}'.")
            return

        # 关闭所有节点
        for node in nodes_to_kill:
            try:
                stop_cmd = f"rosnode kill {node}"
                subprocess.check_call(stop_cmd, shell=True)
                rospy.loginfo(f"Node '{node}' stopped successfully.")
            except subprocess.CalledProcessError as e:
                rospy.logerr(f"Failed to stop node '{node}': {e}")

    def is_any_node_running(self, launch_name):
        # 检查该launch文件关联的节点是否有正在运行的
        nodes = self.launch_to_nodes_map.get(launch_name, {}).get('nodes', [])
        if not nodes:
            rospy.logerr(f"No nodes found for launch '{launch_name}'.")
            return False

        running_nodes = subprocess.check_output(['rosnode', 'list']).decode('utf-8').split('\n')
        return any(node in running_nodes for node in nodes)

    def check_node_status(self, nodes):
        # 检查节点状态，返回是否完整，不完整，或未执行
        running_nodes = subprocess.check_output(['rosnode', 'list']).decode('utf-8').split('\n')
        if all(node in running_nodes for node in nodes):
            return 2  # 完整
        elif any(node in running_nodes for node in nodes):
            return 1  # 不完整
        else:
            return 0  # 未执行

    def update_status(self, event):
        # 检查各个模块的状态并更新消息
        self.auto_manager_msg.mavros_status = self.check_node_status(self.get_nodes_for_launch('MavRos'))
        self.auto_manager_msg.slam_status = self.check_node_status(self.get_nodes_for_launch('LIO'))
        self.auto_manager_msg.planner_status = self.check_node_status(self.get_nodes_for_launch('Planner'))
        self.auto_manager_msg.cam_driver_status = self.check_node_status(self.get_nodes_for_launch('Camera_Diver'))
        self.auto_manager_msg.lidar_driver_status = self.check_node_status(self.get_nodes_for_launch('Lidar_Driver'))
        self.auto_manager_msg.ctrl_status = self.check_node_status(self.get_nodes_for_launch('Ctrl'))

        # 发布 AutoManager 消息
        self.auto_manager_pub.publish(self.auto_manager_msg)

    def get_nodes_for_launch(self, launch_name):
        return self.launch_to_nodes_map.get(launch_name, {}).get('nodes', [])


if __name__ == '__main__':
    try:
        NodeManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
