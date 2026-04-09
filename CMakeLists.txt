cmake_minimum_required(VERSION 3.0.2)
project(astro_manager)

## 找到catkin，并引入相关组件
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## 添加生成的消息文件
add_message_files(
  FILES
  AutoManager.msg
  Command.msg
)

## 生成消息目标
generate_messages(
  DEPENDENCIES
  std_msgs
)

## 声明catkin的包
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

## 将Python脚本安装到ROS包的bin目录下
catkin_install_python(PROGRAMS
  scripts/node_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 如果需要添加C++源文件的编译，请在这里进行相关配置
# add_executable(${PROJECT_NAME}_node src/your_ros_package_node.cpp)

## 将库和目标链接到catkin的包
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
