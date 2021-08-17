cmake_minimum_required(VERSION 3.5)

project(p3dxmsgs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
		"srv/State.srv"
)
		
install(DIRECTORY srv
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
