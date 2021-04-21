cmake_minimum_required(VERSION 3.5)

project(controller)

if(NOT CMAKE_CXX_STANDARD)

    set(CMAKE_CXX_STANDARD 17)

endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

    add_compile_options(-Wall -Wextra -Wpedantic)

endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
    rclcpp
    geometry_msgs
)

include_directories(include)

add_executable(controller src/main.cpp)

add_library(controller_core SHARED
    src/controller.cpp
)

ament_target_dependencies(controller ${dependencies})
target_include_directories(controller
    PUBLIC include
)
target_link_libraries(controller controller_core)

ament_target_dependencies(controller_core ${dependencies})
target_include_directories(controller_core
    PUBLIC include
)
target_link_libraries(controller_core)

install(TARGETS controller_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(TARGETS controller
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

endif()

ament_export_include_directories(include)
ament_export_libraries(controller_core)
ament_export_dependencies(${dependencies})

ament_package()
