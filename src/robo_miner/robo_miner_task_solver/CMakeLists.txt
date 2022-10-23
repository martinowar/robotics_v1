cmake_minimum_required(VERSION 3.5)
project(robo_miner_task_solver)

set(_INC_FOLDER_NAME include)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_miner_interfaces REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/RoboMinerTaskSolver.cpp
)

target_include_directories(
    ${PROJECT_NAME} 
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp robo_miner_interfaces)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
