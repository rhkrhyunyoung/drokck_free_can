cmake_minimum_required(VERSION 3.8)
project(drokck)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# 1. 필요한 외부 패키지 로드
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# -----------------------------------------------------------------
# 2-1. [1~4구간 노드] 실행 파일 및 빌드 소스 지정
# -----------------------------------------------------------------
add_executable(track_1_4_node
  track_1_4_node.cpp       # 변경된 파일 이름 적용
  RMD_command.cpp
  kinematics.cpp
)

target_include_directories(track_1_4_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(track_1_4_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(track_1_4_node
  yaml-cpp
)

# -----------------------------------------------------------------
# 2-2. [5구간 노드] 실행 파일 및 빌드 소스 지정
# -----------------------------------------------------------------
add_executable(track_5_node
  track_5_node.cpp         # 변경된 파일 이름 적용
  RMD_command.cpp          # 모터 구동은 동일하게 필요하므로 똑같이 링크
  kinematics.cpp
)

target_include_directories(track_5_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(track_5_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(track_5_node
  yaml-cpp
)

# -----------------------------------------------------------------
# 3. ROS2 빌드 후 시스템에 실행 파일 등록 및 설치 경로 지정
# -----------------------------------------------------------------
install(TARGETS
  track_1_4_node
  track_5_node
  DESTINATION lib/${PROJECT_NAME}
)

# 인클루드 디렉토리 설치 설정
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
