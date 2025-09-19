# CMake 최소 버전 설정
cmake_minimum_required(VERSION 3.0.2)
project(waypoint_follower)

# ROS 패키지 의존성 찾기
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# 빌드 설정
catkin_package()

# 실행 파일 생성
add_executable(dual_gps_pure_pursuit_node src/dual_gps_pure_pursuit_node.cpp)

# 실행 파일에 라이브러리 연결
target_link_libraries(dual_gps_pure_pursuit_node
  ${catkin_LIBRARIES}
)