cmake_minimum_required(VERSION 3.8)
project(false_sensors)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS
    src/fake_lidar.py
    src/fake_camera.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
