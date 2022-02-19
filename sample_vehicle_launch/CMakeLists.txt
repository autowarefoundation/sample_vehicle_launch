cmake_minimum_required(VERSION 3.5)
project(lexus_launch)

add_compile_options(-std=c++17)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
