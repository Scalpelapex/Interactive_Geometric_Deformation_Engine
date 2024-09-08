# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required (VERSION 3.8)

project ("EDMap_DeformationController")

# 包含子项目。
add_subdirectory ("DeformationController")
add_subdirectory ("deps/polyscope")
add_subdirectory ("deps/eigen")
add_subdirectory ("deps/utils")
add_subdirectory ("deps/yaml-cpp")
#add_subdirectory ("deps/libGraphCpp")

