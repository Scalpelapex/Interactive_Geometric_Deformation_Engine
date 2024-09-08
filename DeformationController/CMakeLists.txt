# CMakeList.txt: DeformationController 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

# 将源代码添加到此项目的可执行文件。
include_directories(include)

aux_source_directory (. DIR_SRCS)
add_executable(DeformationController ${DIR_SRCS})  #制定生成目标

target_link_libraries(DeformationController polyscope)
target_link_libraries(DeformationController eigen)
target_link_libraries(DeformationController utils)
target_link_libraries(DeformationController yaml-cpp)
# target_link_libraries(DeformationController libGraphCpp)
# TODO: 如有需要，请添加测试并安装目标。
