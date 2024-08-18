# Set the minimum required CMake version and define the project
cmake_minimum_required(VERSION 3.20)
project(view_planning)

# Find necessary Catkin packages for ROS
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  rospy
  sensor_msgs
  message_generation
  dynamic_reconfigure
  rviz_visual_tools
)

# Find external libraries like PCL, VTK, CGAL, OctoMap, mlpack, and CUDA
find_package(PCL 1.8 REQUIRED)
find_package(VTK REQUIRED)
find_package(CGAL REQUIRED)
find_package(octomap REQUIRED)
find_package(mlpack)
find_package(CUDA REQUIRED)
find_package(Open3D REQUIRED)

# Include directories for header files
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${mlpack_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  include
  include/visioncraft  # Add this line to include your library headers
  ${catkin_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}  # Include rviz_visual_tools headers
)

include(${VTK_USE_FILE})

# Specify Catkin package details like include directories and dependencies
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_conversions pcl_ros sensor_msgs message_runtime roscpp rviz_visual_tools
)

# Define an executable and link libraries for the executable
add_executable(model_loader_tests tests/model_loader_tests.cpp src/visioncraft/model_loader.cpp src/visioncraft/utils/poisson_sampler.cpp)
target_link_libraries(model_loader_tests ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OCTOMAP_LIBRARIES} Open3D::Open3D ${CUDA_LIBRARIES})
add_dependencies(model_loader_tests ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(viewpoint_tests tests/viewpoint_tests.cpp src/visioncraft/viewpoint.cpp)
target_link_libraries(viewpoint_tests ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OCTOMAP_LIBRARIES} Open3D::Open3D ${CUDA_LIBRARIES})
add_dependencies(viewpoint_tests ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Define an executable and link libraries for the executable
add_executable(visualizer_tests tests/visualizer_tests.cpp src/visioncraft/visualizer.cpp src/visioncraft/viewpoint.cpp src/visioncraft/model_loader.cpp src/visioncraft/my_cuda_code.cu)
target_link_libraries(visualizer_tests ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OCTOMAP_LIBRARIES} Open3D::Open3D ${CUDA_LIBRARIES})
add_dependencies(visualizer_tests ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rviz_visualizer_tests tests/rviz_visualizer_tests.cpp src/visioncraft/rviz_visualizer.cpp src/visioncraft/viewpoint.cpp src/visioncraft/model_loader.cpp)
target_link_libraries(rviz_visualizer_tests ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OCTOMAP_LIBRARIES} Open3D::Open3D ${CUDA_LIBRARIES} ${rviz_visual_tools_LIBRARIES})  # Link rviz_visual_tools
add_dependencies(rviz_visualizer_tests ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

CUDA_ADD_LIBRARY(CudaTesterLib STATIC src/visioncraft/my_cuda_code.cu)
add_executable(cuda_tester tests/cuda_tester.cpp)
target_link_libraries(cuda_tester ${CUDA_LIBRARIES} CudaTesterLib)
add_dependencies(cuda_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(exploration_view_tests tests/exploration_view_tests.cpp src/visioncraft/model_loader.cpp src/visioncraft/viewpoint.cpp src/visioncraft/rviz_visualizer.cpp)
target_link_libraries(exploration_view_tests ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} Open3D::Open3D ${CUDA_LIBRARIES} ${VTK_LIBRARIES} )
add_dependencies(exploration_view_tests ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})