cmake_minimum_required(VERSION 3.16)
project(xarm_motion)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(xarm_sdk REQUIRED)
find_package(xarm_description REQUIRED)
find_program(XACRO_EXECUTABLE xacro REQUIRED)

# 공식 모델은 읽기만 하며 생성물은 제어 PC 빌드 폴더에 둔다.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/models")
foreach(model IN ITEMS false true)
  execute_process(
    COMMAND "${XACRO_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/urdf/xarm7.urdf.xacro"
      "model1300:=${model}" -o "${CMAKE_CURRENT_BINARY_DIR}/models/xarm7_${model}.urdf"
    RESULT_VARIABLE result ERROR_VARIABLE error)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "공식 xArm 모델 생성 실패: ${error}")
  endif()
endforeach()

add_library(motion_core SHARED src/motion_core.cpp)
target_include_directories(motion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(motion_core PUBLIC Eigen3::Eigen yaml-cpp)
ament_target_dependencies(motion_core PUBLIC orocos_kdl kdl_parser urdf)

add_library(xarm_backend SHARED src/xarm_backend.cpp)
target_link_libraries(xarm_backend PUBLIC motion_core)
target_include_directories(xarm_backend SYSTEM PUBLIC ${xarm_sdk_INCLUDE_DIRS})
ament_target_dependencies(xarm_backend PUBLIC xarm_sdk)

add_executable(xarm_motion_node src/motion_node.cpp)
set_target_properties(xarm_motion_node PROPERTIES OUTPUT_NAME xarm_motion)
target_link_libraries(xarm_motion_node motion_core xarm_backend)
ament_target_dependencies(xarm_motion_node ament_index_cpp rclcpp sensor_msgs geometry_msgs std_msgs)
# 로컬 공통 설정 경로를 고정하며 CLI·ROS·환경 변수 재정의를 제공하지 않는다.
get_filename_component(CONTROL_PC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." REALPATH)
target_compile_definitions(xarm_motion_node PRIVATE XARM_CONTROL_PC_DIR="${CONTROL_PC_DIR}")

install(TARGETS motion_core xarm_backend EXPORT xarm_motion_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS xarm_motion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/models/" DESTINATION share/${PROJECT_NAME}/models)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(xarm_motion_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp orocos_kdl kdl_parser urdf xarm_sdk)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motion test/test_motion.cpp)
  target_link_libraries(test_motion motion_core)
  target_compile_definitions(test_motion PRIVATE
    TEST_CONTROL_PC_DIR="${CONTROL_PC_DIR}"
    TEST_MODEL="${CMAKE_CURRENT_BINARY_DIR}/models/xarm7_false.urdf")
endif()
ament_package()
