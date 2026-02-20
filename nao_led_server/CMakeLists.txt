cmake_minimum_required(VERSION 3.8)
project(nao_led_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nao_led_interfaces REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)


# Include Cpp "include" directory
include_directories(include)


#################### LED SERVER ###########################
add_library(led_server SHARED
  src/led_action_server.cpp)

target_include_directories(led_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(led_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")

rclcpp_components_register_node(led_server
  PLUGIN "nao_led_action_server::LedsPlayActionServer" 
  EXECUTABLE led_action_server
)

set(LED_SERVER_INCLUDE_DEPENDS
  "ament_index_cpp"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nao_lola_command_msgs"
  "nao_lola_sensor_msgs"
  "nao_led_interfaces"
  "std_msgs"
  "std_srvs"
  "rcutils")

ament_target_dependencies(led_server ${LED_SERVER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${LED_SERVER_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS led_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

#################### LED CLIENT ###########################
add_library(led_client SHARED
  src/led_action_client.cpp
)

target_include_directories(led_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(led_client
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")

rclcpp_components_register_node(led_client
  PLUGIN "nao_led_action_client::LedsPlayActionClient" 
  EXECUTABLE led_action_client
)

set(LED_CLIENT_INCLUDE_DEPENDS
  "ament_index_cpp"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nao_lola_command_msgs"
  "nao_lola_sensor_msgs"
  "nao_led_interfaces"
  "std_msgs"
  )

ament_target_dependencies(led_client ${LED_CLIENT_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${LED_CLIENT_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS led_client
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)



#################### TEST ###############################
if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
#set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
