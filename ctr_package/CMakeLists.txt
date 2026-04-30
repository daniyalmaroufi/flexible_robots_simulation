cmake_minimum_required(VERSION 3.8)
project(ctr_package)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf/
)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes/
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
  FILES_MATCHING PATTERN "*.py"
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)
install(PROGRAMS
  scripts/ctr_key_listener.py
  scripts/ctr_keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
