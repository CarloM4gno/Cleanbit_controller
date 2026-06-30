cmake_minimum_required(VERSION 3.8)
project(cleanbit_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Aggiungi opzioni di compilazione se usi C++ (utile se in futuro aggiungi nodi in C++)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Installa cartelle di configurazione e modelli
install(
  DIRECTORY launch model config
  DESTINATION share/${PROJECT_NAME}
)


# Installs python scripts as programs
install(PROGRAMS
  cleanbit_control/arduino_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Automatic lints (only if testing is enabled)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Temporarily disables lints that require license/copyrights
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

