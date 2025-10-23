cmake_minimum_required(VERSION 3.10)
project(rmf_manager_cloud)

find_package(ament_cmake REQUIRED)

# Install the Python node as an executable script (no setup.py as requested)
install(PROGRAMS
  scripts/cloud_server_node.py
  scripts/rmf_influx_exporter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch + web assets
install(DIRECTORY launch web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
