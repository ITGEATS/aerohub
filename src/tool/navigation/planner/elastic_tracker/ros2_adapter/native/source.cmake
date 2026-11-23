# Build the AeroHub-maintained ROS 2 port; upstream/ is reference only.
set(ELASTIC_SOURCE "${CMAKE_CURRENT_LIST_DIR}/port/src")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
install(FILES "${ELASTIC_SOURCE}/../LICENSE" DESTINATION share/${PROJECT_NAME} RENAME LICENSE.Elastic-Tracker)
