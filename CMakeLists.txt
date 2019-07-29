cmake_minimum_required(VERSION 2.8.3)
project(draco_point_cloud_transport)

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        message_generation
        point_cloud_transport
        sensor_msgs
        std_msgs)

find_package(Draco REQUIRED)
message("${draco_INCLUDE_DIR}")
message("${draco_LIBRARY_DIR}")

add_message_files(
        FILES
        CompressedPointCloud2.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

generate_dynamic_reconfigure_options(cfg/DracoPublisher.cfg cfg/DracoSubscriber.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${draco_LIBRARY_DIR}
  CATKIN_DEPENDS dynamic_reconfigure point_cloud_transport sensor_msgs std_msgs message_runtime
  DEPENDS draco
)

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${draco_INCLUDE_DIR}/..
        ${draco_INCLUDE_DIR})

link_directories(${draco_LIBRARY_DIR})

add_library(${PROJECT_NAME}
        src/draco_publisher.cpp
        src/draco_subscriber.cpp
        src/manifest.cpp)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg draco_point_cloud_transport_generate_messages_cpp draco)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} libdracoenc.so)

class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)



# add xml file
install(FILES draco_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
