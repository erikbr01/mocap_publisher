add_executable(mocap_publisher
src/main.cpp
)

target_include_directories(mocap_publisher PUBLIC include)

target_link_libraries(mocap_publisher PRIVATE
comm_lib
)

# Third party dependencies
target_link_libraries(mocap_publisher PRIVATE
ViconDataStreamSDK_lib
fastcdr 
fastrtps
)
