
target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
        include/third_party/FastCRC
        include/third_party/spdlog
        include/third_party/cmdline
        PRIVATE
        src)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "include/livox_def.h;include/livox_sdk.h")

# set(PLATFORM win)

# target_sources(${SDK_LIBRARY}
#         PRIVATE
#         src/third_party/FastCRC/FastCRC_tables.hpp
#         src/third_party/FastCRC/FastCRCsw.cpp
#         src/base/io_loop.h
#         src/base/io_loop.cpp
#         src/base/thread_base.h
#         src/base/thread_base.cpp
#         src/base/io_thread.h
#         src/base/io_thread.cpp
#         src/base/network/network_util.h
#         src/base/network/${PLATFORM}/network_util.cpp
#         src/base/logging.h
#         src/base/logging.cpp
#         src/base/noncopyable.h
#         src/livox_sdk.cpp
#         src/device_discovery.h
#         src/device_discovery.cpp
#         src/device_manager.h
#         src/device_manager.cpp
#         src/comm/comm_port.cpp
#         src/comm/sdk_protocol.h
#         src/comm/sdk_protocol.cpp
#         src/data_handler/data_handler.h
#         src/data_handler/data_handler.cpp
#         src/data_handler/hub_data_handler.h
#         src/data_handler/hub_data_handler.cpp
#         src/data_handler/lidar_data_handler.h
#         src/data_handler/lidar_data_handler.cpp
#         src/command_handler/command_handler.h
#         src/command_handler/command_handler.cpp
#         src/command_handler/command_channel.h
#         src/command_handler/command_channel.cpp
#         src/command_handler/hub_command_handler.h
#         src/command_handler/hub_command_handler.cpp
#         src/command_handler/lidar_command_handler.h
#         src/command_handler/lidar_command_handler.cpp
#         src/command_handler/command_impl.cpp
#         src/command_handler/command_impl.h
#         src/base/multiple_io/multiple_io_base.h
#         src/base/multiple_io/multiple_io_base.cpp
#         src/base/multiple_io/multiple_io_factory.h
#         src/base/multiple_io/multiple_io_epoll.h
#         src/base/multiple_io/multiple_io_epoll.cpp
#         src/base/multiple_io/multiple_io_poll.h
#         src/base/multiple_io/multiple_io_poll.cpp
#         src/base/multiple_io/multiple_io_select.h
#         src/base/multiple_io/multiple_io_select.cpp
#         src/base/multiple_io/multiple_io_kqueue.h
#         src/base/multiple_io/multiple_io_kqueue.cpp
#         src/base/wake_up/wake_up_pipe.h
#         src/base/wake_up/${PLATFORM}/wake_up_pipe.cpp)
