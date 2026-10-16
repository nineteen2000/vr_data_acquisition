cmake_minimum_required(VERSION 3.14)
project(LHandProLib_EtherCAT_Test LANGUAGES CXX)

# 定位到 SDK 根目录
set(SDK_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../..)
message(STATUS "SDK Root Directory: ${SDK_ROOT_DIR}")

# 设置输出路径统一管理
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
# 覆盖各配置的输出路径，忽略 Debug/Release 子目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# 添加头文件目录
include_directories(${SDK_ROOT_DIR}/include/LHandProLib)

option(ETHERCAT_DEBUG "Enable EtherCAT print of test module LHandProLib_EtherCAT_Test" OFF)
add_compile_definitions(ETHERCAT_DEBUG=$<BOOL:${ETHERCAT_DEBUG}>)

# 构建SOEM库
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/SOEM)

# 构建测试程序
add_executable(LHandProLib_EtherCAT_Test main.cpp EthercatMaster.cpp)

# 链接主库
find_library(LHANDPRO_LIB NAMES LHandProLib PATHS ${SDK_ROOT_DIR}/lib REQUIRED)
target_link_libraries(LHandProLib_EtherCAT_Test PRIVATE
    ${LHANDPRO_LIB}
    soem
)

# 导出头文件
target_include_directories(LHandProLib_EtherCAT_Test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# 区分 Windows 和 Linux 平台
if(WIN32)
    # Windows (x64)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        # 64-bit (x64)
        message(STATUS "Building for 64-bit (x64)")
    
        # 安装对应的 DLL   
        set(NPCAP_DLLS
            ${CMAKE_CURRENT_SOURCE_DIR}/x64/Packet.dll
            ${CMAKE_CURRENT_SOURCE_DIR}/x64/wpcap.dll
            ${SDK_ROOT_DIR}/bin/LHandProLib.dll
        )
        
        # 遍历每个 DLL 文件
        foreach(dll_path ${NPCAP_DLLS})
            # 获取文件名（如 Packet.dll）
            get_filename_component(dll_name "${dll_path}" NAME)

            # 目标路径
            set(dst_path "${CMAKE_BINARY_DIR}/bin/${dll_name}")

            # 添加复制命令：构建目标后复制文件（如果不同则复制）
            add_custom_command(
                TARGET LHandProLib_EtherCAT_Test POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${dll_path}" "${dst_path}"
                COMMENT "Copying ${dll_name} to output directory"
                VERBATIM
            )
        endforeach()
        
        install(FILES 
            ${NPCAP_DLLS}
            DESTINATION bin
        )
    else()
        message(FATAL_ERROR "Other architecture: ${CMAKE_SIZEOF_VOID_P}")
    endif()
    
elseif(UNIX AND NOT APPLE)
    # Linux: 链接系统安装
endif()

# 安装测试程序
install(TARGETS LHandProLib_EtherCAT_Test
    RUNTIME DESTINATION bin
)

if(UNIX AND NOT APPLE)
    # 计算绝对路径
    get_filename_component(LIB_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib" ABSOLUTE)
    message(STATUS "Absolute lib path: ${LIB_SOURCE_PATH}")
    install(DIRECTORY 
        ${LIB_SOURCE_PATH}/
        DESTINATION bin
    )
endif()