cmake_minimum_required(VERSION 3.22)


# 设置机器人ID
# 包含机器人配置（定义ROBOT_NAME）
include(${CMAKE_CURRENT_SOURCE_DIR}/PYRo/cmake/config/pyro_robot_id_config.cmake)



# ==============================================================================
# 1. 编译器与工具链配置 (已由 gcc-arm-none-eabi.cmake 处理)
# ==============================================================================
include(CubeMX/cmake/gcc-arm-none-eabi.cmake)

# ==============================================================================
# 2. 项目定义
# ==============================================================================
set(CMAKE_PROJECT_NAME PYRo)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()

# ==============================================================================
# 3. 额外性能优化 (保留工具链未包含的指令)
# ==============================================================================
add_compile_options(
        "-mthumb"
        "-Wdouble-promotion"
        "-funsafe-math-optimizations"
)

# ==============================================================================
# 4. CMSIS-DSP & dsppp 库引入
# ==============================================================================
set(DSP_REPO "${CMAKE_CURRENT_SOURCE_DIR}/third_party/CMSIS-DSP")

add_compile_definitions(
        ARM_MATH_CM7
        ARM_MATH_LOOPUNROLL
)

set(CMSISDSP_BUILD_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)
add_subdirectory(${DSP_REPO} build_cmsis_dsp)

target_include_directories(CMSISDSP PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/CubeMX/Drivers/CMSIS/Include"
)

add_library(dsppp_lib INTERFACE)
target_include_directories(dsppp_lib INTERFACE
        ${DSP_REPO}/dsppp/Include
        ${DSP_REPO}/Include
)

# ==============================================================================
# 5. 源文件配置 (按分类排序)
# ==============================================================================
add_executable(${CMAKE_PROJECT_NAME}
        PYRo/Core/ETL/map.cpp
        PYRo/Core/Lock/pyro_mutex.cpp
        PYRo/Core/Lock/pyro_rw_lock.cpp
        PYRo/Core/Task/pyro_task.cpp
        PYRo/Core/Memory/pyro_core_dma_heap.c
        PYRo/Core/Memory/pyro_core_mem.cpp
        PYRo/Core/Task/pyro_task.cpp

        # --- Peripheral ---
        PYRo/Peripheral/CAN/pyro_can_drv.cpp
        PYRo/Peripheral/DWT/pyro_dwt_drv.cpp
        PYRo/Peripheral/UART/pyro_uart_drv.cpp
        PYRo/Peripheral/UART/pyro_bsp_uart.cpp

        # --- Algorithm ---
        PYRo/Algorithm/Kinematics/pyro_kin_hybrid.cpp
        PYRo/Algorithm/Kinematics/pyro_kin_mec.cpp
        PYRo/Algorithm/Kinematics/pyro_kin_rudder.cpp
        PYRo/Algorithm/Kinematics/pyro_kin_wl.cpp
        PYRo/Algorithm/OLS/pyro_algo_ols.cpp
        PYRo/Algorithm/PID/pyro_algo_pid.cpp
        PYRo/Algorithm/EKF/QuaternionEKF.cpp
        PYRo/Algorithm/KF/kalman_filter.cpp
        PYRo/Algorithm/Common/pyro_algo_common.cpp

        # --- Component ---
        PYRo/Component/CRC/PYRo_crc.cpp
        PYRo/Component/Motor/pyro_dji_motor_drv.cpp
        PYRo/Component/Motor/pyro_dm_motor_drv.cpp
        PYRo/Component/Motor/pyro_motor_base.cpp
        PYRo/Component/Powercontrol/pyro_power_control_drv.cpp
        PYRo/Component/Powermeter/pyro_powermeter.cpp
        PYRo/Component/RC/pyro_dr16_rc_drv.cpp
        PYRo/Component/RC/pyro_rc_base_drv.cpp
        PYRo/Component/RC/pyro_vt03_rc_drv.cpp
        PYRo/Component/Referee/fifo.c
        PYRo/Component/Referee/pyro_referee.cpp
        PYRo/Component/INS/pyro_ins.cpp
        PYRo/Component/Supercap/pyro_supercap_drv.cpp
        # --- Debug ---
        PYRo/Debug/Debug_task.cpp
        PYRo/Debug/JCOM/pyro_jcom.cpp
        PYRo/Debug/VOFA/pyro_vofa.cpp


        # --- Communication ---
        PYRo/Communication/pyro_com_cantx.cpp
        PYRo/Communication/pyro_com_canrx.cpp
        PYRo/Communication/pyro_uart_comm.cpp


        # --- Device ---
        PYRo/Device/BMI088/BMI088_driver.cpp
        PYRo/Device/BMI088/BMI088_middleware.cpp
        PYRo/Component/Referee/UI/pyro_ui_drv.cpp


)
if (ROBOT_ID EQUAL HERO_ID)
    include(${CMAKE_CURRENT_SOURCE_DIR}/robot/hero_target.cmake)
elseif (ROBOT_ID EQUAL SUB_HERO_ID)
    include(${CMAKE_CURRENT_SOURCE_DIR}/robot/sub_hero_target.cmake)
elseif(ROBOT_ID EQUAL INFANTRY2_ID)
    include(${CMAKE_CURRENT_SOURCE_DIR}/robot/infantry2_target.cmake)
elseif(ROBOT_ID EQUAL SENTRY_ID)
    include(${CMAKE_CURRENT_SOURCE_DIR}/robot/sentry_target.cmake)
endif ()

add_subdirectory(CubeMX/cmake/stm32cubemx)

# ==============================================================================
# 6. 包含路径与链接 (按路径排序)
# ==============================================================================
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        # --- Core ---
        PYRo/Core/Config
        PYRo/Core/Def
        PYRo/Core/ETL
        PYRo/Core/FSM
        PYRo/Core/Lock
        PYRo/Core/Memory
        PYRo/Core/Task

        # --- Peripheral ---
        PYRo/Peripheral/CAN
        PYRo/Peripheral/DWT
        PYRo/Peripheral/UART

        # --- Device ---
        PYRo/Device/BMI088

        # --- Algorithm ---
        PYRo/Algorithm/Common
        PYRo/Algorithm/Kinematics
        PYRo/Algorithm/OLS
        PYRo/Algorithm/PID
        PYRo/Algorithm/EKF
        PYRo/Algorithm/KF

        # --- Component ---
        PYRo/Component/CRC
        PYRo/Component/IMU
        PYRo/Component/INS
        PYRo/Component/Motor
        PYRo/Component/Powercontrol
        PYRo/Component/Powermeter
        PYRo/Component/RC
        PYRo/Component/Referee
        PYRo/Component/Referee/UI
        PYRo/Component/Shoot
        PYRo/Component/Supercap

        # --- Module ---
        PYRo/Module

        # --- Communication ---
        PYRo/Communication

        # --- Debug ---
        PYRo/Debug/VOFA
        PYRo/Debug/JCOM
)

target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        CMSISDSP
        dsppp_lib
)

add_dependencies(echo_robot_id ${CMAKE_PROJECT_NAME})

