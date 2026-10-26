cmake_minimum_required(VERSION 3.22)

# 启用 printf 的浮点支持（必须加这行）
add_link_options(-u _printf_float)
#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ins_task)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Inc/all_tou_h.h
        application/INS_task.c
        application/INS_task.h
        application/struct_typedef.h
        application/bsp/imu/bsp_delay.c
        application/bsp/imu/bsp_delay.h
        application/bsp/imu/bsp_imu_pwm.c
        application/bsp/imu/bsp_imu_pwm.h
        application/bsp/imu/bsp_led.h
        application/bsp/imu/bsp_led.c
        application/bsp/imu/bsp_spi.c
        application/bsp/imu/bsp_spi.h
        application/components/algorithm/MahonyAHRS.h
        application/components/algorithm/MahonyAHRS.C
        application/components/algorithm/LESO.c
        application/components/algorithm/LESO.h
        application/components/algorithm/RLS.c
        application/components/algorithm/RLS.h
        application/components/algorithm/user_lib.h
        application/components/algorithm/user_lib.c
        application/components/algorithm/Include/arm_math.h
        application/components/algorithm/Include/arm_common_tables.h
        application/components/algorithm/Include/arm_const_structs.h
        application/components/controller/pid.c
        application/components/controller/pid.h
        application/components/devices/BMI088driver.h
        application/components/devices/BMI088driver.c
        application/components/devices/BMI088Middleware.h
        application/components/devices/BMI088Middleware.c
        application/components/devices/BMI088reg.h
        application/components/devices/ist8310driver.c
        application/components/devices/ist8310driver.h
        application/components/devices/ist8310driver_middleware.h
        application/components/devices/ist8310driver_middleware.c


        modules/oled/oledfont.h
        modules/oled/OLED.h
        modules/oled/OLED.c
        modules/usart_printf/usart_printf_task.c
        modules/usart_printf/usart_printf_task.h

        chassis_translate/bsp/can/bsp_can.c
        chassis_translate/bsp/can/bsp_can.h
        chassis_translate/bsp/can/CAN_receive.c
        chassis_translate/bsp/can/CAN_receive.h
        chassis_translate/bsp/remote/bsp_rc.c
        chassis_translate/bsp/remote/bsp_rc.h
        chassis_translate/bsp/remote/remote_control.c
        chassis_translate/bsp/remote/remote_control.h
        chassis_translate/task/chassis/chassis_task.c
        chassis_translate/task/chassis/chassis_task.h
        chassis_translate/task/chassis/chassis_pid.c
        chassis_translate/task/chassis/chassis_pid.h
        chassis_translate/task/chassis/chassis_behaviour.c
        chassis_translate/task/chassis/chassis_behaviour.h
        chassis_translate/task/chassis/bmi088_date_send.c
        chassis_translate/task/chassis/bmi088_date_send.h

        chassis_translate/task/power_management/referee.c
        chassis_translate/task/power_management/referee.h
        chassis_translate/task/rm_ui_protocol.c
        chassis_translate/task/rm_ui_protocol.h
        chassis_translate/task/yaw_motor/Gimbal_Pid.c
        chassis_translate/task/yaw_motor/Gimbal_Pid.h
        chassis_translate/task/yaw_motor/yaw_motor_task.c
        chassis_translate/task/yaw_motor/yaw_motor_task.h

        chassis_translate/task/Shoot/trigger_task.c
        chassis_translate/task/Shoot/trigger_task.h

        i6x_remote/ix6.h
        i6x_date_receive/gimbal_i6x_date.h


)



# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)



# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        ${CMAKE_SOURCE_DIR}/Core
        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${CMAKE_SOURCE_DIR}/Core/Src


        ${CMAKE_SOURCE_DIR}/modules/usart_printf


        ${CMAKE_SOURCE_DIR}/application
        ${CMAKE_SOURCE_DIR}/application/bsp
        ${CMAKE_SOURCE_DIR}/application/bsp/imu
        ${CMAKE_SOURCE_DIR}/application/components
        ${CMAKE_SOURCE_DIR}/application/components/algorithm
        ${CMAKE_SOURCE_DIR}/application/components/controller
        ${CMAKE_SOURCE_DIR}/application/components/algorithm/Include
        ${CMAKE_SOURCE_DIR}/application/components/devices



        ${CMAKE_SOURCE_DIR}/chassis_translate
        ${CMAKE_SOURCE_DIR}/chassis_translate/bsp
        ${CMAKE_SOURCE_DIR}/chassis_translate/bsp/can
        ${CMAKE_SOURCE_DIR}/chassis_translate/bsp/remote
        ${CMAKE_SOURCE_DIR}/chassis_translate/task
        ${CMAKE_SOURCE_DIR}/chassis_translate/task/chassis
        ${CMAKE_SOURCE_DIR}/chassis_translate/task/yaw_motor

        ${CMAKE_SOURCE_DIR}/chassis_translate/task/Shoot


        ${CMAKE_SOURCE_DIR}/chassis_translate/task/power_management
        ${CMAKE_SOURCE_DIR}/i6x_remote

        ${CMAKE_SOURCE_DIR}/i6x_date_receive
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 添加包含路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

)

