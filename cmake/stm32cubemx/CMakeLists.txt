cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_HAL_DRIVER 
	STM32F427xx
    ARM_MATH_CM4
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/SEGGRE/RTT
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/SEGGRE/RTT/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/ARM/DSP/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../APP/action
    ${CMAKE_CURRENT_SOURCE_DIR}/../../APP/gimbal
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/can
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/dwt
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/log
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/pwm
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/tim
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/usart
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/coordinate_system
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/crc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/kalman
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/linear_algebra
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/pid
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/unicomm
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/daemon
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/motor
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/motor/DJImotor
    ${CMAKE_CURRENT_SOURCE_DIR}/../../TASK/MotorControl_task
    ${CMAKE_CURRENT_SOURCE_DIR}/../../TASK/PostureControl_task
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/freertos.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32f427xx.s
    ${CMAKE_CURRENT_SOURCE_DIR}/../../APP/action/action.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../APP/gimbal/gimbal_motor_monitor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/bsp_init.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/can/bsp_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/dwt/bsp_dwt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/log/bsp_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/pwm/bsp_pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/tim/bsp_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/usart/bsp_usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/motor/DJImotor/dji_motor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/user_lib.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/coordinate_system/coordinate_system.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/crc/crc8.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/crc/crc16.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/kalman/Kalman_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/linear_algebra/linear_algebra.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/pid/controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/algorithm/pid/PID.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/imu/IMU.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/unicomm/unicomm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/daemon/daemon.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../TASK/MotorControl_task/MotorControl_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../TASK/PostureControl_task/PostureControl_task.c
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32f4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# Drivers Midllewares


set(FreeRTOS_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/SEGGRE/RTT/SEGGER_RTT.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/SEGGRE/RTT/SEGGER_RTT_printf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/SEGGRE/RTT/Config/SEGGER_RTT_Conf.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/SEGGRE/RTT/SEGGER_RTT_ASM_ARMv7M.S
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
    FreeRTOS
	
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Create FreeRTOS static library
add_library(FreeRTOS OBJECT)
target_sources(FreeRTOS PRIVATE ${FreeRTOS_Src})
target_link_libraries(FreeRTOS PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
