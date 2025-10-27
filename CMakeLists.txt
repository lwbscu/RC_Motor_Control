#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.17)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(M3508_Motor_Control C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(Core/Inc Drivers/STM32G4xx_HAL_Driver/Inc Drivers/STM32G4xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32G4xx/Include Drivers/CMSIS/Include)

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx)

# 改进文件收集，排除不需要的CMSIS模板和测试文件
file(GLOB_RECURSE SOURCES
        "Core/*.*"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c"          # 基础HAL
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c"    # Cortex内核相关
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c"       # DMA（若依赖）
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c"    # DMA扩展（若依赖）
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c"      # EXTI（若依赖）
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c"     # FDCAN（M3508电机需用）
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c"     # Flash
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c"  # Flash扩展
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c" # Flash RAM函数
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c"      # GPIO
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c"       # PWR
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c"    # PWR扩展
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c"       # RCC时钟
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c"    # RCC扩展
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c"       # TIM定时器（电机控制需用）
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c"    # TIM扩展
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c"      # UART核心驱动（关键！）
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c"   # UART扩展驱动（关键！）
        "Drivers/CMSIS/Device/*.*"
        )

# 排除不需要的CMSIS模板文件
list(FILTER SOURCES EXCLUDE REGEX ".*Template.*")
list(FILTER SOURCES EXCLUDE REGEX ".*TestSuite.*")


set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32G474VETX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")