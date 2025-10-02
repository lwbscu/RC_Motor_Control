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
project(VESC_Motor_Control C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

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

file(GLOB_RECURSE SOURCES
        "Core/*.*"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c"
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c"
        "Drivers/CMSIS/Device/*.*"
        )

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