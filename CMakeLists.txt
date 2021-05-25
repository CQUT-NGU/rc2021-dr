include(cmake/cmake-arm-none-eabi.cmake)

cmake_minimum_required(VERSION 3.10)

project(rc2021_stm32_dr)

set(CMAKE_VERBOSE_MAKEFILE OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Og -ggdb -gdwarf-2")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og -ggdb -gdwarf-2")

include(cmake/cmake-flags.cmake)
add_compile_definitions(
    USE_HAL_DRIVER STM32F407xx
)
add_compile_options(
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -fdata-sections -ffunction-sections
)
add_link_options(
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -fdata-sections -ffunction-sections
)

include_directories("Drivers/CMSIS/Include")
include_directories("Drivers/CMSIS/Device/ST/STM32F4xx/Include")
include_directories("Drivers/STM32F4xx_HAL_Driver/Inc/Legacy")
include_directories("Drivers/STM32F4xx_HAL_Driver/Inc")
aux_source_directory("Drivers/STM32F4xx_HAL_Driver/Src" SOURCE_Drivers)
list(APPEND SOURCE ${SOURCE_Drivers})

include_directories("Middlewares/Third_Party/FreeRTOS/Source/include")
include_directories("Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2")
include_directories("Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F")
# aux_source_directory("Middlewares/Third_Party/FreeRTOS/Source" SOURCE_Middlewares)
# list(APPEND SOURCE_Middlewares "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c")
# list(APPEND SOURCE_Middlewares "Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c")
# list(APPEND SOURCE_Middlewares "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c")
file(GLOB_RECURSE SOURCE_Middlewares Middlewares/*.c)
list(APPEND SOURCE ${SOURCE_Middlewares})

include_directories("Core/Inc")
aux_source_directory("Core/Src" SOURCE_Core)
list(APPEND SOURCE ${SOURCE_Core})

# include_directories("DSP/Include")
# list(APPEND SOURCE "DSP/Source/FastMathFunctions/FastMathFunctions.c")

include_directories("bsp/boards")
include_directories("bsp/devices")
aux_source_directory("bsp/boards" SOURCE_boards)
aux_source_directory("bsp/devices" SOURCE_devices)
list(APPEND SOURCE ${SOURCE_boards} ${SOURCE_devices})

include_directories("application")
aux_source_directory("application" SOURCE_application)
list(APPEND SOURCE ${SOURCE_application})

include_directories("ca")
list(APPEND SOURCE ca/ca.c)

file(GLOB STARTUP *.s)
add_executable(${CMAKE_PROJECT_NAME}.elf ${STARTUP} ${SOURCE})

target_link_libraries(${CMAKE_PROJECT_NAME}.elf c m nosys)

file(GLOB LINKER_SCRIPT  *.ld)
target_link_options(${CMAKE_PROJECT_NAME}.elf PUBLIC
    -Wl,--gc-sections
    --specs=nano.specs
    -T${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
)

# display the size of the output binary after a build is finished
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} --format=berkeley ${CMAKE_PROJECT_NAME}.elf
)
add_custom_target(${CMAKE_PROJECT_NAME}.hex ALL DEPENDS ${CMAKE_PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -Oihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
)
add_custom_target(${CMAKE_PROJECT_NAME}.bin ALL DEPENDS ${CMAKE_PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -Obinary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin
)

# openocd
file(GLOB FLASH_CONFIG *.cfg)
add_custom_target(flash
    COMMAND openocd -f ${FLASH_CONFIG} -c init -c halt -c \"program ${CMAKE_PROJECT_NAME}.elf verify reset exit\"
)
add_custom_target(reset
    COMMAND openocd -f ${FLASH_CONFIG} -c init -c halt -c reset -c shutdown
)
