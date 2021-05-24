# Generate dependency information
# set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -MMD -MP -MF\"$(@:%.obj=%.d)\"")
# set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wa,-a,-ad,-alms=$(notdir $(<:.c=.lst))")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -MMD -MP -MF\"$(@:%.obj=%.d)\"")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-a,-ad,-alms=$(notdir $(<:.c=.lst))")

# C/C++ build debug
set (CMAKE_C_FLAGS_DEBUG "-Og -g -ggdb -gdwarf-2")
set (CMAKE_CXX_FLAGS_DEBUG "-Og -g -ggdb -gdwarf-2")

# C/C++ build release
# set (CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG")
# set (CMAKE_CXX_FLAGS_RELEASE "-Os -DNDEBUG")

add_compile_options (
    -Wall -Wextra -pedantic
)

add_compile_definitions (
    USE_HAL_DRIVER STM32F407xx
)

add_compile_options (
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -fdata-sections -ffunction-sections
)

add_link_options (
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -fdata-sections -ffunction-sections
)
