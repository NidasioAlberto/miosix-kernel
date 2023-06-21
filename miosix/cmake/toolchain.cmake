# Tell CMake that we are building for an embedded ARM system
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Select compiler
set(PREFIX arm-miosix-eabi-)

# From compiler prefix form the name of the compiler and other tools
set(CMAKE_ASM_COMPILER ${PREFIX}as)
set(CMAKE_C_COMPILER   ${PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${PREFIX}g++)
set(CMAKE_AR           ${PREFIX}ar)
set(CMAKE_OBJCOPY      ${PREFIX}objcopy)
set(CMAKE_OBJDUMP      ${PREFIX}objdump)
set(CMAKE_SIZE         ${PREFIX}size)

# Reset default compiler flags
set(CMAKE_ASM_FLAGS_DEBUG   "" CACHE INTERNAL "asm debug compiler flags")
set(CMAKE_ASM_FLAGS_RELEASE "" CACHE INTERNAL "asm release compiler flags")
set(CMAKE_C_FLAGS_DEBUG     "" CACHE INTERNAL "c debug compiler flags")
set(CMAKE_C_FLAGS_RELEASE   "" CACHE INTERNAL "c release compiler flags")
set(CMAKE_CXX_FLAGS_DEBUG   "" CACHE INTERNAL "cxx debug compiler flags")
set(CMAKE_CXX_FLAGS_RELEASE "" CACHE INTERNAL "cxx release compiler flags")
