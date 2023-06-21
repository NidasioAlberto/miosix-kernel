enable_language(C CXX ASM)

# Load in CMAKE_MODULE_PATH the current directory
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

# Load in MBS_BASE the project path (points to the 'miosix' folder)
get_filename_component(MBS_BASE ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)

# Include the kernel and board list
file(GLOB KPATH ${MBS_BASE})
add_subdirectory(${KPATH} EXCLUDE_FROM_ALL)
include(${KPATH}/cmake/boards.cmake)

# Create a custom target to list all the boards
string(REPLACE ";" "\\n" BOARDS_STR "${BOARDS}")
add_custom_target(
    help-boards
    COMMAND printf ${BOARDS_STR}
    COMMENT "All boards available:"
    VERBATIM
)

# Function to link the Miosix libraries to the target
function(mbs_target TARGET OPT_BOARD)
    if(NOT OPT_BOARD)
        message(FATAL_ERROR "No board selected")
    endif()

    # Include miosix library
    target_link_libraries(${TARGET} PRIVATE
        $<TARGET_OBJECTS:Miosix::Boot::${OPT_BOARD}>
        $<LINK_GROUP:RESCAN,Miosix::Kernel::${OPT_BOARD},stdc++,c,m,gcc,atomic>
    )

    # Linker script and linking options are eredited from the kernel library

    # Add a post build command to create the hex file to flash on the board
    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET} ${TARGET}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET} ${TARGET}.bin
        BYPRODUCTS ${TARGET}.hex ${TARGET}.bin
        VERBATIM
    )

    # Save custom flash command to file
    get_target_property(PROGRAM_CMDLINE Miosix::Kernel::${OPT_BOARD} PROGRAM_CMDLINE)
    if(NOT PROGRAM_CMDLINE STREQUAL "PROGRAM_CMDLINE-NOTFOUND")
        file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/flash-${TARGET}.txt" ${PROGRAM_CMDLINE})
    endif()
endfunction()
