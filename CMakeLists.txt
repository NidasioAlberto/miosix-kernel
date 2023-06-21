cmake_minimum_required(VERSION 3.21)
include(miosix/cmake/mbs.cmake)

project(MiosixExamples)

#-----------------------------------------------------------------------------#
#                                  Examples                                   #
#-----------------------------------------------------------------------------#

add_executable(asm miosix/_examples/asm/main.s)
mbs_target(asm stm32f407vg_stm32f4discovery)

add_executable(atsam4l_lcd miosix/_examples/atsam4l_lcd/main.cpp)
mbs_target(atsam4l_lcd atsam4lc2aa_generic)

add_executable(blinking_led miosix/_examples/blinking_led/simple.cpp)
mbs_target(blinking_led stm32f429zi_stm32f4discovery)

add_executable(datalogger
    ${KPATH}/_examples/datalogger/main.cpp
    ${KPATH}/_examples/datalogger/Logger.cpp
    ${KPATH}/_examples/datalogger/tscpp/buffer.cpp
)
target_include_directories(datalogger PUBLIC miosix/_examples/datalogger/)
mbs_target(datalogger stm32f407vg_stm32f4discovery)

add_executable(deep_sleep miosix/_examples/deep_sleep/main-test.cpp)
mbs_target(deep_sleep stm32f407vg_stm32f4discovery)

add_executable(hd44780 miosix/_examples/hd44780/hd44780.cpp)
mbs_target(hd44780 stm32f407vg_stm32f4discovery)

add_executable(ir_decoder miosix/_examples/ir_decoder/ir_decoder.cpp)
mbs_target(ir_decoder stm32f407vg_stm32f4discovery)

add_executable(led_display miosix/_examples/led_display/main.cpp)
mbs_target(led_display stm32f100rb_stm32vldiscovery)

add_executable(sad_trombone
    miosix/_examples/sad_trombone/main.cpp
    miosix/_examples/sad_trombone/player.cpp
    miosix/_examples/sad_trombone/adpcm.c
)
mbs_target(sad_trombone stm32f407vg_stm32f4discovery)

add_executable(servo-prompt miosix/_examples/servo/prompt.cpp)
mbs_target(servo-prompt stm32f407vg_stm32f4discovery)

add_executable(servo-sweep miosix/_examples/servo/sweep.cpp)
mbs_target(servo-sweep stm32f407vg_stm32f4discovery)

add_executable(streamwriter miosix/_examples/streamwriter/streamwriter.cpp)
mbs_target(streamwriter stm32f407vg_stm32f4discovery)

add_executable(termios miosix/_examples/termios/main.cpp)
mbs_target(termios stm32f407vg_stm32f4discovery)

add_executable(thread_native miosix/_examples/thread_native/native_thread_example.cpp)
mbs_target(thread_native stm32f407vg_stm32f4discovery)

add_executable(thread_pthread miosix/_examples/thread_pthread/pthread_example.cpp)
mbs_target(thread_pthread stm32f407vg_stm32f4discovery)

#-----------------------------------------------------------------------------#
#                                   Others                                    #
#-----------------------------------------------------------------------------#

add_executable(delay_test miosix/_tools/delay_test/delay_test.cpp)
mbs_target(delay_test stm32f407vg_stm32f4discovery)

add_executable(feedforward_profiling miosix/_tools/feedforward_profiling/test.cpp)
mbs_target(feedforward_profiling stm32f407vg_stm32f4discovery)

add_executable(fs_backend miosix/_tools/fs_backend/backend_benchmark.cpp)
mbs_target(fs_backend stm32f407vg_stm32f4discovery)

add_executable(fs_misc_testcode miosix/_tools/fs_misc_testcode/dirlist.cpp)
mbs_target(fs_misc_testcode stm32f407vg_stm32f4discovery)

# To build this entrypoint remember to define WITH_PROCESSES in miosix_settings.h
# add_executable(processes miosix/_tools/processes/main_processes.cpp)
# target_include_directories(processes PUBLIC miosix/_tools/processes/)
# add_custom_command(
#   TARGET processes
#   COMMAND make
#   WORKING_DIRECTORY ${KPATH}/_tools/processes/process_template
# )
# mbs_target(processes stm32f407vg_stm32f4discovery)

add_executable(ram_test
    miosix/_tools/ram_test/main.cpp
    miosix/_tools/ram_test/sha1.cpp
)
mbs_target(ram_test stm32f407vg_stm32f4discovery)

# If you want to test the MPU remember to run build.sh in miosix/_tools/testsuite
add_executable(testsuite miosix/_tools/testsuite/testsuite.cpp)
mbs_target(testsuite stm32f407vg_stm32f4discovery)