# Host-side test suite for Vantare Vantage Suit firmware modules.
#
# The firmware feature modules are header-only, so these tests compile the
# headers directly against host g++/clang++ with FatFs + BLE shims from
# stubs/. Configure & run:
#
#   cmake -S host/tests/cpp -B host/tests/cpp/build
#   cmake --build host/tests/cpp/build
#   ctest --test-dir host/tests/cpp/build --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(vantage_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

enable_testing()

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

set(EXO_COMMON_INC   "${REPO_ROOT}/Firmware/common/inc")
set(MASTER_CORE_INC  "${REPO_ROOT}/Firmware/Master/Core/Inc")
set(NODE_CORE_INC    "${REPO_ROOT}/Firmware/Node/Core/Inc")
set(THIRD_PARTY      "${REPO_ROOT}/Firmware/third_party")
set(STUBS            "${CMAKE_CURRENT_LIST_DIR}/stubs")

add_library(exo_test_common INTERFACE)
target_include_directories(exo_test_common INTERFACE
    "${EXO_COMMON_INC}"
    "${THIRD_PARTY}"
    "${THIRD_PARTY}/sh2"
    "${THIRD_PARTY}/w25qxx/src"
    "${THIRD_PARTY}/icm45686-driver"
    "${STUBS}/fatfs"
)
target_compile_definitions(exo_test_common INTERFACE
    EXO_ACQ_DIAG_HOST_TEST=1
)
if(MSVC)
    target_compile_options(exo_test_common INTERFACE /W4 /permissive-)
else()
    target_compile_options(exo_test_common INTERFACE -Wall -Wextra)
    # Session-stager tests allocate sizeable scratch buffers; give them room.
    target_link_options(exo_test_common INTERFACE -Wl,--stack,16777216)
endif()

if(MINGW)
    # Self-contained executables: no MinGW runtime DLLs needed at test time.
    target_link_options(exo_test_common INTERFACE -static -static-libgcc -static-libstdc++)
endif()

function(exo_add_test NAME SOURCE)
    add_executable(${NAME} ${SOURCE})
    target_link_libraries(${NAME} PRIVATE exo_test_common)
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

exo_add_test(test_master_node_reliable_control        test_master_node_reliable_control.cpp)
exo_add_test(test_master_node_session_stager_sequential test_master_node_session_stager_sequential.cpp)
exo_add_test(test_master_node_transfer_window         test_master_node_transfer_window.cpp)
exo_add_test(test_master_training_csv_formatter_v2    test_master_training_csv_formatter_v2.cpp)
exo_add_test(test_master_training_csv_logger_v2       test_master_training_csv_logger_v2.cpp)
exo_add_test(test_node_live_sample_queue              test_node_live_sample_queue.cpp)
exo_add_test(test_remote_transfer_lifecycle_regression test_remote_transfer_lifecycle_regression.cpp)
exo_add_test(test_coderabbit_csv_logger_regressions   test_coderabbit_csv_logger_regressions.cpp)
exo_add_test(test_coderabbit_firmware_regressions     test_coderabbit_firmware_regressions.cpp)
exo_add_test(test_hub_leaf_ble_manager                test_hub_leaf_ble_manager.cpp)
exo_add_test(test_link_tune_state                     test_link_tune_state.cpp)
