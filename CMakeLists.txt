cmake_minimum_required(VERSION 3.16)
project(vantage_vantage_suit)

# Host-side tests only. Firmware builds are managed by STM32CubeIDE projects
# under firmware/master and firmware/node.
add_subdirectory(host/tests/cpp)
