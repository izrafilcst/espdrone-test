cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drone_esp32s3 VERSION 0.1.0 LANGUAGES C CXX)
