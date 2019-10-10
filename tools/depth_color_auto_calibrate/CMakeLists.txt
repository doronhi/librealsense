# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseToolsDepthColorAutoCalibrate)

add_executable(rs-depth-color-auto-calibrate rs-depth-color-auto-calibrate.cpp depth-color-auto-calibrator.cpp depth-color-auto-calibrator.h simplex.h)
set_property(TARGET rs-depth-color-auto-calibrate PROPERTY CXX_STANDARD 11)
target_link_libraries(rs-depth-color-auto-calibrate ${DEPENDENCIES})
include_directories(rs-depth-color-auto-calibrate ../ ../../third-party/tclap/include)
set_target_properties (rs-depth-color-auto-calibrate PROPERTIES FOLDER "Tools")

install(TARGETS rs-depth-color-auto-calibrate RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
