
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "depth-color-auto-calibrator.h"

using namespace std;
// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[])
{
    rs2::log_to_file(RS2_LOG_SEVERITY_DEBUG);

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 0, 640, 480);
    cfg.enable_stream(RS2_STREAM_COLOR, 0, 1920, 1080);
    auto profile = pipe.start(cfg);

    rs2_extrinsics fixed_depth_to_other;

    auto sensor = profile.get_device().first<rs2::depth_sensor>();
    float depth_units = sensor.get_option(RS2_OPTION_DEPTH_UNITS);
    librealsense::depth_color_calib dcac(depth_units);
    while (true)
    {
        rs2::frameset frameset = pipe.wait_for_frames();
        bool is_ok = dcac.calibrate(frameset, fixed_depth_to_other);
    }
    return 0;
}