// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "../include/librealsense2/hpp/rs_frame.hpp"
#include "synthetic-stream.h"
#include "option.h"

namespace librealsense
{
    class depth_smear : public generic_processing_block
    {
    public:
        depth_smear();

        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
    private:
        bool should_process(const rs2::frame& frame) override;
        rs2::frame prepare_output(const rs2::frame_source& source, rs2::frame input, std::vector<rs2::frame> results) override;
        const char * get_option_name(rs2_option option) const override;
    };
    MAP_EXTENSION(RS2_EXTENSION_DEPTH_SMEAR_FILTER, librealsense::depth_smear);
}
