// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "depth-smear.h"
#include <iomanip>
#include "l500/l500-depth.h"

namespace librealsense
{
    depth_smear::depth_smear()
        : generic_processing_block("Depth Smear Fix")
    {
    }

    const char* depth_smear::get_option_name(rs2_option option) const
    {
        return "No Options Yet";
    }

    rs2::frame depth_smear::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {
        std::vector<rs2::frame> result;
        return f;
    }

    bool depth_smear::should_process(const rs2::frame& frame)
    {
        if (auto set = frame.as<rs2::frameset>())
        {
            if (!set.get_depth_frame() || !set.get_infrared_frame())
            {
                return false;
            }
            return true;
        }
        return false;
    }

    rs2::frame depth_smear::prepare_output(const rs2::frame_source & source, rs2::frame input, std::vector<rs2::frame> results)
    {
        if (auto composite = input.as<rs2::frameset>())
        {
            composite.foreach([&](rs2::frame f)
            {
                if (f.get_profile().stream_type() != RS2_STREAM_DEPTH && f.get_profile().stream_type() != RS2_STREAM_INFRARED && f.get_profile().stream_type() != RS2_STREAM_CONFIDENCE &&
                    results.size() > 0)
                    results.push_back(f);
            });
        }
        return source.allocate_composite_frame(results);
    }

}

