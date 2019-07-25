// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "depth-smear.h"
#include <iomanip>
#include "l500/l500-depth.h"

namespace librealsense
{
    template<class T>
    bool depth_smear_invalidation(const uint16_t * depth_data_in, const uint8_t * ir_data, T zero_pixel,
        rs2_intrinsics intrinsics,
        const depth_smear_options& options)
    {
        for (auto i = 0; i < intrinsics.height*intrinsics.width; i++)
        {
            auto zero = (depth_data_in[i] > 0) && (i>(intrinsics.height/2)*intrinsics.width) && (i < (intrinsics.height / 2+40)*intrinsics.width);

            zero_pixel(i, zero);
        }
        return true;
    }

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

        auto data = f.as<rs2::frameset>();

        if (!_source_profile_depth)
            _source_profile_depth = data.get_depth_frame().get_profile();

        if (!_target_profile_depth)
            _target_profile_depth = _source_profile_depth.clone(_source_profile_depth.stream_type(), _source_profile_depth.stream_index(), _source_profile_depth.format());

        auto depth_frame = data.get_depth_frame();
        auto ir_frame = data.get_infrared_frame();
        auto confidence_frame = data.first_or_default(RS2_STREAM_CONFIDENCE);

        auto depth_out = source.allocate_video_frame(_target_profile_depth, depth_frame, 0, 0, 0, 0, RS2_EXTENSION_DEPTH_FRAME);

        rs2::frame confidence_out;
        if (confidence_frame)
        {
            if (!_source_profile_confidence)
                _source_profile_confidence = confidence_frame.get_profile();

            if (!_target_profile_confidence)
                _target_profile_confidence = _source_profile_confidence.clone(_source_profile_confidence.stream_type(), _source_profile_confidence.stream_index(), _source_profile_confidence.format());

            confidence_out = source.allocate_video_frame(_source_profile_confidence, confidence_frame, 0, 0, 0, 0, RS2_EXTENSION_VIDEO_FRAME);
        }
        auto depth_intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        auto depth_output = (uint16_t*)depth_out.get_data();
        uint8_t* confidence_output;

        if (confidence_frame)
        {
            confidence_output = (uint8_t*)confidence_out.get_data();
        }

        if (depth_smear_invalidation((const uint16_t*)depth_frame.get_data(),
            (const uint8_t*)ir_frame.get_data(),
            [&](int index, bool zero)
        {
            depth_output[index] = zero ? 0 : ((uint16_t*)depth_frame.get_data())[index];

            if (confidence_frame)
            {
                confidence_output[index] = zero ? 0 : ((uint8_t*)confidence_frame.get_data())[index];
            }
        },
            depth_intrinsics,
            _options))
        {
            result.push_back(depth_out);
            result.push_back(ir_frame);
            if (confidence_frame)
                result.push_back(confidence_out);
        }
        else
        {
            result.push_back(depth_frame);
            result.push_back(ir_frame);
            if (confidence_frame)
                result.push_back(confidence_frame);
        }
        return source.allocate_composite_frame(result);
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

