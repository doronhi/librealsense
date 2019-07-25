// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "../include/librealsense2/hpp/rs_frame.hpp"
#include "synthetic-stream.h"
#include "option.h"

#define DS_IR_GRADE_THRESHOLD 180
#define DS_IR_THRESHOLD 190
#define DS_NEIGHBOR_PIX 8
#define DS_DZ_AROUND_EDGE_TH 45
#define DS_DZ_IN_NEIGHBOR_PIX_TH 100

namespace librealsense
{
    struct  depth_smear_options
    {
        // params = struct('IR_grad_thresh', 180, 'ir_thresh', 190, 'neighborPix', 8, 'dz_around_edge_th', 45, 'dz_in_neighborPix_th', 100);
        depth_smear_options() :
            ir_grade_threshold(DS_IR_GRADE_THRESHOLD),
            ir_threshold(DS_IR_THRESHOLD),
            neighborPix(DS_NEIGHBOR_PIX),
            dz_around_edge_th(DS_DZ_AROUND_EDGE_TH),
            dz_in_neighborPix_th(DS_DZ_IN_NEIGHBOR_PIX_TH)
        {}

        uint8_t                 ir_grade_threshold;
        uint8_t                 ir_threshold;
        uint8_t                 neighborPix;
        uint16_t                dz_around_edge_th;
        uint16_t                dz_in_neighborPix_th;
    };

    class depth_smear : public generic_processing_block
    {
    public:
        depth_smear();

        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
    private:
        bool should_process(const rs2::frame& frame) override;
        rs2::frame prepare_output(const rs2::frame_source& source, rs2::frame input, std::vector<rs2::frame> results) override;
        const char * get_option_name(rs2_option option) const override;


        rs2::stream_profile     _source_profile_depth;
        rs2::stream_profile     _target_profile_depth;

        rs2::stream_profile     _source_profile_confidence;
        rs2::stream_profile     _target_profile_confidence;

        depth_smear_options      _options;
    };
    MAP_EXTENSION(RS2_EXTENSION_DEPTH_SMEAR_FILTER, librealsense::depth_smear);
}
