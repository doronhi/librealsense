// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "../include/librealsense2/hpp/rs_frame.hpp"
#include "synthetic-stream.h"
#include "option.h"
#include "canny.h"

#define DS_IR_GRAD_THRESHOLD 180
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
            ir_grad_threshold(DS_IR_GRAD_THRESHOLD),
            ir_threshold(DS_IR_THRESHOLD),
            neighborPix(DS_NEIGHBOR_PIX),
            dz_around_edge_th(DS_DZ_AROUND_EDGE_TH),
            dz_in_neighborPix_th(DS_DZ_IN_NEIGHBOR_PIX_TH)
        {}

        uint8_t                 ir_grad_threshold;
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

        template<class T>
        bool depth_smear_invalidation(const uint16_t * depth_data_in, const uint8_t * ir_data, T zero_pixel,
            rs2_intrinsics intrinsics,
            const depth_smear_options& options);

        std::set<int> findPixByNeighbor(const std::vector<uint8_t>& ir_edge, bool is_horizontal);
        std::set<int> setPixelsToCheck(const std::vector<uint8_t>& ir_edge, bool is_horizontal);
        std::set<int> invalid_smear_dir(bool is_horizontal);
        void discard_low_gradients(std::vector<uint8_t>& ir_edge, bool is_horizontal);
        void calcDzAroundClosestEdge(const int x, const int y, const std::vector<uint8_t>& ir_edge, bool is_horizontal, uint8_t& dz_around_edge, int& closestEdgeIx);
        int calcDepthFromEdge(const int x, const int y, bool is_horizontal, const int closestEdgeIx);

    private:
        rs2::stream_profile     _source_profile_depth;
        rs2::stream_profile     _target_profile_depth;

        rs2::stream_profile     _source_profile_confidence;
        rs2::stream_profile     _target_profile_confidence;

        const uint16_t *        _depth_data;
        const uint8_t *         _ir_data;
        int2                    _imsize;
        std::vector<uint8_t>    _ir_edge;
        std::vector<uint8_t>    _temp_ir_edge;
        std::vector<pixel_t>    _temp_G;
        Canny                   _canny;

        
        depth_smear_options      _options;
    };
    MAP_EXTENSION(RS2_EXTENSION_DEPTH_SMEAR_FILTER, librealsense::depth_smear);
}
