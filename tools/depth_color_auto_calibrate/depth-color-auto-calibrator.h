// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once
#include "core/options.h"
#include "types.h"
#include "core/info.h"
#include "../include/librealsense2/hpp/rs_processing.hpp"
#include "../include/librealsense2/rsutil.h"


#define LASER_DIST_CONT_TH 40
#define LASER_BACKGROUND_TH 30000
#define WIN_ROWS 9
#define WIN_COLS 9
#define ALPHA 1/9
#define GAMMA 0.98

namespace librealsense
{
    struct  depth_color_calib_options
    {
        depth_color_calib_options() :
            laser_dist_cont_th(LASER_DIST_CONT_TH),
            laser_background_th(LASER_BACKGROUND_TH),
            win_rows(WIN_ROWS),
            win_cols(WIN_COLS),
            alpha(ALPHA),
            gamma(GAMMA)
        {}

        double          laser_dist_cont_th;
        uint16_t        laser_background_th;
        size_t          win_rows, win_cols;
        double          alpha, gamma;
    };

    //struct ACParams
    //{
    //    rs2_intrinsics& depth_intrin;
    //    rs2_intrinsics& other_intrin;

    //};



    class depth_color_calib : public options_container, public info_container
    {
    public:
        depth_color_calib(float depth_unit);
        ~depth_color_calib();

        bool calibrate(const rs2::frameset frameset, rs2_extrinsics& depth_to_other);
        bool calibrate(const rs2::video_frame& depth, const rs2::video_frame& color, rs2_extrinsics& depth_to_other);

    private:
        bool is_calibrated(const rs2::video_frame& depth, const rs2::video_frame& other, const rs2_extrinsics& depth_to_other);
        std::vector<rs2::vertex> PointCloudProcessing(const rs2::vertex* vertices, const size_t size);
        bool points_to_image(const std::vector<rs2::vertex>& pc, rs2_intrinsics depth_intrinsics, uint16_t* depth_image);
        void FilterBackgroundNoise(rs2_intrinsics depth_intrinsics, uint16_t* depth_image);
        void SubtractLocalCenter(rs2_intrinsics image_intrinsics, double* image);
        void CalculateDistanceTransform(rs2_intrinsics image_intrinsics, double* image);
        float CalculateObjectionFunction(double* gray_image, uint16_t* depth_image, size_t image_size);
        float CalculateExtrinsicsGrade(std::vector<float> transformation);

    private:
        rs2::pointcloud _pc;
        depth_color_calib_options _options;
        float _depth_scale;
        rs2::frameset _frameset;
        double* _gray_image;
    };
}