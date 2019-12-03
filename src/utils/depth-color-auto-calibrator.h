// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once
#include "core/options.h"
#include "types.h"
#include "core/info.h"
#include "../include/librealsense2/hpp/rs_processing.hpp"


#define LASER_DIST_CONT_TH 40
#define LASER_BACKGROUND_TH 30000
#define WIN_ROWS_EDGE 3
#define WIN_COLS_EDGE 3
#define WIN_ROWS_DSF 9
#define WIN_COLS_DSF 9
#define ALPHA 1/9
#define GAMMA 0.98

namespace librealsense
{
    struct  depth_color_calib_options_st
    {
        depth_color_calib_options_st() :
            laser_dist_cont_th(LASER_DIST_CONT_TH),
            laser_background_th(LASER_BACKGROUND_TH),
            win_rows_edge(WIN_ROWS_EDGE),
            win_cols_edge(WIN_COLS_EDGE),
            win_rows_dsf(WIN_ROWS_DSF),
            win_cols_dsf(WIN_COLS_DSF),
            alpha(ALPHA),
            gamma(GAMMA)
        {}

        float          laser_dist_cont_th;
        uint16_t        laser_background_th;
        size_t          win_rows_edge, win_cols_edge;
        size_t          win_rows_dsf, win_cols_dsf;        
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
        void set_frameset(rs2::frameset frameset);
        ~depth_color_calib();

        void prepare_images();
        void calibrate(rs2_extrinsics* depth_to_other);
        //bool old_calibrate(const rs2::frameset frameset, rs2_extrinsics& depth_to_other);
        //bool calibrate(const rs2_frame* depth, const rs2_frame* color, rs2_extrinsics& depth_to_other);
        //bool calibrate(const rs2::frame depth, const rs2::frame color, rs2_extrinsics& depth_to_other);
        float calculate_extrinsics_grade(std::vector<float> transformation, bool save_images = true);

    private:
        void prepare_depth_image();
        void prepare_gray_image();
        bool is_calibrated(const rs2::video_frame& depth, const rs2::video_frame& other, const rs2_extrinsics& depth_to_other);
        std::vector<rs2::vertex> PointCloudProcessing(const rs2::vertex* vertices, const size_t size);
        void filter_depth_edges(rs2_intrinsics depth_intrinsics, uint16_t* depth_image);
        bool points_to_image(const std::vector<rs2::vertex>& pc, rs2_intrinsics depth_intrinsics, uint16_t* depth_image);
        void filter_background_noise(rs2_intrinsics depth_intrinsics, uint16_t* depth_image);
        void subtract_local_center(rs2_intrinsics image_intrinsics, double* image);
        void calculate_distance_transform(rs2_intrinsics image_intrinsics, double* image);
        float calculate_objection_function(double* gray_image, uint16_t* depth_image, size_t image_size);
        const char * get_option_name(rs2_option option) const override;

    private:
        rs2::pointcloud _pc;
        depth_color_calib_options_st _options;
        float _depth_scale;
        rs2::frameset _frameset;
        double* _gray_image;
        uint16_t*   _original_depth_image;
        uint16_t*   _processed_depth_image;
        uint16_t*   _temp_depth_image;
        size_t      _temp_depth_image_size;
    };
    MAP_EXTENSION(RS2_EXTENSION_DEPTH_COLOR_CALIB, librealsense::depth_color_calib);

}