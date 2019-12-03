// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#ifndef LIBREALSENSE_RS2_UTILS_HPP
#define LIBREALSENSE_RS2_UTILS_HPP

#include "rs_types.hpp"
#include "../h/rs_utils.h"
#include "rs_options.hpp"


namespace rs2
{
    class depth_color_calib : public options
    {
    public:
        using options::supports;

        depth_color_calib()
        {
            rs2_error* e = nullptr;
            _dcc = std::shared_ptr<rs2_depth_color_calib>(rs2_create_depth_color_calibrator(&e),
                rs2_delete_depth_color_calibrator);
            options::operator=(_dcc);
            error::handle(e);
        }

        void set_frameset(rs2::frameset frameset)
        {
            rs2_error* e = nullptr;
            rs2_dcc_set_frameset(_dcc.get(), frameset, &e);
            error::handle(e);
        }

        void prepare_images()
        {
            rs2_error* e = nullptr;
            rs2_dcc_prepare_images(_dcc.get(), &e);
            error::handle(e);
        }

        void calibrate(struct rs2_extrinsics * extrin)
        {
            rs2_error* e = nullptr;
            rs2_dcc_calibrate(_dcc.get(), extrin, &e);
            error::handle(e);
        }

        operator rs2_options*() const { return (rs2_options*)get(); }

        float calculate_extrinsics_grade(std::vector<float> transformation)
        {
            rs2_error* e = nullptr;
            return rs2_dcc_calculate_grade(_dcc.get(), transformation, &e);
            error::handle(e);
        }
        rs2_depth_color_calib* get() const
        {
            return _dcc.get();
        }
        explicit operator std::shared_ptr<rs2_depth_color_calib>() const
        {
            return _dcc;
        }

        depth_color_calib(std::shared_ptr<rs2_depth_color_calib> dcc) : _dcc(dcc) {}

    private:
        std::shared_ptr<rs2_depth_color_calib> _dcc;
    };

    void depth_color_auto_calibrate(rs2::frameset frameset, struct rs2_extrinsics * extrin)
    {
        rs2_error* e = nullptr;
        rs2_depth_color_auto_calibrate(frameset, extrin, &e);
        error::handle(e);
    }
}
#endif // LIBREALSENSE_RS2_PROCESSING_HPP
