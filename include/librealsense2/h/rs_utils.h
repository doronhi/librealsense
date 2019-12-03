#ifndef LIBREALSENSE_RS2_UTILS_H
#define LIBREALSENSE_RS2_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs_types.h"
#include "rs_sensor.h"

    typedef struct rs2_depth_color_calib rs2_depth_color_calib;

    rs2_depth_color_calib* rs2_create_depth_color_calibrator(rs2_error** error);
    void rs2_delete_depth_color_calibrator(rs2_depth_color_calib* rdcc);
    void rs2_dcc_set_frameset(rs2_depth_color_calib* rdcc, rs2::frameset frameset, rs2_error** error);
    void rs2_dcc_prepare_images(rs2_depth_color_calib* rdcc, rs2_error** error);
    float rs2_dcc_calculate_grade(rs2_depth_color_calib* rdcc, std::vector<float> transformation, rs2_error** error);
    void rs2_dcc_calibrate(rs2_depth_color_calib* rdcc, struct rs2_extrinsics * extrin, rs2_error** error);

    void rs2_depth_color_auto_calibrate(rs2::frameset frameset, struct rs2_extrinsics * extrin, rs2_error** error);
#ifdef __cplusplus
}
#endif
#endif
