/* License: Apache 2.0. See LICENSE file in root directory.
Copyright(c) 2017 Intel Corporation. All Rights Reserved. */

#include "python.hpp"
#include "../include/librealsense2/rsutil.h"
#include "../include/librealsense2/hpp/rs_frame.hpp"
#include "../include/librealsense2/hpp/rs_utils.hpp"


void init_util(py::module &m) {
    /** rsutil.h **/
    m.def("rs2_project_point_to_pixel", [](const rs2_intrinsics& intrin, const std::array<float, 3>& point)->std::array<float, 2>
    {
        std::array<float, 2> pixel{};
        rs2_project_point_to_pixel(pixel.data(), &intrin, point.data());
        return pixel;
    }, "Given a point in 3D space, compute the corresponding pixel coordinates in an image with no distortion or forward distortion coefficients produced by the same camera",
       "intrin"_a, "point"_a);

    m.def("rs2_deproject_pixel_to_point", [](const rs2_intrinsics& intrin, const std::array<float, 2>& pixel, float depth)->std::array<float, 3>
    {
        std::array<float, 3> point{};
        rs2_deproject_pixel_to_point(point.data(), &intrin, pixel.data(), depth);
        return point;
    }, "Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera",
       "intrin"_a, "pixel"_a, "depth"_a);

    m.def("rs2_transform_point_to_point", [](const rs2_extrinsics& extrin, const std::array<float, 3>& from_point)->std::array<float, 3>
    {
        std::array<float, 3> to_point{};
        rs2_transform_point_to_point(to_point.data(), &extrin, from_point.data());
        return to_point;
    }, "Transform 3D coordinates relative to one sensor to 3D coordinates relative to another viewpoint",
       "extrin"_a, "from_point"_a);

    m.def("rs2_fov", [](const rs2_intrinsics& intrin)->std::array<float, 2>
    {
        std::array<float, 2> to_fow{};
        rs2_fov(&intrin, to_fow.data());
        return to_fow;
    }, "Calculate horizontal and vertical field of view, based on video intrinsics", "intrin"_a);

    m.def("depth_color_auto_calibrate", [](rs2::frameset frameset, struct rs2_extrinsics& extrin)->struct rs2_extrinsics
    {
        struct rs2_extrinsics out_extrin(extrin);
        rs2::depth_color_auto_calibrate(frameset, &out_extrin);
        return out_extrin;
    }, "Calculate extrinsics between depth and color frames", "frames"_a, "extrin"_a);

    m.def("create_extrinsics", [](const std::vector<float>& extrinsics_vector)->struct rs2_extrinsics
    {
        struct rs2_extrinsics out_extrin = rs2_create_extrinsics(extrinsics_vector.data(), extrinsics_vector.size());
        return out_extrin;
    }, "Create a rs2_extrinsics structure based on the given vector", "extrinsics_vector"_a);

    py::class_<rs2::depth_color_calib, rs2::options> depth_color_calib(m, "depth_color_calib", "Class for auto calibrating depth and color");
    depth_color_calib.def(py::init<>(), "To perform calibration set with frameset including depth and color frames.\n")
        .def("set_frameset", &rs2::depth_color_calib::set_frameset, "Set the frameset to calculate grade for. Must include depth and color images.", "frames"_a)
        .def("prepare_images", &rs2::depth_color_calib::prepare_images, "Filter images based on current options.")
        .def("calibrate", [](rs2::depth_color_calib& self)->struct rs2_extrinsics
        {
            struct rs2_extrinsics out_extrin;
            self.calibrate(&out_extrin);
            return out_extrin;
        }, "Calculate extrinsics between depth and color frames")
        .def("calculate_extrinsics_grade", &rs2::depth_color_calib::calculate_extrinsics_grade, "Calculate grade based on color image, depth image and extinsics transformation", "transformation"_a);

    /** end rsutil.h **/
}
