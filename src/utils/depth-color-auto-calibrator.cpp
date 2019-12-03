// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "depth-color-auto-calibrator.h"
#include <iostream>
#include <algorithm>
#include <librealsense2/rs.hpp>
#include "../include/librealsense2/rsutil.h"
#include "simplex.h"
#include <numeric>
#include "option.h"

#include "../core/streaming.h"
#include "context.h"
#include "stream.h"

using namespace std;

using namespace librealsense;

namespace librealsense
{
    const double SQRT_DBL_EPSILON = sqrt(std::numeric_limits<double>::epsilon());

    enum depth_color_calib_options
    {
        RS2_OPTION_DCC_LASER_DIST_CONT_TH = static_cast<rs2_option>(RS2_OPTION_COUNT + 0), /**< IR min threshold used by zero order filter */
        RS2_OPTION_DCC_LASER_BACKGROUND_TH = static_cast<rs2_option>(RS2_OPTION_COUNT + 1), /**< RTD high threshold used by zero order filter */
        RS2_OPTION_DCC_WIN_ROWS_EDGE = static_cast<rs2_option>(RS2_OPTION_COUNT + 2), /**< RTD low threshold used by zero order filter */
        RS2_OPTION_DCC_WIN_COLS_EDGE = static_cast<rs2_option>(RS2_OPTION_COUNT + 3), /**< baseline of camera used by zero order filter */
        RS2_OPTION_DCC_WIN_ROWS_DSF = static_cast<rs2_option>(RS2_OPTION_COUNT + 4), /**< RTD low threshold used by zero order filter */
        RS2_OPTION_DCC_WIN_COLS_DSF = static_cast<rs2_option>(RS2_OPTION_COUNT + 5), /**< baseline of camera used by zero order filter */
        RS2_OPTION_DCC_ALPHA = static_cast<rs2_option>(RS2_OPTION_COUNT + 6), /**< patch size used by zero order filter */
        RS2_OPTION_DCC_GAMMA = static_cast<rs2_option>(RS2_OPTION_COUNT + 7), /**< z max value used by zero order filter */
    };

    void SaveImage(string filename, char* image, size_t size)
    {
        ofstream fout(filename.c_str(), ios::binary);
        fout.write(image, size);
        fout.close();
    }

    template<typename T1, typename T2>
    void RGB2Gray(const T1* color, T2* gray, size_t size)
    {
        for (int i = 0; i < size; i++)
        {
            gray[i] = static_cast<int> (round( (color[i * 3] * 0.299 + color[i * 3 + 1] * 0.587 + color[i * 3 + 2] * 0.114) ));
        }
    }

    float3x3 rotation_x(float theta)
    {
        float3x3 rot;
        float sin_t = sinf(theta);
        float cos_t = cosf(theta);
        rot = { {1, 0, 0},
                {0, cos_t, -sin_t},
                {0, sin_t, cos_t} };
        return rot;
    }

    float3x3 rotation_y(float theta)
    {
        float3x3 rot;
        float sin_t = sinf(theta);
        float cos_t = cosf(theta);
        rot = { {cos_t, 0, sin_t},
                {0, 1, 0},
                {-sin_t, 0, cos_t} };
        return rot;
    }

    float3x3 rotation_z(float theta)
    {
        float3x3 rot;
        float sin_t = sinf(theta);
        float cos_t = cosf(theta);
        rot = { {cos_t, -sin_t, 0},
                {sin_t, cos_t, 0},
                {0, 0, 1} };
        return rot;
    }

    /// Convert orientation angles stored in rodrigues conventions to rotation matrix
    /// for details: http://mesh.brown.edu/en193s08-2003/notes/en193s08-rots.pdf
    //float3x3 calc_rotation_from_rodrigues_angles(const std::vector<double> rot)
    //{
    //    assert(3 == rot.size());
    //    float3x3 rot_mat{};

    //    double theta = sqrt(std::inner_product(rot.begin(), rot.end(), rot.begin(), 0.0));
    //    double r1 = rot[0], r2 = rot[1], r3 = rot[2];
    //    if (theta <= SQRT_DBL_EPSILON) // identityMatrix
    //    {
    //        rot_mat(0, 0) = rot_mat(1, 1) = rot_mat(2, 2) = 1.0;
    //        rot_mat(0, 1) = rot_mat(0, 2) = rot_mat(1, 0) = rot_mat(1, 2) = rot_mat(2, 0) = rot_mat(2, 1) = 0.0;
    //    }
    //    else
    //    {
    //        r1 /= theta;
    //        r2 /= theta;
    //        r3 /= theta;

    //        double c = cos(theta);
    //        double s = sin(theta);
    //        double g = 1 - c;

    //        rot_mat(0, 0) = float(c + g * r1 * r1);
    //        rot_mat(0, 1) = float(g * r1 * r2 - s * r3);
    //        rot_mat(0, 2) = float(g * r1 * r3 + s * r2);
    //        rot_mat(1, 0) = float(g * r2 * r1 + s * r3);
    //        rot_mat(1, 1) = float(c + g * r2 * r2);
    //        rot_mat(1, 2) = float(g * r2 * r3 - s * r1);
    //        rot_mat(2, 0) = float(g * r3 * r1 - s * r2);
    //        rot_mat(2, 1) = float(g * r3 * r2 + s * r1);
    //        rot_mat(2, 2) = float(c + g * r3 * r3);
    //    }

    //    return rot_mat;
    //}

    template<typename T>
    void ShrinkToDepthImage(rs2_intrinsics from_intrinsics, rs2_intrinsics to_intrinsics, T* gray_image)
    {
        int from_width(from_intrinsics.width), from_height(from_intrinsics.height);
        int to_width(to_intrinsics.width), to_height(to_intrinsics.height);

        double step_x((double)from_width / (double)to_width);
        double step_y((double)from_height / (double)to_height);
        T* to_img = gray_image;

        #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
        for (double y = 0; y < from_height; y += step_y)
        {
            size_t idx_y(static_cast<size_t>(y));
            for (double x = 0; x < from_width; x += step_x)
            {
                size_t idx_x(static_cast<size_t>(x));
                *to_img = gray_image[idx_y * from_width + idx_x];
                to_img++;
            }
        }
    }

    depth_color_calib::depth_color_calib(float depth_unit) :
        _gray_image(NULL),
        _original_depth_image(NULL),
        _processed_depth_image(NULL),
        _temp_depth_image(NULL)
    {
//        _options.laser_dist_cont_th
        string option_name(get_option_name(static_cast<rs2_option>(RS2_OPTION_DCC_LASER_DIST_CONT_TH)));
        auto laser_dist_cont_th = std::make_shared<ptr_option<float>>(
            0.0f,
            255.0f,
            1.f,
            float(LASER_DIST_CONT_TH),
            &_options.laser_dist_cont_th,
            option_name);
        register_option(static_cast<rs2_option>(RS2_OPTION_DCC_LASER_DIST_CONT_TH), laser_dist_cont_th);
        
        //        _options.laser_background_th
        option_name = get_option_name(static_cast<rs2_option>(RS2_OPTION_DCC_LASER_BACKGROUND_TH));
        auto laser_background_th = std::make_shared<ptr_option<uint16_t>>(
            0,
            50000,
            1,
            LASER_BACKGROUND_TH,
            &_options.laser_background_th,
            option_name);
        register_option(static_cast<rs2_option>(RS2_OPTION_DCC_LASER_BACKGROUND_TH), laser_background_th);

        //        _options.win_rows_edge
        option_name = get_option_name(static_cast<rs2_option>(RS2_OPTION_DCC_WIN_ROWS_EDGE));
        auto win_rows_edge = std::make_shared<ptr_option<size_t>>(
            0,
            100,
            1,
            WIN_ROWS_EDGE,
            &_options.win_rows_edge,
            option_name);
        register_option(static_cast<rs2_option>(RS2_OPTION_DCC_WIN_ROWS_EDGE), win_rows_edge);

        //        _options.win_cols_edge
        option_name = get_option_name(static_cast<rs2_option>(RS2_OPTION_DCC_WIN_COLS_EDGE));
        auto win_cols_edge = std::make_shared<ptr_option<size_t>>(
            0,
            100,
            1,
            WIN_COLS_EDGE,
            &_options.win_cols_edge,
            option_name);
        register_option(static_cast<rs2_option>(RS2_OPTION_DCC_WIN_COLS_EDGE), win_cols_edge);

        //        _options.win_rows
        option_name = get_option_name(static_cast<rs2_option>(RS2_OPTION_DCC_WIN_ROWS_DSF));
        auto win_rows_dsf = std::make_shared<ptr_option<size_t>>(
            0,
            100,
            1,
            WIN_ROWS_DSF,
            &_options.win_rows_dsf,
            option_name);
        register_option(static_cast<rs2_option>(RS2_OPTION_DCC_WIN_ROWS_DSF), win_rows_dsf);

        //        _options.win_cols
        option_name = get_option_name(static_cast<rs2_option>(RS2_OPTION_DCC_WIN_COLS_DSF));
        auto win_cols_dsf = std::make_shared<ptr_option<size_t>>(
            0,
            100,
            1,
            WIN_COLS_DSF,
            &_options.win_cols_dsf,
            option_name);
        register_option(static_cast<rs2_option>(RS2_OPTION_DCC_WIN_COLS_DSF), win_cols_dsf);

        //        _options.alpha
        option_name = get_option_name(static_cast<rs2_option>(RS2_OPTION_DCC_ALPHA));
        auto alpha = std::make_shared<ptr_option<double>>(
            0.0,
            1.0,
            0.01,
            ALPHA,
            &_options.alpha,
            option_name);
        register_option(static_cast<rs2_option>(RS2_OPTION_DCC_ALPHA), alpha);

        //        _options.gamma
        option_name = get_option_name(static_cast<rs2_option>(RS2_OPTION_DCC_GAMMA));
        auto gamma = std::make_shared<ptr_option<double>>(
            0.0,
            1.0,
            0.01,
            GAMMA,
            &_options.gamma,
            option_name);
        register_option(static_cast<rs2_option>(RS2_OPTION_DCC_GAMMA), gamma);
    }

    depth_color_calib::~depth_color_calib()
    {
        if (_gray_image)
            delete[] _gray_image;
        if (_original_depth_image)
            delete[] _original_depth_image;
        if (_processed_depth_image)
            delete[] _processed_depth_image;
        if (_temp_depth_image)
            delete[] _temp_depth_image;
    }

    float get_depth_scale(rs2::frame depth)
    {
        auto snr = ((frame_interface*)depth.get())->get_sensor().get();
        librealsense::depth_sensor* dss;
        float depth_scale;

        if (Is<librealsense::depth_sensor>(snr))
        {
            dss = As<librealsense::depth_sensor>(snr);
            depth_scale = dss->get_depth_scale();
        }
        else
        {
            throw runtime_error("depth frame is not from a librealsense::depth_sensor.");
        }
        return depth_scale;
    }

    void depth_color_calib::set_frameset(rs2::frameset frameset)
    {
        //_depth_scale = 0.001f;
        rs2::frame depth = frameset.get_depth_frame();
        rs2::frame color = frameset.get_color_frame();
        if (!depth)
        {
            throw runtime_error("No depth frame in frameset - needed for depth-color calibration.");
        }
        if (!color)
        {
            throw runtime_error("No color frame in frameset - needed for depth-color calibration.");
        }
        _depth_scale = get_depth_scale(depth);
        std::cout << "_depth_scale = " << _depth_scale << std::endl;
        _frameset = frameset;
    }

    void depth_color_calib::prepare_images()
    {
        prepare_depth_image();
        prepare_gray_image();
    }

    void depth_color_calib::prepare_depth_image()
    {
        if (!_frameset)
            throw runtime_error("must call set_frameset() before using prepare_depth_image()");

        rs2::frame depth = _frameset.get_depth_frame();
        rs2::frame color = _frameset.get_color_frame();
        rs2::video_stream_profile color_profile = color.get_profile().as<rs2::video_stream_profile>();
        rs2::video_stream_profile depth_profile = depth.get_profile().as<rs2::video_stream_profile>();

        rs2_intrinsics depth_intrinsics = depth_profile.get_intrinsics();
        rs2_intrinsics color_intrinsics = color_profile.get_intrinsics();

        _original_depth_image = new uint16_t[depth_intrinsics.width*depth_intrinsics.height];
        memcpy(_original_depth_image, depth.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
        _processed_depth_image = new uint16_t[depth_intrinsics.width*depth_intrinsics.height];
        memcpy(_processed_depth_image, depth.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
        
        SaveImage("depth_image0.bin", (char*)depth.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
        rs2::points pts = _pc.calculate(depth);
        auto vertices = pts.get_vertices();              // get vertices
        std::vector<rs2::vertex> DepthCP = PointCloudProcessing(vertices, pts.size());
        points_to_image(DepthCP, depth_intrinsics, _processed_depth_image);
        //filter_depth_edges(depth_intrinsics, (uint16_t*)depth.get_data());
        SaveImage("depth_image1.bin", (char*)_processed_depth_image, depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));

        filter_background_noise(depth_intrinsics, _processed_depth_image);
        SaveImage("depth_image2.bin", (char*)_processed_depth_image, depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
    }

    void depth_color_calib::prepare_gray_image()
    {
        if (!_frameset)
            throw runtime_error("must call set_frameset() before using prepare_gray_image()");

        rs2::frame depth = _frameset.get_depth_frame();
        rs2::frame color = _frameset.get_color_frame();

        rs2::video_stream_profile color_profile = color.get_profile().as<rs2::video_stream_profile>();
        rs2::video_stream_profile depth_profile = depth.get_profile().as<rs2::video_stream_profile>();

        rs2_intrinsics depth_intrinsics = depth_profile.get_intrinsics();
        rs2_intrinsics color_intrinsics = color_profile.get_intrinsics();

        _gray_image = new double[color_intrinsics.width*color_intrinsics.height](); //set to zeros
        SaveImage("color_image1.bin", (char*)(color.get_data()), color_intrinsics.width*color_intrinsics.height * 3 * sizeof(char));
        RGB2Gray((unsigned char*)color.get_data(), _gray_image, color_intrinsics.width*color_intrinsics.height);


        //std::cout << "Zero image" << std::endl;
        //double init_value(0);
        //std::fill_n(_gray_image, color_intrinsics.width*color_intrinsics.height, init_value);
        //init_value = 60;
        //std::fill_n(_gray_image + 300 * color_intrinsics.width, color_intrinsics.width * 10, init_value);
        //std::cout << "Zeroed image" << std::endl;


        SaveImage("gray_image1.bin", (char*)_gray_image, color_intrinsics.width*color_intrinsics.height * sizeof(double));
        subtract_local_center(color_intrinsics, _gray_image);
        SaveImage("gray_image2.bin", (char*)_gray_image, color_intrinsics.width*color_intrinsics.height * sizeof(double));
        calculate_distance_transform(color_intrinsics, _gray_image);
        std::cout << "Save gray_image3.bin" << std::endl;
        SaveImage("gray_image3.bin", (char*)_gray_image, color_intrinsics.width*color_intrinsics.height * sizeof(double));
        ShrinkToDepthImage(color_intrinsics, depth_intrinsics, _gray_image);
        SaveImage("gray_image4.bin", (char*)_gray_image, depth_intrinsics.width*depth_intrinsics.height * sizeof(double));
    }

    //bool depth_color_calib::calibrate(const rs2_frame* depth, const rs2_frame* color, rs2_extrinsics& fixed_depth_to_other)
    // bool depth_color_calib::calibrate(const rs2::frame depth, const rs2::frame color, rs2_extrinsics& fixed_depth_to_other)

    //bool depth_color_calib::prepare_(rs2::frameset frameset, rs2_extrinsics& fixed_depth_to_other)
    //{
    //    calculate_extrinsics_grade
    //}

    void depth_color_calib::calibrate(rs2_extrinsics* fixed_depth_to_other)
    {
        if (!_frameset)
            throw runtime_error("must call set_frameset() before using prepare_depth_image()");

        rs2::frame depth = _frameset.get_depth_frame();
        rs2::frame color = _frameset.get_color_frame();

        rs2::video_stream_profile color_profile = color.get_profile().as<rs2::video_stream_profile>();
        rs2::video_stream_profile depth_profile = depth.get_profile().as<rs2::video_stream_profile>();

        rs2_extrinsics depth_to_other = depth_profile.get_extrinsics_to(color_profile);

        std::vector<float> transformation(12);
        memcpy(&transformation[0], &depth_to_other, 12 * sizeof(float));
        std::function<float(std::vector<float>)> calc_grade_func = [this](std::vector<float> transformation) {
            return (-1)*(this->calculate_extrinsics_grade(transformation, false));
        };

        std::vector< std::vector<float> > x;
        std::vector<float> res_extrinsics = BT::Simplex(calc_grade_func, transformation, 1e-5f, x, 10000);

        memcpy(fixed_depth_to_other, res_extrinsics.data(), 12 * sizeof(float));
    }

//    bool depth_color_calib::old_calibrate(const rs2::frameset frameset, rs2_extrinsics& fixed_depth_to_other)
//    {
//        rs2::frame depth = frameset.get_depth_frame();
//        rs2::frame color = frameset.get_color_frame();
//        if (!depth)
//        {
//            cout << ("No depth frame - needed for depth-color calibration.") << endl;
//        }
//        if (!color)
//        {
//            cout << ("No color frame - needed for depth-color calibration.") << endl;
//        }
//
//        rs2_intrinsics depth_intrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
//        rs2_intrinsics color_intrinsics = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
//
//        _depth_scale = 0.001f;
//#if 1
//        //// DEBUG SECTION:
//        if (depth_intrinsics.width == 640 && depth_intrinsics.height == 480)
//        {
//            cout << ("read depth image") << endl;
//            // Debug : read image from file:
//            FILE* fin = fopen("C:\\projects\\IVCam2AutoCalib\\Data\\AutoCalibData\\Depth\\iv2_viewpoint_6_Z_GrayScale_640x480_0006_flipped_lr.bin", "rb");
//            //FILE* fin = fopen("C:\\projects\\librealsense\\build\\tools\\depth_color_auto_calibrate\\depth_image_original.bin", "rb");
//            fread((void*)depth.get_data(), sizeof(uint16_t), depth_intrinsics.width*depth_intrinsics.height, fin);
//            fclose(fin);
//        }
//    
//        if (color_intrinsics.width == 1920 && color_intrinsics.height == 1080)
//        {
//            cout << ("read color image") << endl;
//            // Debug : read image from file:
//            FILE* fin = fopen("C:\\projects\\IVCam2AutoCalib\\Data\\AutoCalibData\\RGB\\rgb_viewpoint_6_YUY2_YUY2_1920x1080_0006_a.bin", "rb");
//            //FILE* fin = fopen("C:\\projects\\librealsense\\build\\tools\\depth_color_auto_calibrate\\color_image_original.bin", "rb");
//            fread((void*)color.get_data(), sizeof(uint8_t), color_intrinsics.width*color_intrinsics.height*3, fin);
//            fclose(fin);
//        }
//#endif
//      ////////////////////
//        SaveImage("depth_image0.bin", (char*)depth.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
//
//        rs2::points pts = _pc.calculate(depth);
//        auto vertices = pts.get_vertices();              // get vertices
//        std::vector<rs2::vertex> DepthCP = PointCloudProcessing(vertices, pts.size());
//        points_to_image(DepthCP, depth_intrinsics, (uint16_t*)depth.get_data());
//        //filter_depth_edges(depth_intrinsics, (uint16_t*)depth.get_data());
//        SaveImage("depth_image1.bin", (char*)depth.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
//
//        filter_background_noise(depth_intrinsics, (uint16_t*)depth.get_data());
//        SaveImage("depth_image2.bin", (char*)depth.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
//
//        rs2::video_stream_profile color_profile = color.get_profile().as<rs2::video_stream_profile>();
//        rs2::video_stream_profile depth_profile = depth.get_profile().as<rs2::video_stream_profile>();
//        rs2_extrinsics depth_to_other = depth_profile.get_extrinsics_to(color_profile);
//
//        if (true)
//        {
//            //float debug_trn[] = { -82.2412f * _depth_scale, -0.356379f * _depth_scale, -12.4257f * _depth_scale };
//
//            //float debug_rot[] = { 0.998891466751534, 0.0465640766472368, -0.00690104462818514,
//            //                      -0.0464540842990243, 0.998802892380759, 0.0153231923491409,
//            //                     0.00760629363810577, -0.0149856243720402, 0.999858777707768 }; //NOISY
//
//            //float debug_rot[] = { 0.998891466751534, -0.0464540842990243, 0.00760629363810577,
//            //                       0.0465640766472368, 0.998802892380759, -0.0149856243720402,
//            //                      -0.00690104462818514, 0.0153231923491409, 0.999858777707768 };
//
//            //float debug_rot[] = { 0.9999952, 0.0019251, -0.0024400,
//            //                     -0.0019467, 0.9999588, -0.0088643,
//            //                      0.0024228, 0.0088690, 0.9999577 };    // +0.5 degree in X
//
//            //float debug_rot[] = { 0.9999952, 0.0019251, -0.0024400,
//            //                     -0.0020505, 0.9986202, -0.0524734,
//            //                      0.0023356, 0.0524782, 0.9986193 };   // +3 degrees
//
//
//            //float debug_rot[] = { 0.999995172, 0.00192511710, -0.00243999087,
//            //           -0.00192545913, 0.999998152, -0.000137818133,
//            //          0.00243972102, 0.000142515564, 0.999997020 }; // original
//
//            float3x3 aa = { {0, 1, 2},
//                            {3, 4, 5},
//                            {6, 7, 8} };
//            float3x3 bb = { {10, 11, 12},
//                            {13, 14, 15},
//                            {16, 17, 18} };
//            float3x3 cc = aa * bb;
//
//            std::vector<double> rot = { 0, 0, 3 };
//            float3x3 origin_rot = { {0.999995172, 0.00192511710, -0.00243999087},
//                                    {-0.00192545913, 0.999998152, -0.000137818133},
//                                    {0.00243972102, 0.000142515564, 0.999997020} };
//            // float3x3 rot_err = calc_rotation_from_rodrigues_angles(rot);
//            //float3x3 rot_err = rotation_x(0.0f) * rotation_y(0.0f) * rotation_z(deg2rad(3.0f));
//            float3x3 rot_err = rotation_z(deg2rad(3.0f)) * rotation_y(0.0f) * rotation_x(0.0f);
//            float3x3 debug_rot = origin_rot * rot_err;
//
//
//
//            float debug_trn[] = { 0.0146882981, 0.000314516481, 0.000251281104 };   // original
//
//            memcpy(depth_to_other.rotation, &debug_rot, 9 * sizeof(float));
//            // memcpy(depth_to_other.translation, debug_trn, 3 * sizeof(float));
//            depth.get_profile().as<rs2::video_stream_profile>().register_extrinsics_to(color_profile, depth_to_other);
//        }
//
//        rs2::align align_to(RS2_STREAM_COLOR);
//
//        rs2::frameset fs2 = align_to.process(frameset);
//        rs2::frame depth2 = fs2.get_depth_frame();
//        rs2_intrinsics depth_intrinsics2 = depth2.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
//        SaveImage("depth_aligned1.bin", (char*)depth2.get_data(), depth_intrinsics2.width*depth_intrinsics2.height * sizeof(uint16_t));
//        uint16_t* depth_image = new uint16_t[depth_intrinsics2.width*depth_intrinsics2.height](); //set to zeros
//        memcpy(depth_image, depth2.get_data(), depth_intrinsics2.width*depth_intrinsics2.height * sizeof(uint16_t));
//        ShrinkToDepthImage(depth_intrinsics2, depth_intrinsics, depth_image);
//        SaveImage("depth_aligned2.bin", (char*)depth_image, depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
//
//
//        double* gray_image = new double[color_intrinsics.width*color_intrinsics.height](); //set to zeros
//        SaveImage("color_image1.bin", (char*)(color.get_data()), color_intrinsics.width*color_intrinsics.height * 3 * sizeof(char));
//        RGB2Gray((unsigned char*)color.get_data(), gray_image, color_intrinsics.width*color_intrinsics.height);
//        SaveImage("gray_image1.bin", (char*)gray_image, color_intrinsics.width*color_intrinsics.height * sizeof(double));
//        subtract_local_center(color_intrinsics, gray_image);
//        SaveImage("gray_image2.bin", (char*)gray_image, color_intrinsics.width*color_intrinsics.height * sizeof(double));
//        calculate_distance_transform(color_intrinsics, gray_image);
//        SaveImage("gray_image3.bin", (char*)gray_image, color_intrinsics.width*color_intrinsics.height * sizeof(double));
//        ShrinkToDepthImage(color_intrinsics, depth_intrinsics, gray_image);
//        SaveImage("gray_image4.bin", (char*)gray_image, depth_intrinsics.width*depth_intrinsics.height * sizeof(double));
//        double jc = calculate_objection_function(gray_image, depth_image, depth_intrinsics.width*depth_intrinsics.height);
//        cout << "NOISY: The objection function J given the NOISY input calibration (R,T) is: " << jc << endl;
//        cout << "jc = " << jc << endl;
//
//        _frameset = frameset;
//        _gray_image = gray_image;
//        std::vector<float> transformation(12);
//        memcpy(&transformation[0], &depth_to_other, 12 * sizeof(float));
//
//        // Testing for GT score:
//        if (true)
//        {
//
//            //float debug_trn[] = { -82.2412f * _depth_scale, -0.356379f * _depth_scale, -12.4257f * _depth_scale };
//
//            //float debug_rot[] = { 0.99928, 0.029116, -0.024352,
//            //                      -0.029176, 0.99957, -0.0021285,
//            //                     0.024279, 0.0028374, 0.9997 }; // GT
//
//            float debug_rot[] = { 0.999995172, 0.00192511710, -0.00243999087,
//                       -0.00192545913, 0.999998152, -0.000137818133,
//                      0.00243972102, 0.000142515564, 0.999997020 }; // original
//
//            float debug_trn[] = { 0.0146882981, 0.000314516481, 0.000251281104 };   // original
//
//            memcpy(depth_to_other.rotation, debug_rot, 9 * sizeof(float));
//            //memcpy(depth_to_other.translation, debug_trn, 3 * sizeof(float));
//
//            std::vector<float> gt_transformation(12);
//            memcpy(&gt_transformation[0], &depth_to_other, 12 * sizeof(float));
//
//            jc = calculate_extrinsics_grade(gt_transformation);
//            cout << "func: GT: The objection function J given the GT input calibration (R,T) is: " << jc << endl;
//        }
//
//
//        std::function<float(std::vector<float>)> calc_grade_func = [this](std::vector<float> transformation) {
//            return (-1)*(this->calculate_extrinsics_grade(transformation));
//        };
//
//        std::vector< std::vector<float> > x;
//        //std::vector<float> res_extrinsics = BT::Simplex(calc_grade_func, transformation, 1e-5f, x, 10000);
//        //jc = calculate_extrinsics_grade(res_extrinsics);
//        //cout << "results: The objection function J given the GT input calibration (R,T) is: " << jc << endl;
//
//        delete[] depth_image;
//
//        return true;
//    }

    float depth_color_calib::calculate_extrinsics_grade(std::vector<float> transformation, bool save_images)
    {
        rs2::frame depth = _frameset.get_depth_frame();
        rs2::frame color = _frameset.get_color_frame();
        if (!depth)
        {
            throw runtime_error("No depth frame in frameset - needed for depth-color calibration.");
        }
        if (!color)
        {
            throw runtime_error("No color frame in frameset - needed for depth-color calibration.");
        }

        rs2::video_stream_profile depth_profile = depth.get_profile().as<rs2::video_stream_profile>();
        rs2::video_stream_profile color_profile = color.get_profile().as<rs2::video_stream_profile>();
        rs2_extrinsics depth_to_other = depth_profile.get_extrinsics_to(color_profile);
        memcpy(&depth_to_other, &transformation[0], 12 * sizeof(float));
        depth.get_profile().as<rs2::video_stream_profile>().register_extrinsics_to(color_profile, depth_to_other);

        rs2_intrinsics depth_intrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        memcpy((uint16_t*)depth.get_data(), _processed_depth_image, depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));

        rs2::align align_to(RS2_STREAM_COLOR);
        rs2::frameset fs2 = align_to.process(_frameset);

        rs2::frame depth2 = fs2.get_depth_frame();
        rs2_intrinsics depth_intrinsics2 = depth2.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        //SaveImage("depth_aligned1.bin", (char*)depth2.get_data(), depth_intrinsics2.width*depth_intrinsics2.height * sizeof(uint16_t));
        memcpy((uint16_t*)depth.get_data(), _original_depth_image, depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));

        size_t depth_image_size(depth_intrinsics2.width*depth_intrinsics2.height);
        if (_temp_depth_image_size != depth_image_size)
        {
            if (_temp_depth_image != NULL)
            {
                delete[] _temp_depth_image;
            }
            _temp_depth_image = new uint16_t[depth_image_size](); //set to zeros
            _temp_depth_image_size = depth_image_size;
        }
        memcpy(_temp_depth_image, depth2.get_data(), depth_intrinsics2.width*depth_intrinsics2.height * sizeof(uint16_t));
        ShrinkToDepthImage(depth_intrinsics2, depth_intrinsics, _temp_depth_image);
        if (save_images)
            SaveImage("depth_aligned2.bin", (char*)_temp_depth_image, depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));

        double jc = calculate_objection_function(_gray_image, _temp_depth_image, depth_intrinsics.width*depth_intrinsics.height);
        //delete[] depth_image;
        std::cout << "jc: " << jc << std::endl;
        return static_cast<float>(jc);
    }

    float depth_color_calib::calculate_objection_function(double* gray_image, uint16_t* depth_image, size_t image_size)
    {
        double* g_img = gray_image;
        uint16_t* d_img = depth_image;
        double max_gray(0), max_depth(0);
        double val(0);
        for (size_t idx = 0; idx < image_size; idx++)
        {
            val += (*g_img) * (*d_img);
            max_gray = max(max_gray, *g_img);
            max_depth = max(max_depth, (double)(*d_img));
            g_img++;
            d_img++;
        }
        val /= (max_gray * max_depth);
        return static_cast<float>(val);
    }

    void depth_color_calib::calculate_distance_transform(rs2_intrinsics image_intrinsics, double* image)
    {
        int width(image_intrinsics.width), height(image_intrinsics.height);
        size_t win_rows(_options.win_rows_dsf), win_cols(_options.win_cols_dsf);
        size_t hcols(win_cols / 2);
        size_t hrows(win_rows / 2);
        double* temp_image = new double[width*height]; //set to zeros
        memcpy(temp_image, image, width*height * sizeof(double));
        for (size_t y = 0; y < height - win_rows; y++)
        {
            for (size_t x = 0; x < width - win_cols; x++)
            {
                size_t iidx(y*width + x);
                double max_val(-1);

                //size_t max_idx[2];
                size_t center_idx = (y + hrows) * width + (x + hcols);
                double tpix(temp_image[center_idx]);
                double* pix(image + center_idx);
                size_t min_dist(max(hcols, hrows));
                for (int yy = 0; yy < win_rows; yy++)
                {
                    for (int xx = 0; xx < win_cols; xx++)
                    {
                        size_t idx_dist = max(abs(xx - (int)hcols), abs(yy - (int)hrows));
                        if ((temp_image[iidx] > max_val) || (temp_image[iidx] == max_val && idx_dist < min_dist))
                        {
                            max_val = temp_image[iidx];
                            min_dist = idx_dist;
                            //max_idx[0] = xx;
                            //max_idx[1] = yy;
                        }
                        iidx++;
                    }
                    iidx += (width - win_cols);
                }
                //D(i, j) = Params.Alpha * E(i, j) + (1 - Params.Alpha)* MaxLocalE * Params.Gamma ^ (max(abs(x - 2), abs(y - 2)));
                //*pix = _options.alpha * tpix + (1 - _options.alpha)*max_val * pow(_options.gamma, max(abs(max_idx[0] - (double)hcols), abs(max_idx[1] - (double)hrows)));
                *pix = _options.alpha * tpix + (1 - _options.alpha)*max_val * pow(_options.gamma, (double)min_dist);
            }
        }
        delete[] temp_image;
        std::cout << "Done phase 1" << std::endl;
    }

    //E = colfilt( GrayImg,[ Params.WinRows, Params.WinCols ],'sliding',fun);
    void depth_color_calib::subtract_local_center(rs2_intrinsics image_intrinsics, double* image)
    {
        int width(image_intrinsics.width), height(image_intrinsics.height);
        size_t win_rows(_options.win_rows_edge), win_cols(_options.win_cols_edge);
        size_t hcols(win_rows / 2);
        size_t hrows(win_cols / 2);
        double* temp_image = new double[width*height]; //set to zeros
        memcpy(temp_image, image, width*height * sizeof(double));
        for (size_t y = 0; y < height - win_rows; y++)
        {
            for (size_t x = 0; x < width - win_cols; x++)
            {
                size_t iidx(y*width + x);
                double max_val(0);
                size_t center_idx = (y + hrows) * width + (x + hcols);
                double* tpix(temp_image + center_idx);
                double* pix(image + center_idx);
                for (size_t yy = 0; yy < win_rows; yy++)
                {
                    for (size_t xx = 0; xx < win_cols; xx++)
                    {
                        max_val = std::max(max_val, std::abs(*tpix - temp_image[iidx]));
                        iidx++;
                    }
                    iidx += (width - win_cols);
                }
                *pix = max_val;
            }
        }
        delete[] temp_image;
    }

    void depth_color_calib::filter_background_noise(rs2_intrinsics depth_intrinsics, uint16_t* depth_image)
    {
        uint16_t laser_background_th_m = _options.laser_background_th; // Filter noise in bakcground

        uint16_t* i_img_end(depth_image + depth_intrinsics.width*depth_intrinsics.height);
        for (uint16_t* i_img = depth_image; i_img < i_img_end; i_img++)
        {
            if (*i_img >= laser_background_th_m)
            {
                *i_img = 0;
            }
        }
    }

    bool depth_color_calib::points_to_image(const std::vector<rs2::vertex>& pc, rs2_intrinsics depth_intrinsics, uint16_t* depth_image)
    {
        memset(depth_image, 0, depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
        for (std::vector<rs2::vertex>::const_iterator i_pc = pc.begin(); i_pc != pc.end(); i_pc++)
        {
            float pixel[2];
            rs2_project_point_to_pixel(pixel, &depth_intrinsics, *i_pc);
            depth_image[int(pixel[1]) * depth_intrinsics.width + int(pixel[0])] = (uint16_t)(i_pc->z / _depth_scale);
        }
        return true;
    }

    void depth_color_calib::filter_depth_edges(rs2_intrinsics depth_intrinsics, uint16_t* depth_image)
    {
        double laser_dist_cont_th_m = pow(_options.laser_dist_cont_th, 2.0) * 0.125;
        size_t size(depth_intrinsics.height * depth_intrinsics.width);

        std::vector<size_t> indices_to_remove;
        indices_to_remove.reserve(size);

        for (size_t y = 0; y < depth_intrinsics.height; y++)
        {
            double left(-1), right(-1);
            for (size_t x = 1; x < depth_intrinsics.width-1; x++)
            {
                size_t idx = y * depth_intrinsics.width + x;
                if (depth_image[idx] == 0)
                    continue;

                // TODO: if left pixel == 0 need to take closest pixel from the left whos not 0. Same for right pixel.
                double left = abs(double(depth_image[idx - 1]) - double(depth_image[idx]));
                double right = abs(double(depth_image[idx + 1]) - double(depth_image[idx]));
                if (max(left, right) < laser_dist_cont_th_m)
                {
                    indices_to_remove.push_back(idx);
                }
            }
        }
        for (size_t idx : indices_to_remove)
        {
            depth_image[idx] = 0;
        }
    }

    std::vector<rs2::vertex> depth_color_calib::PointCloudProcessing(const rs2::vertex* vertices, const size_t size)
    {
        std::vector<rs2::vertex> pc;
        std::vector<rs2::vertex> X;
        pc.reserve(size);
        X.reserve(size);

        // Remove invalid depth points:
        for (const rs2::vertex* v_iter(vertices); v_iter < vertices + size; v_iter++)
        {
            if (v_iter->z != 0)
            {
                pc.push_back(*v_iter);
                //X.push_back(*v_iter);
            }
        }
        std::cout << "num points > 0: " << pc.size() << std::endl;
        //return X;

        double laser_dist_cont_th_m = pow(_options.laser_dist_cont_th, 2.0) * _depth_scale * 0.125;
        double laser_background_th_m = _options.laser_background_th * _depth_scale; // Filter noise in bakcground
        std::cout << "laser_dist_cont_th_m = " << laser_dist_cont_th_m << " = " << _options.laser_dist_cont_th << "^2 * " << _depth_scale  << " * 0.125 " << std::endl;
        // Leave only points with difference greater then "_options.laser_dist_cont_th" from neighbours.
        std::vector<int> indices;
        std::vector<double> diff_vals;
        //int count(0);
        for (int idx(1); idx < pc.size() - 1; idx++)
        {
            double left = pc[idx - 1].z - pc[idx].z;
            double right = pc[idx + 1].z - pc[idx].z;
            diff_vals.push_back(max(left, right));
            //if (pc[idx].z > laser_background_th_m)
            //{
            //    count++;
            //}
            if (/*(pc[idx].z < laser_background_th_m) && */ (max(left, right) > laser_dist_cont_th_m))
            {
                X.push_back(pc[idx]);
                indices.push_back(idx);
            }
        }
        X.resize(X.size());
        ofstream fout("diff_vals.txt");
        for (std::vector<double>::iterator wi = diff_vals.begin(); wi != diff_vals.end(); wi++)
        {
            fout << *wi << endl;
        }
        fout.close();
        
        return X;
    }

    const char * depth_color_calib::get_option_name(rs2_option option) const
    {
        switch (option)
        {
        case depth_color_calib_options::RS2_OPTION_DCC_LASER_DIST_CONT_TH:
            return "laser dist cont th";
        case depth_color_calib_options::RS2_OPTION_DCC_LASER_BACKGROUND_TH:
            return "laser background th";
        case depth_color_calib_options::RS2_OPTION_DCC_WIN_ROWS_EDGE:
            return "win rows edge";
        case depth_color_calib_options::RS2_OPTION_DCC_WIN_COLS_EDGE:
            return "win cols edge";
        case depth_color_calib_options::RS2_OPTION_DCC_WIN_ROWS_DSF:
            return "win rows dsf";
        case depth_color_calib_options::RS2_OPTION_DCC_WIN_COLS_DSF:
            return "win cols dsf";
        case depth_color_calib_options::RS2_OPTION_DCC_ALPHA:
            return "alpha";
        case depth_color_calib_options::RS2_OPTION_DCC_GAMMA:
            return "gamma";
        }

        return options_container::get_option_name(option);
    }
}