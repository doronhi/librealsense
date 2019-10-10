// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "depth-color-auto-calibrator.h"
#include <iostream>
#include <algorithm>
#include <librealsense2/rs.hpp>
#include "../include/librealsense2/rsutil.h"
#include "simplex.h"


using namespace std;

namespace librealsense
{
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

    template<typename T>
    void ShrinkToDepthImage(rs2_intrinsics from_intrinsics, rs2_intrinsics to_intrinsics, T* gray_image)
    {
        int from_width(from_intrinsics.width), from_height(from_intrinsics.height);
        int to_width(to_intrinsics.width), to_height(to_intrinsics.height);

        double step_x((double)from_width / (double)to_width);
        double step_y((double)from_height / (double)to_height);
        T* to_img = gray_image;
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


    depth_color_calib::depth_color_calib(float depth_unit)
    {
//        _options.laser_dist_cont_th
    }

    depth_color_calib::~depth_color_calib()
    {
    }

    bool depth_color_calib::calibrate(const rs2::frameset frameset, rs2_extrinsics& fixed_depth_to_other)
    {
        rs2::frame depth = frameset.get_depth_frame();
        rs2::frame color = frameset.get_color_frame();
        if (!depth)
        {
            cout << ("No depth frame - needed for depth-color calibration.") << endl;
        }
        if (!color)
        {
            cout << ("No color frame - needed for depth-color calibration.") << endl;
        }

        rs2_intrinsics depth_intrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        rs2_intrinsics color_intrinsics = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        _depth_scale = 0.001f;
#if 1
        //// DEBUG SECTION:
        if (depth_intrinsics.width == 640 && depth_intrinsics.height == 480)
        {
            cout << ("read depth image") << endl;
            // Debug : read image from file:
            //FILE* fin = fopen("C:\\projects\\IVCam2AutoCalib\\Data\\AutoCalibData\\Depth\\iv2_viewpoint_6_Z_GrayScale_640x480_0006_flipped_lr.bin", "rb");
            FILE* fin = fopen("C:\\projects\\librealsense\\build\\tools\\depth_color_auto_calibrate\\depth_image_original.bin", "rb");
            fread((void*)depth.get_data(), sizeof(uint16_t), depth_intrinsics.width*depth_intrinsics.height, fin);
            fclose(fin);
        }
    
        if (color_intrinsics.width == 1920 && color_intrinsics.height == 1080)
        {
            cout << ("read color image") << endl;
            // Debug : read image from file:
            //FILE* fin = fopen("C:\\projects\\IVCam2AutoCalib\\Data\\AutoCalibData\\RGB\\rgb_viewpoint_6_YUY2_YUY2_1920x1080_0006_a.bin", "rb");
            FILE* fin = fopen("C:\\projects\\librealsense\\build\\tools\\depth_color_auto_calibrate\\color_image_original.bin", "rb");
            fread((void*)color.get_data(), sizeof(uint8_t), color_intrinsics.width*color_intrinsics.height*3, fin);
            fclose(fin);
        }
#endif
      ////////////////////
        rs2::points pts = _pc.calculate(depth);
        auto vertices = pts.get_vertices();              // get vertices
        std::vector<rs2::vertex> DepthCP = PointCloudProcessing(vertices, pts.size());

        SaveImage("depth_image0.bin", (char*)depth.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));

        points_to_image(DepthCP, depth_intrinsics, (uint16_t*)depth.get_data());

        SaveImage("depth_image1.bin", (char*)depth.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));

        FilterBackgroundNoise(depth_intrinsics, (uint16_t*)depth.get_data());
         SaveImage("depth_image2.bin", (char*)depth.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));

        rs2::video_stream_profile color_profile = color.get_profile().as<rs2::video_stream_profile>();
        rs2::video_stream_profile depth_profile = depth.get_profile().as<rs2::video_stream_profile>();
        rs2_extrinsics depth_to_other = depth_profile.get_extrinsics_to(color_profile);

        if (true)
        {
            //float debug_trn[] = { -82.2412f * _depth_scale, -0.356379f * _depth_scale, -12.4257f * _depth_scale };

            //float debug_rot[] = { 0.998891466751534, 0.0465640766472368, -0.00690104462818514,
            //                      -0.0464540842990243, 0.998802892380759, 0.0153231923491409,
            //                     0.00760629363810577, -0.0149856243720402, 0.999858777707768 }; //NOISY

            //float debug_rot[] = { 0.998891466751534, -0.0464540842990243, 0.00760629363810577,
            //                       0.0465640766472368, 0.998802892380759, -0.0149856243720402,
            //                      -0.00690104462818514, 0.0153231923491409, 0.999858777707768 };

            //float debug_rot[] = { 0.9999952, 0.0019251, -0.0024400,
            //                     -0.0019467, 0.9999588, -0.0088643,
            //                      0.0024228, 0.0088690, 0.9999577 };    // +0.5 degree in X

            float debug_rot[] = { 0.9999952, 0.0019251, -0.0024400,
                                 -0.0020505, 0.9986202, -0.0524734,
                                  0.0023356, 0.0524782, 0.9986193 };   // +3 degrees

            float debug_trn[] = { 0.0146882981, 0.000314516481, 0.000251281104 };   // original

            memcpy(depth_to_other.rotation, debug_rot, 9 * sizeof(float));
            // memcpy(depth_to_other.translation, debug_trn, 3 * sizeof(float));
        }
        depth.get_profile().as<rs2::video_stream_profile>().register_extrinsics_to(color_profile, depth_to_other);

        rs2::align align_to(RS2_STREAM_COLOR);

        rs2::frameset fs2 = align_to.process(frameset);
        rs2::frame depth2 = fs2.get_depth_frame();
        rs2_intrinsics depth_intrinsics2 = depth2.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        SaveImage("depth_aligned1.bin", (char*)depth2.get_data(), depth_intrinsics2.width*depth_intrinsics2.height * sizeof(uint16_t));
        uint16_t* depth_image = new uint16_t[depth_intrinsics2.width*depth_intrinsics2.height](); //set to zeros
        memcpy(depth_image, depth2.get_data(), depth_intrinsics2.width*depth_intrinsics2.height * sizeof(uint16_t));
        ShrinkToDepthImage(depth_intrinsics2, depth_intrinsics, depth_image);
        SaveImage("depth_aligned2.bin", (char*)depth_image, depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));


        double* gray_image = new double[color_intrinsics.width*color_intrinsics.height](); //set to zeros
        SaveImage("color_image1.bin", (char*)(color.get_data()), color_intrinsics.width*color_intrinsics.height * 3 * sizeof(char));
        RGB2Gray((unsigned char*)color.get_data(), gray_image, color_intrinsics.width*color_intrinsics.height);
        SaveImage("gray_image1.bin", (char*)gray_image, color_intrinsics.width*color_intrinsics.height * sizeof(double));
        SubtractLocalCenter(color_intrinsics, gray_image);
        SaveImage("gray_image2.bin", (char*)gray_image, color_intrinsics.width*color_intrinsics.height * sizeof(double));
        CalculateDistanceTransform(color_intrinsics, gray_image);
        SaveImage("gray_image3.bin", (char*)gray_image, color_intrinsics.width*color_intrinsics.height * sizeof(double));
        ShrinkToDepthImage(color_intrinsics, depth_intrinsics, gray_image);
        SaveImage("gray_image4.bin", (char*)gray_image, depth_intrinsics.width*depth_intrinsics.height * sizeof(double));
        double jc = CalculateObjectionFunction(gray_image, depth_image, depth_intrinsics.width*depth_intrinsics.height);
        cout << "NOISY: The objection function J given the NOISY input calibration (R,T) is: " << jc << endl;
        cout << "jc = " << jc << endl;

        _frameset = frameset;
        _gray_image = gray_image;
        std::vector<float> transformation(12);
        memcpy(&transformation[0], &depth_to_other, 12 * sizeof(float));

        // Testing for GT score:
        if (true)
        {

            //float debug_trn[] = { -82.2412f * _depth_scale, -0.356379f * _depth_scale, -12.4257f * _depth_scale };

            //float debug_rot[] = { 0.99928, 0.029116, -0.024352,
            //                      -0.029176, 0.99957, -0.0021285,
            //                     0.024279, 0.0028374, 0.9997 }; // GT

            float debug_rot[] = { 0.999995172, 0.00192511710, -0.00243999087,
                       -0.00192545913, 0.999998152, -0.000137818133,
                      0.00243972102, 0.000142515564, 0.999997020 }; // original

            float debug_trn[] = { 0.0146882981, 0.000314516481, 0.000251281104 };   // original

            memcpy(depth_to_other.rotation, debug_rot, 9 * sizeof(float));
            //memcpy(depth_to_other.translation, debug_trn, 3 * sizeof(float));

            std::vector<float> gt_transformation(12);
            memcpy(&gt_transformation[0], &depth_to_other, 12 * sizeof(float));

            jc = CalculateExtrinsicsGrade(gt_transformation);
            cout << "func: GT: The objection function J given the GT input calibration (R,T) is: " << jc << endl;
        }


        std::function<float(std::vector<float>)> calc_grade_func = [this](std::vector<float> transformation) {
            return (-1)*(this->CalculateExtrinsicsGrade(transformation));
        };

        std::vector< std::vector<float> > x;
        std::vector<float> res_extrinsics = BT::Simplex(calc_grade_func, transformation, 1e-8f, x, 10000);
        jc = CalculateExtrinsicsGrade(res_extrinsics);
        cout << "results: The objection function J given the GT input calibration (R,T) is: " << jc << endl;

        delete[] depth_image;

        return true;
    }

    float depth_color_calib::CalculateExtrinsicsGrade(std::vector<float> transformation)
    {
        rs2::frame depth = _frameset.get_depth_frame();
        rs2::frame color = _frameset.get_color_frame();
        rs2::video_stream_profile depth_profile = depth.get_profile().as<rs2::video_stream_profile>();
        rs2::video_stream_profile color_profile = color.get_profile().as<rs2::video_stream_profile>();
        rs2_extrinsics depth_to_other = depth_profile.get_extrinsics_to(color_profile);
        memcpy(&depth_to_other, &transformation[0], 12 * sizeof(float));
        depth.get_profile().as<rs2::video_stream_profile>().register_extrinsics_to(color_profile, depth_to_other);

        rs2::align align_to(RS2_STREAM_COLOR);
        rs2::frameset fs2 = align_to.process(_frameset);

        rs2::frame depth2 = fs2.get_depth_frame();
        rs2_intrinsics depth_intrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        rs2_intrinsics depth_intrinsics2 = depth2.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        uint16_t* depth_image = new uint16_t[depth_intrinsics2.width*depth_intrinsics2.height](); //set to zeros
        memcpy(depth_image, depth2.get_data(), depth_intrinsics2.width*depth_intrinsics2.height * sizeof(uint16_t));
        ShrinkToDepthImage(depth_intrinsics2, depth_intrinsics, depth_image);
        double jc = CalculateObjectionFunction(_gray_image, depth_image, depth_intrinsics.width*depth_intrinsics.height);
        delete[] depth_image;
        // std::cout << "jc: " << jc << std::endl;
        return static_cast<float>(jc);
    }

    float depth_color_calib::CalculateObjectionFunction(double* gray_image, uint16_t* depth_image, size_t image_size)
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

    void depth_color_calib::CalculateDistanceTransform(rs2_intrinsics image_intrinsics, double* image)
    {
        int width(image_intrinsics.width), height(image_intrinsics.height);
        size_t hcols(_options.win_cols / 2);
        size_t hrows(_options.win_rows / 2);
        double* temp_image = new double[width*height]; //set to zeros
        memcpy(temp_image, image, width*height * sizeof(double));
        for (size_t y = 0; y < height - _options.win_rows; y++)
        {
            for (size_t x = 0; x < width - _options.win_cols; x++)
            {
                size_t iidx(y*width + x);
                double max_val(0);
                size_t max_idx[2];
                size_t center_idx = (y + hrows) * width + (x + hcols);
                double tpix(temp_image[center_idx]);
                double* pix(image + center_idx);
                for (size_t yy = 0; yy < _options.win_rows; yy++)
                {
                    for (size_t xx = 0; xx < _options.win_cols; xx++)
                    {
                        if (temp_image[iidx] > max_val)
                        {
                            max_val = temp_image[iidx];
                            max_idx[0] = xx;
                            max_idx[1] = yy;
                        }
                        iidx++;
                    }
                    iidx += (width - _options.win_cols);
                }
                //D(i, j) = Params.Alpha * E(i, j) + (1 - Params.Alpha)* MaxLocalE * Params.Gamma ^ (max(abs(x - 2), abs(y - 2)));
                *pix = _options.alpha * tpix + (1 - _options.alpha)*max_val * pow(_options.gamma, max(abs(max_idx[0] - (double)hcols), abs(max_idx[1] - (double)hrows)));
            }
        }
        delete[] temp_image;
    }

    //E = colfilt( GrayImg,[ Params.WinRows, Params.WinCols ],'sliding',fun);
    void depth_color_calib::SubtractLocalCenter(rs2_intrinsics image_intrinsics, double* image)
    {
        int width(image_intrinsics.width), height(image_intrinsics.height);
        size_t hcols(_options.win_cols / 2);
        size_t hrows(_options.win_rows / 2);
        double* temp_image = new double[width*height]; //set to zeros
        memcpy(temp_image, image, width*height * sizeof(double));
        for (size_t y = 0; y < height - _options.win_rows; y++)
        {
            for (size_t x = 0; x < width - _options.win_cols; x++)
            {
                size_t iidx(y*width + x);
                double max_val(0);
                size_t center_idx = (y + hrows) * width + (x + hcols);
                double* tpix(temp_image + center_idx);
                double* pix(image + center_idx);
                for (size_t yy = 0; yy < _options.win_rows; yy++)
                {
                    for (size_t xx = 0; xx < _options.win_cols; xx++)
                    {
                        max_val = std::max(max_val, std::abs(*tpix - temp_image[iidx]));
                        iidx++;
                    }
                    iidx += (width - _options.win_cols);
                }
                *pix = max_val;
            }
        }
        delete[] temp_image;
    }

    void depth_color_calib::FilterBackgroundNoise(rs2_intrinsics depth_intrinsics, uint16_t* depth_image)
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
            }
        }

        double laser_dist_cont_th_m = pow(_options.laser_dist_cont_th, 2.0) * _depth_scale * 0.125;
        double laser_background_th_m = _options.laser_background_th * _depth_scale; // Filter noise in bakcground
        // Leave only points with difference greater then "_options.laser_dist_cont_th" from neighbours.
        std::vector<int> indices;
        //int count(0);
        for (int idx(1); idx < pc.size() - 1; idx++)
        {
            double left = pc[idx - 1].z - pc[idx].z;
            double right = pc[idx + 1].z - pc[idx].z;
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
        //ofstream fout("indices_c.txt");
        //for (std::vector<int>::iterator wi = indices.begin(); wi != indices.end(); wi++)
        //{
        //    fout << *wi << endl;
        //}
        //fout.close();
        

        return X;
    }

}