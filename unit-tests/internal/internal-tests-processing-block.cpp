// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include "catch.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include "../../common/tiny-profiler.h"
#include "./../unit-tests-common.h"
#include "./../src/environment.h"
#include "./../src/proc/color-formats-converter.h"
#include <algorithm>

using namespace librealsense;
using namespace librealsense::platform;

TEST_CASE("unpack_yuy2")
{
    class yuy2_converter_test : public yuy2_converter
    {
    public:
        yuy2_converter_test(rs2_format target_format, int width, int height) :
            yuy2_converter(target_format),
            m_width(width),
            m_height(height)
        {
            _target_bpp = get_image_bpp(target_format) / 8;
            m_dest = new byte[m_width*m_height*_target_bpp];
            m_source = new byte[m_width*m_height*_target_bpp];
        };

        ~yuy2_converter_test()
        {
            delete(m_dest);
            delete(m_source);
        }

        void process_function(std::vector<std::chrono::duration<double, std::milli> >& processing_times, const std::chrono::duration<double, std::milli>& max_processing_time_ms)
        {
            //int max_iterations(1000 * 60 * 10); //approximately 17 minutes.
            int max_iterations(170); //approximately 10 seconds.
            std::chrono::duration<double, std::milli> time_elapsed(0);
            int counter(0);
            while (counter < max_iterations && time_elapsed < max_processing_time_ms)
            {
                auto start_time = std::chrono::high_resolution_clock::now();
                yuy2_converter::process_function(&m_dest, m_source, m_width, m_height, m_height * m_width * _target_bpp, 0);
                time_elapsed = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time);
                counter++;
                processing_times.push_back(time_elapsed);
                //std::cout << counter << ". " << "time elapsed: " << time_elapsed.count() / 1000.f << " sec" << std::endl;
            }
        }

    private:
        int m_width, m_height, _target_bpp;
        byte* m_dest;
        byte* m_source;
    };

    std::chrono::duration<double, std::milli> max_processing_time_ms(5);

    yuy2_converter_test test_obj(RS2_FORMAT_RGB8, 848, 480);
    std::vector<std::chrono::duration<double, std::milli> > processing_times;
    test_obj.process_function(processing_times, max_processing_time_ms);
    bool is_ok(*(std::max_element(processing_times.begin(), processing_times.end())) < max_processing_time_ms);
    CAPTURE(*(std::max_element(processing_times.begin(), processing_times.end())));
    CAPTURE(*(std::min_element(processing_times.begin(), processing_times.end())));
    std::cout << "processing times: " << (*(std::min_element(processing_times.begin(), processing_times.end()))).count() << " -> " << (*(std::max_element(processing_times.begin(), processing_times.end()))).count() << "(mSec)" << std::endl;
    REQUIRE(is_ok);
}