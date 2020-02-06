// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering

#include <map>
#include <vector>
#include <chrono>

using namespace std::chrono;

int main(int argc, char * argv[]) try
{
    bool display(true);
    int max_frames(-1);
    std::string serial_number;
    int loops(1);
    for (int ii = 0; ii < argc; ii++)
    {
        if (std::string(argv[ii]) == std::string("--help"))
        {
            std::cout << "USAGE" << std::endl;
            std::cout << "-----" << std::endl;
            std::cout << "rs-multicam.exe [params]" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "Without and params, application connects to all available realsense cameras." << std::endl;
            std::cout << "For each camera it opens 2 windows: one displaying RGB image and the other, the depth image." << std::endl;
            std::cout << "" << std::endl;
            std::cout << "params" << std::endl;
            std::cout << "------" << std::endl;
            std::cout << "--no_display: no display window will be openned." << std::endl;
            std::cout << "--nframes <N>: will stop once received N frames from any camera" << std::endl;
            std::cout << "--sn <serial_number>: will stream images from camera with <serial_number> only." << std::endl;
            std::cout << "--loops <N>: Will restart the camera and stream again N times." << std::endl;
            std::cout << "" << std::endl;
            return 0;
        }

        if (std::string(argv[ii]) == std::string("--no_display"))
            display = false;
        if (std::string(argv[ii]) == std::string("--nframes"))
            max_frames = atoi(argv[ii+1]);
        if (std::string(argv[ii]) == std::string("--sn"))
            serial_number = std::string(argv[ii+1]);
        if (std::string(argv[ii]) == std::string("--loops"))
            loops = atoi(argv[ii+1]);
    }

    while (loops > 0)
    {
        loops--;
        // Create a simple OpenGL window for rendering:

        rs2::context                          ctx;        // Create librealsense context for managing devices

        std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)

        std::vector<rs2::pipeline>            pipelines;

        // Start a streaming pipe per each connected device
        for (auto&& dev : ctx.query_devices())
        {
            rs2::pipeline pipe(ctx);
            rs2::config cfg;
            auto serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            if (!serial_number.empty() && serial_number != serial)
                continue;
            cfg.enable_device(serial);
            pipe.start(cfg);
            pipelines.emplace_back(pipe);
            // Map from each device's serial number to a different colorizer
            colorizers[serial] = rs2::colorizer();
            if (!display)
                std::cout << "Enabled device: " << serial << std::endl;
        }
        if (pipelines.empty())
        {
            std::cerr << "No device was connected." << std::endl;
            return -1;
        }

        // We'll keep track of the last frame of each stream available to make the presentation persistent
        std::map<int, rs2::frame> render_frames;
        std::map<std::string, int> frames_counter;

        double system_time_start = duration<double, std::milli>(system_clock::now().time_since_epoch()).count();
        double system_time_finish = duration<double, std::milli>(system_clock::now().time_since_epoch()).count();

        window* app(NULL);
        if (display)
            app = new window(1280, 960, "CPP Multi-Camera Example");

        bool running(true);

        // Main app loop
        while (running && (!display || *app))
        {
            // Collect the new frames from all the connected devices
            std::vector<rs2::frame> new_frames;
            for (auto &&pipe : pipelines)
            {
                rs2::frameset fs;
                if (pipe.poll_for_frames(&fs))
                {
                    for (const rs2::frame& f : fs)
                        new_frames.emplace_back(f);
                    system_time_start = duration<double, std::milli>(system_clock::now().time_since_epoch()).count();
                }
            }
            system_time_finish = duration<double, std::milli>(system_clock::now().time_since_epoch()).count();
            if (system_time_finish - system_time_start > 3000)
            {
                std::cout << "No frames arrived" << std::endl;
                system_time_start = duration<double, std::milli>(system_clock::now().time_since_epoch()).count();
            }

            // Convert the newly-arrived frames to render-friendly format
            for (const auto& frame : new_frames)
            {
                // Get the serial number of the current frame's device
                auto serial = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                // Apply the colorizer of the matching device and store the colorized frame
                render_frames[frame.get_profile().unique_id()] = colorizers[serial].process(frame);

                if (frames_counter.find(serial) == frames_counter.end())
                    frames_counter.insert(std::pair<std::string, int>(serial, 0));
                frames_counter[serial]++;

            }
            for (auto icount : frames_counter)
            {
                if (max_frames > 0 && icount.second > max_frames)
                {
                    running = false;
                }
            }
            if (display)
            {
                // Present all the collected frames with openGl mosaic
                app->show(render_frames);
            }
        }
        if (!display)
            std::cerr << "active streams: " << frames_counter.size() << " / " << pipelines.size() << std::endl;
        if (app)
            delete app;
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
