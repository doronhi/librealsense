// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

////////////////////////////////////////////////
// Regressions tests against the known issues //
////////////////////////////////////////////////

#include <cmath>
#include "unit-tests-common.h"
#include "../include/librealsense2/rs_advanced_mode.hpp"
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/hpp/rs_frame.hpp>
#include <iostream>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <librealsense2/rsutil.h>

using namespace rs2;

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

TEST_CASE("logger-load", "")
{
    {
        rs2::log_to_file(RS2_LOG_SEVERITY_DEBUG, "lrs_log.txt");
        // rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
        std::vector<std::shared_ptr<std::thread> > threads;
        const size_t num_threads(10);
        std::chrono::seconds test_running_time(10);
        std::chrono::milliseconds thread_sleep_time(1);
        std::chrono::milliseconds max_delay_time(50);
        bool is_running(true);
        std::condition_variable cv;
        std::mutex m;
        bool _break_on_delay(true);
        bool _delete_log_file_on_successful_exit(true);

        for (size_t thread_idx=0; thread_idx < num_threads; thread_idx++)
        {
            LOG_INFO("Add thread: " << thread_idx);
            threads.push_back(std::make_shared<std::thread>([&, thread_idx]()
            {
                auto crnt_time = std::chrono::high_resolution_clock::now();
                auto prev_time = std::chrono::high_resolution_clock::now();
                while (is_running)
                {
                    std::this_thread::sleep_for(thread_sleep_time);
                    crnt_time = std::chrono::high_resolution_clock::now();
                    std::chrono::milliseconds diff = std::chrono::duration_cast<std::chrono::milliseconds>(crnt_time - prev_time);
                    LOG_INFO(std::dec << "thread id:" << thread_idx << " crnt_time: " << std::chrono::duration_cast<std::chrono::milliseconds>(crnt_time.time_since_epoch()).count() << " diff: " << diff.count() << "," << (crnt_time-prev_time > thread_sleep_time + max_delay_time));
                    if (crnt_time-prev_time > thread_sleep_time + max_delay_time)
                    {
                        LOG_ERROR(std::dec << "thread id:" << thread_idx << ":" << " diff: " << diff.count());
                        if (_break_on_delay)
                        {
                            std::unique_lock<std::mutex> lock(m);
                            is_running = false;
                            cv.notify_one();
                        }
                        else
                        {
                            std::cout << std::dec << "thread id:" << thread_idx << " crnt_time: " << std::chrono::duration_cast<std::chrono::milliseconds>(crnt_time.time_since_epoch()).count() << " diff: " << diff.count() << std::endl;
                        }
                    }
                    prev_time = crnt_time;
                }
                LOG_INFO("END thread" << thread_idx);
            }));
        }
        auto start_time = std::chrono::high_resolution_clock::now();
        while(is_running && std::chrono::high_resolution_clock::now()-start_time < test_running_time)
        {
            std::unique_lock<std::mutex> lock(m);
            cv.wait_for(lock, std::chrono::seconds(1), [&] {return (!is_running); });
        }
        bool test_ok(is_running);
        std::chrono::seconds diff = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time);
        CAPTURE(diff.count());
        if (!is_running)
        {
            LOG_ERROR("Test failed after " << diff.count() << "(sec)");
        }
        else
        {
            LOG_INFO("Test succeeded.");
        }
        LOG_INFO("Waiting for threads...");
        is_running = false;
        for (auto& thread : threads)
        {
            thread->join();
        }
        if (test_ok && _delete_log_file_on_successful_exit)
        {
            std::remove("lrs_log.txt");
        }
        REQUIRE(test_ok);
    }
}