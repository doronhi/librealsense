// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "../../func-common.h"
#include <iostream>

using namespace rs2;

TEST_CASE("emitter-on-off works if started before streaming", "[d400][live]")
{
    const size_t record_frames = 60;
    auto devices = find_devices_by_product_line_or_exit(RS2_PRODUCT_LINE_D400);
    auto dev = devices[0];

    auto depth_sens = dev.first< rs2::depth_sensor >();

    REQUIRE_NOTHROW(depth_sens.set_option(RS2_OPTION_EMITTER_ON_OFF, 1));

    std::vector< stream_profile > profiles;
    profiles.push_back(find_profile(depth_sens, RS2_STREAM_DEPTH, RS2_SENSOR_MODE_VGA));
    profiles.push_back(find_profile(depth_sens, RS2_STREAM_INFRARED, RS2_SENSOR_MODE_VGA));

    REQUIRE_NOTHROW(depth_sens.open(profiles));

    std::condition_variable cv;
    std::mutex m;

    bool even = true;
    std::vector<int> emitter_state;
    emitter_state.reserve(record_frames);
    size_t skip_frames(4);

    auto wait_for_streams = [&]() {
        std::unique_lock< std::mutex > lock(m);
        REQUIRE(cv.wait_for(lock, std::chrono::seconds(10), [&]() {
            return emitter_state.size() == record_frames;
            }));
    };

    REQUIRE_NOTHROW(depth_sens.start([&](rs2::frame f) {
        std::unique_lock< std::mutex > lock(m);
        if (((bool(f.get_frame_number() % 2)) != even) && f.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE) &&
            (emitter_state.size() < record_frames) )
        {
            if (skip_frames > 0)
            {
                skip_frames--;
                return;
            }
            even = !even;  // Alternating odd/even frame number is sufficient to avoid duplicates
            auto val = static_cast<int>(f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE));
            // std::cout << "val: " << val << ", " << emitter_state.size() << std::endl;
            emitter_state.push_back(val);
        }
        cv.notify_one();
        }));

    wait_for_streams();
    REQUIRE(std::adjacent_find(emitter_state.begin(), emitter_state.end()) == emitter_state.end());

    REQUIRE_NOTHROW(depth_sens.set_option(RS2_OPTION_EMITTER_ON_OFF, 0));

    REQUIRE_NOTHROW(depth_sens.stop());
    REQUIRE_NOTHROW(depth_sens.close());
}
