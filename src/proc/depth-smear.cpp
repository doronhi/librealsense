// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "depth-smear.h"
#include <iomanip>
#include "l500/l500-depth.h"
#include "canny.h"
#include <algorithm>

using namespace std;

namespace librealsense
{
    void discard_low_gradients(const uint8_t * ir_data, uint8_t * ir_edge, const int nx, const int ny, uint8_t ir_grade_threshold, bool is_horizontal)
    {
        //gaussian_filter(in, out, nx, ny, sigma);

        pixel_t *G = (pixel_t*)calloc(nx * ny, sizeof(pixel_t));

        if (is_horizontal)
        {
            const float kernel[] = { -1, 0, 1,
                                     -2, 0, 2,
                                     -1, 0, 1 };
            convolution_uc(ir_data, G, kernel, nx, ny, 3, 3, false);
            FILE* fout = fopen("Gx.bin", "wb");
            fwrite(G, sizeof(G[0]), nx*ny, fout);
            fclose(fout);
        }
        else
        {
            const float kernel[] = { 1, 2, 1,
                                     0, 0, 0,
                                    -1,-2,-1 };
            convolution_uc(ir_data, G, kernel, nx, ny, 3, 3, false);
            FILE* fout = fopen("Gy.bin", "wb");
            fwrite(G, sizeof(G[0]), nx*ny, fout);
            fclose(fout);
        }

        // Discard threshold where gradient is too low
        for (int i = 0; i < nx*ny; i++)
        {
            if (abs(G[i]) < ir_grade_threshold)
            {
                ir_edge[i] = 0;
            }
        }
        free(G);
    }

    std::set<int> findPixByNeighbor(const uint8_t * ir_edge, const int nx, const int ny, uint8_t neighborPix, bool is_horizontal)
    {
        std::set<int> pixs2Check;
        for (int y = 0; y < ny; y++)
        {
            for (int x = 0; x < nx; x++)
            {
                if (ir_edge[y*nx + x] > 0)
                {
                    if (is_horizontal)
                    {
                        for (int xx = max(0, x - (int)ceil(neighborPix / 2)); xx <= min(x + (int)ceil(neighborPix / 2), nx); xx++)
                        {
                            pixs2Check.insert(y*nx + xx);
                        }
                    }
                    else
                    {
                        for (int yy = max(0, y - (int)ceil(neighborPix / 2)); yy <= min(y + (int)ceil(neighborPix / 2), ny); yy++)
                        {
                            pixs2Check.insert(yy*nx + x);
                        }
                    }
                }
            }
        }
        return pixs2Check;
    }

    std::set<int> setPixelsToCheck(const uint16_t * depth_data_in, const uint8_t * ir_data, const uint8_t * ir_edge,
                          const int nx, const int ny, const depth_smear_options& options, bool is_horizontal)
    {
        std::set<int> pixs2Check = findPixByNeighbor(ir_edge, nx, ny, options.neighborPix, is_horizontal);
        {
            FILE* fout = fopen("pixs2Check_before.bin", "wb");
            for (int idx : pixs2Check)
                fwrite(&idx, sizeof(idx), 1, fout);
            fclose(fout);
        }
        {
            FILE* fout = fopen("depth_data_in.bin", "wb");
            fwrite(depth_data_in, sizeof(depth_data_in[0]), nx*ny, fout);
            fclose(fout);
        }
        {
            FILE* fout = fopen("ir_data.bin", "wb");
            fwrite(ir_data, sizeof(ir_data[0]), nx*ny, fout);
            fclose(fout);
        }

        std::vector<int> pixs_not_check;
        for (int idx : pixs2Check)
        {
            if ((depth_data_in[idx] == 0) || (ir_data[idx] >= options.ir_threshold))
                pixs_not_check.push_back(idx);
        }
        for (int idx : pixs_not_check)
        {
            pixs2Check.erase(idx);
        }
        {
            FILE* fout = fopen("pixs2Check.bin", "wb");
            for (int idx : pixs2Check)
                fwrite(&idx, sizeof(idx), 1, fout);
            fclose(fout);
        }
        return pixs2Check;
    }

    void calcDzAroundClosestEdge(const uint16_t * depth_data_in, const int x, const int y, const uint8_t *ir_edge, const int nx, const int ny,
        bool is_horizontal, uint8_t& dz_around_edge,int& closestEdgeIx)
    {
        int sizeOfAxisDir;
        if (is_horizontal)
        {
            sizeOfAxisDir = nx;
            for (int i = 0; i < nx; i++)
            {
                if (x - i > 0 && ir_edge[y*nx + x - i] > 0)
                {
                    closestEdgeIx = x - i;
                    break;
                }
                if (x + i < nx && ir_edge[y*nx + x + i] > 0)
                {
                    closestEdgeIx = x + i;
                    break;
                }
            }
        }
        else
        {
            sizeOfAxisDir = ny;
            for (int i = 0; i < ny; i++)
            {
                if (y - i > 0 && ir_edge[(y-i)*nx + x] > 0)
                {
                    closestEdgeIx = y - i;
                    break;
                }
                if (y + i < ny && ir_edge[(y+i)*nx + x] > 0)
                {
                    closestEdgeIx = y + i;
                    break;
                }
            }
        }
        int beforEdgeIx = max(1, closestEdgeIx - 1);
        int afterEdgeIx = min(sizeOfAxisDir, closestEdgeIx + 1);
        if (is_horizontal)
        {
            dz_around_edge = abs(depth_data_in[y*nx + afterEdgeIx] - depth_data_in[y*nx + beforEdgeIx]);
        }
        else
        {
            dz_around_edge = abs(depth_data_in[afterEdgeIx*nx + x] - depth_data_in[beforEdgeIx*nx + x]);
        }
    }

    // Need to know if in the neighbouring pixels, any of the depth is nan and what is the index of max depth (ignoring nans)
    int calcDepthFromEdge(const uint16_t * depth_data_in, const int x, const int y, const uint8_t * ir_data, const int nx, const int ny,
        bool is_horizontal, const int closestEdgeIx, const uint8_t neighborPix)
    {
        int end_idx, dir_sign;
        if (is_horizontal)
        {
            int dx_from_edge(x - closestEdgeIx);
            dir_sign = (dx_from_edge < 0) ? -1 : 1;
            if (dx_from_edge == 0)
            {
                dir_sign = (ir_data[y*nx + min(x + 2, nx)] > ir_data[y*nx + max(x - 2, 1)]) ? -1 : 1;
            }
            end_idx = min(max(x + dir_sign * (neighborPix - abs(dx_from_edge)), 1), nx);
            end_idx = y * nx + end_idx;
        }
        else
        {
            int dx_from_edge(y - closestEdgeIx);
            dir_sign = (dx_from_edge < 0) ? -1 : 1;
            if (dx_from_edge == 0)
            {
                dir_sign = (ir_data[min(y+2, ny)*nx + x] > ir_data[max(y-2, 1)*nx + x]) ? -1 : 1;
            }
            end_idx = min(max(y + dir_sign * (neighborPix - abs(dx_from_edge)), 1), ny);
            dir_sign *= nx;
            end_idx = end_idx * nx + x;
        }
        int max_depth_idx(-1);
        uint16_t max_depth(0);
        for (int idx = y * nx + x; idx != end_idx; idx += dir_sign)
        {
            if (depth_data_in[idx] == 0)
            {
                max_depth_idx = -2;
                break;
            }
            if (depth_data_in[idx] > max_depth)
            {
                max_depth = depth_data_in[idx];
                max_depth_idx = idx;
            }
        }
        return max_depth_idx;
    }

    std::set<int> invalid_smear_dir(const uint16_t * depth_data_in, const uint8_t * ir_data, uint8_t * full_ir_edge, const int nx, const int ny,
                           const depth_smear_options& options, bool is_horizontal)
    {
        uint8_t *ir_edge = (uint8_t*)malloc(nx * ny * sizeof(uint8_t));
        memcpy(ir_edge, full_ir_edge, nx*ny * sizeof(uint8_t));
        {
            FILE* fout = fopen("ir_edge_before.bin", "wb");
            fwrite(ir_edge, sizeof(ir_edge[0]), nx*ny, fout);
            fclose(fout);
        }
        discard_low_gradients(ir_data, ir_edge, nx, ny, options.ir_grade_threshold, true);

        {
            FILE* fout = fopen("ir_edge.bin", "wb");
            fwrite(ir_edge, sizeof(ir_edge[0]), nx*ny, fout);
            fclose(fout);
        }

        // Condition #3
        bool* pixs2Check = new bool[nx*ny]{ false };
        std::set<int> idx_to_invalid;
        std::set<int> idx_to_check = setPixelsToCheck(depth_data_in, ir_data, ir_edge, nx, ny, options, is_horizontal);
        {
            FILE* fout = fopen("idx_to_check.bin", "wb");
            for (int idx : idx_to_check)
                fwrite(&idx, sizeof(idx), 1, fout);
            fclose(fout);
        }
        for (int crnt_idx : idx_to_check)
        {
            int x = crnt_idx % nx;
            int y = (crnt_idx - x) / nx;

            uint8_t dz_around_edge;
            int closestEdgeIx;
            calcDzAroundClosestEdge(depth_data_in, x, y, ir_edge, nx, ny, is_horizontal, dz_around_edge, closestEdgeIx);
            if (dz_around_edge > options.dz_around_edge_th) //Condition #4 - if the depth is consistent with IR, no need to invalidate
                continue;
            // Need to know if in the neighbouring pixels, any of the depth is nan and what is the index of max depth (ignoring nans)
            int max_depth_idx = calcDepthFromEdge(depth_data_in, x, y, ir_data, nx, ny, is_horizontal, closestEdgeIx, options.neighborPix);
            if (max_depth_idx < 0 || (abs(depth_data_in[max_depth_idx]-depth_data_in[crnt_idx]) > options.dz_in_neighborPix_th && ir_data[crnt_idx] < ir_data[max_depth_idx]))
            {
                idx_to_invalid.insert(crnt_idx);
            }
        }
        delete[] pixs2Check;
        {
            FILE* fout = fopen("idx_to_invalid.bin", "wb");
            for (int idx : idx_to_invalid)
                fwrite(&idx, sizeof(idx), 1, fout);
            fclose(fout);
        }
        return idx_to_invalid;
    }

    template<class T>
    bool depth_smear_invalidation(const uint16_t * depth_data_in, const uint8_t * ir_data, T zero_pixel,
        rs2_intrinsics intrinsics,
        const depth_smear_options& options)
    {
        const int nx = intrinsics.width;
        const int ny = intrinsics.height;

        uint8_t *ir_edge = new uint8_t[nx * ny];
        canny_edge_detection(ir_data, ir_edge, nx, ny, 0.2f, 0.5f, 1.4142f);
        std::set<int> idx_to_invalid = invalid_smear_dir(depth_data_in, ir_data, ir_edge, nx, ny, options, true);
        std::set<int> idx_to_invalid_y = invalid_smear_dir(depth_data_in, ir_data, ir_edge, nx, ny, options, false);
        for (int idx : idx_to_invalid_y)
        {
            idx_to_invalid.insert(idx);
        }
        for (int idx : idx_to_invalid)
        {
            zero_pixel(idx);
        }


        //for (auto i = 0; i < intrinsics.height*intrinsics.width; i++)
        //{
        //    auto zero = (ir_edge[i] == 0);
        //    zero_pixel(i, zero);
        //}
        delete[] ir_edge;
        return true;
    }

    depth_smear::depth_smear()
        : generic_processing_block("Depth Smear Fix")
    {
    }

    const char* depth_smear::get_option_name(rs2_option option) const
    {
        return "No Options Yet";
    }

    rs2::frame depth_smear::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {
        std::vector<rs2::frame> result;

        auto data = f.as<rs2::frameset>();

        if (!_source_profile_depth)
            _source_profile_depth = data.get_depth_frame().get_profile();

        if (!_target_profile_depth)
            _target_profile_depth = _source_profile_depth.clone(_source_profile_depth.stream_type(), _source_profile_depth.stream_index(), _source_profile_depth.format());

        auto depth_frame = data.get_depth_frame();
        auto ir_frame = data.get_infrared_frame();
        auto confidence_frame = data.first_or_default(RS2_STREAM_CONFIDENCE);

        auto depth_out = source.allocate_video_frame(_target_profile_depth, depth_frame, 0, 0, 0, 0, RS2_EXTENSION_DEPTH_FRAME);

        rs2::frame confidence_out;
        if (confidence_frame)
        {
            if (!_source_profile_confidence)
                _source_profile_confidence = confidence_frame.get_profile();

            if (!_target_profile_confidence)
                _target_profile_confidence = _source_profile_confidence.clone(_source_profile_confidence.stream_type(), _source_profile_confidence.stream_index(), _source_profile_confidence.format());

            confidence_out = source.allocate_video_frame(_source_profile_confidence, confidence_frame, 0, 0, 0, 0, RS2_EXTENSION_VIDEO_FRAME);
        }
        auto depth_intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        auto depth_output = (uint16_t*)depth_out.get_data();
        memcpy(depth_output, (const uint16_t*)depth_frame.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
        //memset(depth_output, 0, depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
        uint8_t* confidence_output;

        if (confidence_frame)
        {
            confidence_output = (uint8_t*)confidence_out.get_data();
        }

        if (depth_smear_invalidation((const uint16_t*)depth_frame.get_data(),
            (const uint8_t*)ir_frame.get_data(),
            [&](int index)
        {
            depth_output[index] = 0;

            if (confidence_frame)
            {
                confidence_output[index] = 0;
            }
        },
            depth_intrinsics,
            _options))
        {
            result.push_back(depth_out);
            result.push_back(ir_frame);
            if (confidence_frame)
                result.push_back(confidence_out);
        }
        else
        {
            result.push_back(depth_frame);
            result.push_back(ir_frame);
            if (confidence_frame)
                result.push_back(confidence_frame);
        }
        return source.allocate_composite_frame(result);
    }

    bool depth_smear::should_process(const rs2::frame& frame)
    {
        if (auto set = frame.as<rs2::frameset>())
        {
            if (!set.get_depth_frame() || !set.get_infrared_frame())
            {
                return false;
            }
            return true;
        }
        return false;
    }

    rs2::frame depth_smear::prepare_output(const rs2::frame_source & source, rs2::frame input, std::vector<rs2::frame> results)
    {
        if (auto composite = input.as<rs2::frameset>())
        {
            composite.foreach([&](rs2::frame f)
            {
                if (f.get_profile().stream_type() != RS2_STREAM_DEPTH && f.get_profile().stream_type() != RS2_STREAM_INFRARED && f.get_profile().stream_type() != RS2_STREAM_CONFIDENCE &&
                    results.size() > 0)
                    results.push_back(f);
            });
        }
        return source.allocate_composite_frame(results);
    }

}

