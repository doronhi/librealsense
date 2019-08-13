// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "depth-smear.h"
#include <iomanip>
#include "l500/l500-depth.h"
#include <algorithm>

using namespace std;
// #define _SAVE_DBG_
namespace librealsense
{
    void depth_smear::discard_low_gradients(std::vector<uint8_t>* ir_edge, bool is_horizontal)
    {
        const int nx(_imsize.x), ny(_imsize.y);

        std::fill(_temp_G.begin(), _temp_G.end(), 0.0f);

        if (is_horizontal)
        {
            const float kernel[] = { -1, 0, 1,
                                     -2, 0, 2,
                                     -1, 0, 1 };
            convolution_uc(_ir_data, _temp_G.data(), kernel, nx, ny, 3, 3, false);
#ifdef _SAVE_DBG_
            {
                FILE* fout = fopen("Gx.bin", "wb");
                fwrite(_temp_G.data(), sizeof(_temp_G[0]), nx*ny, fout);
                fclose(fout);
            }
#endif
        }
        else
        {
            const float kernel[] = { 1, 2, 1,
                                     0, 0, 0,
                                    -1,-2,-1 };
            convolution_uc(_ir_data, _temp_G.data(), kernel, nx, ny, 3, 3, false);
#ifdef _SAVE_DBG_
            {
                FILE* fout = fopen("Gy.bin", "wb");
                fwrite(_temp_G.data(), sizeof(_temp_G[0]), nx*ny, fout);
                fclose(fout);
            }
#endif
        }

        // Discard threshold where gradient is too low
        for (int i = 0; i < nx*ny; i++)
        {
            if (abs(_temp_G[i]) < _options.ir_grad_threshold)
            {
                (*ir_edge)[i] = 0;
            }
        }
    }

    std::set<int> depth_smear::findPixByNeighbor(const std::vector<uint8_t>& ir_edge, bool is_horizontal)
    {
        const int nx(_imsize.x), ny(_imsize.y);
        uint8_t neighborPix(_options.neighborPix);
        std::set<int> pixs2Check;
        for (int y = 0; y < ny; y++)
        {
            for (int x = 0; x < nx; x++)
            {
                if (ir_edge[y*nx + x] > 0)
                {
                    if (is_horizontal)
                    {
                        for (int xx = max(0, x - (int)ceil(neighborPix / 2)); xx < min(x + (int)ceil(neighborPix / 2), nx); xx++)
                        {
                            pixs2Check.insert(y*nx + xx);
                        }
                    }
                    else
                    {
                        for (int yy = max(0, y - (int)ceil(neighborPix / 2)); yy < min(y + (int)ceil(neighborPix / 2), ny); yy++)
                        {
                            pixs2Check.insert(yy*nx + x);
                        }
                    }
                }
            }
        }
        return pixs2Check;
    }

    std::set<int> depth_smear::setPixelsToCheck(const std::vector<uint8_t>& ir_edge, bool is_horizontal)
    {
        std::set<int> pixs2Check = findPixByNeighbor(ir_edge, is_horizontal);
#ifdef _SAVE_DBG_
        const int nx(_imsize.x), ny(_imsize.y);
        {
            FILE* fout = fopen("pixs2Check_before.bin", "wb");
            for (int idx : pixs2Check)
                fwrite(&idx, sizeof(idx), 1, fout);
            fclose(fout);
        }
        {
            FILE* fout = fopen("depth_data_in.bin", "wb");
            fwrite(_depth_data, sizeof(_depth_data[0]), nx*ny, fout);
            fclose(fout);
        }
        {
            FILE* fout = fopen("ir_data.bin", "wb");
            fwrite(_ir_data, sizeof(_ir_data[0]), nx*ny, fout);
            fclose(fout);
        }
#endif
        std::vector<int> pixs_not_check;
        for (int idx : pixs2Check)
        {
            if ((_depth_data[idx] == 0) || (_ir_data[idx] >= _options.ir_threshold))
                pixs_not_check.push_back(idx);
        }
        for (int idx : pixs_not_check)
        {
            pixs2Check.erase(idx);
        }
#ifdef _SAVE_DBG_
        {
            FILE* fout = fopen("pixs2Check.bin", "wb");
            for (int idx : pixs2Check)
                fwrite(&idx, sizeof(idx), 1, fout);
            fclose(fout);
        }
#endif
        return pixs2Check;
    }

    void depth_smear::calcDzAroundClosestEdge(const int x, const int y, const std::vector<uint8_t>& ir_edge, bool is_horizontal, uint8_t& dz_around_edge,int& closestEdgeIx)
    {
        const int nx(_imsize.x), ny(_imsize.y);
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
            dz_around_edge = abs(_depth_data[y*nx + afterEdgeIx] - _depth_data[y*nx + beforEdgeIx]);
        }
        else
        {
            dz_around_edge = abs(_depth_data[afterEdgeIx*nx + x] - _depth_data[beforEdgeIx*nx + x]);
        }
    }

    // Need to know if in the neighbouring pixels, any of the depth is nan and what is the index of max depth (ignoring nans)
    int depth_smear::calcDepthFromEdge(const int x, const int y, bool is_horizontal, const int closestEdgeIx)
    {
        const int nx(_imsize.x), ny(_imsize.y);
        const uint8_t neighborPix(_options.neighborPix);
        int end_idx, dir_sign;
        if (is_horizontal)
        {
            int dx_from_edge(x - closestEdgeIx);
            dir_sign = (dx_from_edge < 0) ? -1 : 1;
            if (dx_from_edge == 0)
            {
                dir_sign = (_ir_data[y*nx + min(x + 2, nx)] > _ir_data[y*nx + max(x - 2, 1)]) ? -1 : 1;
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
                dir_sign = (_ir_data[min(y+2, ny)*nx + x] > _ir_data[max(y-2, 1)*nx + x]) ? -1 : 1;
            }
            end_idx = min(max(y + dir_sign * (neighborPix - abs(dx_from_edge)), 1), ny);
            dir_sign *= nx;
            end_idx = end_idx * nx + x;
        }
        int max_depth_idx(-1);
        uint16_t max_depth(0);
        for (int idx = y * nx + x; idx != end_idx; idx += dir_sign)
        {
            if (_depth_data[idx] == 0)
            {
                max_depth_idx = -2;
                break;
            }
            if (_depth_data[idx] > max_depth)
            {
                max_depth = _depth_data[idx];
                max_depth_idx = idx;
            }
        }
        return max_depth_idx;
    }

    std::set<int> depth_smear::invalid_smear_dir(bool is_horizontal)
    {
        const int nx(_imsize.x), ny(_imsize.y);

        memcpy(_temp_ir_edge.data(), _ir_edge.data(), nx*ny * sizeof(uint8_t));
#ifdef _SAVE_DBG_
        {
            FILE* fout = fopen("ir_edge_before.bin", "wb");
            fwrite(_temp_ir_edge.data(), sizeof(_temp_ir_edge[0]), nx*ny, fout);
            fclose(fout);
        }
#endif
        discard_low_gradients(&_temp_ir_edge, is_horizontal);   // remove from _temp_ir_edge

#ifdef _SAVE_DBG_
        {
            FILE* fout = fopen("ir_edge.bin", "wb");
            fwrite(_temp_ir_edge.data(), sizeof(_temp_ir_edge[0]), nx*ny, fout);
            fclose(fout);
        }
#endif
        // Condition #3
        std::set<int> idx_to_invalid;
        std::set<int> idx_to_check = setPixelsToCheck(_temp_ir_edge, is_horizontal);
#ifdef _SAVE_DBG_
        {
            FILE* fout = fopen("idx_to_check.bin", "wb");
            for (int idx : idx_to_check)
                fwrite(&idx, sizeof(idx), 1, fout);
            fclose(fout);
        }
#endif
        for (int crnt_idx : idx_to_check)
        {
            int x = crnt_idx % nx;
            int y = (crnt_idx - x) / nx;

            uint8_t dz_around_edge;
            int closestEdgeIx;
            calcDzAroundClosestEdge(x, y, _temp_ir_edge, is_horizontal, dz_around_edge, closestEdgeIx);
            if (dz_around_edge > _options.dz_around_edge_th) //Condition #4 - if the depth is consistent with IR, no need to invalidate
                continue;
            // Need to know if in the neighbouring pixels, any of the depth is nan and what is the index of max depth (ignoring nans)
            int max_depth_idx = calcDepthFromEdge(x, y, is_horizontal, closestEdgeIx);
            if (max_depth_idx < 0 || (abs(_depth_data[max_depth_idx]-_depth_data[crnt_idx]) > _options.dz_in_neighborPix_th && _ir_data[crnt_idx] < _ir_data[max_depth_idx]))
            {
                idx_to_invalid.insert(crnt_idx);
            }
        }
#ifdef _SAVE_DBG_
        {
            FILE* fout = fopen("idx_to_invalid.bin", "wb");
            for (int idx : idx_to_invalid)
                fwrite(&idx, sizeof(idx), 1, fout);
            fclose(fout);
        }
#endif
        return idx_to_invalid;
    }

    template<class T>
    bool depth_smear::depth_smear_invalidation(const uint16_t * depth_data_in, const uint8_t * ir_data, T zero_pixel,
        rs2_intrinsics intrinsics,
        const depth_smear_options& options)
    {
        _canny.canny_edge_detection(_ir_data, _ir_edge, _imsize.x, _imsize.y, 0.2f, 0.5f, 1.4142f);
        std::set<int> idx_to_invalid = invalid_smear_dir(true);
        std::set<int> idx_to_invalid_y = invalid_smear_dir(false);
        for (int idx : idx_to_invalid_y)
        {
            idx_to_invalid.insert(idx);
        }
        for (int idx : idx_to_invalid)
        {
            zero_pixel(idx);
        }
        return true;
    }

    enum depth_smear_invalidation_options
    {
        RS2_OPTION_FILTER_DS_IR_GRAD_THRESHOLD = static_cast<rs2_option>(RS2_OPTION_COUNT + 0), /**< min IR gradient to consider an edge valid */
        RS2_OPTION_FILTER_DS_IR_THRESHOLD = static_cast<rs2_option>(RS2_OPTION_COUNT + 1),      /**< min IR value to invalidate by filter */
        RS2_OPTION_FILTER_DS_NEIGHBOUR_PIX = static_cast<rs2_option>(RS2_OPTION_COUNT + 2),     /**< number of pixels from edge to check for invalidation */
        RS2_OPTION_FILTER_DS_DZ_AROUND_EDGE = static_cast<rs2_option>(RS2_OPTION_COUNT + 3),    /**< min depth difference around edge to be considered for invalidation */
        RS2_OPTION_FILTER_DS_DZ_IN_NEIGHBOUR = static_cast<rs2_option>(RS2_OPTION_COUNT + 4),   /**< min depth difference to neighbour pixel for invalidation */
    };


    depth_smear::depth_smear()
        : generic_processing_block("Depth Smear Fix")
    {
        // Set option ir_grad_threshold:
        string option_name(get_option_name(static_cast<rs2_option>(RS2_OPTION_FILTER_DS_IR_GRAD_THRESHOLD)));
        auto ir_grad_threshold = std::make_shared<ptr_option<uint8_t>>(
            0,
            255,
            1,
            DS_IR_GRAD_THRESHOLD,
            &_options.ir_grad_threshold,
            option_name);
        ir_grad_threshold->on_set([ir_grad_threshold, option_name](float val)
        {
            if (!ir_grad_threshold->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported " << option_name << ". " << val << " is out of range.");
        });
        register_option(static_cast<rs2_option>(RS2_OPTION_FILTER_DS_IR_GRAD_THRESHOLD), ir_grad_threshold);

        // Set option ir_threshold:
        option_name = get_option_name(static_cast<rs2_option>(RS2_OPTION_FILTER_DS_IR_THRESHOLD));
        auto ir_threshold = std::make_shared<ptr_option<uint8_t>>(
            0,
            255,
            1,
            DS_IR_THRESHOLD,
            &_options.ir_threshold,
            option_name);
        ir_threshold->on_set([ir_threshold, option_name](float val)
        {
            if (!ir_threshold->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported " << option_name << ". " << val << " is out of range.");
        });
        register_option(static_cast<rs2_option>(RS2_OPTION_FILTER_DS_IR_THRESHOLD), ir_threshold);

        // Set option neighborPix:
        option_name = get_option_name(static_cast<rs2_option>(RS2_OPTION_FILTER_DS_NEIGHBOUR_PIX));
        auto neighborPix = std::make_shared<ptr_option<uint8_t>>(
            0,
            255,
            1,
            DS_NEIGHBOR_PIX,
            &_options.neighborPix,
            option_name);
        neighborPix->on_set([neighborPix, option_name](float val)
        {
            if (!neighborPix->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported " << option_name << ". " << val << " is out of range.");
        });
        register_option(static_cast<rs2_option>(RS2_OPTION_FILTER_DS_NEIGHBOUR_PIX), neighborPix);

        // Set option dz_around_edge_th:
        option_name = get_option_name(static_cast<rs2_option>(RS2_OPTION_FILTER_DS_DZ_AROUND_EDGE));
        auto dz_around_edge_th = std::make_shared<ptr_option<uint16_t>>(
            0,
            255*255,
            1,
            DS_DZ_AROUND_EDGE_TH,
            &_options.dz_around_edge_th,
            option_name);
        dz_around_edge_th->on_set([dz_around_edge_th, option_name](float val)
        {
            if (!dz_around_edge_th->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported " << option_name << ". " << val << " is out of range.");
        });
        register_option(static_cast<rs2_option>(RS2_OPTION_FILTER_DS_DZ_AROUND_EDGE), dz_around_edge_th);

        // Set option dz_in_neighborPix_th:
        option_name = get_option_name(static_cast<rs2_option>(RS2_OPTION_FILTER_DS_DZ_IN_NEIGHBOUR));
        auto dz_in_neighborPix_th = std::make_shared<ptr_option<uint16_t>>(
            0,
            255 * 255,
            1,
            DS_DZ_IN_NEIGHBOR_PIX_TH,
            &_options.dz_in_neighborPix_th,
            option_name);
        dz_in_neighborPix_th->on_set([dz_in_neighborPix_th, option_name](float val)
        {
            if (!dz_in_neighborPix_th->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported " << option_name << ". " << val << " is out of range.");
        });
        register_option(static_cast<rs2_option>(RS2_OPTION_FILTER_DS_DZ_IN_NEIGHBOUR), dz_in_neighborPix_th);
    }

    const char* depth_smear::get_option_name(rs2_option option) const
    {
        switch (option)
        {
        case depth_smear_invalidation_options::RS2_OPTION_FILTER_DS_IR_GRAD_THRESHOLD:
            return "IR-gradient Threshold";
        case depth_smear_invalidation_options::RS2_OPTION_FILTER_DS_IR_THRESHOLD:
            return "IR Threshold";
        case depth_smear_invalidation_options::RS2_OPTION_FILTER_DS_NEIGHBOUR_PIX:
            return "Num Neighbours";
        case depth_smear_invalidation_options::RS2_OPTION_FILTER_DS_DZ_AROUND_EDGE:
            return "edge DZ";
        case depth_smear_invalidation_options::RS2_OPTION_FILTER_DS_DZ_IN_NEIGHBOUR:
            return "neighbour DZ";
        }
        return options_container::get_option_name(option);
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

        //if (depth_intrinsics.width == 640 && depth_intrinsics.height == 480)
        //{
        //    LOG_ERROR("read depth image");
        //    // Debug : read image from file:
        //    FILE* fin = fopen("C:\\projects\\librealsense\\build\\depth_data_in.bin", "rb");
        //    fread((void*)depth_frame.get_data(), sizeof(uint16_t), depth_intrinsics.width*depth_intrinsics.height, fin);
        //    fclose(fin);
        //}
        //if (depth_intrinsics.width == 640 && depth_intrinsics.height == 480)
        //{
        //    LOG_ERROR("read ir image");
        //    FILE* fin = fopen("C:\\projects\\librealsense\\build\\ir_data.bin", "rb");
        //    fread((void*)ir_frame.get_data(), sizeof(uint8_t), depth_intrinsics.width*depth_intrinsics.height, fin);
        //    fclose(fin);
        //}

        auto depth_output = (uint16_t*)depth_out.get_data();
        memcpy(depth_output, (const uint16_t*)depth_frame.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint16_t));
        uint8_t* confidence_output;

        if (confidence_frame)
        {
            confidence_output = (uint8_t*)confidence_out.get_data();
            memcpy(confidence_output, (const uint8_t*)confidence_frame.get_data(), depth_intrinsics.width*depth_intrinsics.height * sizeof(uint8_t));
        }

        if (depth_intrinsics.width != _imsize.x || depth_intrinsics.height != _imsize.y)
        {
            _imsize.x = depth_intrinsics.width;
            _imsize.y = depth_intrinsics.height;
            _ir_edge.resize(_imsize.x*_imsize.y);
            _temp_ir_edge.resize(_imsize.x*_imsize.y);
            _temp_G.resize(_imsize.x*_imsize.y);
        }
        _ir_data = (const uint8_t*)ir_frame.get_data();
        _depth_data = (const uint16_t*)depth_frame.get_data();

        if (depth_smear_invalidation(_depth_data, _ir_data,
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

