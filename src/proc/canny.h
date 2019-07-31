#include "../include/librealsense2/hpp/rs_types.hpp"
#include <stdint.h>

typedef float pixel_t;

template<class T> 
void convolution(const T *in, float *out, const float *kernel,
                 const int nx, const int ny, const int ksize_x, const int ksize_y,
                 const bool normalize);


void convolution_uc(const uint8_t *in, float *out, const float *kernel,
    const int nx, const int ny, const int ksize_x, const int ksize_y,
    const bool normalize);

class Canny
{
public:
    void canny_edge_detection(const uint8_t *in_img, std::vector<uint8_t>& out,
        const int nx, const int ny,
        const float lowThresh, const float highThresh,
        const float sigma);

private:
    void Canny::create_kernel();
    void init_buffers();
    void set_zero_buffers();
    void smoothGradients();
    void unite_gradients();
    void hysteresisThreshold(const std::vector<pixel_t> in, std::vector<pixel_t>& out, const int nx, const int ny, const pixel_t lowThresh, const pixel_t highThresh);

private:
    std::vector<pixel_t> _G;
    std::vector<pixel_t> _Gx;
    std::vector<pixel_t> _Gy;
    std::vector<pixel_t> _nms;
    std::vector<pixel_t> _in;
    std::vector<pixel_t> _conv_buf;
    std::vector<int>     _hys_edges;
    int                  _nx, _ny;
    float                _sigma;
    std::vector<float>   _kernel, _dkernel;
    int                  _kernel_size;

};