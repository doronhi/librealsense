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

void canny_edge_detection(const uint8_t *in_img, uint8_t* out,
    const int nx, const int ny,
    const float lowThresh, const float highThresh,
    const float sigma);