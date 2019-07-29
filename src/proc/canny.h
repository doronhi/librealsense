#include "../include/librealsense2/hpp/rs_types.hpp"
#include <stdint.h>

typedef float pixel_t;

void canny_edge_detection(const uint8_t *in_img, uint8_t* out,
    const int nx, const int ny,
    const float lowThresh, const float highThresh,
    const float sigma, pixel_t *G, pixel_t *Gx, pixel_t *Gy);