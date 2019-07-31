#include "canny.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>


#define MAX_BRIGHTNESS 255

// C99 doesn't define M_PI (GNU-C99 does)
#define M_PI 3.14159265358979323846264338327

/*
 * Loading part taken from
 * http://www.vbforums.com/showthread.php?t=261522
 * BMP info:
 * http://en.wikipedia.org/wiki/BMP_file_format
 *
 * Note: the magic number has been removed from the bmpfile_header_t
 * structure since it causes alignment problems
 *     bmpfile_magic_t should be written/read first
 * followed by the
 *     bmpfile_header_t
 * [this avoids compiler-specific alignment pragmas etc.]
 */


// Use float instead `unsigned char' so that we can
// store negative values and normalize.

// if normalize is true, map pixels to range 0..MAX_BRIGHTNESS
template<class T> 
void convolution(const T *in, float *out, const float *kernel,
    const int nx, const int ny, const int ksize_x, const int ksize_y,
    const bool normalize)
{
    // assert(kn % 2 == 1);
    // assert(nx > kn && ny > kn);
    const int khalf_x = ksize_x / 2;
    const int khalf_y = ksize_y / 2;
    float min = FLT_MAX, max = -FLT_MAX;

    if (normalize)
        for (int m = khalf_x; m < nx - khalf_x; m++)
            for (int n = khalf_y; n < ny - khalf_y; n++) {
                float pixel = 0.0;
                size_t c = 0;
                for (int j = -khalf_y; j <= khalf_y; j++)
                    for (int i = -khalf_x; i <= khalf_x; i++) {
                        pixel += in[(n - j) * nx + m - i] * kernel[c];
                        c++;
                    }
                if (pixel < min)
                    min = pixel;
                if (pixel > max)
                    max = pixel;
                }

    for (int m = khalf_x; m < nx - khalf_x; m++)
        for (int n = khalf_y; n < ny - khalf_y; n++) {
            float pixel = 0.0;
            size_t c = 0;
            for (int j = -khalf_y; j <= khalf_y; j++)
                for (int i = -khalf_x; i <= khalf_x; i++) {
                    pixel += in[(n - j) * nx + m - i] * kernel[c];
                    c++;
                }

            if (normalize)
                pixel = MAX_BRIGHTNESS * (pixel - min) / (max - min);
            out[n * nx + m] = (pixel_t)pixel;
        }
}

void convolution_uc(const uint8_t *in, float *out, const float *kernel,
    const int nx, const int ny, const int ksize_x, const int ksize_y,
    const bool normalize)
{
    convolution(in, out, kernel, nx, ny, ksize_x, ksize_x, normalize);
}
/*
 * gaussianFilter:
 * http://www.songho.ca/dsp/cannyedge/cannyedge.html
 * determine size of kernel (odd #)
 * 0.0 <= sigma < 0.5 : 3
 * 0.5 <= sigma < 1.0 : 5
 * 1.0 <= sigma < 1.5 : 7
 * 1.5 <= sigma < 2.0 : 9
 * 2.0 <= sigma < 2.5 : 11
 * 2.5 <= sigma < 3.0 : 13 ...
 * kernelSize = 2 * int(2*sigma) + 3;
 */
//void gaussian_filter(const pixel_t *in, pixel_t *out,
//                     const int nx, const int ny, const float sigma)
//{
//    const int n = 2 * (int)(2 * sigma) + 3;
//    const float mean = (float)floor(n / 2.0);
//    //float kernel[n * n]; // variable length array
//	float* kernel = (float*)malloc(n * n * sizeof(float));
//
//    fprintf(stderr, "gaussian_filter: kernel size %d, sigma=%g\n",
//            n, sigma);
//    size_t c = 0;
//    for (int i = 0; i < n; i++)
//        for (int j = 0; j < n; j++) {
//			kernel[c] = expf(-0.5f * (powf((i - mean) / sigma, 2.0f) +
//                                    powf((j - mean) / sigma, 2.0f)))
//                        / (2 * (float)M_PI * sigma * sigma);
//            fprintf(stderr, "%g, ", kernel[c]);
//            c++;
//        }
//
//    convolution(in, out, kernel, nx, ny, n, true);
//	free(kernel);
//}

void gradient(const std::vector<float> f, const size_t n, std::vector<float>& g)
{
    if (n > 1)
    {
        g[0] = f[1] - f[0];
        g[n - 1] = f[n - 1] - f[n - 2];
    }
    if (n > 2)
    {
        for (int i = 1; i < n - 1; i++)
        {
            g[i] = (f[i + 1] - f[i - 1]) / 2.0f;
        }
    }
}

void Canny::create_kernel()
{
    const int n = (int)ceil(4 * _sigma);
    const float c = 1 / (sqrtf(2 * (float)M_PI)*_sigma);
    _kernel_size = (n * 2) + 1;
    //float* kernel = (float*)malloc(kernel_size * sizeof(float));
    _kernel.resize(_kernel_size);
    _dkernel.resize(_kernel_size);
    float kernel_sum(0);
    for (int x = -n; x <= n; x++)
    {
        int i(x + n);
        _kernel[i] = c * exp(-(x*x) / (2 * _sigma*_sigma));
        kernel_sum += _kernel[i];
    }

    // Normalize to ensure kernel sums to one
    for (int i = 0; i < _kernel_size; i++)
        _kernel[i] /= kernel_sum;

    // Create 1 - D Derivative of Gaussian Kernel
    //float* dkernel = (float*)malloc(kernel_size * sizeof(float));
    gradient(_kernel, _kernel_size, _dkernel);

    //// Normalize to ensure kernel sums to zero
    float neg_sum(0), pos_sum(0);
    for (int i = 0; i < _kernel_size; i++)
    {
        if (_dkernel[i] > 0)
            pos_sum += _dkernel[i];
        else
            neg_sum += _dkernel[i];
    }
    //neg_sum *= -1;
    pos_sum *= -1;  // Flip dkernel to match flipped image regarding to matlab.
    for (int i = 0; i < _kernel_size; i++)
    {
        if (_dkernel[i] > 0)
            _dkernel[i] /= pos_sum;
        else
            _dkernel[i] /= neg_sum;
    }
}

void Canny::smoothGradients()
{

    //float* GK = (float*)calloc(kernel_size*kernel_size, sizeof(float));
    //for (int i = 0; i < kernel_size; i++)
    //    GK[i*kernel_size + n] = kernel[i];

    int buf_size = _nx * _ny * sizeof(pixel_t);
    //pixel_t* tempbuf = (pixel_t*)malloc(buf_size);

    convolution(_in.data(), _Gx.data(), _kernel.data(), _nx, _ny, 1, _kernel_size, false);
    //{
    //    FILE* fout = fopen("in.bin", "wb");
    //    fwrite(_in.data(), sizeof(pixel_t), _nx*_ny, fout);
    //    fclose(fout);
    //}
    //{
    //    FILE* fout = fopen("kernel.bin", "wb");
    //    fwrite(_kernel.data(), sizeof(pixel_t), 13, fout);
    //    fclose(fout);
    //}
    //{
    //    FILE* fout = fopen("gx.bin", "wb");
    //    fwrite(_Gx.data(), sizeof(pixel_t), _nx*_ny, fout);
    //    fclose(fout);
    //}

    memcpy(_conv_buf.data(), _Gx.data(), buf_size);
    //{
    //    FILE* fout = fopen("buf.bin", "wb");
    //    fwrite(_conv_buf.data(), sizeof(pixel_t), _nx*_ny, fout);
    //    fclose(fout);
    //}
    //memcpy(tempbuf, GX, buf_size);
    //{
    //    FILE* fout = fopen("dkernel.bin", "wb");
    //    fwrite(_dkernel.data(), sizeof(pixel_t), 13, fout);
    //    fclose(fout);
    //}
    convolution(_conv_buf.data(), _Gx.data(), _dkernel.data(), _nx, _ny, _kernel_size, 1, false);
    //{
    //    FILE* fout = fopen("gx.bin", "wb");
    //    fwrite(_Gx.data(), sizeof(pixel_t), _nx*_ny, fout);
    //    fclose(fout);
    //}

    convolution(_in.data(), _Gy.data(), _kernel.data(), _nx, _ny, _kernel_size, 1, false);
    memcpy(_conv_buf.data(), _Gy.data(), buf_size);
    convolution(_conv_buf.data(), _Gy.data(), _dkernel.data(), _nx, _ny, 1, _kernel_size, false);

    const int n((_kernel_size - 1) / 2);
    // Remove kernel size boundaries:
    memset(_Gx.data(), 0, _nx*n * sizeof(pixel_t));                     // Erase top n rows.
    memset(_Gx.data() + (_ny - n)*_nx, 0, _nx*n * sizeof(pixel_t));     // Erase bottom n rows.
    memset(_Gy.data(), 0, _nx*n*sizeof(pixel_t));                       // Erase top n rows.
    memset(_Gy.data() + (_ny - n)*_nx, 0, _nx*n * sizeof(pixel_t));     // Erase bottom n rows.
    for (int i = n; i < _ny - n; i++)                                    // Erase n left and right columns.
    {
        memset(_Gx.data() + i * _nx, 0, n * sizeof(pixel_t));
        memset(_Gx.data() + i * _nx + (_nx - n - 1), 0, n * sizeof(pixel_t));
        memset(_Gy.data() + i * _nx, 0, n * sizeof(pixel_t));
        memset(_Gy.data() + i * _nx + (_nx - n - 1), 0, n * sizeof(pixel_t));
    }

    //free(tempbuf);
    //free(kernel);
    //free(dkernel);
}

#ifndef __NO_COMPILE__

/*
 * Links:
 * http://en.wikipedia.org/wiki/Canny_edge_detector
 * http://www.tomgibara.com/computer-vision/CannyEdgeDetector.java
 * http://fourier.eng.hmc.edu/e161/lectures/canny/node1.html
 * http://www.songho.ca/dsp/cannyedge/cannyedge.html
 *
 * Note: T1 and T2 are lower and upper thresholds.
 */

// Tracing edges with hysteresis . Non-recursive implementation.
// return original values on traces.
void Canny::hysteresisThreshold(const std::vector<pixel_t> in, std::vector<pixel_t>& out, const int nx, const int ny, const pixel_t lowThresh, const pixel_t highThresh)
{
    std::fill(out.begin(), out.end(), 0.0f);
    int *edges = (int*)calloc(nx * ny, sizeof(int));
    size_t c = 1;
    for (int j = 1; j < ny - 1; j++)
        for (int i = 1; i < nx - 1; i++) {
            if (in[c] >= highThresh && out[c] == 0) { // trace edges
                out[c] = in[c];
                int nedges = 1;
                _hys_edges[0] = (int)c;

                do {
                    nedges--;
                    const int t = _hys_edges[nedges];

                    int nbs[8]; // neighbours
                    nbs[0] = t - nx;     // nn
                    nbs[1] = t + nx;     // ss
                    nbs[2] = t + 1;      // ww
                    nbs[3] = t - 1;      // ee
                    nbs[4] = nbs[0] + 1; // nw
                    nbs[5] = nbs[0] - 1; // ne
                    nbs[6] = nbs[1] + 1; // sw
                    nbs[7] = nbs[1] - 1; // se

                    for (int k = 0; k < 8; k++)
                        if (in[nbs[k]] >= lowThresh && out[nbs[k]] == 0) {
                            out[nbs[k]] = in[nbs[k]];
                            _hys_edges[nedges] = nbs[k];
                            nedges++;
                        }
                } while (nedges > 0);
            }
            c++;
        }
}

void Canny::init_buffers()
{
    _G.resize(_nx*_ny);
    _Gx.resize(_nx*_ny);
    _Gy.resize(_nx*_ny);
    _nms.resize(_nx*_ny);
    _in.resize(_nx*_ny);
    _conv_buf.resize(_nx*_ny);
    _hys_edges.resize(_nx*_ny);
}

void Canny::set_zero_buffers()
{
    std::fill(_G.begin(), _G.end(), 0.0f);
    std::fill(_Gx.begin(), _Gx.end(), 0.0f);
    std::fill(_Gy.begin(), _Gy.end(), 0.0f);
    std::fill(_nms.begin(), _nms.end(), 0.0f);
    std::fill(_hys_edges.begin(), _hys_edges.end(), 0);
}

void Canny::unite_gradients()
{
    pixel_t magmax(0);
    int max_i, max_j;
    for (int i = 1; i < _nx - 1; i++)
        for (int j = 1; j < _ny - 1; j++) {
            const int c = i + _nx * j;
            _G[c] = (pixel_t)hypot(_Gx[c], _Gy[c]);
            if (_G[c] > magmax)
            {
                magmax = _G[c];
                max_i = i;
                max_j = j;
            }
        }
    for (int i = 0; i < _nx*_ny; i++)
        _G[i] /= magmax;
}

void Canny::canny_edge_detection(const uint8_t *in_img, std::vector<uint8_t>& out,
                          const int nx, const int ny,
                          const float lowThresh, const float highThresh,
                          const float sigma)
{
    if (_nx != nx || _ny != ny)
    {
        _nx = nx;
        _ny = ny;
        init_buffers();
    }
    if (_sigma || sigma)
    {
        _sigma = sigma;
        create_kernel();
    }
    set_zero_buffers();

    // Copy in_img from uint8_t to pixel_t (range 0-1):
    for (int i = 0; i < nx*ny; i++)
    {
        _in[i] = in_img[i] / (float)MAX_BRIGHTNESS;
    }

    smoothGradients();  // calc _Gx, _Gy
    unite_gradients();  // calc _G


    // thinAndThreshold(Gx, Gy, G, lowThresh, highThresh, out);
    // Non-maximum suppression, straightforward implementation.
    for (int i = 1; i < nx - 1; i++)
        for (int j = 1; j < ny - 1; j++) {
            const int c = i + nx * j;
            const int nn = c - nx;
            const int ss = c + nx;
            const int ww = c + 1;
            const int ee = c - 1;
            const int nw = nn + 1;
            const int ne = nn - 1;
            const int sw = ss + 1;
            const int se = ss - 1;

            const float dir = (float)(fmod(atan2(_Gy[c],
                                                 _Gx[c]) + M_PI,
                                           M_PI) / M_PI) * 8;

            if (((dir <= 1 || dir > 7) && _G[c] > _G[ee] &&
                 _G[c] > _G[ww]) || // 0 deg
                ((dir > 1 && dir <= 3) && _G[c] > _G[nw] &&
                 _G[c] > _G[se]) || // 45 deg
                ((dir > 3 && dir <= 5) && _G[c] > _G[nn] &&
                 _G[c] > _G[ss]) || // 90 deg
                ((dir > 5 && dir <= 7) && _G[c] > _G[ne] &&
                 _G[c] > _G[sw]))   // 135 deg
                _nms[c] = _G[c];
            else
                _nms[c] = 0;
        }

    // Reuse array
    // used as a stack. nx*ny/2 elements should be enough.
    //int *edges = (int*) Gy;
    //{
    //    FILE* fout = fopen("nms.bin", "wb");
    //    fwrite(_nms.data(), sizeof(_nms[0]), nx*ny, fout);
    //    fclose(fout);
    //}
    //{
    //    FILE* fout = fopen("Gy.bin", "wb");
    //    fwrite(_Gy.data(), sizeof(pixel_t), nx*ny, fout);
    //    fclose(fout);
    //}
    //{
    //    FILE* fout = fopen("G.bin", "wb");
    //    fwrite(_G.data(), sizeof(pixel_t), nx*ny, fout);
    //    fclose(fout);
    //}
    hysteresisThreshold(_nms, _in, nx, ny, lowThresh, lowThresh);     // use in as temporary buffer.
    hysteresisThreshold(_in, _nms, nx, ny, lowThresh, highThresh);    // use nms as temporary buffer.

    for (int i = 0; i < nx*ny; i++)
        out[i] = MAX_BRIGHTNESS * (_nms[i] != 0);
}

#endif // __NO_COMPILE__

