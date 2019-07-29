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
void convolution(const pixel_t *in, pixel_t *out, const float *kernel,
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

void gradient(const float* f, const size_t n, float* g)
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

void smoothGradient(const pixel_t *in, pixel_t *GX, pixel_t* GY, 
                    const int nx, const int ny, const float sigma)
{
    const int n = (int)ceil(4 * sigma);
    const float c = 1 / (sqrtf(2 * (float)M_PI)*sigma);
    const int kernel_size = (n * 2) + 1;
    float* kernel = (float*)malloc(kernel_size * sizeof(float));
    float kernel_sum(0);
    for (int x = -n; x <= n; x++)
    {
        int i(x + n);
        kernel[i] = c * exp(-(x*x) / (2 * sigma*sigma));
        kernel_sum += kernel[i];
    }
    
    // Normalize to ensure kernel sums to one
    for (int i = 0; i < kernel_size; i++)
        kernel[i] /= kernel_sum;

    // Create 1 - D Derivative of Gaussian Kernel
    float* dkernel = (float*)malloc(kernel_size * sizeof(float));
    gradient(kernel, kernel_size, dkernel);

    //// Normalize to ensure kernel sums to zero
    float neg_sum(0), pos_sum(0);
    for (int i = 0; i < kernel_size; i++)
    {
        if (dkernel[i] > 0)
            pos_sum += dkernel[i];
        else
            neg_sum += dkernel[i];
    }
    //neg_sum *= -1;
    pos_sum *= -1;  // Flip dkernel to match flipped image regarding to matlab.
    for (int i = 0; i < kernel_size; i++)
    {
        if (dkernel[i] > 0)
            dkernel[i] /= pos_sum;
        else
            dkernel[i] /= neg_sum;
    }

    //float* GK = (float*)calloc(kernel_size*kernel_size, sizeof(float));
    //for (int i = 0; i < kernel_size; i++)
    //    GK[i*kernel_size + n] = kernel[i];

    int buf_size = nx * ny * sizeof(pixel_t);
    pixel_t* tempbuf = (pixel_t*)malloc(buf_size);

    convolution(in, GX, kernel, nx, ny, 1, kernel_size, false);
    memcpy(tempbuf, GX, buf_size);
    convolution(tempbuf, GX, dkernel, nx, ny, kernel_size, 1, false);

    convolution(in, GY, kernel, nx, ny, kernel_size, 1, false);
    memcpy(tempbuf, GY, buf_size);
    convolution(tempbuf, GY, dkernel, nx, ny, 1, kernel_size, false);

    // Remove kernel size boundaries:
    memset(GX, 0, nx*n * sizeof(pixel_t));
    memset(GX + (ny - n)*nx, 0, nx*n * sizeof(pixel_t));
    memset(GY, 0, nx*n*sizeof(pixel_t));
    memset(GY + (ny - n)*nx, 0, nx*n * sizeof(pixel_t));
    for (int i = n; i < ny - n; i++)
    {
        memset(GX + i * nx, 0, n * sizeof(pixel_t));
        memset(GX + i * nx + (nx - n - 1), 0, n * sizeof(pixel_t));
        memset(GY + i * nx, 0, n * sizeof(pixel_t));
        memset(GY + i * nx + (nx - n - 1), 0, n * sizeof(pixel_t));
    }

    free(tempbuf);
    free(kernel);
    free(dkernel);
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
void hysteresisThreshold(const pixel_t* in, pixel_t* out, const int nx, const int ny, const pixel_t lowThresh, const pixel_t highThresh)
{
    memset(out, 0, sizeof(pixel_t) * nx * ny);
    int *edges = (int*)calloc(nx * ny, sizeof(int));
    size_t c = 1;
    for (int j = 1; j < ny - 1; j++)
        for (int i = 1; i < nx - 1; i++) {
            if (in[c] >= highThresh && out[c] == 0) { // trace edges
                out[c] = in[c];
                int nedges = 1;
                edges[0] = (int)c;

                do {
                    nedges--;
                    const int t = edges[nedges];

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
                            edges[nedges] = nbs[k];
                            nedges++;
                        }
                } while (nedges > 0);
            }
            c++;
        }
    free(edges);
}

void canny_edge_detection(const uint8_t *in_img, uint8_t* out,
                          const int nx, const int ny,
                          const float lowThresh, const float highThresh,
                          const float sigma, pixel_t *G, pixel_t *Gx, pixel_t *Gy)
{
    //pixel_t *G = (pixel_t*)calloc(nx * ny, sizeof(pixel_t));
    //pixel_t *Gx = (pixel_t*)calloc(nx * ny, sizeof(pixel_t));
    //pixel_t *Gy = (pixel_t*)calloc(nx * ny, sizeof(pixel_t));
    pixel_t *nms = (pixel_t*)calloc(nx * ny, sizeof(pixel_t));
    pixel_t *in = (pixel_t*)malloc(nx * ny * sizeof(pixel_t));

    if (G == NULL || Gx == NULL || Gy == NULL ||
        nms == NULL || out == NULL || in == NULL ) {
        fprintf(stderr, "canny_edge_detection:"
                " Failed memory allocation(s).\n");
        exit(1);
    }

    // Copy in_img from uint8_t to pixel_t (range 0-1):
    for (int i = 0; i < nx*ny; i++)
    {
        in[i] = in_img[i] / 255.0f;
    }

    smoothGradient(in, Gx, Gy, nx, ny, sigma);

    pixel_t magmax(0);
    int max_i, max_j;
    for (int i = 1; i < nx - 1; i++)
        for (int j = 1; j < ny - 1; j++) {
            const int c = i + nx * j;
            // G[c] = abs(Gx[c]) + abs(Gy[c]);
            G[c] = (pixel_t)hypot(Gx[c], Gy[c]);
            if (G[c] > magmax)
            {
                magmax = G[c];
                max_i = i;
                max_j = j;
            }
        }
    for (int i = 0; i < nx*ny; i++)
        G[i] /= magmax;

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

            const float dir = (float)(fmod(atan2(Gy[c],
                                                 Gx[c]) + M_PI,
                                           M_PI) / M_PI) * 8;

            if (((dir <= 1 || dir > 7) && G[c] > G[ee] &&
                 G[c] > G[ww]) || // 0 deg
                ((dir > 1 && dir <= 3) && G[c] > G[nw] &&
                 G[c] > G[se]) || // 45 deg
                ((dir > 3 && dir <= 5) && G[c] > G[nn] &&
                 G[c] > G[ss]) || // 90 deg
                ((dir > 5 && dir <= 7) && G[c] > G[ne] &&
                 G[c] > G[sw]))   // 135 deg
                nms[c] = G[c];
            else
                nms[c] = 0;
        }

    // Reuse array
    // used as a stack. nx*ny/2 elements should be enough.
    //int *edges = (int*) Gy;

    hysteresisThreshold(nms, in, nx, ny, lowThresh, lowThresh);     // use in as temporary buffer.
    hysteresisThreshold(in, nms, nx, ny, lowThresh, highThresh);    // use nms as temporary buffer.

    for (int i = 0; i < nx*ny; i++)
        out[i] = MAX_BRIGHTNESS * (nms[i] != 0);

    //free(Gx);
    //free(Gy);
    //free(G);
    free(nms);
    free(in);
}

#endif // __NO_COMPILE__

