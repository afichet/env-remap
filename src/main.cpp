/**
 * Copyright (c) 2021 Alban Fichet
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *  * Neither the name of %ORGANIZATION% nor the names of its contributors may be
 * used to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>
#include <iostream>

#define TINYEXR_IMPLEMENTATION
#include <tinyexr.h>

void pano_getThetaPhi(float u, float v, float& theta, float& phi)
{
    // u in [0..2]
    // v in [0..1]
    u *= 2.f;

    theta = M_PI * (u - 1);
    phi = M_PI * v;
}

void pano_getDir(float u, float v, float& dx, float& dy, float& dz)
{
    float theta, phi;
    pano_getThetaPhi(u, v, theta, phi);

    dx = sin(phi) * sin(theta);
    dy = cos(phi);
    dz = -sin(phi)*cos(theta);
}

void pano_getUV(float dx, float dy, float dz, float& u, float& v)
{
    u = 1 + atan2(dx, -dz) / M_PI;
    v = acos(dy) / M_PI;

    u /= 2.f;
}

/*
 Thus, if we consider the images to be normalized to have coordinates u=[-1,1], v=[-1,1], we have theta=atan2(v,u), phi=pi*sqrt(u*u+v*v). The unit vector pointing in the corresponding direction is obtained by rotating (0,0,-1) by phi degrees around the y (up) axis and then theta degrees around the -z (forward) axis. If for a direction vector in the world (Dx, Dy, Dz), the corresponding (u,v) coordinate in the light probe image is (Dx*r,Dy*r) where r=(1/pi)*acos(Dz)/sqrt(Dx^2 + Dy^2).
 */
void probe_getThetaPhi(float u, float v, float& theta, float& phi)
{
    // u, v in [-1, 1]
    u = 2.f * u - 1.f;
    v = 2.f * v - 1.f;

    theta = atan2(v, u);
    phi = M_PI * sqrt(u * u + v * v);
}

void probe_getUV(float dx, float dy, float dz, float& u, float& v)
{
    const float r = (1.f / M_PI) * acos(dz) / sqrt(dx * dx + dy * dy);

    u = dx * r;
    v = dy * r;

    // remap in 0..1
    u = (u + 1.f) / 2.f;
    v = (v + 1.f) / 2.f;
}

bool SaveEXR(const float* rgb, int width, int height, const char* outfilename);

int main(int argc, char* argv[])
{
    const char* input = argv[1];
    const char* output = argv[2];
    float* rgba; // width * height * RGBA
    int width;
    int height;
    const char* err = nullptr;

    int ret = LoadEXR(&rgba, &width, &height, input, &err);

    if (ret != TINYEXR_SUCCESS) {
        if (err) {
            fprintf(stderr, "ERR : %s\n", err);
            FreeEXRErrorMessage(err); // release memory of error message.
        }

        return -1;
    }

    const size_t out_width = 1024;
    const size_t out_height = 512;
    float* out_rgba = new float[out_width * out_height * 4];

    for (size_t y = 0; y < out_height; y++) {
        float v_target = 1.f - float(y) / float(out_height);
        for (size_t x = 0; x < out_width; x++) {
            float u_target = float(x) / float(out_width);
            float dx, dy, dz;
            pano_getDir(u_target, v_target, dx, dy, dz);

            float u_source, v_source;
            probe_getUV(dx, dy, dz, u_source, v_source);

            size_t x_source = u_source * width;
            size_t y_source = v_source * height;

            for (int c = 0; c < 3; c++) {
                out_rgba[4 * (y * out_width + x) + c] = rgba[4 * (y_source * width + x_source) + c];
            }

            out_rgba[4 * (y * out_width + x) + 3] = 1.f;
        }
    }

    SaveEXR(out_rgba, out_width, out_height, output);

    delete[] out_rgba;

    free(rgba); // release memory of image data

    return 0;
}

// See `examples/rgbe2exr/` for more details.
bool SaveEXR(const float* rgba, int width, int height, const char* outfilename)
{
    EXRHeader header;
    InitEXRHeader(&header);

    EXRImage image;
    InitEXRImage(&image);

    image.num_channels = 3;

    std::vector<float> images[3];

    // Split RGBRGBRGB... into R, G and B layer
    for (size_t c = 0; c < image.num_channels; c++) {
        images[c].resize(width * height);

        for (int i = 0; i < width * height; i++) {
            images[c][i] = rgba[4 * i + c];
        }
    }

    float* image_ptr[3];
    image_ptr[0] = &(images[2].at(0)); // B
    image_ptr[1] = &(images[1].at(0)); // G
    image_ptr[2] = &(images[0].at(0)); // R
    // image_ptr[3] = &(images[3].at(0)); // A

    image.images = (unsigned char**)image_ptr;
    image.width = width;
    image.height = height;

    header.num_channels = 3;
    header.channels = (EXRChannelInfo*)malloc(sizeof(EXRChannelInfo) * header.num_channels);
    // Must be (A)BGR order, since most of EXR viewers expect this channel order.
    strncpy(header.channels[0].name, "B", 255);
    header.channels[0].name[strlen("B")] = '\0';
    strncpy(header.channels[1].name, "G", 255);
    header.channels[1].name[strlen("G")] = '\0';
    strncpy(header.channels[2].name, "R", 255);
    header.channels[2].name[strlen("R")] = '\0';
    // strncpy(header.channels[3].name, "A", 255);
    // header.channels[3].name[strlen("A")] = '\0';

    header.pixel_types = (int*)malloc(sizeof(int) * header.num_channels);
    header.requested_pixel_types = (int*)malloc(sizeof(int) * header.num_channels);
    for (int i = 0; i < header.num_channels; i++) {
        header.pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT; // pixel type of input image
        header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT; // pixel type of output image to be stored in .EXR
    }

    const char* err = nullptr;
    int ret = SaveEXRImageToFile(&image, &header, outfilename, &err);
    if (ret != TINYEXR_SUCCESS) {
        fprintf(stderr, "Save EXR err: %s\n", err);
        FreeEXRErrorMessage(err); // free's buffer for an error message
        return false;
    }

    printf("Saved exr file. [ %s ] \n", outfilename);

    free(header.channels);
    free(header.pixel_types);
    free(header.requested_pixel_types);

    return true;
}