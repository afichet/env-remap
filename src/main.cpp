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
#include <tclap/CmdLine.h>
#include <tinyexr.h>

bool SaveEXR(const float* rgb, int width, int height, const char* outfilename);

// Parnoramic parametrisation: https://vgl.ict.usc.edu/Data/HighResProbes/
bool pano_getThetaPhi(float u, float v, float& theta, float& phi)
{
    // u in [0..2]
    // v in [0..1]
    u *= 2.f;

    theta = M_PI * (u - 1);
    phi = M_PI * v;

    return true;
}

bool pano_getDir(float u, float v, float& dx, float& dy, float& dz)
{
    float theta, phi;
    pano_getThetaPhi(u, v, theta, phi);

    dx = sin(phi) * sin(theta);
    dy = cos(phi);
    dz = -sin(phi) * cos(theta);

    return true;
}

void pano_getUV(float dx, float dy, float dz, float& u, float& v)
{
    u = 1 + atan2(dx, -dz) / M_PI;
    v = acos(dy) / M_PI;

    u /= 2.f;
}

// Probe parametrisation: https://www.pauldebevec.com/Probes/
bool probe_getThetaPhi(float u, float v, float& theta, float& phi)
{
    // u, v in [-1, 1]
    u = 2.f * u - 1.f;
    v = 2.f * v - 1.f;

    if (u*u + v*v < 1.) {
        theta = atan2(v, u);
        phi = M_PI * sqrt(u * u + v * v);
    } else {
        return false;
    }

    return true;
}

bool probe_getDir(float u, float v, float& dx, float& dy, float& dz)
{
    float theta, phi;
    const bool dirExists = probe_getThetaPhi(u, v, theta, phi);

    if (dirExists) {
        dx = sin(phi) * sin(theta);
        dy = cos(phi);
        dz = -sin(phi) * cos(theta);
    } else {
        return false;
    }

    return true;
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

int main(int argc, char* argv[])
{
    std::string input, output;

    float* in_rgba; // width * height * RGBA
    int in_width;
    int in_height;
    size_t out_width = 1024;
    size_t out_height = 512;

    const char* err = nullptr;

    void (*source_getUV)(float, float, float, float&, float&) = nullptr;
    bool (*target_getDir)(float, float, float&, float&, float&) = nullptr;

    try {
        TCLAP::CmdLine cmd("Conversion of envmap parametrizations", ' ', "0.9");

        TCLAP::ValueArg<std::string> sourceArg("s", "source", "Source parametrization (probe or pano)", true, "probe", "string");
        TCLAP::ValueArg<std::string> targetArg("t", "target", "Target parametrization (probe or pano)", true, "pano", "string");
        TCLAP::ValueArg<std::string> inputArg("i", "in", "Input image (EXR)", true, "in", "string");
        TCLAP::ValueArg<std::string> outputArg("o", "out", "Output image (EXR)", true, "out", "string");
        TCLAP::ValueArg<int> sizeArg("w", "width", "Output image width", false, 512, "Integer");

        cmd.add(sourceArg);
        cmd.add(targetArg);
        cmd.add(inputArg);
        cmd.add(outputArg);
        cmd.add(sizeArg);

        // Parse the argv array.
        cmd.parse(argc, argv);

        // Get the value parsed by each arg.
        std::string sourceParam = sourceArg.getValue();
        std::string targetParam = targetArg.getValue();
        input = inputArg.getValue();
        output = outputArg.getValue();

        if (sourceParam == "probe") {
            source_getUV = &probe_getUV;
        } else if (sourceParam == "pano") {
            source_getUV = &pano_getUV;
        } else {
            std::cerr << "Source parametrization is incorrect. It can be only [probe, pano]" << std::endl;
            return -1;
        }

        if (targetParam == "probe") {
            target_getDir = &probe_getDir;
            out_width = sizeArg.getValue();
            out_height = out_width;
        } else if (targetParam == "pano") {
            target_getDir = &pano_getDir;
            out_width = sizeArg.getValue();
            out_height = out_width / 2;
        } else {
            std::cerr << "Target parametrization is incorrect. It can be only [probe, pano]" << std::endl;
            return -1;
        }

    } catch (TCLAP::ArgException& e) // catch exceptions
    {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }

    int ret = LoadEXR(&in_rgba, &in_width, &in_height, input.c_str(), &err);

    if (ret != TINYEXR_SUCCESS) {
        if (err) {
            fprintf(stderr, "ERR : %s\n", err);
            FreeEXRErrorMessage(err); // release memory of error message.
        }

        return -1;
    }

    float* out_rgb = new float[3 * out_width * out_height];

    for (size_t y = 0; y < out_height; y++) {
        float v_target = 1.f - float(y) / float(out_height);

        for (size_t x = 0; x < out_width; x++) {
            float u_target = float(x) / float(out_width);
            float u_source, v_source;
            float dx, dy, dz;

            const bool dir_exists = target_getDir(u_target, v_target, dx, dy, dz);
            source_getUV(dx, dy, dz, u_source, v_source);
            
            if ( dir_exists
             && u_source >= 0.f && u_source <= 1.f
             && v_source >= 0.f && v_source <= 1.f) { 
                const size_t x_source = u_source * (in_width - 1);
                const size_t y_source = v_source * (in_height - 1);

                for (int c = 0; c < 3; c++) {
                    out_rgb[3 * (y * out_width + x) + c] = in_rgba[4 * (y_source * in_width + x_source) + c];
                }
            } else {
                for (int c = 0; c < 3; c++) {
                    out_rgb[3 * (y * out_width + x) + c] = 0.f;
                }
            }
        }
    }

    SaveEXR(out_rgb, out_width, out_height, output.c_str());

    free(in_rgba);
    delete[] out_rgb;

    return 0;
}

// See `examples/rgbe2exr/` for more details.
bool SaveEXR(const float* rgb, int width, int height, const char* outfilename)
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

        for (size_t i = 0; i < width * height; i++) {
            images[c][i] = rgb[3 * i + c];
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