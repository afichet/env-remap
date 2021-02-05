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

#pragma once
#include <cmath>

// Probe parametrisation: https://www.pauldebevec.com/Probes/
bool sphere_getThetaPhi(float u, float v, float& theta, float& phi)
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

bool sphere_getDir(float u, float v, float& dx, float& dy, float& dz)
{
    float theta, phi;
    const bool dirExists = sphere_getThetaPhi(u, v, theta, phi);

    if (dirExists) {
        dx = sin(phi) * sin(theta);
        dy = cos(phi);
        dz = -sin(phi) * cos(theta);
    } else {
        return false;
    }

    return true;
}

void sphere_getUV(float dx, float dy, float dz, float& u, float& v)
{
    const float r = (1.f / M_PI) * acos(dz) / sqrt(dx * dx + dy * dy);

    u = dx * r;
    v = dy * r;

    // remap in 0..1
    u = (u + 1.f) / 2.f;
    v = (v + 1.f) / 2.f;
}