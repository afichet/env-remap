#pragma once
#include <cmath>

// https://en.wikipedia.org/wiki/Cube_mapping
void cube_getDir(int index, float u, float v, float& dx, float &dy, float &dz) {
  // convert range 0 to 1 to -1 to 1
  float uc = 2.0f * u - 1.0f;
  float vc = 2.0f * v - 1.0f;

  switch (index)
  {
    case 0: dx =  1.0f; dy =    vc; dz =   -uc; break;	// POSITIVE X
    case 1: dx = -1.0f; dy =    vc; dz =    uc; break;	// NEGATIVE X
    case 2: dx =    uc; dy =  1.0f; dz =   -vc; break;	// POSITIVE Y
    case 3: dx =    uc; dy = -1.0f; dz =    vc; break;	// NEGATIVE Y
    case 4: dx =    uc; dy =    vc; dz =  1.0f; break;	// POSITIVE Z
    case 5: dx =   -uc; dy =    vc; dz = -1.0f; break;	// NEGATIVE Z
  }
}

void cube_getUV(float dx, float dy, float dz, int &index, float &u, float &v)
{
  float absX = fabs(dx);
  float absY = fabs(dy);
  float absZ = fabs(dz);
  
  const bool isXPositive = dx > 0 ? 1 : 0;
  const bool isYPositive = dy > 0 ? 1 : 0;
  const bool isZPositive = dz > 0 ? 1 : 0;
  
  float maxAxis, uc, vc;
  
  // POSITIVE X
  if (isXPositive && absX >= absY && absX >= absZ) {
    // u (0 to 1) goes from +z to -z
    // v (0 to 1) goes from -y to +y
    maxAxis = absX;
    uc = -dz;
    vc = dy;
    index = 0;
  }
  // NEGATIVE X
  if (!isXPositive && absX >= absY && absX >= absZ) {
    // u (0 to 1) goes from -z to +z
    // v (0 to 1) goes from -y to +y
    maxAxis = absX;
    uc = dz;
    vc = dy;
    index = 1;
  }
  // POSITIVE Y
  if (isYPositive && absY >= absX && absY >= absZ) {
    // u (0 to 1) goes from -x to +x
    // v (0 to 1) goes from +z to -z
    maxAxis = absY;
    uc = dx;
    vc = -dz;
    index = 2;
  }
  // NEGATIVE Y
  if (!isYPositive && absY >= absX && absY >= absZ) {
    // u (0 to 1) goes from -x to +x
    // v (0 to 1) goes from -z to +z
    maxAxis = absY;
    uc = dx;
    vc = dz;
    index = 3;
  }
  // POSITIVE Z
  if (isZPositive && absZ >= absX && absZ >= absY) {
    // u (0 to 1) goes from -x to +x
    // v (0 to 1) goes from -y to +y
    maxAxis = absZ;
    uc = dx;
    vc = dy;
    index = 4;
  }
  // NEGATIVE Z
  if (!isZPositive && absZ >= absX && absZ >= absY) {
    // u (0 to 1) goes from +x to -x
    // v (0 to 1) goes from -y to +y
    maxAxis = absZ;
    uc = -dx;
    vc = dy;
    index = 5;
  }

  // Convert range from -1 to 1 to 0 to 1
  u = 0.5f * (uc / maxAxis + 1.0f);
  v = 0.5f * (vc / maxAxis + 1.0f);
}

void cubehorizcross_getUV(float dx, float dy, float dz, float& u, float& v) {
  int index;

  cube_getUV(-dx, dy, dz, index, u, v);

  switch (index) {
    case 0: u = 1.f/4.f * u + 1.f/2.f; v = 1.f/3.f * v + 1.f/3.f; break; // Positive X
    case 1: u = 1.f/4.f * u          ; v = 1.f/3.f * v + 1.f/3.f; break; // Negative X
    case 2: u = 1.f/4.f * u + 1.f/4.f; v = 1.f/3.f * v + 2.f/3.f; break; // Positive Y
    case 3: u = 1.f/4.f * u + 1.f/4.f; v = 1.f/3.f * v          ; break; // Negative Y
    case 4: u = 1.f/4.f * u + 1.f/4.f; v = 1.f/3.f * v + 1.f/3.f; break; // Positive Z
    case 5: u = 1.f/4.f * u + 3.f/4.f; v = 1.f/3.f * v + 1.f/3.f; break; // Negative Z
  }

  // TODO: Shamefull fix
  v = 1.f - v;
}

bool cubehorizcross_getDir(float u, float v, float &dx, float &dy, float &dz) {
  int index;

  // TODO: Shamefull fix
  v = 1.f - v;
  
  if (v < 1.f/3.f) {
    if (u > 1.f/4.f && u < 1.f/2.f) { // Negative Y
      index = 3; 
      u = 4.f * (u - 1.f/4.f);
      v *= 3.f;
    } else {
      return false;
    }
  }
  else if (v > 2.f/3.f) {
    if (u > 1.f/4.f && u < 1.f/2.f) { // Positive Y
      index = 2;
      u = 4.f * (u - 1.f/4.f);
      v = 3.f * (v - 2.f/3.f);
    } else {
      return false;
    }
  }
  else {
    v = 3.f * (v - 1.f/3.f);
    if (u < 1.f/4.f) {              // Negative X
      index = 1;
      u *= 4.f;
    } else if (u < 1.f/2.f) {       // Positive Z
      index = 4;
      u = 4.f * (u - 1.f/4.f);
    } else if (u < 3.f / 4.f) {     // Positive X
      index = 0;
      u = 4.f * (u - 1.f / 2.f);
    } else {                        // Negative Z
      index = 5;
      u = 4.f * (u - 3.f / 4.f);
    }
  }

  cube_getDir(index, u, v, dx, dy, dz);

  dx = -dx;
  return true;
}