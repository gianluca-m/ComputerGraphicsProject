#if !defined(__PERLINNOISE_H__)
#define __PERLINNOISE_H__

#include <nori/object.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

// Credits to https://github.com/daniilsjb/perlin-noise

class PerlinNoise {
public:
    /// @brief Calculates the perlin noise value of a 3D input point
    static float get3DPerlinNoise(Point3f point) {
        // Top-left coordinates of the unit-cube
        int xi = int(std::floor(point.x()));
        int yi = int(std::floor(point.y()));
        int zi = int(std::floor(point.z()));

        // Input location in the unit-cube
        float xf0 = point.x() - float(xi);
        float yf0 = point.y() - float(yi);
        float zf0 = point.z() - float(zi);
        float xf1 = xf0 - 1.0f;
        float yf1 = yf0 - 1.0f;
        float zf1 = zf0 - 1.0f;

        // Wrap to range 0-255
        xi &= 255;
        yi &= 255;
        zi &= 255;

        // Generate hash values for each point of the unit-cube
        int h000 = perm[perm[perm[xi + 0] + yi + 0] + zi + 0];
        int h001 = perm[perm[perm[xi + 0] + yi + 0] + zi + 1];
        int h010 = perm[perm[perm[xi + 0] + yi + 1] + zi + 0];
        int h011 = perm[perm[perm[xi + 0] + yi + 1] + zi + 1];
        int h100 = perm[perm[perm[xi + 1] + yi + 0] + zi + 0];
        int h101 = perm[perm[perm[xi + 1] + yi + 0] + zi + 1];
        int h110 = perm[perm[perm[xi + 1] + yi + 1] + zi + 0];
        int h111 = perm[perm[perm[xi + 1] + yi + 1] + zi + 1];

        // Apply the fade function to the location
        float u = fade(xf0);
        float v = fade(yf0);
        float w = fade(zf0);

        // Linearly interpolate between dot products of each gradient with its distance to the input location
        float x11 = lerp(u, direction(h000, xf0, yf0, zf0), direction(h100, xf1, yf0, zf0));
        float x12 = lerp(u, direction(h010, xf0, yf1, zf0), direction(h110, xf1, yf1, zf0));
        float x21 = lerp(u, direction(h001, xf0, yf0, zf1), direction(h101, xf1, yf0, zf1));
        float x22 = lerp(u, direction(h011, xf0, yf1, zf1), direction(h111, xf1, yf1, zf1));

        float y1 = lerp(v, x11, x12);
        float y2 = lerp(v, x21, x22);

        return lerp(w, y1, y2);
    }


private:
    // Permutation table
    static constexpr int perm[512] = {     
        151,160,137, 91, 90, 15,131, 13,201, 95, 96, 53,194,233,  7,225,
        140, 36,103, 30, 69,142,  8, 99, 37,240, 21, 10, 23,190,  6,148,
        247,120,234, 75,  0, 26,197, 62, 94,252,219,203,117, 35, 11, 32,
         57,177, 33, 88,237,149, 56, 87,174, 20,125,136,171,168, 68,175,
         74,165, 71,134,139, 48, 27,166, 77,146,158,231, 83,111,229,122,
         60,211,133,230,220,105, 92, 41, 55, 46,245, 40,244,102,143, 54,
         65, 25, 63,161,  1,216, 80, 73,209, 76,132,187,208, 89, 18,169,
        200,196,135,130,116,188,159, 86,164,100,109,198,173,186,  3, 64,
         52,217,226,250,124,123,  5,202, 38,147,118,126,255, 82, 85,212,
        207,206, 59,227, 47, 16, 58, 17,182,189, 28, 42,223,183,170,213,
        119,248,152,  2, 44,154,163, 70,221,153,101,155,167, 43,172,  9,
        129, 22, 39,253, 19, 98,108,110, 79,113,224,232,178,185,112,104,
        218,246, 97,228,251, 34,242,193,238,210,144, 12,191,179,162,241,
         81, 51,145,235,249, 14,239,107, 49,192,214, 31,181,199,106,157,
        184, 84,204,176,115,121, 50, 45,127,  4,150,254,138,236,205, 93,
        222,114, 67, 29, 24, 72,243,141,128,195, 78, 66,215, 61,156,180,

        151,160,137, 91, 90, 15,131, 13,201, 95, 96, 53,194,233,  7,225,
        140, 36,103, 30, 69,142,  8, 99, 37,240, 21, 10, 23,190,  6,148,
        247,120,234, 75,  0, 26,197, 62, 94,252,219,203,117, 35, 11, 32,
         57,177, 33, 88,237,149, 56, 87,174, 20,125,136,171,168, 68,175,
         74,165, 71,134,139, 48, 27,166, 77,146,158,231, 83,111,229,122,
         60,211,133,230,220,105, 92, 41, 55, 46,245, 40,244,102,143, 54,
         65, 25, 63,161,  1,216, 80, 73,209, 76,132,187,208, 89, 18,169,
        200,196,135,130,116,188,159, 86,164,100,109,198,173,186,  3, 64,
         52,217,226,250,124,123,  5,202, 38,147,118,126,255, 82, 85,212,
        207,206, 59,227, 47, 16, 58, 17,182,189, 28, 42,223,183,170,213,
        119,248,152,  2, 44,154,163, 70,221,153,101,155,167, 43,172,  9,
        129, 22, 39,253, 19, 98,108,110, 79,113,224,232,178,185,112,104,
        218,246, 97,228,251, 34,242,193,238,210,144, 12,191,179,162,241,
         81, 51,145,235,249, 14,239,107, 49,192,214, 31,181,199,106,157,
        184, 84,204,176,115,121, 50, 45,127,  4,150,254,138,236,205, 93,
        222,114, 67, 29, 24, 72,243,141,128,195, 78, 66,215, 61,156,180
    };

    static float fade(float d) {
        return d * d * d * (d * (d * 6.0f - 15.0f) + 10.0f);
    }

    static float direction(int hash, float x, float y, float z) {
        switch (hash & 0xF) {
            case 0x0: return  x + y;
            case 0x1: return -x + y;
            case 0x2: return  x - y;
            case 0x3: return -x - y;
            case 0x4: return  x + z;
            case 0x5: return -x + z;
            case 0x6: return  x - z;
            case 0x7: return -x - z;
            case 0x8: return  y + z;
            case 0x9: return -y + z;
            case 0xA: return  y - z;
            case 0xB: return -y - z;
            case 0xC: return  y + x;
            case 0xD: return -y + z;
            case 0xE: return  y - x;
            case 0xF: return -y - z;
            default:  return  0.0f;
        }
    }
};

NORI_NAMESPACE_END
#endif
