#pragma once
namespace micropilot::rendering
{
/** Bilinear sample of a (H,W,3) image with edge clamp (matches NumpyRenderer). */
__device__ __forceinline__ void bilinear(const float* img, int W, int H, float x, float y,
                                          float& r, float& g, float& b)
{
    x = fminf(fmaxf(x, 0.0f), W - 1.0f);
    y = fminf(fmaxf(y, 0.0f), H - 1.0f);
    int x0 = (int)floorf(x), y0 = (int)floorf(y);
    int x1 = min(x0 + 1, W - 1), y1 = min(y0 + 1, H - 1);
    float wx = x - x0, wy = y - y0;
    auto P = [&](int yy, int xx, int c) { return img[((yy)*W + (xx)) * 3 + c]; };
    for (int c = 0; c < 3; ++c)
    {
        float top = P(y0, x0, c) * (1 - wx) + P(y0, x1, c) * wx;
        float bot = P(y1, x0, c) * (1 - wx) + P(y1, x1, c) * wx;
        float v = top * (1 - wy) + bot * wy;
        if (c == 0) r = v; else if (c == 1) g = v; else b = v;
    }
}
}  // namespace micropilot::rendering
