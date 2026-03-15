#pragma once

#include <cstdint>

#ifdef CARLA_VERSION_0100

#include "carla/rpc/Texture.h"

namespace carla_rust {
namespace rpc {

class FfiTextureColor {
public:
    FfiTextureColor(uint32_t width, uint32_t height) : inner_(width, height) {}

    uint32_t GetWidth() const { return inner_.GetWidth(); }
    uint32_t GetHeight() const { return inner_.GetHeight(); }

    void SetPixel(uint32_t x, uint32_t y, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
        inner_.At(x, y) = carla::sensor::data::Color(r, g, b, a);
    }

    uint8_t GetPixelR(uint32_t x, uint32_t y) const { return inner_.At(x, y).r; }
    uint8_t GetPixelG(uint32_t x, uint32_t y) const { return inner_.At(x, y).g; }
    uint8_t GetPixelB(uint32_t x, uint32_t y) const { return inner_.At(x, y).b; }
    uint8_t GetPixelA(uint32_t x, uint32_t y) const { return inner_.At(x, y).a; }

    const carla::rpc::TextureColor& inner() const { return inner_; }
    carla::rpc::TextureColor& inner_mut() { return inner_; }

private:
    carla::rpc::TextureColor inner_;
};

class FfiTextureFloatColor {
public:
    FfiTextureFloatColor(uint32_t width, uint32_t height) : inner_(width, height) {}

    uint32_t GetWidth() const { return inner_.GetWidth(); }
    uint32_t GetHeight() const { return inner_.GetHeight(); }

    void SetPixel(uint32_t x, uint32_t y, float r, float g, float b, float a) {
        inner_.At(x, y) = carla::rpc::FloatColor(r, g, b, a);
    }

    float GetPixelR(uint32_t x, uint32_t y) const { return inner_.At(x, y).r; }
    float GetPixelG(uint32_t x, uint32_t y) const { return inner_.At(x, y).g; }
    float GetPixelB(uint32_t x, uint32_t y) const { return inner_.At(x, y).b; }
    float GetPixelA(uint32_t x, uint32_t y) const { return inner_.At(x, y).a; }

    const carla::rpc::TextureFloatColor& inner() const { return inner_; }
    carla::rpc::TextureFloatColor& inner_mut() { return inner_; }

private:
    carla::rpc::TextureFloatColor inner_;
};

}  // namespace rpc
}  // namespace carla_rust

#else  // !CARLA_VERSION_0100

namespace carla_rust {
namespace rpc {

// Stub types for versions prior to 0.10.0
struct FfiTextureColor {
    uint8_t _unused;
};

struct FfiTextureFloatColor {
    uint8_t _unused;
};

}  // namespace rpc
}  // namespace carla_rust

#endif  // CARLA_VERSION_0100
