#pragma once

#include "carla/client/DebugHelper.h"
#include "carla/geom/BoundingBox.h"
#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/sensor/data/Color.h"
#include "../geom.hpp"
#include "../sensor/data/color.hpp"
#include <string>

namespace carla_rust {
namespace client {
using carla::client::DebugHelper;
using carla::geom::BoundingBox;
using carla::geom::Location;
using carla::geom::Rotation;
using carla::sensor::data::Color;

// FFI wrapper for DebugHelper
class FfiDebugHelper {
public:
    FfiDebugHelper(DebugHelper&& helper) : helper_(std::move(helper)) {}

    // Draw a point in the simulation
    void DrawPoint(const Location& location, float size, const Color& color, float life_time,
                   bool persistent_lines) const {
        helper_.DrawPoint(location, size, color, life_time, persistent_lines);
    }

    // Draw a line between two points
    void DrawLine(const Location& begin, const Location& end, float thickness, const Color& color,
                  float life_time, bool persistent_lines) const {
        helper_.DrawLine(begin, end, thickness, color, life_time, persistent_lines);
    }

    // Draw an arrow from begin to end
    void DrawArrow(const Location& begin, const Location& end, float thickness, float arrow_size,
                   const Color& color, float life_time, bool persistent_lines) const {
        helper_.DrawArrow(begin, end, thickness, arrow_size, color, life_time, persistent_lines);
    }

    // Draw a bounding box
    void DrawBox(const BoundingBox& box, const Rotation& rotation, float thickness,
                 const Color& color, float life_time, bool persistent_lines) const {
        helper_.DrawBox(box, rotation, thickness, color, life_time, persistent_lines);
    }

    // Draw a text string at a location
    void DrawString(const Location& location, const std::string& text, bool draw_shadow,
                    const Color& color, float life_time, bool persistent_lines) const {
        helper_.DrawString(location, text, draw_shadow, color, life_time, persistent_lines);
    }

private:
    mutable DebugHelper helper_;
};

// Free functions for FFI using POD types
inline void FfiDebugHelper_DrawPoint(const FfiDebugHelper& helper,
                                     const geom::FfiLocation& location, float size,
                                     const sensor::data::FfiColor& color, float life_time,
                                     bool persistent_lines) {
    const_cast<FfiDebugHelper&>(helper).DrawPoint(
        location.as_native(), size, const_cast<sensor::data::FfiColor&>(color).as_builtin(),
        life_time, persistent_lines);
}

inline void FfiDebugHelper_DrawLine(const FfiDebugHelper& helper, const geom::FfiLocation& begin,
                                    const geom::FfiLocation& end, float thickness,
                                    const sensor::data::FfiColor& color, float life_time,
                                    bool persistent_lines) {
    const_cast<FfiDebugHelper&>(helper).DrawLine(
        begin.as_native(), end.as_native(), thickness,
        const_cast<sensor::data::FfiColor&>(color).as_builtin(), life_time, persistent_lines);
}

inline void FfiDebugHelper_DrawArrow(const FfiDebugHelper& helper, const geom::FfiLocation& begin,
                                     const geom::FfiLocation& end, float thickness,
                                     float arrow_size, const sensor::data::FfiColor& color,
                                     float life_time, bool persistent_lines) {
    const_cast<FfiDebugHelper&>(helper).DrawArrow(
        begin.as_native(), end.as_native(), thickness, arrow_size,
        const_cast<sensor::data::FfiColor&>(color).as_builtin(), life_time, persistent_lines);
}

inline void FfiDebugHelper_DrawBox(const FfiDebugHelper& helper, const geom::FfiBoundingBox& box,
                                   const Rotation& rotation, float thickness,
                                   const sensor::data::FfiColor& color, float life_time,
                                   bool persistent_lines) {
    const_cast<FfiDebugHelper&>(helper).DrawBox(
        box.as_native(), rotation, thickness,
        const_cast<sensor::data::FfiColor&>(color).as_builtin(), life_time, persistent_lines);
}

inline void FfiDebugHelper_DrawString(const FfiDebugHelper& helper,
                                      const geom::FfiLocation& location, const std::string& text,
                                      bool draw_shadow, const sensor::data::FfiColor& color,
                                      float life_time, bool persistent_lines) {
    const_cast<FfiDebugHelper&>(helper).DrawString(
        location.as_native(), text, draw_shadow,
        const_cast<sensor::data::FfiColor&>(color).as_builtin(), life_time, persistent_lines);
}

}  // namespace client
}  // namespace carla_rust
