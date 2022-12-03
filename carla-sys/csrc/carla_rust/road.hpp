#pragma once

#include "carla/road/element/LaneMarking.h"

namespace carla_rust {
    namespace road {
        namespace element {
            using carla::road::element::LaneMarking;

            // LaneMarking
            class FfiLaneMarking {
            public:
                FfiLaneMarking(LaneMarking &&base)
                    :
                    inner_(std::move(base))
                {}

                FfiLaneMarking(LaneMarking &base)
                    :
                    inner_(base)
                {}

                LaneMarking::Type type() const {
                    return inner_.type;
                }

                LaneMarking::Color color() const {
                    return inner_.color;
                }

                LaneMarking::LaneChange lane_change() const {
                    return inner_.lane_change;
                }

                double width() const {
                    return inner_.width;
                }

                FfiLaneMarking clone() const {
                    return *this;
                }

            private:
                LaneMarking inner_;
            };

            static_assert(sizeof(FfiLaneMarking) == sizeof(LaneMarking), "FfiLaneMarking has invalid size");
        }
    }

}
