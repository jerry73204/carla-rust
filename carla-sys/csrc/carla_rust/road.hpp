#include "carla/road/element/LaneMarking.h"

namespace carla_rust {
    namespace road {
        namespace element {
            using carla::road::element::LaneMarking;

            class FfiLaneMarking {
            public:
                FfiLaneMarking(LaneMarking &&base)
                    :
                    inner_(std::move(base))
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

            private:
                LaneMarking inner_;
            };

        }
    }

}
