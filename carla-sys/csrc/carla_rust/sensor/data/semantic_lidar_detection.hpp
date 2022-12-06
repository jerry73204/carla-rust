#pragma once

#include "carla/sensor/data/SemanticLidarData.h"


namespace carla_rust
{
    namespace sensor {
        namespace data {
            using carla::sensor::data::SemanticLidarDetection;

            // SemanticLidarDetection
            class FfiSemanticLidarDetection {
            public:
                FfiLocation point;
                float cos_inc_angle;
                uint32_t object_idx;
                uint32_t object_tag;


                FfiSemanticLidarDetection(SemanticLidarDetection &&base)
                    :
                    point(reinterpret_cast<FfiLocation&&>(std::move(base.point))),
                    cos_inc_angle(std::move(base.cos_inc_angle)),
                    object_idx(std::move(base.object_idx)),
                    object_tag(std::move(base.object_tag))
                {

                }

                SemanticLidarDetection& as_builtin() {
                    return reinterpret_cast<SemanticLidarDetection&>(*this);
                }
            };
            static_assert(sizeof(FfiSemanticLidarDetection) == sizeof(SemanticLidarDetection), "FfiSemanticLidarDetection and SemanticLidarDetection size mismatch");

       }
    }
}
