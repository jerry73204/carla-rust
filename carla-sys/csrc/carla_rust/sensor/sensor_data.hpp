#pragma once

#include "data/mod.hpp"
#include "carla_rust/compat.hpp"
#include "carla_rust/client/result.hpp"

namespace carla_rust {
namespace sensor {
using carla::SharedPtr;
using carla::geom::Transform;
using carla::sensor::SensorData;
using carla::sensor::data::CollisionEvent;
using carla::sensor::data::DVSEventArray;
using carla::sensor::data::GnssMeasurement;
using carla::sensor::data::Image;
using carla::sensor::data::IMUMeasurement;
using carla::sensor::data::LaneInvasionEvent;
using carla::sensor::data::LidarMeasurement;
using carla::sensor::data::ObstacleDetectionEvent;
using carla::sensor::data::OpticalFlowImage;
using carla::sensor::data::RadarMeasurement;
using carla::sensor::data::SemanticLidarMeasurement;
using carla_rust::client::ffi_call;
using carla_rust::client::FfiError;
using carla_rust::geom::FfiTransform;
using carla_rust::sensor::data::FfiCollisionEvent;
using carla_rust::sensor::data::FfiCustomV2XEvent;
using carla_rust::sensor::data::FfiDVSEventArray;
using carla_rust::sensor::data::FfiGnssMeasurement;
using carla_rust::sensor::data::FfiImage;
using carla_rust::sensor::data::FfiImuMeasurement;
using carla_rust::sensor::data::FfiLaneInvasionEvent;
using carla_rust::sensor::data::FfiLidarMeasurement;
using carla_rust::sensor::data::FfiObstacleDetectionEvent;
using carla_rust::sensor::data::FfiOpticalFlowImage;
using carla_rust::sensor::data::FfiRadarMeasurement;
using carla_rust::sensor::data::FfiSemanticLidarMeasurement;

class FfiSensorData {
public:
    FfiSensorData(SharedPtr<SensorData>&& base) : inner_(std::move(base)) {}

    size_t GetFrame() const { return inner_->GetFrame(); }

    /// Simulation-time when the data was generated.
    double GetTimestamp() const { return inner_->GetTimestamp(); }

    /// Sensor's transform when the data was generated.
    const FfiTransform GetSensorTransform() const {
        auto transform = inner_->GetSensorTransform();
        return FfiTransform(std::move(transform));
    }

    std::shared_ptr<FfiImage> to_image() const {
        auto ptr = carla_dynamic_pointer_cast<Image>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiImage>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiGnssMeasurement> to_gnss_measurement() const {
        auto ptr = carla_dynamic_pointer_cast<GnssMeasurement>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiGnssMeasurement>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiImuMeasurement> to_imu_measurement() const {
        auto ptr = carla_dynamic_pointer_cast<IMUMeasurement>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiImuMeasurement>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiCollisionEvent> to_collision_event() const {
        auto ptr = carla_dynamic_pointer_cast<CollisionEvent>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiCollisionEvent>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiObstacleDetectionEvent> to_obstacle_detection_event() const {
        auto ptr = carla_dynamic_pointer_cast<ObstacleDetectionEvent>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiObstacleDetectionEvent>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiLaneInvasionEvent> to_lane_invasion_event() const {
        auto ptr = carla_dynamic_pointer_cast<LaneInvasionEvent>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiLaneInvasionEvent>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiLidarMeasurement> to_lidar_measurement() const {
        auto ptr = carla_dynamic_pointer_cast<LidarMeasurement>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiLidarMeasurement>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiSemanticLidarMeasurement> to_semantic_lidar_measurement() const {
        auto ptr = carla_dynamic_pointer_cast<SemanticLidarMeasurement>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiSemanticLidarMeasurement>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiRadarMeasurement> to_radar_measurement() const {
        auto ptr = carla_dynamic_pointer_cast<RadarMeasurement>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiRadarMeasurement>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiDVSEventArray> to_dvs_event_array() const {
        auto ptr = carla_dynamic_pointer_cast<DVSEventArray>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiDVSEventArray>(std::move(ptr));
        }
    }

    /// The messages of a `sensor.other.v2x_custom` (CARLA 0.9.16); null for
    /// any other data or an older CARLA.
    std::shared_ptr<FfiCustomV2XEvent> to_custom_v2x_event() const {
#ifdef CARLA_VERSION_0916
        auto ptr = carla_dynamic_pointer_cast<carla::sensor::data::CustomV2XEvent>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiCustomV2XEvent>(std::move(ptr));
        }
#else
        return nullptr;
#endif
    }

    std::shared_ptr<FfiOpticalFlowImage> to_optical_flow_image() const {
        auto ptr = carla_dynamic_pointer_cast<OpticalFlowImage>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiOpticalFlowImage>(std::move(ptr));
        }
    }

private:
    SharedPtr<SensorData> inner_;
};
}  // namespace sensor

}  // namespace carla_rust
