#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include "carla/Time.h"
#include "carla/Memory.h"
#include "carla/client/Client.h"
#include "carla/client/World.h"
#include "carla/client/Map.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/Sensor.h"
#include "carla/client/Vehicle.h"
#include "carla/client/TimeoutException.h"
#include "carla/client/ActorList.h"
#include "carla/client/Landmark.h"
#include "carla/client/detail/EpisodeProxy.h"
#include "carla/rpc/AttachmentType.h"
#include "carla/rpc/MapLayer.h"
#include "carla/rpc/OpendriveGenerationParameters.h"
#include "carla/rpc/LabelledPoint.h"
#include "carla/rpc/VehicleControl.h"
#include "carla/rpc/VehiclePhysicsControl.h"
#include "carla/rpc/VehicleDoor.h"
#include "carla/rpc/VehicleWheels.h"
#include "carla/rpc/VehicleLightState.h"
#include "carla/rpc/TrafficLightState.h"
#include "carla/road/element/LaneMarking.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Vector2D.h"
#include "carla/geom/Vector3D.h"
#include "carla/geom/BoundingBox.h"
#include "carla/sensor/SensorData.h"
#include "carla/sensor/data/Color.h"
#include "carla/sensor/data/Image.h"
#include "carla/sensor/data/LidarData.h"
#include "carla/sensor/data/SemanticLidarData.h"
#include "carla/sensor/data/IMUMeasurement.h"
#include "carla/sensor/data/GnssMeasurement.h"
#include "carla/sensor/data/ObstacleDetectionEvent.h"
#include "carla/sensor/data/CollisionEvent.h"
#include "carla/sensor/data/LaneInvasionEvent.h"
#include "carla_rust/road.hpp"

namespace carla_rust {
namespace sensor {
namespace data {
using carla::SharedPtr;
using carla::client::detail::WeakEpisodeProxy;
using carla::geom::Vector2D;
using carla::geom::Vector3D;
using carla::road::element::LaneMarking;
using carla::sensor::data::CollisionEvent;
using carla::sensor::data::Color;
using carla::sensor::data::GnssMeasurement;
using carla::sensor::data::Image;
using carla::sensor::data::IMUMeasurement;
using carla::sensor::data::LaneInvasionEvent;
using carla::sensor::data::LidarDetection;
using carla::sensor::data::ObstacleDetectionEvent;
using carla::sensor::data::SemanticLidarData;
using carla::sensor::data::SemanticLidarDetection;
using carla_rust::client::FfiActor;
using carla_rust::geom::FfiLocation;
using carla_rust::geom::FfiTransform;
using carla_rust::road::element::FfiLaneMarking;

// LaneInvasionEvent
class FfiLaneInvasionEvent {
public:
    FfiLaneInvasionEvent(SharedPtr<LaneInvasionEvent>&& base) : inner_(std::move(base)) {}

    std::shared_ptr<FfiActor> GetActor() const {
        return std::make_shared<FfiActor>(std::move(inner_->GetActor()));
    }

    const std::vector<FfiLaneMarking>& GetCrossedLaneMarkings() const {
        const std::vector<LaneMarking>& orig = inner_->GetCrossedLaneMarkings();
        const std::vector<FfiLaneMarking>& new_ =
            reinterpret_cast<const std::vector<FfiLaneMarking>&>(orig);
        return new_;
    }

private:
    SharedPtr<LaneInvasionEvent> inner_;
};
}  // namespace data
}  // namespace sensor
}  // namespace carla_rust
