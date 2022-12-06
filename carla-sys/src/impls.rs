// carla::client
unsafe impl Send for crate::carla::client::ActorBlueprint {}
unsafe impl Sync for crate::carla::client::ActorBlueprint {}
unsafe impl Send for crate::carla::client::WorldSnapshot {}

// carla_rust::client
unsafe impl Send for crate::carla_rust::client::FfiClient {}
unsafe impl Send for crate::carla_rust::client::FfiWorld {}
unsafe impl Send for crate::carla_rust::client::FfiActor {}
unsafe impl Sync for crate::carla_rust::client::FfiActor {}
unsafe impl Send for crate::carla_rust::client::FfiBlueprintLibrary {}
unsafe impl Sync for crate::carla_rust::client::FfiBlueprintLibrary {}
unsafe impl Send for crate::carla_rust::client::FfiLandmark {}
unsafe impl Sync for crate::carla_rust::client::FfiLandmark {}
unsafe impl Send for crate::carla_rust::client::FfiMap {}
unsafe impl Sync for crate::carla_rust::client::FfiMap {}
unsafe impl Send for crate::carla_rust::client::FfiSensor {}
unsafe impl Sync for crate::carla_rust::client::FfiSensor {}
unsafe impl Send for crate::carla_rust::client::FfiVehicle {}
unsafe impl Sync for crate::carla_rust::client::FfiVehicle {}
unsafe impl Send for crate::carla_rust::client::FfiWaypoint {}
unsafe impl Sync for crate::carla_rust::client::FfiWaypoint {}
unsafe impl Send for crate::carla_rust::client::FfiActorList {}
unsafe impl Sync for crate::carla_rust::client::FfiActorList {}

// carla::geom

// carla_rust::geom

// carla::road

// carla_rust::road::element
unsafe impl Send for crate::carla_rust::road::element::FfiLaneMarking {}

// carla::rpc
unsafe impl Send for crate::carla::rpc::EpisodeSettings {}
unsafe impl Send for crate::carla::rpc::GearPhysicsControl {}
unsafe impl Send for crate::carla::rpc::LabelledPoint {}
unsafe impl Send for crate::carla::rpc::OpendriveGenerationParameters {}
unsafe impl Send for crate::carla::rpc::VehicleLightState {}
unsafe impl Send for crate::carla::rpc::VehiclePhysicsControl {}
unsafe impl Send for crate::carla::rpc::WalkerBoneControlIn {}
unsafe impl Send for crate::carla::rpc::WalkerBoneControlOut {}
unsafe impl Send for crate::carla::rpc::WalkerControl {}
unsafe impl Send for crate::carla::rpc::WheelPhysicsControl {}

// carla_rust::rpc
unsafe impl Send for crate::carla_rust::rpc::FfiEpisodeSettings {}

// carla_rust::sensor
unsafe impl Send for crate::carla_rust::sensor::FfiSensorData {}
unsafe impl Sync for crate::carla_rust::sensor::FfiSensorData {}

// carla_rust::sensor::data
unsafe impl Send for crate::carla_rust::sensor::data::FfiCollisionEvent {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiCollisionEvent {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiGnssMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiGnssMeasurement {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiImage {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiImage {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiImuMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiImuMeasurement {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiLaneInvasionEvent {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiLaneInvasionEvent {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiLidarMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiLidarMeasurement {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiObstacleDetectionEvent {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiObstacleDetectionEvent {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiRadarMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiRadarMeasurement {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiSemanticLidarMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiSemanticLidarMeasurement {}
