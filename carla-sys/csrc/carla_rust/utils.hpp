#pragma once

#include <cstdint>
#include <vector>

namespace carla {
    namespace geom {
        class Vector2D;
    }

    namespace rpc {
        class GearPhysicsControl;
        class WheelPhysicsControl;
    }
}

namespace carla_rust
{
    // namespace client {
    //     class FfiActor;
    // }

    namespace geom {
        class FfiLocation;
    }

    namespace utils {
        using carla::geom::Vector2D;
        using carla::rpc::GearPhysicsControl;
        using carla::rpc::WheelPhysicsControl;
        using carla_rust::geom::FfiLocation;
        // using carla_rust::client::FfiActor;

        template<typename T>
        class VectorRef {
        public:
            VectorRef(const std::vector<T> &ref)
                : inner_(ref)
            {}

            size_t len() const {
                return inner_.size();
            }

            const T* data() const {
                return inner_.data();
            }

        private:
            const std::vector<T> &inner_;
        };

        // Using the boost::shared_ptr to std::shared_ptr trick here:
        // https://stackoverflow.com/questions/71572186/question-on-converting-boost-shared-pointer-to-standard-shared-pointer
        // template<class T>
        // std::shared_ptr<T> as_std_shared_ptr(boost::shared_ptr<T> bp)
        // {
        //     if (!bp) return nullptr;
        //     auto pq = std::make_shared<boost::shared_ptr<T>>(std::move(bp));
        //     return std::shared_ptr<T>(pq, pq.get()->get());
        // }

        std::vector<uint8_t> new_uint8_t_vector() {
            return std::vector<uint8_t>();
        }

        std::vector<uint32_t> new_uint32_t_vector() {
            return std::vector<uint32_t>();
        }

        std::vector<uint64_t> new_uint64_t_vector() {
            return std::vector<uint64_t>();
        }

        std::vector<Vector2D> new_vector_2d_vector() {
            return std::vector<Vector2D> {};
        }

        std::vector<FfiLocation> new_ffi_location_vector() {
            return std::vector<FfiLocation> {};
        }

        std::vector<GearPhysicsControl> new_gear_physics_control_vector() {
            return std::vector<GearPhysicsControl> {};
        }

        std::vector<WheelPhysicsControl> new_wheel_physics_control_vector() {
            return std::vector<WheelPhysicsControl> {};
        }

        // std::vector<std::shared_ptr<FfiActor>> new_ffi_actor_vector() {
        //     return std::vector<std::shared_ptr<FfiActor>> {};
        // }
    }
}
