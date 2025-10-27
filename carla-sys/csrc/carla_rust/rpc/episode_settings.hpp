#pragma once

#include "carla/rpc/EpisodeSettings.h"

namespace carla_rust {
namespace rpc {
using carla::rpc::EpisodeSettings;

class FfiEpisodeSettings {
public:
    FfiEpisodeSettings() = default;

    FfiEpisodeSettings(EpisodeSettings&& base) : inner_(std::move(base)) {}

    FfiEpisodeSettings(bool synchronous_mode, bool no_rendering_mode, double fixed_delta_seconds,
                       bool substepping, double max_substep_delta_time, int max_substeps,
                       float max_culling_distance, bool deterministic_ragdolls,
                       float tile_stream_distance, float actor_active_distance)
        : inner_(synchronous_mode, no_rendering_mode, fixed_delta_seconds, substepping,
                 max_substep_delta_time, max_substeps, max_culling_distance, deterministic_ragdolls,
                 tile_stream_distance, actor_active_distance) {}

    const EpisodeSettings& inner() const { return inner_; }

    bool synchronous_mode() const { return inner_.synchronous_mode; }

    bool no_rendering_mode() const { return inner_.no_rendering_mode; }

    // return NaN if it's not set.
    double fixed_delta_seconds() const {
        auto opt = inner_.fixed_delta_seconds;

        if (opt) {
            return *opt;
        } else {
            return std::numeric_limits<double>::quiet_NaN();
        }
    }

    bool substepping() const { return inner_.substepping; }

    double max_substep_delta_time() const { return inner_.max_substep_delta_time; }

    int max_substeps() const { return inner_.max_substeps; }

    float max_culling_distance() const { return inner_.max_culling_distance; }

    bool deterministic_ragdolls() const { return inner_.deterministic_ragdolls; }

    float tile_stream_distance() const { return inner_.tile_stream_distance; }

    float actor_active_distance() const { return inner_.actor_active_distance; }

private:
    EpisodeSettings inner_;
};
}  // namespace rpc
}  // namespace carla_rust
