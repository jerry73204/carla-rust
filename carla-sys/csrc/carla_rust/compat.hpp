#pragma once

// Compatibility layer for boost::shared_ptr (0.9.x) vs std::shared_ptr (0.10.0)
//
// CARLA 0.10.0 switched from boost::shared_ptr to std::shared_ptr.
// These macros select the appropriate pointer cast functions.

#ifdef CARLA_VERSION_0100

#include <memory>

template <typename T, typename U>
std::shared_ptr<T> carla_dynamic_pointer_cast(const std::shared_ptr<U>& ptr) {
    return std::dynamic_pointer_cast<T>(ptr);
}

template <typename T, typename U>
std::shared_ptr<T> carla_static_pointer_cast(const std::shared_ptr<U>& ptr) {
    return std::static_pointer_cast<T>(ptr);
}

#else

#include <boost/shared_ptr.hpp>

template <typename T, typename U>
boost::shared_ptr<T> carla_dynamic_pointer_cast(const boost::shared_ptr<U>& ptr) {
    return boost::dynamic_pointer_cast<T>(ptr);
}

template <typename T, typename U>
boost::shared_ptr<T> carla_static_pointer_cast(const boost::shared_ptr<U>& ptr) {
    return boost::static_pointer_cast<T>(ptr);
}

#endif
