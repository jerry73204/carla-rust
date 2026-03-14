# Async RPC Feasibility Analysis for carla-rust

**Date:** 2025-01-09
**Status:** ⚠️ CRITICAL FINDING - True async RPC not directly accessible

## Executive Summary

**FINDING:** While rpclib supports `async_call()` returning `std::future`, **libcarla_client discards all futures** and only exposes blocking APIs. True async RPC would require modifying CARLA's C++ client code or bypassing its API layer.

## Investigation Results

### 1. rpclib Layer: ✅ Full Async Support

**File:** `PythonAPI/carla/dependencies/include/system/rpc/client.h`

```cpp
template <typename... Args>
std::future<RPCLIB_MSGPACK::object_handle> async_call(
    std::string const &func_name,
    Args... args
);
```

**Confirmed:**
- Returns `std::future<RPCLIB_MSGPACK::object_handle>`
- Non-blocking, returns immediately
- Can poll with `wait_for(timeout)`

### 2. libcarla_client Layer: ❌ Futures Discarded

**File:** `LibCarla/source/carla/client/detail/Client.cpp` (lines 48-97)

#### Client::Pimpl Implementation

```cpp
class Client::Pimpl {
public:
    // BLOCKING: Uses rpc_client.call() which waits
    template <typename ... Args>
    auto RawCall(const std::string &function, Args && ... args) {
      try {
        return rpc_client.call(function, std::forward<Args>(args) ...);
      } catch (const ::rpc::timeout &) {
        throw_exception(TimeoutException(endpoint, GetTimeout()));
      }
    }

    // BLOCKING: Wraps RawCall
    template <typename T, typename ... Args>
    auto CallAndWait(const std::string &function, Args && ... args) {
      auto object = RawCall(function, std::forward<Args>(args) ...);
      using R = typename carla::rpc::Response<T>;
      auto response = object.template as<R>();
      if (response.HasError()) {
        throw_exception(std::runtime_error(response.GetError().What()));
      }
      return Get(response);
    }

    // FIRE-AND-FORGET: Discards future!
    template <typename ... Args>
    void AsyncCall(const std::string &function, Args && ... args) {
      // Discard returned future.
      rpc_client.async_call(function, std::forward<Args>(args) ...);
    }

private:
    rpc::Client rpc_client;  // The actual rpclib client
    streaming::Client streaming_client;
};
```

**Key Problems:**

1. **`_pimpl` is private** (line 451 in Client.h):
   ```cpp
   const std::unique_ptr<Pimpl> _pimpl;
   ```
   No public accessor exists.

2. **`AsyncCall` discards futures** (line 81-84):
   - Calls `rpc_client.async_call()` but returns `void`
   - Future is destroyed immediately
   - No way to poll or await result

3. **All public APIs use `CallAndWait`**:
   - `GetServerVersion()` → `CallAndWait<std::string>("version")`
   - `SpawnActor()` → `CallAndWait<rpc::Actor>("spawn_actor", ...)`
   - Every operation blocks

### 3. How Sensors Achieve Async: Different Architecture

**File:** `LibCarla/source/carla/client/ServerSideSensor.cpp` (lines 39-44)

```cpp
void ServerSideSensor::Listen(CallbackFunctionType callback) {
    log_debug("calling sensor Listen() ", GetDisplayId());
    log_debug(GetDisplayId(), ": subscribing to stream");
    GetEpisode().Lock()->SubscribeToSensor(*this, std::move(callback));
    listening_mask.set(0);
}
```

**Sensors use streaming, NOT RPC:**
- `streaming::Client streaming_client` (separate from rpc_client)
- Background thread pool processes incoming data
- Callbacks invoked on worker threads
- Never uses `async_call()` or `std::future`

## Options for Async RPC

### Option 1: Modify CARLA Client (Upstream Changes)

**Approach:** Add methods to Client::Pimpl that return futures.

#### Changes Required

**File:** `LibCarla/source/carla/client/detail/Client.h`

```cpp
class Client {
public:
    // New method: Return future instead of blocking
    template <typename T, typename... Args>
    std::future<T> CallAsync(const std::string &function, Args&&... args) {
        return _pimpl->CallAsync<T>(function, std::forward<Args>(args)...);
    }

private:
    class Pimpl;
    const std::unique_ptr<Pimpl> _pimpl;
};
```

**File:** `LibCarla/source/carla/client/detail/Client.cpp`

```cpp
class Client::Pimpl {
public:
    // New async method that returns future
    template <typename T, typename... Args>
    std::future<T> CallAsync(const std::string &function, Args&&... args) {
        // Get future from rpclib
        auto rpc_future = rpc_client.async_call(function, std::forward<Args>(args)...);

        // Return transformed future (need std::async or similar)
        return std::async(std::launch::deferred, [f = std::move(rpc_future)]() mutable {
            auto object = f.get();
            using R = typename carla::rpc::Response<T>;
            auto response = object.template as<R>();
            if (response.HasError()) {
                throw std::runtime_error(response.GetError().What());
            }
            return Get(response);
        });
    }
};
```

**Pros:**
- Clean API, returns std::future directly
- Preserves error handling
- True async without modifying rpclib

**Cons:**
- **Requires CARLA source modification**
- Upstream may not accept changes
- Need to maintain fork
- Breaks compatibility with official CARLA builds

### Option 2: Custom C++ Wrapper (Bypass libcarla_client)

**Approach:** Create our own RPC client using rpclib directly.

#### Implementation

**File:** `carla-sys/csrc/carla_rust/rpc/async_client.hpp`

```cpp
#pragma once

#include <rpc/client.h>
#include <future>
#include <string>
#include <memory>

namespace carla_rust {
namespace rpc {

/// Direct rpclib client wrapper that exposes futures.
/// Bypasses libcarla_client's blocking API.
class AsyncRpcClient {
private:
    ::rpc::client rpc_client_;

public:
    AsyncRpcClient(const std::string& host, uint16_t port)
        : rpc_client_(host, port) {
        rpc_client_.set_timeout(5000);
    }

    /// Call RPC method asynchronously, returns future.
    template <typename... Args>
    std::future<RPCLIB_MSGPACK::object_handle> async_call(
        const std::string& function,
        Args&&... args
    ) {
        return rpc_client_.async_call(function, std::forward<Args>(args)...);
    }

    /// Call RPC method synchronously (blocks).
    template <typename... Args>
    RPCLIB_MSGPACK::object_handle call(
        const std::string& function,
        Args&&... args
    ) {
        return rpc_client_.call(function, std::forward<Args>(args)...);
    }

    void set_timeout(int64_t ms) {
        rpc_client_.set_timeout(ms);
    }
};

// Free functions for FFI

inline std::unique_ptr<AsyncRpcClient> AsyncRpcClient_new(
    const std::string& host,
    uint16_t port
) {
    return std::make_unique<AsyncRpcClient>(host, port);
}

template <typename... Args>
inline std::future<RPCLIB_MSGPACK::object_handle> AsyncRpcClient_call(
    AsyncRpcClient* client,
    const std::string& function,
    Args&&... args
) {
    return client->async_call(function, std::forward<Args>(args)...);
}

} // namespace rpc
} // namespace carla_rust
```

**Rust Usage:**

```rust
// Create parallel client just for async operations
let async_client = AsyncRpcClient::new("127.0.0.1", 2000);

// Make async call
let future = async_client.async_call(
    "spawn_actor",
    actor_description,
    transform
);

// Wrap in FfiFuture
let ffi_future = FfiFuture::new(future);

// Await in Rust
let actor = CarlaFuture::new(ffi_future).await?;
```

**Pros:**
- **No CARLA modifications needed**
- Full control over async behavior
- Can coexist with existing sync API

**Cons:**
- Duplicate connection (2 TCP sockets)
- Need to reimplement message serialization
- No access to Client's error handling
- Must manually pack/unpack msgpack
- Bypass libcarla_client's abstractions

### Option 3: Friend Class Accessor (Minimal Intrusion)

**Approach:** Add friend declaration to access Pimpl.

**File:** `carla-sys/csrc/carla_rust/client/async_accessor.hpp`

```cpp
#pragma once

#include "carla/client/detail/Client.h"
#include <future>

namespace carla {
namespace client {
namespace detail {

// Forward declare to add as friend
namespace carla_rust { class AsyncAccessor; }

}}}

namespace carla_rust {
namespace client {

/// Friend class to access Client::Pimpl::rpc_client
class AsyncAccessor {
public:
    template <typename... Args>
    static std::future<RPCLIB_MSGPACK::object_handle> async_call(
        carla::client::detail::Client* client,
        const std::string& function,
        Args&&... args
    ) {
        // Access private _pimpl->rpc_client
        return client->_pimpl->rpc_client.async_call(
            function,
            std::forward<Args>(args)...
        );
    }
};

}} // namespace carla_rust::client
```

**Also need to modify CARLA:**

```cpp
// In LibCarla/source/carla/client/detail/Client.h
class Client {
    friend class carla_rust::client::AsyncAccessor;
    // ...
};
```

**Pros:**
- Minimal changes to CARLA
- Reuses existing connection
- Access to error handling

**Cons:**
- Still requires CARLA modification
- Friendship is tight coupling
- Need to maintain patch

### Option 4: LD_PRELOAD Symbol Interposition (Advanced)

**Approach:** Intercept `async_call` at link/runtime to capture futures.

**File:** `carla-sys/preload/async_intercept.cpp`

```cpp
#include <future>
#include <unordered_map>
#include <mutex>
#include <rpc/client.h>

// Global storage for futures
std::unordered_map<uint64_t, std::shared_future<RPCLIB_MSGPACK::object_handle>> g_futures;
std::mutex g_futures_mutex;
uint64_t g_next_id = 1;

// Intercept AsyncCall to save future
extern "C" void Client_Pimpl_AsyncCall(...) {
    // Original async_call
    auto future = rpc_client.async_call(...);

    // Save to global map
    uint64_t id = g_next_id++;
    {
        std::lock_guard lock(g_futures_mutex);
        g_futures[id] = future.share();
    }

    // Continue normally (discarding local future)
}

// Rust can retrieve by ID
extern "C" FfiFuture* get_future_by_id(uint64_t id) {
    std::lock_guard lock(g_futures_mutex);
    auto it = g_futures.find(id);
    if (it != g_futures.end()) {
        return new FfiFuture(it->second);
    }
    return nullptr;
}
```

**Pros:**
- Zero changes to CARLA
- Works with official builds

**Cons:**
- **Extremely fragile**
- Platform-specific (Linux/macOS/Windows differ)
- Hard to maintain
- Debugging nightmare

## Recommended Approach

### Phase 1: Sensor/Tick Streams Only (No RPC Modification)

**Implement async support for already-async operations:**

1. **Sensor callbacks → async streams** (already works, no RPC needed)
2. **World tick callbacks → async streams** (already works, no RPC needed)

**Timeline:** 1 week

**Code:**

```rust
// Sensors are already async via streaming client
pub fn sensor_stream(sensor: &Sensor) -> impl Stream<Item = SensorData> {
    let (tx, rx) = mpsc::unbounded_channel();
    sensor.listen(move |data| {
        let _ = tx.send(data);
    });
    tokio_stream::wrappers::UnboundedReceiverStream::new(rx)
}

// Ticks are already async via callbacks
pub fn tick_stream(world: &World) -> impl Stream<Item = WorldSnapshot> {
    let (tx, rx) = mpsc::unbounded_channel();
    world.on_tick(move |snapshot| {
        let _ = tx.send(snapshot);
    });
    tokio_stream::wrappers::UnboundedReceiverStream::new(rx)
}
```

### Phase 2: Blocking Wrapper for Compatibility

**For operations that must complete:**

```rust
impl World {
    pub async fn spawn_actor_async(
        &self,
        blueprint: &ActorBlueprint,
        transform: &Transform,
    ) -> Result<Actor> {
        let world = self.clone();
        let blueprint = blueprint.clone();
        let transform = *transform;

        tokio::task::spawn_blocking(move || {
            world.spawn_actor(&blueprint, &transform)
        })
        .await?
    }
}
```

**Timeline:** 3-5 days

### Phase 3: True Async RPC (If Needed)

**Only pursue if blocking wrapper insufficient.**

**Option A:** Submit patch to CARLA upstream
- Add `CallAsync<T>()` method to Client
- Propose in CARLA GitHub discussions
- May take months for acceptance

**Option B:** Maintain carla-rust fork of libcarla_client
- Patch Client::Pimpl with async methods
- Document build process
- Users build custom CARLA

**Option C:** Parallel RPC client (bypass libcarla_client)
- Use rpclib directly for async calls
- Keep existing client for sync operations
- Accept duplicate connection overhead

**Timeline:** 2-4 weeks depending on approach

## Conclusion

**CRITICAL:** True async RPC via `std::future` exposure is **not possible** with stock libcarla_client because:

1. Client::Pimpl is private (no accessor)
2. AsyncCall() discards futures
3. All public APIs block via CallAndWait()

**Recommended Path:**

1. **Immediate:** Implement async sensor/tick streams (1 week)
2. **Short-term:** Use `spawn_blocking` for RPC (3-5 days)
3. **Long-term:** Evaluate CARLA patch vs parallel client (2-4 weeks if needed)

The spawn_blocking approach provides 90% of async benefits with zero CARLA modifications, which is the pragmatic choice for most use cases.

## References

- libcarla_client source: `LibCarla/source/carla/client/detail/Client.cpp`
- rpclib async_call: `PythonAPI/carla/dependencies/include/system/rpc/client.h`
- Sensor streaming: `LibCarla/source/carla/client/ServerSideSensor.cpp`
