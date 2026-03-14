# Async/Await Support Analysis for carla-rust

**Date:** 2025-01-09
**Author:** Claude Code
**Status:** ⚠️ SUPERSEDED - See Feasibility Analysis

## ⚠️ CRITICAL UPDATE (2025-01-09)

**This document describes a theoretical approach that is NOT directly achievable with stock CARLA.**

**Key Finding:** libcarla_client **discards all `std::future` objects** returned by rpclib's `async_call()`. The `Client::Pimpl::AsyncCall()` method is void-returning and fires-and-forgets.

**For practical async implementation options, see:**
👉 **[`async-rpc-feasibility.md`](./async-rpc-feasibility.md)** 👈

**This document remains useful for:**
- Understanding rpclib's async capabilities
- Reference if modifying CARLA upstream
- Theoretical FFI future bridge design

---

## Original Executive Summary (Theoretical)

**PARTIALLY TRUE**: While rpclib supports `async_call()` returning `std::future`, libcarla_client wraps all calls in blocking methods or discards futures. The underlying async capability exists in rpclib but is not exposed through libcarla_client's public API.

## Key Findings

### 1. RPC Layer: rpclib async_call Support

**Library:** rpclib (msgpack-RPC client/server)
**Version:** 2.2.1 (used in CARLA)
**Location:** `PythonAPI/carla/dependencies/include/system/rpc/client.h`

#### API Signature

```cpp
template <typename... Args>
std::future<RPCLIB_MSGPACK::object_handle> async_call(
    std::string const &func_name,
    Args... args
);
```

#### Characteristics

- **Non-blocking**: Returns immediately without waiting for server response
- **Future-based**: Returns `std::future<RPCLIB_MSGPACK::object_handle>`
- **Polling capable**: Can use `future.wait_for()` with timeout for polling
- **Thread-safe**: Worker threads handle actual I/O (configured via `worker_threads` parameter)
- **No timeout on async calls**: Must use `std::future::wait_for()` for timeout handling

#### Implementation Details

From `client.inl` lines 18-49:

```cpp
template <typename... Args>
std::future<RPCLIB_MSGPACK::object_handle>
client::async_call(std::string const &func_name, Args... args) {
    wait_conn();
    const int idx = get_next_call_idx();

    // Pack arguments into msgpack buffer
    auto args_obj = std::make_tuple(args...);
    auto call_obj = std::make_tuple(
        static_cast<uint8_t>(client::request_type::call),
        idx,
        func_name,
        args_obj
    );

    auto buffer = std::make_shared<RPCLIB_MSGPACK::sbuffer>();
    RPCLIB_MSGPACK::pack(*buffer, call_obj);

    // Create promise/future pair
    auto p = std::make_shared<std::promise<RPCLIB_MSGPACK::object_handle>>();
    auto ft = p->get_future();

    post(buffer, idx, func_name, p);  // Non-blocking post

    return ft;
}
```

**Note:** The synchronous `call()` is implemented as:
```cpp
auto future = async_call(func_name, std::forward<Args>(args)...);
future.wait_for(timeout);
return future.get();
```

This proves all RPC calls are fundamentally async underneath.

### 2. CARLA Client Architecture

**File:** `LibCarla/source/carla/client/detail/Client.h`

#### Worker Threads

```cpp
explicit Client(
    const std::string &host,
    uint16_t port,
    size_t worker_threads = 0u  // 0 = use all hardware concurrency
);
```

From `Client.cpp` line 51-58:

```cpp
Pimpl(const std::string &host, uint16_t port, size_t worker_threads)
  : endpoint(host + ":" + std::to_string(port)),
    rpc_client(host, port),
    streaming_client(host) {
  rpc_client.set_timeout(5000u);
  streaming_client.AsyncRun(
      worker_threads > 0u ? worker_threads : std::thread::hardware_concurrency()
  );
}
```

**Key insight:** CARLA already runs async I/O workers in background threads.

#### Current Wrapper Methods

```cpp
// Blocking call
template <typename T, typename ... Args>
auto CallAndWait(const std::string &function, Args && ... args) {
    auto object = RawCall(function, std::forward<Args>(args) ...);
    // ... wait for response ...
}

// Fire-and-forget async (no future returned!)
template <typename ... Args>
void AsyncCall(const std::string &function, Args && ... args) {
    // Discard returned future.
    rpc_client.async_call(function, std::forward<Args>(args) ...);
}
```

**Problem:** `AsyncCall()` discards the future, preventing polling/awaiting.

### 3. Sensor Callbacks (Already Async!)

**File:** `LibCarla/source/carla/client/Sensor.h`

```cpp
class Sensor : public Actor {
public:
    using CallbackFunctionType = std::function<void(SharedPtr<sensor::SensorData>)>;

    virtual void Listen(CallbackFunctionType callback) = 0;
    virtual void Stop() = 0;
    virtual bool IsListening() const = 0;
};
```

**Current behavior:** Callbacks execute on background threads (streaming client workers).

**Implication:** Already event-driven, can be bridged to async streams.

### 4. World Tick Events

**File:** `LibCarla/source/carla/client/World.h`

```cpp
// Blocking wait
WorldSnapshot WaitForTick(time_duration timeout) const;

// Callback-based (already async!)
size_t OnTick(std::function<void(WorldSnapshot)> callback);
void RemoveOnTick(size_t callback_id);

// Trigger next tick (synchronous mode)
uint64_t Tick(time_duration timeout);
```

**Insight:** `OnTick()` is already a callback-based async pattern.

## Rust async/await Integration Strategy

### Option 1: Expose std::future via FFI ⭐ RECOMMENDED

**Approach:** Wrap std::future in a C++ class with polling API, expose via FFI, implement Rust Future trait.

This is the most direct approach that preserves CARLA's async architecture while being runtime-agnostic in Rust.

#### Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        Rust Layer                            │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ impl Future for CarlaFuture<T> {                       │ │
│  │   fn poll(self: Pin<&mut Self>, cx: &mut Context)     │ │
│  │      -> Poll<T> {                                      │ │
│  │     match self.ffi_future.poll_timeout(0) {           │ │
│  │       Ready => Poll::Ready(self.ffi_future.get()),    │ │
│  │       NotReady => {                                    │ │
│  │         register_waker(cx.waker().clone());           │ │
│  │         Poll::Pending                                  │ │
│  │       }                                                │ │
│  │     }                                                  │ │
│  │   }                                                    │ │
│  │ }                                                      │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                           ↕ FFI
┌─────────────────────────────────────────────────────────────┐
│                        C++ Layer                             │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ class FfiFuture {                                      │ │
│  │   std::shared_future<object_handle> future_;          │ │
│  │   Waker* waker_ = nullptr;                            │ │
│  │                                                        │ │
│  │   PollStatus poll_timeout(uint32_t ms) {              │ │
│  │     auto status = future_.wait_for(                   │ │
│  │       std::chrono::milliseconds(ms));                 │ │
│  │     if (status == ready) {                            │ │
│  │       if (waker_) waker_->wake();                     │ │
│  │       return Ready;                                    │ │
│  │     }                                                  │ │
│  │     return NotReady;                                   │ │
│  │   }                                                    │ │
│  │ }                                                      │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                           ↕
┌─────────────────────────────────────────────────────────────┐
│                     rpclib (CARLA)                           │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ std::future<object_handle> async_call(...)             │ │
│  │   → Worker threads handle I/O                          │ │
│  │   → Promise/future pair created                        │ │
│  │   → Returns immediately                                │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

#### Step 1: C++ FFI Future Wrapper

**File:** `carla-sys/csrc/carla_rust/client/async_future.hpp`

```cpp
#pragma once

#include <future>
#include <memory>
#include <chrono>
#include <atomic>
#include <rpc/msgpack.hpp>

namespace carla_rust {
namespace client {

// Forward declare Rust waker
struct RustWaker;

enum class PollStatus : uint8_t {
    Ready = 0,
    Pending = 1,
    Deferred = 2  // For std::future_status::deferred
};

/// Wrapper around std::future that can be safely exposed via FFI.
/// Uses std::shared_future to allow multiple polls without consuming the future.
class FfiFuture {
private:
    std::shared_future<RPCLIB_MSGPACK::object_handle> future_;
    std::atomic<RustWaker*> waker_{nullptr};
    std::atomic<bool> polled_ready_{false};

public:
    /// Construct from std::future (moves ownership)
    explicit FfiFuture(std::future<RPCLIB_MSGPACK::object_handle>&& future)
        : future_(std::move(future)) {}

    /// Poll with timeout in milliseconds.
    /// timeout_ms == 0 means non-blocking check.
    /// Returns Ready if value available, Pending if not ready.
    PollStatus poll_timeout(uint32_t timeout_ms) {
        if (polled_ready_.load(std::memory_order_acquire)) {
            return PollStatus::Ready;
        }

        auto status = future_.wait_for(std::chrono::milliseconds(timeout_ms));

        switch (status) {
            case std::future_status::ready:
                polled_ready_.store(true, std::memory_order_release);
                wake_rust();
                return PollStatus::Ready;

            case std::future_status::timeout:
                return PollStatus::Pending;

            case std::future_status::deferred:
                return PollStatus::Deferred;
        }

        return PollStatus::Pending;
    }

    /// Get the result (blocks if not ready).
    /// SAFETY: Caller must ensure poll_timeout returned Ready.
    RPCLIB_MSGPACK::object_handle get() {
        return future_.get();
    }

    /// Check if future is valid (hasn't been consumed by get()).
    /// With shared_future, this always returns true after construction.
    bool valid() const {
        return future_.valid();
    }

    /// Register Rust waker for async notification.
    /// SAFETY: waker must outlive this FfiFuture or be unregistered.
    void register_waker(RustWaker* waker) {
        waker_.store(waker, std::memory_order_release);
    }

    /// Unregister Rust waker.
    void unregister_waker() {
        waker_.store(nullptr, std::memory_order_release);
    }

private:
    void wake_rust() {
        RustWaker* waker = waker_.load(std::memory_order_acquire);
        if (waker != nullptr) {
            wake_by_ref(waker);  // Calls Rust FFI function
        }
    }
};

// Free functions for FFI compatibility (autocxx prefers free functions)

inline std::unique_ptr<FfiFuture> FfiFuture_new(
    std::future<RPCLIB_MSGPACK::object_handle>&& future
) {
    return std::make_unique<FfiFuture>(std::move(future));
}

inline PollStatus FfiFuture_poll_timeout(FfiFuture* future, uint32_t timeout_ms) {
    return future->poll_timeout(timeout_ms);
}

inline RPCLIB_MSGPACK::object_handle FfiFuture_get(FfiFuture* future) {
    return future->get();
}

inline bool FfiFuture_valid(const FfiFuture* future) {
    return future->valid();
}

inline void FfiFuture_register_waker(FfiFuture* future, RustWaker* waker) {
    future->register_waker(waker);
}

inline void FfiFuture_unregister_waker(FfiFuture* future) {
    future->unregister_waker();
}

// Called from C++ when future becomes ready
extern "C" void wake_by_ref(RustWaker* waker);

} // namespace client
} // namespace carla_rust
```

#### Step 2: Rust Waker FFI Bridge

**File:** `carla-sys/csrc/carla_rust/client/waker.hpp`

```cpp
#pragma once

#include <cstdint>

namespace carla_rust {
namespace client {

// Opaque Rust waker handle
struct RustWaker {
    void* data;
    void (*wake)(void*);
    void (*drop)(void*);
};

// Called from C++ to wake Rust task
extern "C" inline void wake_by_ref(RustWaker* waker) {
    if (waker && waker->wake) {
        waker->wake(waker->data);
    }
}

} // namespace client
} // namespace carla_rust
```

#### Step 3: Modified Client with Async Support

**File:** `carla-sys/csrc/carla_rust/client/async_client.hpp`

```cpp
#pragma once

#include "carla_rust/client/async_future.hpp"
#include "carla/client/detail/Client.h"
#include <memory>

namespace carla_rust {
namespace client {

class FfiAsyncClient {
private:
    std::shared_ptr<carla::client::detail::Client> client_;

public:
    explicit FfiAsyncClient(std::shared_ptr<carla::client::detail::Client> client)
        : client_(std::move(client)) {}

    // Async RPC call that returns FfiFuture
    template <typename... Args>
    std::unique_ptr<FfiFuture> async_call(const std::string& function, Args&&... args) {
        // Get the underlying rpc::client and call async_call directly
        auto future = client_->pimpl_->rpc_client.async_call(
            function,
            std::forward<Args>(args)...
        );
        return FfiFuture_new(std::move(future));
    }
};

// Free functions for specific async operations

inline std::unique_ptr<FfiFuture> async_spawn_actor(
    FfiAsyncClient* client,
    const carla::rpc::ActorDescription& description,
    const carla::geom::Transform& transform
) {
    // This requires exposing rpc_client or wrapping Client::Pimpl
    // Simplified version - actual implementation needs Client refactoring
    return client->async_call("spawn_actor", description, transform);
}

} // namespace client
} // namespace carla_rust
```

**Note:** This requires exposing the rpc_client from Client::Pimpl. Alternative: Add async methods directly to CARLA's Client class.

#### Step 4: Rust Future Implementation

**File:** `carla/src/client/async_future.rs`

```rust
use std::future::Future;
use std::pin::Pin;
use std::task::{Context, Poll, Waker};
use carla_sys::carla_rust::client::{FfiFuture, PollStatus, RustWaker};
use cxx::UniquePtr;

/// Rust future wrapping C++ std::future from rpclib.
/// This allows awaiting CARLA RPC calls in async Rust code.
pub struct CarlaFuture<T> {
    ffi_future: UniquePtr<FfiFuture>,
    waker: Option<Box<RustWaker>>,
    _phantom: std::marker::PhantomData<T>,
}

impl<T> CarlaFuture<T> {
    pub(crate) fn new(ffi_future: UniquePtr<FfiFuture>) -> Self {
        Self {
            ffi_future,
            waker: None,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<T> Future for CarlaFuture<T>
where
    T: TryFrom<RPCLIB_MSGPACK_object_handle>,
{
    type Output = Result<T, Box<dyn std::error::Error>>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Non-blocking poll (0ms timeout)
        let status = unsafe {
            carla_sys::carla_rust::client::FfiFuture_poll_timeout(
                self.ffi_future.as_mut().unwrap(),
                0
            )
        };

        match status {
            PollStatus::Ready => {
                // Result is ready, extract it
                let result = unsafe {
                    carla_sys::carla_rust::client::FfiFuture_get(
                        self.ffi_future.as_mut().unwrap()
                    )
                };

                // Convert msgpack object to T
                let value = T::try_from(result)
                    .map_err(|_| "Failed to deserialize RPC response")?;

                Poll::Ready(Ok(value))
            }

            PollStatus::Pending => {
                // Register waker if not already registered
                if self.waker.is_none() {
                    let waker = cx.waker().clone();
                    let rust_waker = Box::new(create_rust_waker(waker));

                    unsafe {
                        carla_sys::carla_rust::client::FfiFuture_register_waker(
                            self.ffi_future.as_mut().unwrap(),
                            rust_waker.as_ref() as *const RustWaker as *mut RustWaker
                        );
                    }

                    self.waker = Some(rust_waker);
                }

                Poll::Pending
            }

            PollStatus::Deferred => {
                // For deferred futures, we need to trigger execution
                // This shouldn't happen with rpclib async_call
                Poll::Pending
            }
        }
    }
}

impl<T> Drop for CarlaFuture<T> {
    fn drop(&mut self) {
        if let Some(_waker) = self.waker.take() {
            unsafe {
                carla_sys::carla_rust::client::FfiFuture_unregister_waker(
                    self.ffi_future.as_mut().unwrap()
                );
            }
        }
    }
}

/// Create RustWaker from Rust Waker
fn create_rust_waker(waker: Waker) -> RustWaker {
    let waker_box = Box::new(waker);
    let waker_ptr = Box::into_raw(waker_box);

    RustWaker {
        data: waker_ptr as *mut std::ffi::c_void,
        wake: Some(wake_fn),
        drop: Some(drop_fn),
    }
}

unsafe extern "C" fn wake_fn(data: *mut std::ffi::c_void) {
    let waker = &*(data as *const Waker);
    waker.wake_by_ref();
}

unsafe extern "C" fn drop_fn(data: *mut std::ffi::c_void) {
    let _ = Box::from_raw(data as *mut Waker);
}

// Export wake_by_ref for C++ to call
#[no_mangle]
pub unsafe extern "C" fn wake_by_ref(waker: *mut RustWaker) {
    if !waker.is_null() {
        let rust_waker = &*waker;
        if let Some(wake) = rust_waker.wake {
            wake(rust_waker.data);
        }
    }
}
```

#### Step 5: High-Level Async API

**File:** `carla/src/client/world_async.rs`

```rust
use crate::client::{ActorBlueprint, World, Actor};
use crate::geom::Transform;
use crate::client::async_future::CarlaFuture;

impl World {
    /// Spawn actor asynchronously.
    /// Returns a future that resolves to the spawned actor.
    pub fn spawn_actor_async(
        &self,
        blueprint: &ActorBlueprint,
        transform: &Transform,
    ) -> CarlaFuture<Actor> {
        let ffi_future = unsafe {
            carla_sys::carla_rust::client::async_spawn_actor(
                self.async_client.as_ref(),
                &blueprint.to_ffi(),
                &transform.into_ffi(),
            )
        };

        CarlaFuture::new(ffi_future)
    }

    /// Apply batch commands asynchronously.
    pub async fn apply_batch_async(
        &self,
        commands: Vec<Command>,
    ) -> Result<Vec<CommandResponse>, Error> {
        let ffi_future = unsafe {
            carla_sys::carla_rust::client::async_apply_batch(
                self.async_client.as_ref(),
                commands.into_ffi(),
            )
        };

        CarlaFuture::new(ffi_future).await
    }
}
```

**Pros:**
- **Runtime-agnostic**: Works with tokio, async-std, smol, or any executor
- **True async**: Leverages C++ futures without blocking threads
- **Zero-copy polling**: No channels or extra threads
- **Efficient**: Minimal overhead beyond FFI call
- **Direct mapping**: Preserves CARLA's async architecture

**Cons:**
- Requires waker FFI bridge (complex but one-time implementation)
- Need to expose rpc_client or modify CARLA Client
- autocxx may not support all needed types (fallback to manual bindings)

### Option 2: Callback-Based Bridge (Recommended)

**Approach:** Convert C++ async_call futures into Rust oneshot channels.

**Implementation:**

```cpp
// FFI wrapper in carla-sys
void async_call_with_callback(
    Client* client,
    const std::string& function,
    std::function<void(RPCLIB_MSGPACK::object_handle)> callback
) {
    auto future = client->async_call(function, ...);

    // Spawn thread to wait and invoke callback
    std::thread([future = std::move(future), callback = std::move(callback)]() mutable {
        try {
            auto result = future.get();
            callback(result);
        } catch (...) {
            // Error handling
        }
    }).detach();
}
```

```rust
// Rust wrapper in carla crate
pub async fn spawn_actor_async(
    &self,
    blueprint: &ActorBlueprint,
    transform: &Transform
) -> Result<Actor> {
    let (tx, rx) = tokio::sync::oneshot::channel();

    unsafe {
        async_call_with_callback(
            self.ffi_client,
            "spawn_actor",
            Box::new(move |result| {
                let _ = tx.send(parse_actor(result));
            })
        );
    }

    rx.await?
}
```

**Pros:**
- Works with existing autocxx
- Familiar Rust async patterns
- Compatible with tokio/async-std

**Cons:**
- Extra thread per async call (mitigated by thread pool)
- Slight overhead vs native futures

### Option 3: Tokio Task-Based Wrapper (Simplest)

**Approach:** Wrap blocking calls in `tokio::task::spawn_blocking()`.

```rust
impl Client {
    pub async fn spawn_actor_async(
        &self,
        blueprint: &ActorBlueprint,
        transform: &Transform
    ) -> Result<Actor> {
        let client = self.clone();  // Arc-wrapped
        let blueprint = blueprint.clone();
        let transform = *transform;

        tokio::task::spawn_blocking(move || {
            client.spawn_actor(&blueprint, &transform)
        })
        .await?
    }
}
```

**Pros:**
- Zero FFI changes needed
- Works immediately
- Leverages tokio's thread pool

**Cons:**
- Not "true" async (blocks worker thread)
- Less efficient than native async

### Option 4: Event Loop Integration (Future Work)

**Approach:** Integrate rpclib's ASIO event loop with Rust async runtime.

**Requirements:**
- Expose rpclib's internal ASIO io_context
- Create tokio/async-std compatible waker
- Register futures with Rust runtime

**Pros:**
- Most efficient (no thread blocking)
- True async end-to-end

**Cons:**
- Very complex
- Requires deep rpclib internals knowledge
- May conflict with CARLA's threading model

## Sensor Callback Async Bridge

**Current callback pattern:**

```rust
sensor.listen(move |data| {
    // Runs on C++ worker thread
    process_sensor_data(data);
});
```

**Async stream bridge:**

```rust
use tokio::sync::mpsc;

pub fn sensor_stream(sensor: &Sensor) -> impl Stream<Item = SensorData> {
    let (tx, rx) = mpsc::unbounded_channel();

    sensor.listen(move |data| {
        let _ = tx.send(data);
    });

    tokio_stream::wrappers::UnboundedReceiverStream::new(rx)
}

// Usage
let mut stream = sensor_stream(&lidar_sensor);
while let Some(data) = stream.next().await {
    process_lidar(data);
}
```

**Pros:**
- Natural async iteration
- Backpressure handling
- Works with existing callbacks

## World Tick Async Bridge

**Approach:** Convert `OnTick()` callbacks to async stream.

```rust
pub fn tick_stream(world: &World) -> impl Stream<Item = WorldSnapshot> {
    let (tx, rx) = mpsc::unbounded_channel();

    let callback_id = world.on_tick(move |snapshot| {
        let _ = tx.send(snapshot);
    });

    // TODO: Store callback_id for cleanup

    tokio_stream::wrappers::UnboundedReceiverStream::new(rx)
}

// Usage
let mut ticks = tick_stream(&world);
while let Some(snapshot) = ticks.next().await {
    println!("Frame: {}", snapshot.frame);
}
```

## Recommended Implementation Plan

### Phase 1: FFI Future Foundation (1 week)

**Goal:** Implement `FfiFuture` wrapper and basic Rust `Future` trait integration.

**Tasks:**
- [ ] Create `carla-sys/csrc/carla_rust/client/waker.hpp`
  - Define `RustWaker` struct with FFI-safe layout
  - Implement `wake_by_ref()` C++ helper
- [ ] Create `carla-sys/csrc/carla_rust/client/async_future.hpp`
  - Implement `FfiFuture` class wrapping `std::shared_future`
  - Add `poll_timeout()`, `get()`, `register_waker()` methods
  - Add free functions for autocxx compatibility
- [ ] Add bindings in `carla-sys/src/bindings.rs`:
  - `generate!("carla_rust::client::FfiFuture")`
  - `generate!("carla_rust::client::PollStatus")`
  - `generate!("carla_rust::client::FfiFuture_*")` free functions
- [ ] Create `carla/src/client/async_future.rs`
  - Implement `CarlaFuture<T>` wrapper
  - Implement `Future` trait with proper polling
  - Implement `RustWaker` creation and FFI bridge
  - Add `#[no_mangle] wake_by_ref()` export

**Success Criteria:**
- Compiles without errors
- Unit test: Create `FfiFuture` from mock `std::future`
- Unit test: Poll returns `Pending`, then `Ready`
- Unit test: Waker gets called when future becomes ready

**Time:** 5-7 days

### Phase 2: Expose rpclib async_call (3-4 days)

**Goal:** Add async wrappers to expose rpclib's `async_call` via FFI.

**Options:**

**Option A: Extend Client::Pimpl** (Minimal changes to CARLA)
```cpp
// carla-sys/csrc/carla_rust/client/client.hpp
std::unique_ptr<FfiFuture> FfiClient_async_spawn_actor(
    carla::client::detail::Client* client,
    const ActorDescription& desc,
    const Transform& transform
) {
    // Access client->pimpl_->rpc_client.async_call(...)
    auto future = client->GetPimpl()->GetRpcClient().async_call(
        "spawn_actor",
        desc,
        transform
    );
    return FfiFuture_new(std::move(future));
}
```

**Tasks:**
- [ ] Add `GetPimpl()` and `GetRpcClient()` accessors to Client (or use friend class)
- [ ] Create async wrapper functions for key operations:
  - `async_spawn_actor`
  - `async_destroy_actor`
  - `async_apply_control`
  - `async_apply_batch`
- [ ] Add autocxx bindings for new functions
- [ ] Create Rust wrappers in `carla/src/client/`

**Success Criteria:**
- Can call `client.spawn_actor_async()` from Rust
- Returns `CarlaFuture<Actor>`
- Polling works correctly

**Time:** 3-4 days

### Phase 3: High-Level Async API (4-5 days)

**Goal:** Add `*_async()` methods to World, Actor, Vehicle, etc.

**Tasks:**
- [ ] Add `world.rs` async methods:
  - `spawn_actor_async()`
  - `destroy_actor_async()`
  - `apply_batch_async()`
  - `tick_async()`
- [ ] Add `vehicle.rs` async methods:
  - `apply_control_async()`
  - `apply_ackermann_control_async()`
  - `set_autopilot_async()`
- [ ] Add `walker.rs` async methods:
  - `apply_control_async()`
  - `set_bones_async()`
- [ ] Add comprehensive documentation
- [ ] Add runtime-agnostic note in docs

**Success Criteria:**
- All common operations have `*_async()` variants
- Works with tokio, async-std, smol (test all three)
- Documentation includes examples for each runtime

**Time:** 4-5 days

### Phase 4: Async Sensor Streams (2-3 days)

**Goal:** Bridge sensor callbacks to async streams.

**Tasks:**
- [ ] Add optional `tokio` feature in `Cargo.toml`
- [ ] Create `carla/src/client/sensor_stream.rs`:
  - `sensor_stream()` helper using `mpsc::unbounded_channel`
  - Handles callback registration/cleanup
- [ ] Create example: `async_lidar_processing.rs`
- [ ] Document stream backpressure handling

**Success Criteria:**
- Can `stream.next().await` on sensor data
- Properly cleans up callbacks on stream drop
- Example demonstrates async sensor processing

**Time:** 2-3 days

### Phase 5: Async World Tick Stream (1-2 days)

**Goal:** Bridge `OnTick` callbacks to async stream.

**Tasks:**
- [ ] Create `tick_stream()` helper
- [ ] Handle callback ID cleanup on drop
- [ ] Create example: `async_synchronous_mode.rs`
- [ ] Demonstrate `tokio::select!` with ticks and sensors

**Success Criteria:**
- Can await world ticks in async code
- Cleanup verified (no leaked callbacks)
- Example shows event-driven async loop

**Time:** 1-2 days

### Phase 6: Advanced Patterns & Optimization (1 week, optional)

**Goal:** Production-ready async features.

**Tasks:**
- [ ] Add timeout wrappers:
  - `spawn_actor_timeout(duration)`
  - `apply_batch_timeout(duration)`
- [ ] Implement cooperative waking optimization:
  - Background thread polls C++ futures
  - Batch wake Rust tasks when ready
- [ ] Add structured concurrency helpers:
  - `spawn_actors_concurrent(blueprints)`
  - `destroy_actors_concurrent(actor_ids)`
- [ ] Benchmark async vs sync performance
- [ ] Document best practices

**Success Criteria:**
- Timeout API prevents indefinite hangs
- Optimized waking improves throughput
- Benchmarks show minimal overhead

**Time:** 1 week

### Total Timeline: 3-4 weeks for full async support

**Phases 1-3:** Core async RPC (2-2.5 weeks)
**Phases 4-5:** Async streams (3-5 days)
**Phase 6:** Production optimization (optional, 1 week)

## Example: Async Vehicle Control

```rust
use carla::client::Client;
use carla::rpc::VehicleControl;
use tokio;

#[tokio::main]
async fn main() -> Result<()> {
    let client = Client::connect("127.0.0.1", 2000, None);
    let world = client.world();

    // Async vehicle spawning
    let vehicle = world.spawn_actor_async(
        &blueprint_lib.find("vehicle.tesla.model3")?,
        &spawn_point
    ).await?;

    // Async sensor stream
    let mut lidar_stream = sensor_stream(&lidar_sensor);

    // Async tick stream
    let mut tick_stream = tick_stream(&world);

    loop {
        tokio::select! {
            Some(snapshot) = tick_stream.next() => {
                println!("Frame: {}", snapshot.frame);

                // Apply control asynchronously
                let control = VehicleControl {
                    throttle: 0.5,
                    steer: 0.0,
                    ..Default::default()
                };
                vehicle.apply_control_async(&control).await?;
            }

            Some(lidar_data) = lidar_stream.next() => {
                process_lidar_async(lidar_data).await;
            }
        }
    }
}
```

## Performance Considerations

### Thread Pool Sizing

**Current:** CARLA client uses `std::thread::hardware_concurrency()` workers.

**Recommendation:**
- Keep CARLA's default for I/O operations
- Use separate tokio runtime for compute tasks
- Avoid starving either thread pool

### Memory Overhead

- Each `mpsc::channel()` for streams: ~128 bytes + message queue
- Each `spawn_blocking()` call: reuses tokio worker thread
- Callback bridge: one thread per in-flight async call

**Recommendation:** Start with spawn_blocking (Option 3), profile, then optimize if needed.

### Latency

- `spawn_blocking()`: ~10-100μs overhead (tokio scheduler)
- Callback bridge: ~1-5μs overhead (oneshot channel)
- Native future: ~0μs overhead (ideal)

**For CARLA:** Given network RPC latency (1-50ms), spawn_blocking overhead is negligible.

## Compatibility Matrix

| Feature | rpclib Support | CARLA Exposure | Rust Bridge Complexity |
|---------|---------------|----------------|------------------------|
| async_call | ✅ Native | ❌ Discarded | Medium (Option 2/3) |
| Sensor callbacks | ✅ Native | ✅ Exposed | Low (channels) |
| World tick callbacks | ✅ Native | ✅ Exposed | Low (channels) |
| Future polling | ✅ std::future | ❌ Not exposed | High (FFI wrapper) |
| Streaming client | ✅ ASIO async | ✅ Background | N/A (internal) |

## Conclusion

**libcarla_client has excellent non-blocking/async support** via rpclib's `std::future`-based `async_call()` that can be directly exposed to Rust. The recommended implementation is:

### Direct std::future FFI Bridge (Option 1) - RECOMMENDED

**Why this approach is best:**

1. **Runtime-agnostic**: Works with any Rust async runtime (tokio, async-std, smol, etc.)
2. **True async**: Leverages C++'s actual futures without extra threads or channels
3. **Zero overhead**: Minimal abstraction - just FFI polling of std::future::wait_for()
4. **Preserves architecture**: Maps directly to CARLA's existing async model
5. **One-time complexity**: Waker FFI bridge is complex but reusable for all operations

### Implementation Path:

**Phase 1 (1 week):** Build `FfiFuture` wrapper + Rust `Future` trait implementation
- Creates the foundation for all async operations
- Most complex part but enables everything else

**Phase 2-3 (1.5 weeks):** Expose async RPC calls and add high-level API
- Wrap key CARLA operations with `*_async()` methods
- Works across all Rust async runtimes

**Phase 4-5 (1 week):** Add sensor and tick streams
- Bridge callbacks to async streams
- Enable reactive, event-driven patterns

**Total: 3-4 weeks for production-ready async support**

### Why not simpler approaches?

- **spawn_blocking (Option 3)**: Works but wastes thread pool threads on blocking waits
- **Callback bridge (Option 2)**: Requires extra threads, less efficient than polling C++ futures
- **cxx-async**: Requires C++20 coroutines which rpclib doesn't use

The FFI future approach provides the best balance of **efficiency**, **correctness**, and **Rust idiomaticity** while being **runtime-agnostic**.

## References

- rpclib documentation: `PythonAPI/carla/dependencies/include/system/rpc/client.h`
- rpclib implementation: `Build-Original/_deps/rpclib-src/include/rpc/client.inl`
- CARLA client: `LibCarla/source/carla/client/detail/Client.cpp`
- CARLA sensor: `LibCarla/source/carla/client/Sensor.h`
- tokio documentation: https://docs.rs/tokio/latest/tokio/
