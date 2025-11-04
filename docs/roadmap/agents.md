# Navigation Agents Implementation

**Priority:** HIGH (Blocks Phase 13.1: Automatic Control GUI Example)
**Estimated Effort:** 4-6 weeks
**Status:** In Progress - BasicAgent, BehaviorAgent, ConstantVelocityAgent complete. Full A* GlobalRoutePlanner complete.
**Dependencies:** Existing Navigation APIs (Map, Waypoint), Vehicle Control APIs

## Contents

- [Overview](#overview)
- [Python Agent API Reference](#python-agent-api-reference)
- [Implementation Phases](#implementation-phases)
- [Missing Rust APIs](#missing-rust-apis)
- [Verification Strategy](#verification-strategy)
- [Timeline and Estimates](#timeline-and-estimates)

## Overview

This document outlines the implementation of CARLA's navigation agent system in Rust. The agent system provides autonomous vehicle control capabilities through a layered architecture of planners and controllers.

### Component Hierarchy

```
GlobalRoutePlanner (high-level path planning)
    ↓
LocalPlanner (trajectory following + PID control)
    ↓
VehiclePIDController (low-level vehicle control)
    ├─ PIDLongitudinalController (throttle/brake)
    └─ PIDLateralController (steering)
```

### Why This Phase is Needed

- **Blocks Phase 13.1**: The automatic_control GUI example requires agent APIs
- **Feature Parity**: Matches Python's autonomous navigation capabilities
- **Testing**: Enables autonomous traffic scenarios and testing
- **Building Blocks**: Provides reusable components for custom behaviors

### Complexity Assessment

**Complexity Level:** HIGH

**Challenges:**
- Graph algorithms (networkx → petgraph translation)
- PID control tuning to match Python behavior
- FFI bindings for complex APIs (traffic light waypoints, map queries)
- Geometry calculations (bounding box intersection)
- Real-time performance requirements

## Python Agent API Reference

### Agent Inheritance Hierarchy

The Python implementation uses **class inheritance** where agents extend each other:

```
BasicAgent (base class - 508 lines)
    ├─ BehaviorAgent (extends BasicAgent - 318 lines)
    └─ ConstantVelocityAgent (extends BasicAgent - 130 lines)
```

**Key Observation:** There is **NO abstract base class** in Python. `BasicAgent` is a concrete class that serves as both:
1. A usable agent on its own
2. A base class for more specialized agents

**Inheritance Pattern:**
- `BehaviorAgent` **extends** `BasicAgent` via `super().__init__()`
- `ConstantVelocityAgent` **extends** `BasicAgent` via `super().__init__()`
- Both override `run_step()` to customize control logic
- Both reuse helper methods (`_affected_by_traffic_light`, `_vehicle_obstacle_detected`)

### BasicAgent

**File:** `PythonAPI/carla/agents/navigation/basic_agent.py` (508 lines)

**Purpose:** Traffic-aware navigation agent with obstacle detection and traffic light compliance.

**Constructor:**
```python
BasicAgent(vehicle, target_speed=20, opt_dict={}, map_inst=None, grp_inst=None)
```

**Parameters:**
- `vehicle`: Actor - The vehicle to control
- `target_speed`: float - Target cruise speed in km/h (default: 20)
- `opt_dict`: dict - Optional configuration parameters
- `map_inst`: Map - Optional map instance (default: queries from vehicle's world)
- `grp_inst`: GlobalRoutePlanner - Optional planner instance (default: creates new)

**Public Methods (15):**

| Method                                                                      | Return Type        | Purpose                              |
|-----------------------------------------------------------------------------|--------------------|--------------------------------------|
| `set_target_speed(speed)`                                                   | None               | Set cruise speed (km/h)              |
| `follow_speed_limits(value=True)`                                           | None               | Enable dynamic speed limits from map |
| `set_destination(end_location, start_location=None)`                        | None               | Set navigation goal                  |
| `set_global_plan(plan, stop_waypoint_creation, clean_queue)`                | None               | Apply pre-computed route             |
| `trace_route(start_waypoint, end_waypoint)`                                 | List[Tuple]        | Compute route between waypoints      |
| `run_step()`                                                                | VehicleControl     | Execute one control cycle            |
| `done()`                                                                    | bool               | Check if destination reached         |
| `ignore_traffic_lights(active)`                                             | None               | Toggle red light compliance          |
| `ignore_stop_signs(active)`                                                 | None               | Toggle stop sign compliance          |
| `ignore_vehicles(active)`                                                   | None               | Toggle vehicle obstacle detection    |
| `set_offset(offset)`                                                        | None               | Set lateral positioning offset       |
| `lane_change(direction, same_lane_time, other_lane_time, lane_change_time)` | None               | Execute lane change maneuver         |
| `get_local_planner()`                                                       | LocalPlanner       | Access LocalPlanner instance         |
| `get_global_planner()`                                                      | GlobalRoutePlanner | Access GlobalRoutePlanner instance   |
| `add_emergency_stop(control)`                                               | VehicleControl     | Apply maximum braking                |

**Dependencies:**
- LocalPlanner
- GlobalRoutePlanner
- Utility functions: `get_speed()`, `is_within_distance()`, `compute_distance()`
- Polygon intersection for bounding box checks (shapely)

### BehaviorAgent

**File:** `PythonAPI/carla/agents/navigation/behavior_agent.py` (780 lines)

**Purpose:** Extended agent with behavior profiles (cautious/normal/aggressive) and advanced hazard detection.

**Extends:** BasicAgent

**Constructor:**
```python
BehaviorAgent(vehicle, behavior='normal', opt_dict={}, map_inst=None, grp_inst=None)
```

**Behavior Profiles:**
- `'cautious'` - Conservative driving, large safety margins, low speed
- `'normal'` - Standard driving behavior
- `'aggressive'` - Fast driving, small safety margins, quick lane changes

**Additional Methods (7):**
- `update_information()` - Refresh vehicle state and look-ahead
- `traffic_light_manager()` - Traffic light detection and compliance
- `_tailgating()` - Detect lane change opportunities
- `collision_and_car_avoid_manager()` - Obstacle detection with time-to-collision
- `pedestrian_avoid_manager()` - Walker collision prevention
- `car_following_manager()` - Adaptive cruise control
- `emergency_stop()` - Hard brake override

**Note:** BehaviorAgent is optional for Phase 13.1 - BasicAgent is sufficient initially.

### ConstantVelocityAgent

**File:** `PythonAPI/carla/agents/navigation/constant_velocity_agent.py` (42 lines)

**Purpose:** Simple agent that maintains constant velocity for testing.

**Extends:** BasicAgent

**Override:** Disables speed limit following and traffic light detection.

### LocalPlanner

**File:** `PythonAPI/carla/agents/navigation/local_planner.py` (350 lines)

**Purpose:** Executes waypoint-following with PID control and speed management.

**Constructor:**
```python
LocalPlanner(vehicle, opt_dict={}, map_inst=None)
```

**Configuration Parameters (opt_dict):**
```python
{
    'dt': 1.0/20.0,  # Simulation timestep
    'target_speed': 20.0,  # km/h
    'sampling_radius': 2.0,  # Waypoint look-ahead distance
    'lateral_control_dict': {'K_P': 1.95, 'K_D': 0.01, 'K_I': 1.4},
    'longitudinal_control_dict': {'K_P': 1.0, 'K_D': 0, 'K_I': 1},
    'max_throttle': 0.75,
    'max_brake': 0.3,
    'max_steering': 0.8,
    'offset': 0
}
```

**Public Methods (8):**
- `set_speed(speed)` - Set target speed (km/h)
- `follow_speed_limits(value=True)` - Enable speed limit compliance
- `set_global_plan(current_plan, stop_waypoint_creation, clean_queue)` - Load waypoint sequence
- `set_offset(offset)` - Set lateral offset
- `run_step(debug=False)` - Execute planning cycle, returns VehicleControl
- `get_incoming_waypoint_and_direction(steps=3)` - Look ahead N waypoints
- `get_plan()` - Get current waypoint queue
- `done()` - Check if route completed

**RoadOption Enum:**
```python
class RoadOption(IntEnum):
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6
```

### GlobalRoutePlanner

**File:** `PythonAPI/carla/agents/navigation/global_route_planner.py` (440 lines)

**Purpose:** High-level route computation using A* pathfinding on road topology graph.

**Constructor:**
```python
GlobalRoutePlanner(wmap, sampling_resolution)
```

**Parameters:**
- `wmap`: Map - CARLA map instance
- `sampling_resolution`: float - Distance between graph nodes (meters)

**Public Methods (1):**
- `trace_route(origin, destination)` → `List[Tuple[Waypoint, RoadOption]]`

**Algorithm:**
1. Build graph from Map::topology()
2. Add lane change edges (zero-cost lateral moves)
3. Run A* with Euclidean distance heuristic
4. Annotate path with RoadOption (STRAIGHT/LEFT/RIGHT)
5. Return waypoint sequence with turn instructions

**Dependencies:**
- `networkx` (graph data structure and A* algorithm)
- `numpy` (numerical operations)

### VehiclePIDController

**File:** `PythonAPI/carla/agents/navigation/controller.py` (270 lines)

**Purpose:** Low-level vehicle control using PID controllers for steering and speed.

**Constructor:**
```python
VehiclePIDController(vehicle, args_lateral, args_longitudinal, offset=0,
                     max_throttle=0.75, max_brake=0.3, max_steering=0.8)
```

**PID Parameters:**
- **Lateral** (steering): `{'K_P': 1.95, 'K_D': 0.01, 'K_I': 1.4}`
- **Longitudinal** (speed): `{'K_P': 1.0, 'K_D': 0, 'K_I': 1}`

**Public Methods (4):**
- `run_step(target_speed, waypoint)` → VehicleControl
- `change_longitudinal_PID(args_longitudinal)` - Update speed PID
- `change_lateral_PID(args_lateral)` - Update steering PID
- `set_offset(offset)` - Set lateral offset

**Control Logic:**
- Steering changes limited to ±0.1 per frame for smoothness
- Throttle/brake selected based on acceleration sign
- Configurable control limits (max_throttle, max_brake, max_steering)

### Utility Functions

**File:** `PythonAPI/carla/agents/tools/misc.py`

**Required Functions:**
- `get_speed(vehicle)` → float - Calculate velocity magnitude in km/h
- `is_within_distance(target_transform, reference_transform, max_distance, angle_interval)` → bool - Proximity check
- `compute_distance(location_1, location_2)` → float - Euclidean distance
- `get_trafficlight_trigger_location(traffic_light)` → Location - Trigger volume location
- `compute_magnitude_angle(target_location, current_location, orientation)` → (float, float) - Distance and relative angle
- `distance_vehicle(waypoint, vehicle_transform)` → float - 2D distance
- `vector(location_1, location_2)` → Vector3D - Normalized direction vector
- `draw_waypoints(world, waypoints, z)` → None - Debug visualization (optional)
- `positive(num)` → float - Returns number if positive, else 0.0

## Rust Design Architecture

### Summary

The Rust agent implementation uses **composition over inheritance** to achieve code reuse while maintaining type safety and zero-cost abstractions.

**Key Design Decisions:**

1. **AgentCore as Shared Component:** All agents contain an `AgentCore` struct that provides:
   - Hazard detection methods (traffic lights, obstacles)
   - Lane change path generation
   - Configuration management
   - Planner instances (LocalPlanner, GlobalRoutePlanner)

2. **Concrete Types, Not Trait Objects:** Each agent type (`BasicAgent`, `BehaviorAgent`, `ConstantVelocityAgent`) is a distinct struct, not variations of a base class. This provides:
   - Zero-cost abstraction (no vtable overhead)
   - Type-specific APIs (each agent can have unique methods)
   - Clear ownership semantics

3. **Optional Trait for Polymorphism:** An `Agent` trait will be implemented **only if needed** for polymorphic use cases (e.g., storing different agent types in collections).

4. **Code Reuse via Composition:** Instead of inheriting methods from a base class, all agents:
   - Own an `AgentCore` instance
   - Delegate to `core.affected_by_traffic_light()`, `core.vehicle_obstacle_detected()`, etc.
   - Share the same underlying detection and planning logic

### Design Philosophy: Composition over Inheritance

Unlike Python's class inheritance approach, the Rust implementation will use **composition with shared components**.

**Python vs Rust Approach:**

```
Python (Inheritance):                    Rust (Composition):
┌─────────────────┐                      ┌─────────────────┐
│   BasicAgent    │                      │   AgentCore     │
│  (base class)   │                      │ (shared logic)  │
└────────┬────────┘                      └────────┬────────┘
         │ extends                                │ contains
    ┌────┴────┐                           ┌──────┴──────┐
    ▼         ▼                           ▼             ▼
┌─────────┐ ┌─────────┐         ┌─────────────┐ ┌──────────────┐
│Behavior │ │Constant │         │ BasicAgent  │ │BehaviorAgent │
│ Agent   │ │Velocity │         │ {core: ..}  │ │ {core: ..}   │
└─────────┘ └─────────┘         └─────────────┘ └──────────────┘
```

**Why composition in Rust?**
1. Rust doesn't have classical inheritance
2. Trait objects have runtime overhead (avoided for performance)
3. Composition is more flexible and idiomatic in Rust
4. Code sharing via shared structs is clearer and more explicit
5. Each agent type can have unique APIs without base class constraints

### Proposed Architecture

#### Core Components (Shared Structs)

```rust
// carla/src/agents/navigation/agent_core.rs

/// Shared core state and behavior detection used by all agents
pub struct AgentCore {
    vehicle: Vehicle,
    world: World,
    map: Map,

    // Planners
    local_planner: LocalPlanner,
    global_planner: GlobalRoutePlanner,

    // Detection flags
    ignore_traffic_lights: bool,
    ignore_stop_signs: bool,
    ignore_vehicles: bool,

    // Configuration
    base_tlight_threshold: f64,
    base_vehicle_threshold: f64,
    speed_ratio: f64,
    max_brake: f64,
    offset: f64,
    use_bbs_detection: bool,

    // Cached state
    lights_map: HashMap<u32, Waypoint>, // traffic_light.id -> trigger waypoint
    last_traffic_light: Option<TrafficLight>,
}

impl AgentCore {
    /// Core hazard detection methods shared by all agents
    pub fn affected_by_traffic_light(
        &mut self,
        lights_list: &ActorList,
        max_distance: f64,
    ) -> TrafficLightDetectionResult { /* ... */ }

    pub fn vehicle_obstacle_detected(
        &self,
        vehicle_list: &ActorList,
        max_distance: f64,
        up_angle_th: f64,
        low_angle_th: f64,
        lane_offset: i32,
    ) -> ObstacleDetectionResult { /* ... */ }

    pub fn generate_lane_change_path(
        waypoint: &Waypoint,
        direction: LaneChangeDirection,
        distance_same_lane: f64,
        distance_other_lane: f64,
        lane_change_distance: f64,
        check: bool,
        lane_changes: u32,
        step_distance: f64,
    ) -> Vec<(Waypoint, RoadOption)> { /* ... */ }

    // Configuration methods
    pub fn ignore_traffic_lights(&mut self, active: bool) { /* ... */ }
    pub fn ignore_stop_signs(&mut self, active: bool) { /* ... */ }
    pub fn ignore_vehicles(&mut self, active: bool) { /* ... */ }
    pub fn set_offset(&mut self, offset: f64) { /* ... */ }
}
```

#### Agent Types (Individual Structs)

Each agent type is its own struct that **contains** an `AgentCore`:

```rust
// carla/src/agents/navigation/basic_agent.rs

pub struct BasicAgent {
    core: AgentCore,
    target_speed: f64,
    sampling_resolution: f64,
}

impl BasicAgent {
    pub fn new(
        vehicle: Vehicle,
        target_speed: f64,
        opt_dict: AgentConfig,
        map_inst: Option<Map>,
        grp_inst: Option<GlobalRoutePlanner>,
    ) -> Result<Self> { /* ... */ }

    pub fn run_step(&mut self) -> Result<VehicleControl> {
        // BasicAgent-specific control logic
        let hazard_detected = false;

        // Use core for detection
        let vehicle_list = self.core.world.get_actors().filter("*vehicle*");
        let vehicle_speed = misc::get_speed(&self.core.vehicle) / 3.6;
        let max_vehicle_distance = self.core.base_vehicle_threshold
            + self.core.speed_ratio * vehicle_speed;

        let (affected_by_vehicle, _, _) = self.core.vehicle_obstacle_detected(
            &vehicle_list, max_vehicle_distance, 90.0, 0.0, 0
        );
        if affected_by_vehicle {
            hazard_detected = true;
        }

        // ... check traffic lights via core ...

        let mut control = self.core.local_planner.run_step()?;
        if hazard_detected {
            control = self.add_emergency_stop(control);
        }

        Ok(control)
    }

    pub fn set_destination(&mut self, end_location: Location, start_location: Option<Location>) { /* ... */ }
    pub fn add_emergency_stop(&self, control: VehicleControl) -> VehicleControl { /* ... */ }
    pub fn done(&self) -> bool { self.core.local_planner.done() }
    // ... other BasicAgent methods ...
}
```

```rust
// carla/src/agents/navigation/behavior_agent.rs

pub struct BehaviorAgent {
    core: AgentCore,  // Reuses all core detection logic
    behavior: BehaviorType,

    // BehaviorAgent-specific state
    look_ahead_steps: usize,
    speed: f64,
    speed_limit: f64,
    direction: Option<RoadOption>,
    incoming_direction: Option<RoadOption>,
    incoming_waypoint: Option<Waypoint>,
    min_speed: f64,
    sampling_resolution: f64,
}

#[derive(Clone)]
pub enum BehaviorType {
    Cautious(BehaviorParams),
    Normal(BehaviorParams),
    Aggressive(BehaviorParams),
}

pub struct BehaviorParams {
    pub max_speed: f64,
    pub speed_lim_dist: f64,
    pub speed_decrease: f64,
    pub safety_time: f64,
    pub min_proximity_threshold: f64,
    pub braking_distance: f64,
    pub tailgate_counter: i32,
}

impl BehaviorAgent {
    pub fn new(
        vehicle: Vehicle,
        behavior: BehaviorType,
        opt_dict: AgentConfig,
        map_inst: Option<Map>,
        grp_inst: Option<GlobalRoutePlanner>,
    ) -> Result<Self> { /* ... */ }

    pub fn run_step(&mut self, debug: bool) -> Result<VehicleControl> {
        self.update_information();

        // 1. Traffic light check (reuses core)
        if self.traffic_light_manager() {
            return Ok(self.emergency_stop());
        }

        // 2. Pedestrian avoidance
        let (walker_state, walker, w_distance) = self.pedestrian_avoid_manager()?;
        if walker_state {
            let distance = w_distance - /* bounding box calculation */;
            if distance < self.behavior.params().braking_distance {
                return Ok(self.emergency_stop());
            }
        }

        // 3. Car following with TTC calculation
        let (vehicle_state, vehicle, distance) = self.collision_and_car_avoid_manager()?;
        if vehicle_state {
            let distance = distance - /* bounding box calculation */;
            if distance < self.behavior.params().braking_distance {
                return Ok(self.emergency_stop());
            } else {
                return Ok(self.car_following_manager(vehicle, distance, debug)?);
            }
        }

        // 4. Normal control
        let target_speed = /* ... */;
        self.core.local_planner.set_speed(target_speed);
        self.core.local_planner.run_step(debug)
    }

    fn update_information(&mut self) { /* ... */ }
    fn traffic_light_manager(&mut self) -> bool { /* ... */ }
    fn collision_and_car_avoid_manager(&mut self) -> Result<ObstacleDetectionResult> { /* ... */ }
    fn pedestrian_avoid_manager(&mut self) -> Result<ObstacleDetectionResult> { /* ... */ }
    fn car_following_manager(&mut self, vehicle: Actor, distance: f64, debug: bool) -> Result<VehicleControl> { /* ... */ }
    fn tailgating(&mut self, waypoint: &Waypoint, vehicle_list: &ActorList) { /* ... */ }
    fn emergency_stop(&self) -> VehicleControl { /* ... */ }
}
```

```rust
// carla/src/agents/navigation/constant_velocity_agent.rs

pub struct ConstantVelocityAgent {
    core: AgentCore,  // Reuses core detection

    // ConstantVelocityAgent-specific state
    use_basic_behavior: bool,
    target_speed: f64,  // m/s
    current_speed: f64,
    constant_velocity_stop_time: Option<f64>,
    collision_sensor: Option<Sensor>,
    restart_time: f64,
    is_constant_velocity_active: bool,
}

impl ConstantVelocityAgent {
    pub fn run_step(&mut self) -> Result<VehicleControl> {
        if !self.is_constant_velocity_active {
            // Check restart timer or use basic behavior
            // ...
        }

        // Simplified hazard detection (reuses core methods)
        let mut hazard_speed = self.target_speed;

        // ... vehicle and traffic light checks via core ...

        self.set_constant_velocity(hazard_speed);
        self.core.local_planner.run_step()
    }

    fn set_constant_velocity(&mut self, speed: f64) { /* ... */ }
    fn set_collision_sensor(&mut self) { /* ... */ }
    pub fn destroy_sensor(&mut self) { /* ... */ }
}
```

### Optional: Trait for Polymorphism (If Needed)

If we need to store different agent types in a collection or pass them generically:

```rust
pub trait Agent {
    fn run_step(&mut self) -> Result<VehicleControl>;
    fn done(&self) -> bool;
    fn set_destination(&mut self, end_location: Location, start_location: Option<Location>);
    fn set_target_speed(&mut self, speed: f64);
    // ... common interface methods ...
}

impl Agent for BasicAgent { /* ... */ }
impl Agent for BehaviorAgent { /* ... */ }
impl Agent for ConstantVelocityAgent { /* ... */ }

// Usage (if needed):
fn run_agent_loop(agent: &mut dyn Agent) {
    while !agent.done() {
        let control = agent.run_step().unwrap();
        // apply control...
    }
}
```

**Decision:** Implement the trait **only if user code requires polymorphism**. For initial implementation, concrete types are sufficient.

### Module Structure

```
carla/src/agents/
├── mod.rs                              # Public re-exports
├── navigation/
│   ├── mod.rs                          # Navigation module exports
│   ├── types.rs                        # RoadOption, AgentConfig, etc.
│   ├── agent_core.rs                   # Shared AgentCore struct
│   ├── basic_agent.rs                  # BasicAgent struct
│   ├── behavior_agent.rs               # BehaviorAgent struct + BehaviorType enum
│   ├── constant_velocity_agent.rs      # ConstantVelocityAgent struct
│   ├── local_planner.rs                # LocalPlanner
│   ├── global_route_planner.rs         # GlobalRoutePlanner
│   ├── controller.rs                   # VehiclePIDController
│   └── pid.rs                          # PID controller implementations
└── tools/
    ├── mod.rs                          # Tools module exports
    ├── misc.rs                         # Utility functions
    └── types.rs                        # ObstacleDetectionResult, TrafficLightDetectionResult
```

### Benefits of This Design

1. **Code Reuse:** All agents share `AgentCore` for common detection logic
2. **Type Safety:** Each agent is a concrete type with specific behavior
3. **No Runtime Overhead:** No trait objects unless explicitly needed
4. **Clear Ownership:** Each agent owns its core, planners, and state
5. **Extensibility:** Easy to add new agent types by creating new structs with `AgentCore`
6. **Idiomatic Rust:** Uses composition, not inheritance
7. **Testing:** Can test `AgentCore` independently from agent types

## Implementation Phases

### Implementation Summary

**Current Status:** BasicAgent implementation complete and functional ✅

**What's Implemented:**
- ✅ **Phase A.1:** PID controllers, utility functions, RoadOption enum (COMPLETE)
- ✅ **Phase A.2:** LocalPlanner with waypoint queue management and PID integration (COMPLETE)
- ✅ **Phase A.3.1:** GlobalRoutePlanner simplified implementation (COMPLETE)
- ✅ **Phase A.3.2:** Full A* GlobalRoutePlanner with graph-based pathfinding (COMPLETE)
- ✅ **Phase A.4.1:** BasicAgent and AgentCore (COMPLETE)
- ✅ **Phase A.4.2:** Lane change support (COMPLETE)
- ✅ **Phase A.4.4:** BehaviorAgent with behavior profiles (COMPLETE)
- ✅ **Phase A.4.5:** ConstantVelocityAgent (COMPLETE)
- ✅ **Phase A.4.6:** Agent trait for polymorphism (COMPLETE)

**What's Deferred:**
- 🔒 **Phase A.4.3:** Enhanced detection (blocked by missing APIs - bounding box intersection needs `geo` crate)

**Files Created:**
- `carla/src/agents/navigation/local_planner.rs` (249 lines)
- `carla/src/agents/navigation/global_route_planner.rs` (303 lines - upgraded to A* with petgraph)
- `carla/src/agents/navigation/agent_core.rs` (394 lines - with lane change generation)
- `carla/src/agents/navigation/basic_agent.rs` (434 lines - with lane change support)
- `carla/src/agents/navigation/behavior_agent.rs` (599 lines - NEW)
- `carla/src/agents/navigation/constant_velocity_agent.rs` (354 lines - NEW)
- `carla/examples/basic_agent_demo.rs` (112 lines)
- `carla/examples/behavior_agent_demo.rs` (169 lines - NEW)
- `carla/examples/constant_velocity_agent_demo.rs` (175 lines - NEW)

**Key Implementation Decisions:**

1. **Composition Pattern:** AgentCore shared via composition (not inheritance)
2. **Full A* Implementation:**
   - GlobalRoutePlanner: Graph-based A* pathfinding with petgraph
   - Road topology graph construction from Map::topology()
   - Lane change edges with configurable costs
   - Turn decision logic using vector cross products
3. **Behavior Profiles:**
   - Three agent personalities: Cautious, Normal, Aggressive
   - Time-to-collision (TTC) based adaptive cruise control
   - Pedestrian and vehicle hazard detection
4. **Testing Mode:**
   - ConstantVelocityAgent for controlled testing scenarios
   - Optional basic behavior mode with hazard detection

**Testing Status:**
- ✅ All unit tests pass
- ✅ `make lint-rust` passes with no warnings
- ✅ `make build` successful
- ✅ Three working examples demonstrate full agent capabilities

**Blocking Issues:**
- 🔒 Phase A.4.3.1: Missing `TrafficLight::get_affected_lane_waypoints()` API
- 🔒 Phase A.4.3.2: Missing `Vehicle::bounding_box()` API and `geo` crate

**Ready for Use:**
- ✅ Phase 13.1 (Automatic Control GUI Example) can proceed with all agent types
- ✅ BasicAgent provides autonomous navigation with hazard detection
- ✅ BehaviorAgent provides advanced driving with behavior profiles
- ✅ ConstantVelocityAgent provides testing capabilities
- ✅ Full A* pathfinding produces optimal routes
- ✅ Lane change support for advanced maneuvers
- ✅ Agent trait enables polymorphic agent handling

---

### Phase A.1: Core Infrastructure and PID Controllers ✅ COMPLETE

**Status:** ✅ Complete (all items implemented and tested)

**Work Items:**

- [x] **A.1.1: Utility Functions Module**
  - File: `carla/src/agents/tools/misc.rs`
  - Implement 8 utility functions from misc.py
  - Functions: `get_speed`, `is_within_distance`, `compute_distance`, `get_trafficlight_trigger_location`, `compute_magnitude_angle`, `distance_vehicle`, `vector`, `draw_waypoints`
  - Tests: Unit tests for each function with known inputs/outputs
  - Time: 2-3 days

- [x] **A.1.2: PID Controller Base**
  - File: `carla/src/agents/navigation/pid.rs`
  - Implement `PIDLongitudinalController` (throttle/brake control)
  - Implement `PIDLateralController` (steering control)
  - PID formula: `output = K_P * error + K_I * integral + K_D * derivative`
  - Tests: PID response curves with known inputs
  - Time: 2-3 days

- [x] **A.1.3: VehiclePIDController**
  - File: `carla/src/agents/navigation/controller.rs`
  - Combine lateral + longitudinal controllers
  - Public methods: `run_step`, `change_longitudinal_pid`, `change_lateral_pid`, `set_offset`
  - Implement smooth steering changes (max ±0.1 per frame)
  - Tests: Integration tests with vehicle control outputs
  - Time: 2-3 days

- [x] **A.1.4: RoadOption Enum**
  - File: `carla/src/agents/navigation/types.rs`
  - Define `RoadOption` enum: `Void`, `Left`, `Right`, `Straight`, `LaneFollow`, `ChangeLaneLeft`, `ChangeLaneRight`
  - Implement `Display`, `Debug`, `FromStr` traits
  - Tests: Enum conversion and display tests
  - Time: 1 day

**Success Criteria:**
- PID controllers produce smooth control outputs
- Steering limited to ±0.1 per frame change
- Throttle/brake applied correctly based on acceleration sign
- All utility functions match Python behavior

### Phase A.2: LocalPlanner Implementation ✅ COMPLETE

**Estimated Effort:** 1.5-2 weeks
**Status:** ✅ Complete (all items implemented and tested)

**Work Items:**

- [x] **A.2.1: Waypoint Queue Management**
  - Data structure: `VecDeque<(Waypoint, RoadOption)>`
  - Waypoint sampling at `sampling_radius` intervals
  - Distance tracking and queue advancement
  - Target point computation from queue front
  - Tests: Queue operations, waypoint advancement logic
  - Time: 2-3 days

- [x] **A.2.2: LocalPlanner Core**
  - File: `carla/src/agents/navigation/local_planner.rs`
  - Constructor with `opt_dict` configuration
  - Implement `set_global_plan`, `set_speed`, `set_offset`, `follow_speed_limits`
  - Internal state management
  - Tests: Plan loading and parameter changes
  - Time: 3-4 days

- [x] **A.2.3: LocalPlanner Control Loop**
  - Implement `run_step()` - main planning cycle
  - PID controller integration
  - Speed limit following logic (query from waypoint)
  - Waypoint advancement when within sampling radius
  - Tests: Control output validation
  - Time: 3-4 days

- [x] **A.2.4: LocalPlanner Query Methods**
  - Implement `get_incoming_waypoint_and_direction(steps)` - Look ahead
  - Implement `get_plan()` - Return current queue
  - Implement `done()` - Check if queue empty
  - Tests: Query methods return correct data
  - Time: 1-2 days
  - **Note:** `reset_vehicle()` not implemented (not needed in Rust ownership model)

**Success Criteria:**
- LocalPlanner follows waypoint sequences smoothly
- Speed limits respected when enabled
- Lateral offset applied correctly
- Control outputs smooth and stable
- Done detection works correctly

### Phase A.3: GlobalRoutePlanner Implementation

#### Phase A.3.1: Simplified Implementation ✅ COMPLETE

**Status:** ✅ Complete - Functional but not optimal
**Estimated Effort:** 1 day

**Implementation Note:** The current implementation uses a **greedy waypoint selection** approach instead of full A* pathfinding. This simplified version:
- Uses `map.waypoint()` to get start/end waypoints
- Follows `waypoint.next()` greedily towards destination
- Selects next waypoint with minimum distance to goal
- Suitable for BasicAgent but may not find optimal routes

**Work Items:**

- [x] **A.3.1.1: Public API (Simplified)**
  - File: `carla/src/agents/navigation/global_route_planner.rs`
  - Implement `trace_route(origin: Location, destination: Location)` → `Vec<(Waypoint, RoadOption)>`
  - Greedy waypoint selection using `waypoint.next()`
  - Basic RoadOption assignment (all LaneFollow for now)
  - Tests: End-to-end route computation
  - Time: 1 day

**Success Criteria:**
- ✅ Routes computed between arbitrary locations
- ✅ Greedy selection produces drivable paths
- ⚠️ Optimal routes not guaranteed

#### Phase A.3.2: Full A* Implementation ✅ COMPLETE

**Status:** ✅ Complete - Full graph-based A* pathfinding
**Estimated Effort:** 1.5-2 weeks
**Actual Time:** Completed in session

**Dependencies:**
- ✅ Added `petgraph = "0.6"` to Cargo.toml (Rust graph library)

**Work Items:**

- [x] **A.3.2.1: Graph Data Structures**
  - Defined `RoadNode` struct with waypoint and location
  - Defined `RoadEdge` struct with length and road option
  - Uses `petgraph::Graph<RoadNode, RoadEdge, Directed>`
  - HashMap for waypoint ID to node index mapping
  - Time: Completed

- [x] **A.3.2.2: Graph Construction from Topology**
  - Implemented `build_graph()` - Process Map::topology() into graph
  - Phase 1: Create nodes for all waypoints in topology
  - Phase 2: Create edges with computed lengths and road options
  - Phase 3: Add lane change edges
  - Time: Completed

- [x] **A.3.2.3: Lane Change Links**
  - Implemented `add_lane_change_edges()` - Add lane change edges
  - Detects left/right lane availability using `waypoint.left()`, `waypoint.right()`
  - Connects parallel lanes with small edge cost (1.0)
  - Uses deferred edge addition to avoid borrow checker issues
  - Time: Completed

- [x] **A.3.2.4: Pathfinding**
  - Implemented using `petgraph::algo::astar`
  - Euclidean distance heuristic for goal estimation
  - `find_closest_node()` maps locations to nearest graph node
  - Converts node path to waypoint route with road options
  - Time: Completed

- [x] **A.3.2.5: Turn Decision Logic**
  - Implemented `compute_road_option()` - Classify turns
  - Uses vector cross product to determine turn direction
  - Uses dot product to distinguish straight from turns
  - Thresholds: STRAIGHT (dot > 0.7), LEFT (cross > 0.1), RIGHT (cross < -0.1)
  - Time: Completed

**Success Criteria (Full Implementation):**
- ✅ Graph built from Map topology with correct connectivity
- ✅ A* pathfinding produces optimal routes
- ✅ RoadOption annotations correct (LEFT/RIGHT/STRAIGHT at turns)
- ✅ Lane change options included
- ✅ Build and linter pass successfully

### Phase A.4: Agent Implementations

#### Phase A.4.1: BasicAgent Implementation ✅ COMPLETE

**Status:** ✅ Complete - Fully functional autonomous navigation
**Estimated Effort:** 1 week
**Priority:** HIGH (required for Phase 13.1)

**Work Items:**

- [x] **A.4.1.1: AgentCore Shared Structure**
  - File: `carla/src/agents/navigation/agent_core.rs`
  - Define `AgentCore` struct with vehicle, world, map, planners, flags, config
  - Constructor: `AgentCore::new(vehicle, map_inst, grp_inst, opt_dict)`
  - Configuration methods: `ignore_traffic_lights()`, `ignore_vehicles()`, `set_offset()`
  - Tests: Core instantiation and configuration
  - Time: 1 day

- [x] **A.4.1.2: AgentCore Traffic Light Detection (Simplified)**
  - Implement `affected_by_traffic_light(&mut self, max_distance)` → `TrafficLightDetectionResult`
  - Logic: Check red lights ahead using simple distance check
  - Cache traffic lights list for efficiency
  - Tests: Traffic light detection in various scenarios
  - Time: 2-3 days
  - **Note:** Simplified without trigger volume waypoint matching (see Phase A.4.3 for enhancement)

- [x] **A.4.1.3: AgentCore Vehicle Obstacle Detection (Simplified)**
  - Implement `vehicle_obstacle_detected(&self, max_distance)` → `ObstacleDetectionResult`
  - Simplified detection mode: distance + forward vector dot product
  - Tests: Obstacle detection with various vehicle configurations
  - Time: 3-4 days
  - **Note:** Simplified geometry check (see Phase A.4.3 for bounding box enhancement)

- [x] **A.4.1.4: BasicAgent Structure and Navigation**
  - File: `carla/src/agents/navigation/basic_agent.rs`
  - Define `BasicAgent { core: AgentCore, target_speed, sampling_resolution }`
  - Constructor: `BasicAgent::new(vehicle, target_speed, opt_dict, map_inst, grp_inst)`
  - Navigation methods:
    - `set_destination(end_location, start_location, clean_queue)`
    - `set_global_plan(plan, stop_waypoint_creation, clean_queue)`
    - `trace_route(start_waypoint, end_waypoint)` - delegates to global planner
    - `done()` - delegates to `core.local_planner.done()`
  - Tests: Agent instantiation and route planning
  - Time: 2 days

- [x] **A.4.1.5: BasicAgent Control Loop**
  - Implement `run_step(&mut self)` → `Result<VehicleControl>`
  - Control flow:
    1. Get vehicle speed
    2. Check vehicle obstacles (via `core.vehicle_obstacle_detected()`)
    3. Check traffic lights (via `core.affected_by_traffic_light()`)
    4. Get control from `core.local_planner.run_step()`
    5. Apply emergency stop if hazard detected
  - Implement `add_emergency_stop(control)` - set throttle=0, brake=max_brake
  - Configuration methods: `set_target_speed()`, `ignore_traffic_lights()`, `ignore_vehicles()`, `set_offset()`
  - Tests: Control output in various scenarios (clear road, obstacles, red lights)
  - Time: 2-3 days

- [x] **A.4.1.6: BasicAgent Example**
  - File: `carla/examples/basic_agent_demo.rs`
  - Demonstrates BasicAgent usage with spawn point navigation
  - Shows agent control loop with world tick and control application
  - Time: 1 day

**Success Criteria:**
- ✅ `AgentCore` provides reusable hazard detection via composition pattern
- ✅ `BasicAgent` navigates to destination autonomously
- ✅ Traffic light detection works (stops at red lights)
- ✅ Vehicle obstacle detection prevents collisions
- ✅ Code reuse via composition (BasicAgent uses AgentCore)
- ✅ All unit tests pass (59 tests)
- ✅ Example `basic_agent_demo.rs` demonstrates full navigation flow

#### Phase A.4.2: Lane Change Support ✅ COMPLETE

**Status:** ✅ Complete - Full lane change path generation and execution
**Estimated Effort:** 3-4 days
**Actual Time:** Completed in session

**Work Items:**

- [x] **A.4.2.1: AgentCore Lane Change Path Generation**
  - Implemented static method `generate_lane_change_path(waypoint, direction, distances, ...)`
  - Generates 3-phase waypoint sequence: same lane → lane change → other lane
  - Validates lane change is possible (checks lane availability)
  - Returns `Vec<(Waypoint, RoadOption)>` or empty if impossible
  - Uses configurable distances for each phase
  - Time: Completed

- [x] **A.4.2.2: BasicAgent Lane Change Method**
  - Implemented `lane_change(direction, same_lane_time, other_lane_time, lane_change_time)`
  - Converts time parameters to distances based on current speed
  - Calls `AgentCore::generate_lane_change_path()` to generate path
  - Applies path via `set_global_plan()`
  - Returns error if lane change not possible
  - Time: Completed

**Success Criteria:**
- ✅ Lane change maneuvers generate correct waypoint sequences
- ✅ Lane availability validation prevents invalid maneuvers
- ✅ Three-phase approach provides smooth transitions
- ✅ Build and tests pass

#### Phase A.4.3: Enhanced Detection (Blocked)

**Status:** Blocked - Missing core APIs
**Estimated Effort:** 1 week
**Priority:** Medium (improvements over simplified detection)

**Blocking Issues:**

1. **Traffic Light Trigger Volumes** (BLOCKED)
   - **Missing API:** `TrafficLight::get_affected_lane_waypoints() → Vec<Waypoint>`
   - **Current Workaround:** Simple distance check
   - **Why Blocked:** Cannot determine which lanes are controlled by traffic light
   - **Impact:** May detect traffic lights that don't affect current lane
   - **FFI Effort:** 3-4 days (complex vector return type)
   - **Status:** ⚠️ API already implemented as `TrafficLight::affected_lane_waypoints()` - Phase A.4.3.1 unblocked

2. **Bounding Box Intersection** (PARTIALLY UNBLOCKED)
   - **API Status:** ✅ `Vehicle::bounding_box() → BoundingBox<f32>` implemented (see carla/src/client/vehicle.rs:240-263)
   - **Remaining Blocker:** `geo = "0.27"` crate for polygon intersection not yet added
   - **Current Workaround:** Distance + dot product check
   - **What's Available:** Vehicle bounding boxes in world coordinates with extent and transform
   - **Impact of Missing:** Less accurate obstacle detection in complex scenarios
   - **Remaining Effort:** 1 day for geo crate integration + polygon intersection logic

**Work Items:**

- [ ] **A.4.3.1: Traffic Light Trigger Volume Detection** (UNBLOCKED)
  - **API Status:** ✅ `TrafficLight::affected_lane_waypoints()` already implemented
  - Implement `get_trafficlight_trigger_location()` utility
  - Use trigger volume waypoints to determine if traffic light affects current lane
  - Cache trigger waypoints in `lights_map`
  - Tests: Accurate lane-specific traffic light detection
  - Time: 2-3 days
  - **Note:** API is available, implementation can proceed

- [ ] **A.4.3.2: Bounding Box Intersection Detection** (PARTIALLY UNBLOCKED)
  - **API Status:** ✅ `Vehicle::bounding_box() → BoundingBox<f32>` implemented (vehicle.rs:240-263)
  - **Remaining Blocker:** `geo = "0.27"` crate dependency
  - Add `geo = "0.27"` to Cargo.toml
  - Build route polygon from planned waypoints
  - Query bounding boxes for all nearby vehicles using new `Vehicle::bounding_box()` API
  - Check polygon intersection for precise obstacle detection
  - Tests: Accurate collision detection with various vehicle configurations
  - Time: 1-2 days (API available, just need geo crate + integration)

**Success Criteria:**
- Traffic light detection only triggers for lights affecting current lane
- Bounding box intersection provides accurate collision prediction
- No false positives from adjacent lanes

#### Phase A.4.4: BehaviorAgent ✅ COMPLETE

**Status:** ✅ Complete - Advanced agent with behavior profiles
**Estimated Effort:** 1.5 weeks
**Actual Time:** Completed in session

**Work Items:**

- [x] **A.4.4.1: BehaviorAgent Structure and Behavior Types**
  - File: `carla/src/agents/navigation/behavior_agent.rs` (599 lines)
  - Defined `BehaviorType` enum: `Cautious(BehaviorParams)`, `Normal(BehaviorParams)`, `Aggressive(BehaviorParams)`
  - Defined `BehaviorParams` struct with behavior-specific parameters
  - Implemented presets: cautious (40 km/h, safety 3s), normal (50 km/h, safety 2s), aggressive (70 km/h, safety 1s)
  - Defined `BehaviorAgent { core: AgentCore, behavior: BehaviorType, ... }`
  - Constructor: `BehaviorAgent::new(vehicle, config, map_inst, grp_inst)`
  - Time: Completed

- [x] **A.4.4.2: BehaviorAgent Advanced Detection**
  - Implemented `update_information(&mut self)` - Updates speed, speed limit, direction, look-ahead
  - Implemented `traffic_light_manager(&mut self)` - Wrapper around core detection
  - Implemented `collision_and_car_avoid_manager(&mut self)` - Vehicle detection with TTC
  - Implemented `pedestrian_avoid_manager(&mut self)` - Walker detection
  - Implemented `tailgating(&mut self, waypoint, vehicle_list)` - Lane change opportunity detection
  - Time: Completed

- [x] **A.4.4.3: BehaviorAgent Control Logic**
  - Implemented `run_step_with_debug(&mut self, debug: bool)` → `Result<VehicleControl>`
  - 4-tier decision control flow:
    1. Red lights → emergency stop
    2. Pedestrians → emergency stop or adjust speed
    3. Vehicles → car following manager (TTC-based adaptive cruise)
    4. Normal → speed limit compliance
  - Implemented `car_following_manager(vehicle, distance, debug)` - TTC calculation and adaptive speed
  - Implemented `emergency_stop()` - Creates emergency stop control
  - Time: Completed

**Success Criteria:**
- ✅ `BehaviorAgent` exhibits measurable differences between Cautious/Normal/Aggressive modes
- ✅ Time-to-collision calculations for adaptive cruise control
- ✅ Pedestrian avoidance implemented
- ✅ Build and linter pass successfully
- ✅ Example demonstrates all three behavior profiles

#### Phase A.4.5: ConstantVelocityAgent ✅ COMPLETE

**Status:** ✅ Complete - Testing utility agent
**Estimated Effort:** 2-3 days
**Actual Time:** Completed in session

**Work Items:**

- [x] **A.4.5.1: ConstantVelocityAgent Implementation**
  - File: `carla/src/agents/navigation/constant_velocity_agent.rs` (354 lines)
  - Defined `ConstantVelocityAgent { core: AgentCore, use_basic_behavior, target_speed, ... }`
  - Implemented two modes: constant velocity mode and optional basic behavior mode
  - Implemented `run_step_with_debug()` - Simplified control with constant velocity override
  - Implemented `set_constant_velocity(speed)` - Set target speed in m/s
  - Implemented `ignore_vehicles()` - Toggle vehicle detection
  - Optional basic behavior mode uses hazard detection from core
  - Time: Completed

**Success Criteria:**
- ✅ `ConstantVelocityAgent` maintains constant speed (10 m/s default)
- ✅ Speed can be changed dynamically
- ✅ Optional basic behavior mode with hazard detection
- ✅ Build and linter pass successfully
- ✅ Example demonstrates constant velocity and speed changes

#### Phase A.4.6: Agent Trait ✅ COMPLETE

**Status:** ✅ Complete - Polymorphic agent interface
**Estimated Effort:** 1 day
**Actual Time:** Completed in session

**Work Items:**

- [x] **A.4.6.1: Agent Trait Implementation**
  - File: `carla/src/agents/navigation/types.rs`
  - Defined `pub trait Agent` with common interface:
    - `run_step()` → Result<VehicleControl>
    - `done()` → bool
    - `set_destination(end_location, start_location, clean_queue)` → Result<()>
    - `set_target_speed(speed)` → ()
    - `set_global_plan(plan, stop_waypoint_creation, clean_queue)` → ()
    - `trace_route(start_waypoint, end_waypoint)` → Result<Vec<(Waypoint, RoadOption)>>
    - `ignore_traffic_lights(active)` → ()
    - `ignore_stop_signs(active)` → ()
    - `ignore_vehicles(active)` → ()
  - Implemented `Agent` for `BasicAgent`, `BehaviorAgent`, `ConstantVelocityAgent`
  - Time: Completed

**Success Criteria:**
- ✅ Trait allows polymorphic agent handling
- ✅ All three agent types implement the trait
- ✅ Common interface for agent operations
- ✅ Build and linter pass successfully

## Missing Rust APIs

### Already Implemented ✅

**Navigation APIs:**
- `Map::topology()` ✅ (Road connectivity graph)
- `Map::waypoint(location)` ✅ (Location to waypoint mapping)
- `Waypoint::next()`, `previous()`, `left()`, `right()` ✅ (Navigation)
- `Map::recommended_spawn_points()` ✅ (Spawn locations)

**Control APIs:**
- `Vehicle::apply_control()` ✅ (Control application)
- `Actor::velocity()`, `acceleration()` ✅ (Physics queries)
- `Actor::transform()` ✅ (Position and rotation)

**Detection APIs:**
- `TrafficLight::state()` ✅ (Traffic light state)
- `World::actors()` ✅ (Actor queries)
- `ActorList::filter()` ✅ (Actor filtering)

**Utility APIs:**
- `World::debug()` ✅ (Debug visualization)
- `World::tick()` ✅ (Simulation step)

### APIs Blocking Phase A.4.3 (Enhanced Detection)

These APIs are required for enhanced detection features but **not required for BasicAgent** functionality:

#### 1. Traffic Light Trigger Volumes (UNBLOCKED ✅)

**API Status:** ✅ IMPLEMENTED as `TrafficLight::affected_lane_waypoints() → Vec<Waypoint>`

- **File:** `carla/src/client/traffic_light.rs`
- **Purpose:** Get waypoints controlled by this traffic light (trigger volume)
- **Python:** `traffic_light.get_affected_lane_waypoints()`
- **Implementation Status:** Available and working
- **Usage:** Can now implement precise traffic light detection in Phase A.4.3.1
- **Current Workaround Still Used:** Simple distance check (can be replaced)
- **Unblocks:** Phase A.4.3.1

#### 2. Vehicle Bounding Boxes (PARTIALLY UNBLOCKED ✅)

**API Status:** ✅ IMPLEMENTED as `Vehicle::bounding_box() → BoundingBox<f32>`

- **File:** `carla/src/client/vehicle.rs` (lines 240-263)
- **Purpose:** Get vehicle bounding box in world coordinates for precise collision detection
- **Python:** `vehicle.bounding_box` (local) + `vehicle.get_transform()` → world space
- **Implementation Status:** Available and working
- **FFI Implementation:** ✅ Complete
  - Bound to `carla::client::Vehicle::GetBoundingBox()`
  - Returns `BoundingBox<f32>` with extent and transform in world coordinates
  - Tested in `carla/examples/vehicle_bounding_box.rs`
- **Remaining Dependency:** `geo = "0.27"` crate for polygon intersection (not yet added)
- **What Can Be Done Now:** Query vehicle bounding boxes, use extent data for distance checks
- **What Remains:** Add `geo` crate and implement polygon intersection logic
- **Partially Unblocks:** Phase A.4.3.2 (can start implementation, pending geo crate)

### APIs for Future Enhancements (Lower Priority)

These APIs would be useful for future phases but are not currently blocking:

#### 3. Waypoint Lane Information (for Lane Changes)

**Missing API:** `Waypoint::lane_id() → i32`

- **File:** `carla/src/client/waypoint.rs`
- **Purpose:** Get lane identifier for lane change logic
- **Python:** `waypoint.lane_id`
- **Why Needed:** Validate lane change paths and detect lane changes
- **FFI Implementation:** Property binding to waypoint
- **Effort:** 1 day
- **Useful for:** Phase A.4.2 (Lane Change Support)

#### 4. Landmark Detection (for Stop Signs, etc.)

**Missing API:** `Waypoint::get_landmarks(distance: f64) → Vec<Landmark>`

- **File:** `carla/src/client/waypoint.rs`
- **Purpose:** Get nearby landmarks (stop signs, yield signs, etc.)
- **Python:** `waypoint.get_landmarks(distance)`
- **Why Needed:** Detect and respond to stop signs and other road signs
- **FFI Implementation:** Vector return with landmark type handling
- **Effort:** 2-3 days
- **Useful for:** BehaviorAgent stop sign compliance

### External Dependencies

#### Polygon Intersection Library (for Phase A.4.3.2)

**Crate:** `geo = "0.27"` (recommended)

- **Purpose:** Bounding box intersection for collision detection
- **Python Equivalent:** `shapely.geometry.Polygon`
- **Usage:**
  - Create polygons from vehicle bounding box corners
  - Create route polygon from waypoint sequence
  - Check intersection for precise collision detection
- **Integration Effort:** 1 day (add dependency, write integration code)
- **Blocks:** Phase A.4.3.2 (along with bounding_box() API)

## Verification Strategy

### Unit Tests

**Location:** Within each module using `#[cfg(test)]`

**Test Coverage:**
- PID controller response curves (step input, ramp input)
- Utility function accuracy (distance, angle calculations)
- Graph construction correctness (node count, edge connectivity)
- Turn decision logic (known waypoint pairs → expected RoadOption)
- Waypoint queue operations (push, pop, distance tracking)

### Integration Tests

**Location:** `carla/examples/` (examples serve as integration tests)

**Test Cases:**

1. **Minimal Agent Test** (Created)
   - File: `carla/examples/basic_agent_minimal.rs`
   - Reference: `carla/examples/python/basic_agent_minimal.py`
   - Spawn vehicle, set destination, verify arrival
   - Success: Agent reaches destination within distance threshold
   - Time limit: 1000 steps (detect failures)

2. **Route Following Test**
   - Verify agent follows GlobalRoutePlanner route
   - Check waypoint accuracy (lateral error < 1m)
   - Measure destination error (< 2m)

3. **Hazard Detection Test**
   - Spawn obstacle vehicle ahead
   - Verify agent stops before collision
   - Spawn traffic light at red
   - Verify agent stops at light

4. **Lane Change Test**
   - Trigger lane change maneuver
   - Verify smooth trajectory (lateral acceleration limits)
   - Check completion (vehicle in target lane)

### Comparison Testing

**Method:** Run identical scenarios in Python and Rust, compare outputs

**Scenarios:**
1. **Simple A→B Navigation**
   - Same start/end points in both versions
   - Compare route waypoint count and positions
   - Compare control outputs at matching simulation times

2. **Traffic Light Scenario**
   - Fixed traffic light state sequence
   - Compare stopping distances
   - Compare resume behavior

3. **Performance Benchmark**
   - Route computation time (GlobalRoutePlanner)
   - Control loop time (LocalPlanner + PID)
   - Target: < 5ms per step for real-time performance

### Manual Testing

**Tools:**
- automatic_control example with visualization
- Debug drawing of waypoints, routes, hazards
- HUD showing agent state

**Test Scenarios:**
- Navigate between distant spawn points
- Complex routes with multiple turns
- Dense traffic scenarios
- Various weather conditions

## Timeline and Estimates

### Optimistic: 4 Weeks

**Week 1:** Phase A.1 (PID + utilities)
**Week 2:** Phase A.2 (LocalPlanner)
**Week 3:** Phase A.3 (GlobalRoutePlanner)
**Week 4:** Phase A.4 (BasicAgent + integration tests)

**Assumptions:**
- No major FFI issues
- PID tuning works on first try
- Graph algorithms translate cleanly
- All missing APIs straightforward

### Realistic: 5-6 Weeks

**Week 1-1.5:** Phase A.1 (PID + utilities + some missing APIs)
**Week 2-3:** Phase A.2 (LocalPlanner + debugging)
**Week 3-4.5:** Phase A.3 (GlobalRoutePlanner + graph algorithm adaptation)
**Week 4.5-6:** Phase A.4 (BasicAgent + integration tests + BehaviorAgent)

**Assumptions:**
- Expected FFI challenges (1-2 days debugging)
- PID tuning requires iteration (2-3 iterations)
- Graph algorithm needs some adaptation
- Comprehensive testing included

### Conservative: 6-8 Weeks

**Week 1-2:** Missing APIs + Phase A.1
**Week 2-3.5:** Phase A.2 with extensive testing
**Week 3.5-5:** Phase A.3 with performance optimization
**Week 5-7:** Phase A.4 with BehaviorAgent full implementation
**Week 7-8:** Integration testing, bug fixes, documentation

**Assumptions:**
- Significant FFI issues (traffic light waypoints complex)
- PID tuning challenging (5+ iterations)
- Graph algorithm performance issues require optimization
- Bounding box intersection library integration problems
- Comprehensive testing and documentation

### Critical Path

```
Week 1-2: Missing APIs (parallel) + PID Controllers (sequential)
    ↓
Week 2-3: LocalPlanner (depends on PID)
    ↓
Week 3-4: GlobalRoutePlanner (parallel with LocalPlanner Week 3)
    ↓
Week 4-6: BasicAgent (depends on Local + Global)
    ↓
Week 5-6: Integration Testing (depends on BasicAgent)
```

### Risk Factors

**High Risk:**
- **Graph algorithms:** networkx → petgraph translation may need custom adaptations
- **FFI complexity:** Traffic light waypoints API may be challenging to bind

**Medium Risk:**
- **PID tuning:** May require multiple iterations to match Python smoothness
- **Bounding box intersection:** Geometry library integration (mitigate with well-tested `geo` crate)

**Low Risk:**
- **Utility functions:** Straightforward implementations
- **RoadOption enum:** Simple Rust enum

### Mitigation Strategies

1. **Graph Algorithms:** Start with simple test maps (Town03), validate algorithm before optimization
2. **FFI Issues:** Use FfiWaypoint wrapper pattern (proven in existing code)
3. **PID Tuning:** Start with Python default values, tune incrementally with logging
4. **Performance:** Profile early, optimize hot paths (route computation, control loop)

## Dependencies and Blockers

### Blocks

- **Phase 13.1: Automatic Control GUI Example** (HIGH priority)
  - Requires BasicAgent at minimum
  - Can proceed without BehaviorAgent initially
  - Timeline: Agent Phase must complete before Phase 13.1 starts

### Unblocks

- Phase 13.1: Automatic Control (agent-based navigation GUI)
- Advanced traffic scenarios (multi-agent coordination)
- Autonomous testing frameworks (automated driving tests)
- Custom agent behaviors (research and development)

### External Crate Dependencies

```toml
[dependencies]
petgraph = "0.6"   # Graph algorithms (networkx equivalent)
geo = "0.27"       # Polygon intersection (shapely equivalent)
# nalgebra already included in carla crate for vector math
```

---

## Implementation Notes

### Completed Work

**Date:** January 2025
**Implemented By:** Claude Code assisted implementation

**What Was Implemented:**

1. **Phase A.1 - Core Infrastructure** ✅ Complete
   - All PID controllers (longitudinal, lateral, combined)
   - All utility functions (get_speed, compute_distance, etc.)
   - RoadOption enum with full trait support
   - Comprehensive unit tests

2. **Phase A.2 - LocalPlanner** ✅ Complete
   - Waypoint queue management with VecDeque
   - PID controller integration
   - Speed management and configuration
   - Distance-based waypoint advancement
   - Query methods for plan inspection

3. **Phase A.3 - GlobalRoutePlanner** ✅ Complete (Full A* Implementation)
   - Graph-based A* pathfinding with petgraph
   - Road topology graph construction from Map::topology()
   - Lane change edges with configurable costs
   - Turn decision logic using vector math
   - Optimal route computation between locations

4. **Phase A.4 - Agent Implementations** ✅ Complete (All Agents)
   - AgentCore with composition pattern and lane change generation
   - BasicAgent with full navigation and lane change capability
   - BehaviorAgent with three personality profiles (Cautious, Normal, Aggressive)
   - ConstantVelocityAgent for testing scenarios
   - Agent trait for polymorphic agent handling
   - Sophisticated hazard detection (traffic lights, vehicles, pedestrians)
   - Time-to-collision (TTC) based adaptive cruise control
   - Working examples: basic_agent_demo.rs, behavior_agent_demo.rs, constant_velocity_agent_demo.rs

**Design Decisions:**

1. **Composition over Inheritance:** Rust implementation uses AgentCore struct contained by each agent type rather than inheritance hierarchy
2. **Full A* Implementation:** Graph-based pathfinding with petgraph for optimal routes
3. **Behavior Profiles:** Three distinct agent personalities with measurable parameter differences
4. **Type Conversions:** Consistent use of `Location::from_na_translation()` for nalgebra::Isometry3 conversion
5. **Agent Trait:** Common interface for polymorphic agent handling

**Known Limitations:**

1. Traffic light detection doesn't use trigger volumes (uses distance-based detection)
2. Vehicle detection doesn't use bounding box intersection (uses distance + dot product)
3. Pedestrian detection simplified (no full bounding box analysis)

**Future Work:**

1. Add `geo = "0.27"` crate for precise bounding box intersection detection
2. Implement traffic light trigger volume waypoint matching (API exists: `TrafficLight::affected_lane_waypoints()`)
3. Enhanced collision prediction using vehicle bounding boxes (API exists: `Vehicle::bounding_box()`)
4. Stop sign detection using landmarks API (when available)

---

**Status:** All agent types complete and ready for use ✅
**Next Steps:** Phase 13.1 (Automatic Control GUI) can now proceed with full agent support
**Available Agents:** BasicAgent, BehaviorAgent (Cautious/Normal/Aggressive), ConstantVelocityAgent
**Future Enhancements:** See "Future Work" section above for optional improvements
