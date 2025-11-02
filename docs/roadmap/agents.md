# Navigation Agents Implementation

**Priority:** HIGH (Blocks Phase 13.1: Automatic Control GUI Example)
**Estimated Effort:** 4-6 weeks
**Status:** Not Started
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

### Phase A.1: Core Infrastructure and PID Controllers

**Estimated Effort:** 1-1.5 weeks

**Work Items:**

- [ ] **A.1.1: Utility Functions Module**
  - File: `carla/src/agents/tools/misc.rs`
  - Implement 8 utility functions from misc.py
  - Functions: `get_speed`, `is_within_distance`, `compute_distance`, `get_trafficlight_trigger_location`, `compute_magnitude_angle`, `distance_vehicle`, `vector`, `draw_waypoints`
  - Tests: Unit tests for each function with known inputs/outputs
  - Time: 2-3 days

- [ ] **A.1.2: PID Controller Base**
  - File: `carla/src/agents/navigation/pid.rs`
  - Implement `PIDLongitudinalController` (throttle/brake control)
  - Implement `PIDLateralController` (steering control)
  - PID formula: `output = K_P * error + K_I * integral + K_D * derivative`
  - Tests: PID response curves with known inputs
  - Time: 2-3 days

- [ ] **A.1.3: VehiclePIDController**
  - File: `carla/src/agents/navigation/controller.rs`
  - Combine lateral + longitudinal controllers
  - Public methods: `run_step`, `change_longitudinal_pid`, `change_lateral_pid`, `set_offset`
  - Implement smooth steering changes (max ±0.1 per frame)
  - Tests: Integration tests with vehicle control outputs
  - Time: 2-3 days

- [ ] **A.1.4: RoadOption Enum**
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

### Phase A.2: LocalPlanner Implementation

**Estimated Effort:** 1.5-2 weeks

**Work Items:**

- [ ] **A.2.1: Waypoint Queue Management**
  - Data structure: `VecDeque<(Waypoint, RoadOption)>`
  - Waypoint sampling at `sampling_radius` intervals
  - Distance tracking and queue advancement
  - Target point computation from queue front
  - Tests: Queue operations, waypoint advancement logic
  - Time: 2-3 days

- [ ] **A.2.2: LocalPlanner Core**
  - File: `carla/src/agents/navigation/local_planner.rs`
  - Constructor with `opt_dict` configuration
  - Implement `set_global_plan`, `set_speed`, `set_offset`, `follow_speed_limits`
  - Internal state management
  - Tests: Plan loading and parameter changes
  - Time: 3-4 days

- [ ] **A.2.3: LocalPlanner Control Loop**
  - Implement `run_step()` - main planning cycle
  - PID controller integration
  - Speed limit following logic (query from waypoint)
  - Waypoint advancement when within sampling radius
  - Tests: Control output validation
  - Time: 3-4 days

- [ ] **A.2.4: LocalPlanner Query Methods**
  - Implement `get_incoming_waypoint_and_direction(steps)` - Look ahead
  - Implement `get_plan()` - Return current queue
  - Implement `done()` - Check if queue empty
  - Implement `reset_vehicle()` - Clear vehicle reference
  - Tests: Query methods return correct data
  - Time: 1-2 days

**Success Criteria:**
- LocalPlanner follows waypoint sequences smoothly
- Speed limits respected when enabled
- Lateral offset applied correctly
- Control outputs smooth and stable
- Done detection works correctly

### Phase A.3: GlobalRoutePlanner Implementation

**Estimated Effort:** 1.5-2 weeks

**Dependencies:**
- Add `petgraph = "0.6"` to Cargo.toml (Rust graph library, networkx equivalent)

**Work Items:**

- [ ] **A.3.1: Graph Data Structures**
  - File: `carla/src/agents/navigation/global_route_planner.rs`
  - Define graph node type (road segment + lane info)
  - Define edge type (distance, waypoint list, lane change flag)
  - Tests: Basic graph construction
  - Time: 2 days

- [ ] **A.3.2: Graph Construction from Topology**
  - Implement `_build_topology()` - Process Map::topology() into segments
  - Implement `_build_graph()` - Convert segments to petgraph DiGraph
  - Sample waypoints along each segment at `sampling_resolution` intervals
  - Tests: Complete graph for test map (Town01 or Town03)
  - Time: 3 days

- [ ] **A.3.3: Lane Change Links**
  - Implement `_lane_change_link()` - Add zero-cost lane edges
  - Detect left/right lane availability using `waypoint.left()`, `waypoint.right()`
  - Connect parallel lanes at matching locations
  - Tests: Lane change edges added correctly
  - Time: 2 days

- [ ] **A.3.4: Pathfinding**
  - Implement `_path_search()` using `petgraph::algo::astar`
  - Implement `_distance_heuristic()` - Euclidean distance to goal
  - Implement `_localize(location)` - Map location to nearest graph node
  - Tests: A* finds shortest path between known points
  - Time: 3 days

- [ ] **A.3.5: Turn Decision Logic**
  - Implement `_turn_decision(waypoint, next_waypoint)` - Classify as STRAIGHT/LEFT/RIGHT
  - Use vector cross product to determine turn direction
  - Threshold angle for STRAIGHT vs LEFT/RIGHT classification
  - Tests: Turn decisions match expected for known routes
  - Time: 2 days

- [ ] **A.3.6: Public API**
  - Implement `trace_route(origin: Location, destination: Location)` → `Vec<(Waypoint, RoadOption)>`
  - Localize origin and destination to graph nodes
  - Run A* pathfinding
  - Annotate path with RoadOption
  - Tests: End-to-end route computation
  - Time: 1 day

**Success Criteria:**
- Graph built from Map topology with correct connectivity
- A* pathfinding produces valid routes
- RoadOption annotations correct (LEFT/RIGHT/STRAIGHT at turns)
- Lane change options included
- Performance acceptable (< 100ms for typical routes)

### Phase A.4: Agent Implementations

**Estimated Effort:** 1.5-2 weeks

**Dependencies:**
- Add `geo = "0.27"` to Cargo.toml (Polygon intersection for bounding boxes)

**Work Items:**

- [ ] **A.4.1: AgentCore Shared Structure**
  - File: `carla/src/agents/navigation/agent_core.rs`
  - Define `AgentCore` struct with vehicle, world, map, planners, flags, config
  - Constructor: `AgentCore::new(vehicle, map_inst, grp_inst, opt_dict)`
  - Configuration methods: `ignore_traffic_lights()`, `ignore_vehicles()`, `set_offset()`
  - Accessor methods: `local_planner()`, `global_planner()`, `vehicle()`, `world()`, `map()`
  - Tests: Core instantiation and configuration
  - Time: 1 day

- [ ] **A.4.2: AgentCore Traffic Light Detection**
  - Implement `affected_by_traffic_light(&mut self, lights_list, max_distance)` → `TrafficLightDetectionResult`
  - Logic: Check red lights ahead using trigger volume and waypoint matching
  - Cache traffic light trigger waypoints in `lights_map`
  - Track `last_traffic_light` for efficiency
  - Tests: Traffic light detection in various scenarios
  - Time: 2-3 days

- [ ] **A.4.3: AgentCore Vehicle Obstacle Detection**
  - Implement `vehicle_obstacle_detected(&self, vehicle_list, max_distance, up_angle_th, low_angle_th, lane_offset)` → `ObstacleDetectionResult`
  - Two detection modes:
    1. Bounding box intersection (use `geo` crate polygon intersection)
    2. Simplified waypoint-based detection
  - Build route polygon from planned waypoints
  - Check intersection with target vehicle bounding boxes
  - Tests: Obstacle detection with various vehicle configurations
  - Time: 3-4 days

- [ ] **A.4.4: AgentCore Lane Change Path Generation**
  - Implement static method `generate_lane_change_path(waypoint, direction, distances, ...)`
  - Generate waypoint sequence: same lane → lane change → other lane
  - Validate lane change is possible (lane availability, lane markings)
  - Return `Vec<(Waypoint, RoadOption)>` or empty if impossible
  - Tests: Lane change path generation for left/right changes
  - Time: 2 days

- [ ] **A.4.5: BasicAgent Structure and Navigation**
  - File: `carla/src/agents/navigation/basic_agent.rs`
  - Define `BasicAgent { core: AgentCore, target_speed, sampling_resolution }`
  - Constructor: `BasicAgent::new(vehicle, target_speed, opt_dict, map_inst, grp_inst)`
  - Navigation methods:
    - `set_destination(end_location, start_location, clean_queue)`
    - `set_global_plan(plan, stop_waypoint_creation, clean_queue)`
    - `trace_route(start_waypoint, end_waypoint)`
    - `done()` - delegates to `core.local_planner.done()`
  - Tests: Agent instantiation and route planning
  - Time: 2 days

- [ ] **A.4.6: BasicAgent Control Loop**
  - Implement `run_step(&mut self)` → `Result<VehicleControl>`
  - Control flow:
    1. Get vehicle list and speed
    2. Check vehicle obstacles (via `core.vehicle_obstacle_detected()`)
    3. Check traffic lights (via `core.affected_by_traffic_light()`)
    4. Get control from `core.local_planner.run_step()`
    5. Apply emergency stop if hazard detected
  - Implement `add_emergency_stop(control)` - set throttle=0, brake=max_brake
  - Configuration methods: `set_target_speed()`, `follow_speed_limits()`, `ignore_traffic_lights()`, `ignore_vehicles()`, `set_offset()`
  - Tests: Control output in various scenarios (clear road, obstacles, red lights)
  - Time: 2-3 days

- [ ] **A.4.7: BasicAgent Lane Change**
  - Implement `lane_change(direction, same_lane_time, other_lane_time, lane_change_time)`
  - Call `AgentCore::generate_lane_change_path()` to generate path
  - Apply path via `set_global_plan()`
  - Tests: Lane change execution
  - Time: 1 day

- [ ] **A.4.8: BehaviorAgent Structure and Behavior Types**
  - File: `carla/src/agents/navigation/behavior_agent.rs`
  - Define `BehaviorType` enum: `Cautious(BehaviorParams)`, `Normal(BehaviorParams)`, `Aggressive(BehaviorParams)`
  - Define `BehaviorParams` struct with behavior-specific parameters
  - Implement `Default` for each behavior preset
  - Define `BehaviorAgent { core: AgentCore, behavior: BehaviorType, ... }`
  - Constructor: `BehaviorAgent::new(vehicle, behavior, opt_dict, map_inst, grp_inst)`
  - Tests: Behavior type instantiation
  - Time: 1-2 days

- [ ] **A.4.9: BehaviorAgent Advanced Detection**
  - Implement `update_information(&mut self)` - Update speed, speed limit, direction, look-ahead
  - Implement `traffic_light_manager(&mut self)` - Wrapper around core detection
  - Implement `collision_and_car_avoid_manager(&mut self)` - Vehicle detection with lane offset handling
  - Implement `pedestrian_avoid_manager(&mut self)` - Walker detection
  - Implement `tailgating(&mut self, waypoint, vehicle_list)` - Lane change opportunity detection
  - Tests: Detection methods with various scenarios
  - Time: 3 days

- [ ] **A.4.10: BehaviorAgent Control Logic**
  - Implement `run_step(&mut self, debug: bool)` → `Result<VehicleControl>`
  - Control flow (4-tier decision):
    1. Red lights → emergency stop
    2. Pedestrians → emergency stop or adjust speed
    3. Vehicles → car following manager (TTC-based)
    4. Normal → speed limit compliance
  - Implement `car_following_manager(vehicle, distance, debug)` - TTC calculation and adaptive speed
  - Implement `emergency_stop()` - Create emergency stop control
  - Tests: Behavior agent control in complex scenarios
  - Time: 3-4 days

- [ ] **A.4.11: ConstantVelocityAgent**
  - File: `carla/src/agents/navigation/constant_velocity_agent.rs`
  - Define `ConstantVelocityAgent { core: AgentCore, use_basic_behavior, target_speed, ... }`
  - Constructor sets up collision sensor listener
  - Implement `run_step()` - Simplified control with constant velocity override
  - Implement `set_constant_velocity(speed)` - Use vehicle's constant velocity feature
  - Implement `set_collision_sensor()` - Attach collision sensor with stop callback
  - Implement `destroy_sensor()` - Cleanup method
  - Tests: Constant velocity maintenance and collision handling
  - Time: 2 days

- [ ] **A.4.12: Optional Agent Trait (If Needed)**
  - File: `carla/src/agents/navigation/mod.rs`
  - Define `pub trait Agent` with common interface: `run_step()`, `done()`, `set_destination()`, `set_target_speed()`
  - Implement `Agent` for `BasicAgent`, `BehaviorAgent`, `ConstantVelocityAgent`
  - **Decision:** Only implement if Phase 13.1 requires polymorphic agent handling
  - Tests: Trait object usage
  - Time: 1 day (optional)

**Success Criteria:**
- `AgentCore` provides reusable hazard detection for all agents
- `BasicAgent` navigates to destination autonomously
- Traffic light detection works (stops at red lights)
- Vehicle obstacle detection prevents collisions
- Lane change maneuvers execute smoothly
- `BehaviorAgent` exhibits measurable differences between Cautious/Normal/Aggressive modes
- `ConstantVelocityAgent` maintains constant speed and handles collisions
- Agent behavior matches Python agent behavior in test scenarios
- Code reuse via composition (all agents use same `AgentCore`)

## Missing Rust APIs

### Already Implemented ✅

- `Map::topology()` ✅ (Road connectivity graph)
- `Waypoint::next()`, `previous()`, `left()`, `right()` ✅ (Navigation)
- `Vehicle::apply_control()` ✅ (Control application)
- `Actor::velocity()`, `acceleration()` ✅ (Physics queries)
- `TrafficLight::state()` ✅ (Traffic light state)
- `World::debug()` ✅ (Debug visualization)

### Need to Implement

#### Critical (Required for Agents)

1. **Map::get_waypoint(location: &Location) → Option<Waypoint>**
   - **File:** `carla/src/client/map.rs`
   - **Purpose:** Map arbitrary location to nearest road waypoint
   - **Python:** `map.get_waypoint(location)`
   - **Implementation:** FFI binding to `carla::client::Map::GetWaypoint(location)`
   - **Effort:** 2-3 days (FFI + autocxx + Rust wrapper)

2. **Waypoint::lane_id() → i32**
   - **File:** `carla/src/client/waypoint.rs`
   - **Purpose:** Get lane identifier for lane change logic
   - **Python:** `waypoint.lane_id`
   - **Implementation:** FFI binding to waypoint property
   - **Effort:** 1 day

3. **Vehicle::bounding_box() → BoundingBox**
   - **File:** `carla/src/client/vehicle.rs`
   - **Purpose:** Get vehicle bounding box in world coordinates
   - **Python:** `vehicle.bounding_box` (local) + transform to world
   - **Implementation:** Query bounding box and transform to world space
   - **Effort:** 2-3 days (geometry transform logic)

4. **TrafficLight::get_affected_lane_waypoints() → Vec<Waypoint>**
   - **File:** `carla/src/client/traffic_light.rs`
   - **Purpose:** Get waypoints controlled by this traffic light
   - **Python:** `traffic_light.get_affected_lane_waypoints()`
   - **Implementation:** FFI binding returning vector of waypoints
   - **Effort:** 3-4 days (FFI complexity with vector returns)

5. **Waypoint::get_landmarks(distance: f64) → Vec<Landmark>**
   - **File:** `carla/src/client/waypoint.rs`
   - **Purpose:** Get nearby landmarks (traffic lights, stop signs, etc.)
   - **Python:** `waypoint.get_landmarks(distance)`
   - **Implementation:** FFI binding with landmark type handling
   - **Effort:** 2-3 days

#### External Dependencies

6. **Polygon Intersection Library**
   - **Crate:** `geo = "0.27"` (recommended) or `parry2d`
   - **Purpose:** Bounding box intersection for collision detection
   - **Python:** Uses `shapely.geometry.Polygon`
   - **Implementation:** Add dependency, create polygon from bounding box corners
   - **Effort:** 1 day (integration and testing)

#### Verification (May Exist)

7. **Map::get_spawn_points() → Vec<Transform>**
   - May already exist as `Map::recommended_spawn_points()` ✅
   - Verify availability and add alias if needed

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

**Status:** Not Started
**Next Steps:** Begin Phase A.1 implementation (PID controllers and utilities)
**Review Date:** After Phase A.1 completion (reassess timeline)
