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

### BasicAgent

**File:** `PythonAPI/carla/agents/navigation/basic_agent.py` (510 lines)

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

| Method | Return Type | Purpose |
|--------|-------------|---------|
| `set_target_speed(speed)` | None | Set cruise speed (km/h) |
| `follow_speed_limits(value=True)` | None | Enable dynamic speed limits from map |
| `set_destination(end_location, start_location=None)` | None | Set navigation goal |
| `set_global_plan(plan, stop_waypoint_creation, clean_queue)` | None | Apply pre-computed route |
| `trace_route(start_waypoint, end_waypoint)` | List[Tuple] | Compute route between waypoints |
| `run_step()` | VehicleControl | Execute one control cycle |
| `done()` | bool | Check if destination reached |
| `ignore_traffic_lights(active)` | None | Toggle red light compliance |
| `ignore_stop_signs(active)` | None | Toggle stop sign compliance |
| `ignore_vehicles(active)` | None | Toggle vehicle obstacle detection |
| `set_offset(offset)` | None | Set lateral positioning offset |
| `lane_change(direction, same_lane_time, other_lane_time, lane_change_time)` | None | Execute lane change maneuver |
| `get_local_planner()` | LocalPlanner | Access LocalPlanner instance |
| `get_global_planner()` | GlobalRoutePlanner | Access GlobalRoutePlanner instance |
| `add_emergency_stop(control)` | VehicleControl | Apply maximum braking |

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

- [ ] **A.4.1: BasicAgent Structure**
  - File: `carla/src/agents/navigation/basic_agent.rs`
  - Constructor with `opt_dict`
  - Internal state: LocalPlanner, GlobalRoutePlanner, configuration flags
  - Tests: Agent instantiation with various configurations
  - Time: 1 day

- [ ] **A.4.2: BasicAgent Navigation**
  - Implement `set_destination(end_location, start_location)` - Compute and set route
  - Implement `set_global_plan(plan)` - Apply pre-computed route
  - Implement `trace_route(start, end)` - Route computation wrapper
  - Implement `done()` - Check if destination reached
  - Implement `get_local_planner()`, `get_global_planner()` - Accessor methods
  - Tests: Destination setting and route computation
  - Time: 2 days

- [ ] **A.4.3: BasicAgent Hazard Detection**
  - Implement `_affected_by_traffic_light()` - Check traffic light state ahead
  - Implement `_vehicle_obstacle_detected()` - Bounding box intersection check
  - Use `geo` crate for polygon intersection
  - Implement `ignore_traffic_lights(active)`, `ignore_vehicles(active)` - Toggle detection
  - Tests: Hazard detection in controlled scenarios
  - Time: 3-4 days

- [ ] **A.4.4: BasicAgent Control**
  - Implement `run_step()` - Main agent control loop
    1. Check hazards (traffic lights, vehicles)
    2. Compute target speed (speed limits, hazards, target speed)
    3. Get control from LocalPlanner
    4. Apply emergency stop if needed
  - Implement `add_emergency_stop(control)` - Maximum braking
  - Implement `set_target_speed(speed)`, `follow_speed_limits(value)`
  - Tests: Control output validation in various scenarios
  - Time: 2-3 days

- [ ] **A.4.5: BasicAgent Lane Change**
  - Implement `lane_change(direction, same_lane_time, other_lane_time, lane_change_time)`
  - Implement `_generate_lane_change_path()` - Create waypoint sequence for lane change
  - Tests: Lane change trajectory generation and execution
  - Time: 2 days

- [ ] **A.4.6: BehaviorAgent (Optional)**
  - File: `carla/src/agents/navigation/behavior_agent.rs`
  - Extends BasicAgent
  - Behavior profiles: `Cautious`, `Normal`, `Aggressive` (affect safety margins, speeds)
  - Additional methods: `car_following_manager`, `pedestrian_avoid_manager`, `tailgating`
  - Tests: Behavior profile differences measurable
  - **Note:** Can be deferred if time is limited; Phase 13.1 can use BasicAgent only
  - Time: 3-4 days

- [ ] **A.4.7: ConstantVelocityAgent**
  - File: `carla/src/agents/navigation/constant_velocity_agent.rs`
  - Extends BasicAgent
  - Override: Disable speed limit following and hazard detection
  - Simple agent for testing purposes
  - Tests: Constant velocity maintained
  - Time: 1 day

**Success Criteria:**
- BasicAgent navigates to destination autonomously
- Traffic light detection works (stops at red lights)
- Vehicle obstacle detection prevents collisions
- Lane change maneuvers execute smoothly
- Agent behavior matches Python BasicAgent in test scenarios

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
