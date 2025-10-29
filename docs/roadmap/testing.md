# Cross-Phase Testing

**[← Back to Roadmap Index](../roadmap.md)**

This document covers the cross-phase testing strategy for the carla-rust project. This includes test infrastructure, methodologies, and best practices that span across all development phases.

---

## Cross-Phase Testing

### Integration Test Scenarios

These tests verify interactions between multiple phases:

1. **Complete Autonomous Driving Scenario**
   - Spawn vehicle with sensors
   - Enable Traffic Manager
   - Navigate to destination
   - Handle traffic lights and pedestrians
   - Record entire session
   - Replay and verify behavior

2. **Multi-Agent Simulation**
   - Spawn 50 vehicles and 100 pedestrians
   - Control subset with AI
   - Monitor collisions and blocked actors
   - Use debug drawing for visualization
   - Batch operations for efficiency

3. **Sensor Data Collection Pipeline**
   - Spawn vehicle with RGB, depth, semantic, lidar
   - Navigate using autopilot
   - Collect and save sensor data
   - Verify data consistency across sensors
   - Test different weather and lighting conditions

4. **Dynamic World Manipulation**
   - Load map with layers
   - Spawn environment objects
   - Enable/disable layers at runtime
   - Freeze and manipulate traffic lights
   - Record and replay with different configurations

---

## Testing Infrastructure

### Test Organization

```
carla/
  tests/
    unit/              # Unit tests for individual components
      rpc/             # RPC type tests
      client/          # Client API tests
      sensor/          # Sensor data tests
    integration/       # Integration tests requiring simulator
      vehicle/         # Vehicle-related integration tests
      sensor/          # Sensor integration tests
      walker/          # Pedestrian integration tests
      world/           # World operations tests
      batch/           # Batch operations tests
    performance/       # Performance benchmarks
      spawn.rs
      sensor_throughput.rs
      batch_operations.rs
```

### Test Requirements

1. **Unit Tests**
   - No simulator required
   - Fast execution (< 1 second per test)
   - Test data structures, serialization, and pure logic

2. **Integration Tests**
   - Require running CARLA simulator
   - Use `#[ignore]` attribute for CI without simulator
   - Run with `cargo test -- --ignored` when simulator available
   - Test actual simulation behavior

3. **Performance Tests**
   - Benchmarks using criterion
   - Measure latency, throughput, memory
   - Track regressions over time

### Continuous Integration

- Unit tests run on every commit
- Integration tests run nightly with simulator
- Performance tests run weekly, track trends
- Test coverage target: >80% for unit tests

---

## Version Compatibility Strategy

All new features must consider version compatibility:

1. **Version-Specific Features**
   - Use `#[cfg(carla_0914)]`, `#[cfg(carla_0915)]`, `#[cfg(carla_0916)]` attributes
   - Document version requirements in API docs
   - Test across all supported versions (0.9.14, 0.9.15, 0.9.16)

2. **Deprecation Policy**
   - Features removed in newer CARLA versions get deprecation warnings
   - Maintain backward compatibility for at least one major version
   - Clear migration guides in documentation

3. **Testing Matrix**
   - Test each phase against all supported CARLA versions
   - Version-specific tests use appropriate `#[cfg]` attributes
   - CI runs tests for each version separately

---

## Success Criteria

Each phase is considered complete when:

1. All work items implemented and merged
2. Unit tests achieve >90% code coverage
3. Integration tests pass consistently
4. Documentation complete with examples
5. Code reviewed and approved
6. Tested across all supported CARLA versions

---

## References

- [CARLA Python API Documentation](https://carla.readthedocs.io/en/latest/python_api/)
- [CARLA Python Examples](https://github.com/carla-simulator/carla/tree/master/PythonAPI/examples)
- [CARLA C++ Client LibCarla Source](https://github.com/carla-simulator/carla/tree/master/LibCarla/source/carla/client)
- [CARLA CHANGELOG](https://github.com/carla-simulator/carla/blob/master/CHANGELOG.md)
- [carla-rust API Differences](./carla_api_differences.md)
- [Example Implementation Plan](./example_implementation_plan.md)
