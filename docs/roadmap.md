# CARLA Rust API Development Roadmap

This document serves as the index for the carla-rust development roadmap. For detailed phase information, see the linked documents below.

## Overview

The carla-rust library provides Rust bindings for the CARLA autonomous driving simulator. This roadmap tracks the development of APIs and example implementations to achieve feature parity with the C++ client library.

**Total Phases:** 15 (Phase 0 - Phase 14)
**Estimated Total Effort:** ~31-39 weeks
**Last Updated:** 2025-10-29

## Current Status

**Completed:** Phases 0, 3, 4, 8 (Section 1), 9 (HIGH priority), 10, 11
**Active Development:** Phase 12 (Manual Control), Phase 13 (GUI Examples with Macroquad)
**Total Examples Implemented:** 20 (6 test framework + 8 simple + 6 intermediate)

## Roadmap Documents

### Core API Development

**[Core APIs (Phases 0-9)](roadmap/core-apis.md)**

Foundation APIs for walker control, sensors, recording, batch operations, and utilities.

- Phase 0: Cargo Examples for Demonstration ✅
- Phase 1: Walker/Pedestrian Control
- Phase 2: Debug and Visualization Utilities
- Phase 3: Recording and Playback ✅
- Phase 4: Advanced Vehicle Features ✅
- Phase 5: Batch Operations and Commands
- Phase 6: Advanced Sensor Features
- Phase 7: Advanced World Operations
- Phase 8: Navigation and Path Planning ✅ (Section 1)
- Phase 9: Additional Utilities and Refinements ✅ (HIGH priority)

### Example Implementations

**[Simple Examples (Phase 10)](roadmap/examples-simple.md)** ✅

Basic CARLA operations demonstrated in straightforward examples.

- 8 examples: tutorial, vehicle gallery, weather, physics, recording/replay

**[Intermediate Examples (Phase 11)](roadmap/examples-intermediate.md)** ✅

Complex headless examples suitable for automated testing and CI/CD.

- 6 examples: automatic control, synchronous mode, sensor sync, traffic generation, LiDAR projection

**[GUI Examples (Phases 12-14)](roadmap/examples-gui.md)**

Interactive examples with Macroquad-based visualization and real-time controls.

- Phase 12: Manual Control - Complete Interactive Example (37 work items, 12 subphases)
- Phase 13: Advanced GUI Examples (bounding boxes, skeleton, multi-sensor visualization)
- Phase 14: Specialized Examples (reference/future)

### Testing Strategy

**[Cross-Phase Testing](roadmap/testing.md)**

Test infrastructure, methodologies, and best practices spanning all phases.

---

**Repository:** https://github.com/jerry73204/carla-rust
**Documentation:** https://docs.rs/carla/
