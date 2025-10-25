# Changelog

## [Unreleased]

### Added
- Nextest test groups for sequential CARLA test execution
- `.config/nextest.toml` configuration for proper test isolation
- `test_verify_nextest_configuration` test to validate nextest setup
- Comprehensive documentation for nextest migration in `NEXTEST.md`
- Sensor callback foundation with true event-driven architecture
- `SensorCallbackManager` for managing sensor callbacks
- FFI-safe callback types and per-sensor storage design
- Defensive actor operations with atomic state tracking

### Changed
- **BREAKING**: Migrated from file-based locking to nextest test groups for CARLA test coordination
- **BREAKING**: Merged SafeActor functionality into base Actor implementation
- `make test` now runs only library and unit tests (no CARLA server required)
- Test infrastructure simplified by removing file locking complexity
- Updated documentation to reflect nextest requirements and usage
- Improved Makefile help text for clarity
- Actor now includes defensive methods (try_transform, try_velocity) for crash protection
- Sensor callbacks redesigned from polling to true event-driven architecture

### Removed
- File-based locking infrastructure (`coordination.rs` and related code)
- `FileLockCoordinator` and `FileLockGuard` structs
- `ServerResource` wrapper in test server management
- `libc` dependency from `carla-test-server` crate
- Concurrency tracking atomics from sequential execution tests
- Separate SafeActor module (functionality merged into base Actor)
- Global sensor data storage variables (replaced with per-sensor design)

### Fixed
- Test execution reliability by eliminating stale lock file issues
- Proper test isolation using nextest's native concurrency control
- Simplified error handling by removing lock acquisition failures
- Helper timeout function to use sensible defaults

### Technical Details

#### Migration Summary
- **Phase 1**: Removed all file locking code for immediate simplification
- **Phase 2**: Verified single test execution works correctly
- **Phase 3**: Implemented nextest configuration with `carla-server` test group
- **Phase 4**: Updated sequential execution tests to use nextest verification
- **Phase 5**: Updated all documentation to reflect new architecture
- **Phase 6**: Full validation with build, lint, and test verification

#### Test Organization
- **Library Tests**: Run with `make test` (84 tests, no CARLA server needed)
- **Integration Tests**: Run with `make test-server` (requires CARLA server)
- **Test Groups**: All CARLA tests assigned to `carla-server` group with `max-threads = 1`

#### Files Modified
- `carla-test-server/src/lib.rs` - Simplified server management
- `carla-test-server/src/server.rs` - Removed ServerResource wrapper
- `carla-test-server/src/config.rs` - Removed coordination configuration
- `carla-test-server/src/helpers.rs` - Fixed timeout handling

#### Sensor Callback Implementation
- **Design**: True callbacks with FFI-safe function pointers
- **Storage**: Per-sensor callback map replacing global variables
- **Safety**: Thread-safe with Arc<Mutex<>> and handle system
- **API**: `SensorCallbackManager` for lifecycle management
- **Status**: Foundation complete, C++ bridge implementation pending

#### Additional Files Modified
- `carla-test-server/Cargo.toml` - Removed libc dependency
- `carla/.config/nextest.toml` - Added nextest configuration
- `carla/tests/test_sequential_execution.rs` - Simplified and updated
- `Makefile` - Updated test commands and descriptions
- `TEST_SERVER.md` - Updated architecture documentation
- `CLAUDE.md` - Added nextest requirements

#### Benefits Achieved
- ✅ ~200 lines of complex file locking code removed
- ✅ No more stale lock files or infinite blocking
- ✅ Better test execution visibility with nextest
- ✅ Declarative test configuration that's easier to understand
- ✅ Improved debugging with nextest's built-in features
- ✅ Cleaner error handling and test output

---

## Previous Versions

*Note: This changelog was introduced during the nextest migration. Previous changes were not systematically tracked in this format.*

### Development History
- CARLA Rust client library development with support for CARLA 0.10.0
- Implementation of safe Rust bindings using CXX for C++ interoperability
- Creation of `carla-sys` FFI layer and high-level `carla` API
- Development of test infrastructure with `carla-test-server` crate
- Integration of procedural macros for automatic CARLA server management
