# carla

Rust client library for Carla simulator.

## Usage

The usage can be learnt from snipplet [here](examples/spawn.rs).

This crate prepares Carla source code and builds bindings
automatically. You may experience long waiting time in the first
compilation. The bindings are cached in later builds and can save you
much time.

If you prefer to use custom Carla source code, set the source
directory to `CARLA_DIR` environment variable.

## License

MIT license
