pub mod config;
pub mod pool;

use std::{sync::LazyLock, time::Duration};

use carla::client::{Client, World};
use pool::ServerLease;

/// The server lease — acquired once per test binary.
static LEASE: LazyLock<ServerLease> = LazyLock::new(ServerLease::acquire);

/// Shared CARLA client — connected once per test binary.
static CLIENT: LazyLock<Client> = LazyLock::new(|| {
    let port = LEASE.port();
    let mut client =
        Client::connect("127.0.0.1", port, None).expect("CARLA server must be running");
    client
        .set_timeout(Duration::from_secs(30))
        .expect("Failed to set timeout");
    client
});

/// Get a reference to the shared client.
pub fn client() -> &'static Client {
    &CLIENT
}

/// Get a world handle from the shared client.
pub fn world() -> World {
    CLIENT.world().expect("Failed to get world")
}
