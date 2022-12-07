use carla::client::Client;
use clap::Parser;
use std::time::Duration;

#[derive(Parser)]
struct Opts {
    pub world: Option<String>,
}

fn main() {
    let world_name = Opts::parse().world;
    let mut client = Client::default();
    client.set_timeout(Duration::from_secs(5));

    let world = match &world_name {
        Some(name) => client.load_world(name),
        None => client.world(),
    };
    let map = world.map();
    let opendrive = map.to_open_drive();
    print!("{opendrive}");
}
