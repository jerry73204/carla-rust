use anyhow::Result;
use carla::{client::Client, prelude::*, rpc::ActorId};
use clap::Parser;
use nalgebra::{Isometry3, Translation3, Vector3};
use serde::Serialize;
use std::{collections::HashMap, fs, path::PathBuf, time::Duration};

#[derive(Parser)]
struct Opts {
    #[clap(short = 'o', long)]
    pub output_file: Option<PathBuf>,
    #[clap(long)]
    pub watch: bool,
}

#[derive(Serialize)]
struct ActorDesc {
    pub id: ActorId,
    pub display_id: String,
    pub type_id: String,
    pub parent_id: ActorId,
    pub location: Translation3<f32>,
    pub transform: Isometry3<f32>,
    pub velocity: Vector3<f32>,
    pub acceleration: Vector3<f32>,
    pub angular_velocity: Vector3<f32>,
    pub attributes: HashMap<String, String>,
}

fn main() -> Result<()> {
    let opts = Opts::parse();

    let mut client = Client::default();
    client.set_timeout(Duration::from_secs(5));

    let mut world = client.load_world("Town07");
    let is_sync = world.settings().synchronous_mode;

    loop {
        if is_sync {
            world.tick();
        } else {
            world.wait_for_tick();
        }

        let descs: Vec<_> = world
            .actors()
            .iter()
            .map(|actor| ActorDesc {
                id: actor.id(),
                display_id: actor.display_id(),
                type_id: actor.type_id(),
                parent_id: actor.parent_id(),
                location: actor.location(),
                transform: actor.transform(),
                velocity: actor.velocity(),
                acceleration: actor.acceleration(),
                angular_velocity: actor.angular_velocity(),
                attributes: actor
                    .attributes()
                    .iter()
                    .map(|attr| (attr.id(), attr.value_string()))
                    .collect(),
            })
            .collect();
        let text = serde_json::to_string_pretty(&descs)?;

        match &opts.output_file {
            Some(path) => fs::write(path, text)?,
            None => print!("{text}"),
        }

        if !opts.watch {
            break;
        }
    }
    Ok(())
}
