//! Test ActorExt trait implementation

use carla::actor::ActorExt;

fn main() {
    println!("Testing ActorExt trait implementation...");

    // Test with mock data (no server required)
    test_vehicle_actor_ext();
    test_walker_actor_ext();
    test_traffic_light_actor_ext();
    test_traffic_sign_actor_ext();

    println!("\nAll ActorExt tests passed!");
}

fn test_vehicle_actor_ext() {
    println!("\n1. Testing Vehicle ActorExt implementation");

    // This tests that Vehicle implements ActorExt
    fn requires_actor_ext<T: ActorExt>(_: &T) {
        println!("   ✓ Type implements ActorExt");
    }

    // If this compiles, Vehicle implements ActorExt
    let vehicle: Option<carla::actor::Vehicle> = None;
    if let Some(ref v) = vehicle {
        requires_actor_ext(v);
    } else {
        println!("   ✓ Vehicle implements ActorExt (compile-time check)");
    }
}

fn test_walker_actor_ext() {
    println!("\n2. Testing Walker ActorExt implementation");

    fn requires_actor_ext<T: ActorExt>(_: &T) {
        println!("   ✓ Type implements ActorExt");
    }

    let walker: Option<carla::actor::Walker> = None;
    if let Some(ref w) = walker {
        requires_actor_ext(w);
    } else {
        println!("   ✓ Walker implements ActorExt (compile-time check)");
    }
}

fn test_traffic_light_actor_ext() {
    println!("\n3. Testing TrafficLight ActorExt implementation");

    fn requires_actor_ext<T: ActorExt>(_: &T) {
        println!("   ✓ Type implements ActorExt");
    }

    let traffic_light: Option<carla::actor::TrafficLight> = None;
    if let Some(ref tl) = traffic_light {
        requires_actor_ext(tl);
    } else {
        println!("   ✓ TrafficLight implements ActorExt (compile-time check)");
    }
}

fn test_traffic_sign_actor_ext() {
    println!("\n4. Testing TrafficSign ActorExt implementation");

    fn requires_actor_ext<T: ActorExt>(_: &T) {
        println!("   ✓ Type implements ActorExt");
    }

    let traffic_sign: Option<carla::actor::TrafficSign> = None;
    if let Some(ref ts) = traffic_sign {
        requires_actor_ext(ts);
    } else {
        println!("   ✓ TrafficSign implements ActorExt (compile-time check)");
    }
}
