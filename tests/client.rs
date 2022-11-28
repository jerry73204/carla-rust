use carla_rust::Client;

#[test]
fn client_connect() {
    let client = Client::default();
    let _world = client.load_world("Town01");
}
