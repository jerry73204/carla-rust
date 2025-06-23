#include <carla/client/Client.h>
#include <iostream>
#include <memory>

int main() {
  try {
    std::cout << "Creating CARLA client..." << std::endl;

    // Create client with 0 worker threads
    auto client = std::make_unique<carla::client::Client>("localhost", 2000, 0);

    std::cout << "Client created successfully!" << std::endl;

    // Try to get server version
    std::string version = client->GetServerVersion();
    std::cout << "Server version: " << version << std::endl;

    // Try to get world
    std::cout << "Getting world..." << std::endl;
    auto world = client->GetWorld();
    std::cout << "World ID: " << world.GetId() << std::endl;

    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
