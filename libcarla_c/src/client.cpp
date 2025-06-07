#include "carla_c/client.h"
#include "carla_c/world.h"
#include "internal.h"
#include <cstring>
#include <cstdlib>
#include <vector>

// Helper function to duplicate a string
static char* duplicate_string(const std::string& str) {
    char* result = (char*)malloc(str.length() + 1);
    if (result) {
        std::strcpy(result, str.c_str());
    }
    return result;
}

// Client creation and destruction
carla_client_t* carla_client_new(const char* host, uint16_t port, size_t worker_threads) {
    try {
        auto* wrapper = new carla_client();
        wrapper->client = std::make_unique<carla::client::Client>(host, port, worker_threads);
        return wrapper;
    } catch (...) {
        return nullptr;
    }
}

void carla_client_free(carla_client_t* client) {
    delete client;
}

// Client configuration
void carla_client_set_timeout(carla_client_t* client, uint64_t timeout_ms) {
    if (client && client->client) {
        client->client->SetTimeout(std::chrono::milliseconds(timeout_ms));
    }
}

uint64_t carla_client_get_timeout(const carla_client_t* client) {
    if (client && client->client) {
        auto timeout = client->client->GetTimeout();
        return timeout.milliseconds();
    }
    return 0;
}

// Version information
char* carla_client_get_client_version(const carla_client_t* client) {
    if (client && client->client) {
        return duplicate_string(client->client->GetClientVersion());
    }
    return nullptr;
}

char* carla_client_get_server_version(const carla_client_t* client) {
    if (client && client->client) {
        return duplicate_string(client->client->GetServerVersion());
    }
    return nullptr;
}

// World management
carla_world_t* carla_client_get_world(carla_client_t* client) {
    if (client && client->client) {
        try {
            client->current_world = std::make_shared<carla::client::World>(client->client->GetWorld());
            auto* world_wrapper = new carla_world_t{client->current_world};
            return world_wrapper;
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_world_t* carla_client_reload_world(carla_client_t* client, bool reset_settings) {
    if (client && client->client) {
        try {
            client->current_world = std::make_shared<carla::client::World>(client->client->ReloadWorld(reset_settings));
            auto* world_wrapper = new carla_world_t{client->current_world};
            return world_wrapper;
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_world_t* carla_client_load_world(carla_client_t* client, const char* map_name, 
                                       bool reset_settings, carla_map_layer_t layers) {
    if (client && client->client && map_name) {
        try {
            auto rpc_layers = static_cast<carla::rpc::MapLayer>(layers);
            client->current_world = std::make_shared<carla::client::World>(
                client->client->LoadWorld(map_name, reset_settings, rpc_layers));
            auto* world_wrapper = new carla_world_t{client->current_world};
            return world_wrapper;
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

void carla_client_load_world_if_different(carla_client_t* client, const char* map_name,
                                         bool reset_settings, carla_map_layer_t layers) {
    if (client && client->client && map_name) {
        try {
            auto rpc_layers = static_cast<carla::rpc::MapLayer>(layers);
            client->client->LoadWorldIfDifferent(map_name, reset_settings, rpc_layers);
        } catch (...) {
            // Silently ignore errors
        }
    }
}

// Map queries
carla_string_list_t* carla_client_get_available_maps(const carla_client_t* client) {
    if (client && client->client) {
        try {
            auto maps = client->client->GetAvailableMaps();
            auto* list = new carla_string_list();
            list->count = maps.size();
            list->strings = (char**)calloc(list->count, sizeof(char*));
            
            for (size_t i = 0; i < list->count; ++i) {
                list->strings[i] = duplicate_string(maps[i]);
            }
            return list;
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

// File management
bool carla_client_set_files_base_folder(carla_client_t* client, const char* path) {
    if (client && client->client && path) {
        try {
            return client->client->SetFilesBaseFolder(path);
        } catch (...) {
            return false;
        }
    }
    return false;
}

carla_string_list_t* carla_client_get_required_files(const carla_client_t* client, 
                                                     const char* folder, bool download) {
    if (client && client->client) {
        try {
            std::string folder_str = folder ? folder : "";
            auto files = client->client->GetRequiredFiles(folder_str, download);
            auto* list = new carla_string_list();
            list->count = files.size();
            list->strings = (char**)calloc(list->count, sizeof(char*));
            
            for (size_t i = 0; i < list->count; ++i) {
                list->strings[i] = duplicate_string(files[i]);
            }
            return list;
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

void carla_client_request_file(carla_client_t* client, const char* name) {
    if (client && client->client && name) {
        try {
            client->client->RequestFile(name);
        } catch (...) {
            // Silently ignore errors
        }
    }
}

// Traffic manager
uint16_t carla_client_get_traffic_manager_port(carla_client_t* client, uint16_t port) {
    if (client && client->client) {
        try {
            return client->client->GetInstanceTM(port).Port();
        } catch (...) {
            return 0;
        }
    }
    return 0;
}

// String list operations
size_t carla_string_list_size(const carla_string_list_t* list) {
    return list ? list->count : 0;
}

const char* carla_string_list_get(const carla_string_list_t* list, size_t index) {
    if (list && index < list->count) {
        return list->strings[index];
    }
    return nullptr;
}