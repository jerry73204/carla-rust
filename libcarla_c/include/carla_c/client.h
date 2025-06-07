#ifndef CARLA_C_CLIENT_H
#define CARLA_C_CLIENT_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Client creation and destruction
carla_client_t* carla_client_new(const char* host, uint16_t port, size_t worker_threads);
void carla_client_free(carla_client_t* client);

// Client configuration
void carla_client_set_timeout(carla_client_t* client, uint64_t timeout_ms);
uint64_t carla_client_get_timeout(const carla_client_t* client);

// Version information
char* carla_client_get_client_version(const carla_client_t* client);
char* carla_client_get_server_version(const carla_client_t* client);

// World management
carla_world_t* carla_client_get_world(carla_client_t* client);
carla_world_t* carla_client_reload_world(carla_client_t* client, bool reset_settings);
carla_world_t* carla_client_load_world(carla_client_t* client, const char* map_name, 
                                       bool reset_settings, carla_map_layer_t layers);
void carla_client_load_world_if_different(carla_client_t* client, const char* map_name,
                                         bool reset_settings, carla_map_layer_t layers);

// Map queries
carla_string_list_t* carla_client_get_available_maps(const carla_client_t* client);

// File management
bool carla_client_set_files_base_folder(carla_client_t* client, const char* path);
carla_string_list_t* carla_client_get_required_files(const carla_client_t* client, 
                                                     const char* folder, bool download);
void carla_client_request_file(carla_client_t* client, const char* name);

// Traffic manager
uint16_t carla_client_get_traffic_manager_port(carla_client_t* client, uint16_t port);

// String list operations
size_t carla_string_list_size(const carla_string_list_t* list);
const char* carla_string_list_get(const carla_string_list_t* list, size_t index);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_CLIENT_H