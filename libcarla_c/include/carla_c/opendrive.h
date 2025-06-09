#ifndef CARLA_C_OPENDRIVE_H
#define CARLA_C_OPENDRIVE_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// OpenDRIVE Map Management

// Load OpenDRIVE map from XML string
carla_opendrive_map_t *
carla_opendrive_load_from_string(const char *opendrive_xml, size_t xml_length);

// Load OpenDRIVE map from file
carla_opendrive_map_t *carla_opendrive_load_from_file(const char *file_path);

// Generate world from OpenDRIVE content
carla_world_t *carla_client_generate_opendrive_world(
    carla_client_t *client, const char *opendrive_xml,
    const carla_opendrive_generation_params_t *params, bool reset_settings);

// Get OpenDRIVE content from existing map
carla_error_t carla_map_get_opendrive_content(const carla_map_t *map,
                                              char **opendrive_xml,
                                              size_t *xml_length);

// Destroy OpenDRIVE map and free memory
void carla_opendrive_map_destroy(carla_opendrive_map_t *map);

// OpenDRIVE Map Information

// Get map data structure with roads and junctions
carla_error_t carla_opendrive_map_get_data(const carla_opendrive_map_t *map,
                                           carla_opendrive_map_data_t *data);

// Get map name
const char *carla_opendrive_map_get_name(const carla_opendrive_map_t *map);

// Get OpenDRIVE version
double carla_opendrive_map_get_version(const carla_opendrive_map_t *map);

// Get map bounds
carla_error_t carla_opendrive_map_get_bounds(const carla_opendrive_map_t *map,
                                             carla_vector3d_t *min_bounds,
                                             carla_vector3d_t *max_bounds);

// Road Network Access

// Get number of roads in map
size_t carla_opendrive_map_get_road_count(const carla_opendrive_map_t *map);

// Get road by index
carla_road_t *carla_opendrive_map_get_road(const carla_opendrive_map_t *map,
                                           size_t index);

// Get road by ID
carla_road_t *
carla_opendrive_map_get_road_by_id(const carla_opendrive_map_t *map,
                                   carla_road_id_t road_id);

// Get road data
carla_error_t carla_road_get_data(const carla_road_t *road,
                                  carla_road_data_t *data);

// Get road ID
carla_road_id_t carla_road_get_id(const carla_road_t *road);

// Get road name
const char *carla_road_get_name(const carla_road_t *road);

// Get road length
double carla_road_get_length(const carla_road_t *road);

// Junction Access

// Get number of junctions in map
size_t carla_opendrive_map_get_junction_count(const carla_opendrive_map_t *map);

// Get junction by index
carla_junction_t *
carla_opendrive_map_get_junction(const carla_opendrive_map_t *map,
                                 size_t index);

// Get junction by ID
carla_junction_t *
carla_opendrive_map_get_junction_by_id(const carla_opendrive_map_t *map,
                                       carla_junction_id_t junction_id);

// Get junction data
carla_error_t carla_junction_get_data(const carla_junction_t *junction,
                                      carla_junction_data_t *data);

// Lane Access

// Get lane section from road
carla_error_t carla_road_get_lane_section(const carla_road_t *road,
                                          size_t section_index,
                                          carla_lane_section_data_t *section);

// Get lane from section
carla_error_t
carla_lane_section_get_lane(const carla_lane_section_data_t *section,
                            carla_lane_id_t lane_id, carla_lane_data_t *lane);

// Waypoint Generation and Navigation

// Get waypoint at world location
carla_waypoint_t *carla_opendrive_map_get_waypoint(
    const carla_opendrive_map_t *map, const carla_vector3d_t *location,
    bool project_to_road, carla_lane_type_t lane_type);

// Get waypoint by OpenDRIVE coordinates
carla_waypoint_t *
carla_opendrive_map_get_waypoint_xodr(const carla_opendrive_map_t *map,
                                      carla_road_id_t road_id,
                                      carla_lane_id_t lane_id, double s);

// Generate waypoints at regular intervals
carla_error_t carla_opendrive_map_generate_waypoints(
    const carla_opendrive_map_t *map, double distance,
    carla_waypoint_t ***waypoints, size_t *waypoint_count);

// Get next waypoints from current waypoint
carla_error_t carla_waypoint_get_next(const carla_waypoint_t *waypoint,
                                      double distance,
                                      carla_waypoint_t ***next_waypoints,
                                      size_t *next_count);

// Get previous waypoints from current waypoint
carla_error_t
carla_waypoint_get_previous(const carla_waypoint_t *waypoint, double distance,
                            carla_waypoint_t ***previous_waypoints,
                            size_t *previous_count);

// Get waypoint data
carla_error_t carla_waypoint_get_data(const carla_waypoint_t *waypoint,
                                      carla_waypoint_data_t *data);

// Destroy waypoint
void carla_waypoint_destroy(carla_waypoint_t *waypoint);

// Destroy waypoint array
void carla_waypoint_array_destroy(carla_waypoint_t **waypoints, size_t count);

// Map Topology

// Get map topology (waypoint connections)
carla_error_t carla_opendrive_map_get_topology(const carla_opendrive_map_t *map,
                                               carla_map_topology_t *topology);

// Free topology structure
void carla_map_topology_destroy(carla_map_topology_t *topology);

// OpenDRIVE Generation Parameters

// Create default generation parameters
carla_opendrive_generation_params_t carla_opendrive_create_default_params(void);

// Validate generation parameters
carla_error_t carla_opendrive_validate_params(
    const carla_opendrive_generation_params_t *params);

// Mesh Generation from OpenDRIVE

// Generate mesh from OpenDRIVE map
carla_error_t carla_opendrive_map_generate_mesh(
    const carla_opendrive_map_t *map,
    const carla_mesh_generation_options_t *options, carla_mesh_data_t *mesh);

// Generate chunked mesh with parameters
carla_error_t carla_opendrive_map_generate_chunked_mesh(
    const carla_opendrive_map_t *map,
    const carla_opendrive_generation_params_t *params,
    carla_mesh_data_t **meshes, size_t *mesh_count);

// Generate crosswalk mesh
carla_error_t
carla_opendrive_map_generate_crosswalk_mesh(const carla_opendrive_map_t *map,
                                            carla_mesh_data_t *mesh);

// Generate walls mesh
carla_error_t
carla_opendrive_map_generate_walls_mesh(const carla_opendrive_map_t *map,
                                        double distance, float wall_height,
                                        carla_mesh_data_t *mesh);

// Destroy mesh data
void carla_mesh_data_destroy(carla_mesh_data_t *mesh);

// Destroy mesh array
void carla_mesh_data_array_destroy(carla_mesh_data_t *meshes, size_t count);

// OpenDRIVE Building and Construction

// Create new OpenDRIVE map builder
carla_opendrive_map_t *carla_opendrive_map_builder_create(const char *map_name,
                                                          double version);

// Add road to map being built
carla_error_t carla_opendrive_map_builder_add_road(
    carla_opendrive_map_t *map, carla_road_id_t road_id, const char *name,
    double length, carla_junction_id_t junction_id, carla_road_id_t predecessor,
    carla_road_id_t successor);

// Add geometry to road
carla_error_t carla_opendrive_map_builder_add_road_geometry_line(
    carla_opendrive_map_t *map, carla_road_id_t road_id, double s, double x,
    double y, double hdg, double length);

carla_error_t carla_opendrive_map_builder_add_road_geometry_arc(
    carla_opendrive_map_t *map, carla_road_id_t road_id, double s, double x,
    double y, double hdg, double length, double curvature);

carla_error_t carla_opendrive_map_builder_add_road_geometry_spiral(
    carla_opendrive_map_t *map, carla_road_id_t road_id, double s, double x,
    double y, double hdg, double length, double curv_start, double curv_end);

// Add lane section to road
carla_error_t carla_opendrive_map_builder_add_lane_section(
    carla_opendrive_map_t *map, carla_road_id_t road_id,
    carla_section_id_t section_id, double s);

// Add lane to section
carla_error_t carla_opendrive_map_builder_add_lane(
    carla_opendrive_map_t *map, carla_road_id_t road_id,
    carla_section_id_t section_id, carla_lane_id_t lane_id,
    carla_lane_type_t lane_type, bool level, carla_lane_id_t predecessor,
    carla_lane_id_t successor);

// Add junction to map
carla_error_t
carla_opendrive_map_builder_add_junction(carla_opendrive_map_t *map,
                                         carla_junction_id_t junction_id,
                                         const char *name);

// Add connection to junction
carla_error_t carla_opendrive_map_builder_add_connection(
    carla_opendrive_map_t *map, carla_junction_id_t junction_id,
    carla_connection_id_t connection_id, carla_road_id_t incoming_road,
    carla_road_id_t connecting_road);

// Finalize map construction
carla_error_t carla_opendrive_map_builder_build(carla_opendrive_map_t *map);

// Export map to OpenDRIVE XML
carla_error_t carla_opendrive_map_export_xml(const carla_opendrive_map_t *map,
                                             char **xml_content,
                                             size_t *xml_length);

// Utility Functions

// Create default mesh generation options
carla_mesh_generation_options_t
carla_mesh_generation_create_default_options(void);

// Validate mesh generation options
carla_error_t carla_mesh_generation_validate_options(
    const carla_mesh_generation_options_t *options);

// Convert lane type to string
const char *carla_lane_type_to_string(carla_lane_type_t lane_type);

// Convert lane type from string
carla_lane_type_t carla_lane_type_from_string(const char *lane_type_str);

// Convert geometry type to string
const char *
carla_road_geometry_type_to_string(carla_road_geometry_type_t geometry_type);

// Convert geometry type from string
carla_road_geometry_type_t
carla_road_geometry_type_from_string(const char *geometry_type_str);

// Memory Management

// Free OpenDRIVE map data structure
void carla_opendrive_map_data_destroy(carla_opendrive_map_data_t *data);

// Free road data structure
void carla_road_data_destroy(carla_road_data_t *data);

// Free junction data structure
void carla_junction_data_destroy(carla_junction_data_t *data);

// Free waypoint path structure
void carla_waypoint_path_destroy(carla_waypoint_path_t *path);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_OPENDRIVE_H
