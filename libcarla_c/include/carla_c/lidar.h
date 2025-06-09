#ifndef CARLA_C_LIDAR_H
#define CARLA_C_LIDAR_H

#include "carla_c/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// LiDAR Data Analysis Functions
// ============================================================================

/// Calculate basic statistics for a LiDAR point cloud
/// @param data LiDAR sensor data
/// @param stats Output statistics structure
/// @return Error code
carla_error_t carla_lidar_calculate_stats(const carla_sensor_data_t *data,
                                          carla_lidar_stats_t *stats);

/// Calculate bounding box/bounds for a LiDAR point cloud
/// @param data LiDAR sensor data
/// @param bounds Output bounds structure
/// @return Error code
carla_error_t carla_lidar_calculate_bounds(const carla_sensor_data_t *data,
                                           carla_lidar_bounds_t *bounds);

/// Get the distance from sensor origin to a specific point
/// @param point LiDAR detection point
/// @return Distance in meters
float carla_lidar_point_distance(const carla_lidar_detection_t *point);

/// Get the distance between two LiDAR points
/// @param point1 First point
/// @param point2 Second point
/// @return Distance in meters
float carla_lidar_points_distance(const carla_lidar_detection_t *point1,
                                  const carla_lidar_detection_t *point2);

// ============================================================================
// LiDAR Data Filtering Functions
// ============================================================================

/// Filter LiDAR points based on intensity, distance, and height criteria
/// @param data Input LiDAR sensor data
/// @param filter Filter criteria
/// @param output_points Pre-allocated buffer for filtered points
/// @param max_output_points Maximum number of points the buffer can hold
/// @param actual_output_points Actual number of filtered points (output)
/// @return Error code
carla_error_t carla_lidar_filter_points(const carla_sensor_data_t *data,
                                        const carla_lidar_filter_t *filter,
                                        carla_lidar_detection_t *output_points,
                                        size_t max_output_points,
                                        size_t *actual_output_points);

/// Filter points by intensity range
/// @param data Input LiDAR sensor data
/// @param min_intensity Minimum intensity value
/// @param max_intensity Maximum intensity value
/// @param output_points Pre-allocated buffer for filtered points
/// @param max_output_points Maximum number of points the buffer can hold
/// @param actual_output_points Actual number of filtered points (output)
/// @return Error code
carla_error_t carla_lidar_filter_by_intensity(
    const carla_sensor_data_t *data, float min_intensity, float max_intensity,
    carla_lidar_detection_t *output_points, size_t max_output_points,
    size_t *actual_output_points);

/// Filter points by distance from sensor origin
/// @param data Input LiDAR sensor data
/// @param min_distance Minimum distance in meters
/// @param max_distance Maximum distance in meters
/// @param output_points Pre-allocated buffer for filtered points
/// @param max_output_points Maximum number of points the buffer can hold
/// @param actual_output_points Actual number of filtered points (output)
/// @return Error code
carla_error_t carla_lidar_filter_by_distance(
    const carla_sensor_data_t *data, float min_distance, float max_distance,
    carla_lidar_detection_t *output_points, size_t max_output_points,
    size_t *actual_output_points);

/// Filter points by height (Z coordinate) range
/// @param data Input LiDAR sensor data
/// @param min_height Minimum height in meters
/// @param max_height Maximum height in meters
/// @param output_points Pre-allocated buffer for filtered points
/// @param max_output_points Maximum number of points the buffer can hold
/// @param actual_output_points Actual number of filtered points (output)
/// @return Error code
carla_error_t carla_lidar_filter_by_height(
    const carla_sensor_data_t *data, float min_height, float max_height,
    carla_lidar_detection_t *output_points, size_t max_output_points,
    size_t *actual_output_points);

/// Remove points within a spherical region (useful for removing ego vehicle)
/// @param data Input LiDAR sensor data
/// @param center Center of exclusion sphere
/// @param radius Radius of exclusion sphere in meters
/// @param output_points Pre-allocated buffer for filtered points
/// @param max_output_points Maximum number of points the buffer can hold
/// @param actual_output_points Actual number of filtered points (output)
/// @return Error code
carla_error_t carla_lidar_remove_sphere(const carla_sensor_data_t *data,
                                        const carla_vector3d_t *center,
                                        float radius,
                                        carla_lidar_detection_t *output_points,
                                        size_t max_output_points,
                                        size_t *actual_output_points);

// ============================================================================
// Point Cloud Downsampling Functions
// ============================================================================

/// Downsample point cloud using specified method
/// @param data Input LiDAR sensor data
/// @param method Downsampling method
/// @param target_points Target number of points after downsampling
/// @param output_points Pre-allocated buffer for downsampled points
/// @param max_output_points Maximum number of points the buffer can hold
/// @param actual_output_points Actual number of downsampled points (output)
/// @return Error code
carla_error_t carla_lidar_downsample(const carla_sensor_data_t *data,
                                     carla_lidar_downsample_method_t method,
                                     size_t target_points,
                                     carla_lidar_detection_t *output_points,
                                     size_t max_output_points,
                                     size_t *actual_output_points);

/// Downsample using uniform random sampling
/// @param data Input LiDAR sensor data
/// @param target_points Target number of points
/// @param output_points Pre-allocated buffer for downsampled points
/// @param max_output_points Maximum number of points the buffer can hold
/// @param actual_output_points Actual number of downsampled points (output)
/// @return Error code
carla_error_t carla_lidar_downsample_uniform(
    const carla_sensor_data_t *data, size_t target_points,
    carla_lidar_detection_t *output_points, size_t max_output_points,
    size_t *actual_output_points);

/// Downsample using grid-based voxel filtering
/// @param data Input LiDAR sensor data
/// @param voxel_size Size of each voxel in meters
/// @param output_points Pre-allocated buffer for downsampled points
/// @param max_output_points Maximum number of points the buffer can hold
/// @param actual_output_points Actual number of downsampled points (output)
/// @return Error code
carla_error_t
carla_lidar_downsample_grid(const carla_sensor_data_t *data, float voxel_size,
                            carla_lidar_detection_t *output_points,
                            size_t max_output_points,
                            size_t *actual_output_points);

// ============================================================================
// Coordinate Transformation Functions
// ============================================================================

/// Transform point cloud to different coordinate system
/// @param data Input LiDAR sensor data
/// @param target_coords Target coordinate system
/// @param sensor_transform Transform of the sensor (required for world
/// coordinates)
/// @param vehicle_transform Transform of the vehicle (required for world
/// coordinates)
/// @param output_points Pre-allocated buffer for transformed points
/// @param max_output_points Maximum number of points the buffer can hold
/// @param actual_output_points Actual number of transformed points (output)
/// @return Error code
carla_error_t
carla_lidar_transform_coordinates(const carla_sensor_data_t *data,
                                  carla_lidar_coordinate_system_t target_coords,
                                  const carla_transform_t *sensor_transform,
                                  const carla_transform_t *vehicle_transform,
                                  carla_lidar_detection_t *output_points,
                                  size_t max_output_points,
                                  size_t *actual_output_points);

/// Transform a single point to world coordinates
/// @param point Input point in sensor coordinates
/// @param sensor_transform Transform of the sensor
/// @param result Output point in world coordinates
/// @return Error code
carla_error_t
carla_lidar_point_to_world(const carla_lidar_detection_t *point,
                           const carla_transform_t *sensor_transform,
                           carla_lidar_detection_t *result);

/// Transform a single point to vehicle coordinates
/// @param point Input point in sensor coordinates
/// @param sensor_to_vehicle Transform from sensor to vehicle frame
/// @param result Output point in vehicle coordinates
/// @return Error code
carla_error_t
carla_lidar_point_to_vehicle(const carla_lidar_detection_t *point,
                             const carla_transform_t *sensor_to_vehicle,
                             carla_lidar_detection_t *result);

// ============================================================================
// Ground Segmentation Functions
// ============================================================================

/// Segment ground points from non-ground points using plane fitting
/// @param data Input LiDAR sensor data
/// @param max_ground_distance Maximum distance from ground plane to be
/// considered ground
/// @param max_ground_angle Maximum angle from horizontal to be considered
/// ground (radians)
/// @param result Pre-allocated ground segmentation result structure
/// @param ground_buffer Pre-allocated buffer for ground points
/// @param max_ground_points Maximum number of ground points the buffer can hold
/// @param non_ground_buffer Pre-allocated buffer for non-ground points
/// @param max_non_ground_points Maximum number of non-ground points the buffer
/// can hold
/// @return Error code
carla_error_t carla_lidar_segment_ground(
    const carla_sensor_data_t *data, float max_ground_distance,
    float max_ground_angle, carla_lidar_ground_segmentation_t *result,
    carla_lidar_detection_t *ground_buffer, size_t max_ground_points,
    carla_lidar_detection_t *non_ground_buffer, size_t max_non_ground_points);

/// Extract only ground points
/// @param data Input LiDAR sensor data
/// @param max_ground_distance Maximum distance from ground plane
/// @param max_ground_angle Maximum angle from horizontal (radians)
/// @param ground_points Pre-allocated buffer for ground points
/// @param max_ground_points Maximum number of ground points the buffer can hold
/// @param actual_ground_points Actual number of ground points (output)
/// @return Error code
carla_error_t carla_lidar_extract_ground(const carla_sensor_data_t *data,
                                         float max_ground_distance,
                                         float max_ground_angle,
                                         carla_lidar_detection_t *ground_points,
                                         size_t max_ground_points,
                                         size_t *actual_ground_points);

/// Remove ground points, keeping only non-ground points
/// @param data Input LiDAR sensor data
/// @param max_ground_distance Maximum distance from ground plane
/// @param max_ground_angle Maximum angle from horizontal (radians)
/// @param non_ground_points Pre-allocated buffer for non-ground points
/// @param max_non_ground_points Maximum number of non-ground points the buffer
/// can hold
/// @param actual_non_ground_points Actual number of non-ground points (output)
/// @return Error code
carla_error_t carla_lidar_remove_ground(
    const carla_sensor_data_t *data, float max_ground_distance,
    float max_ground_angle, carla_lidar_detection_t *non_ground_points,
    size_t max_non_ground_points, size_t *actual_non_ground_points);

// ============================================================================
// Obstacle Detection Functions
// ============================================================================

/// Detect obstacles using clustering algorithm
/// @param data Input LiDAR sensor data (should be non-ground points)
/// @param cluster_tolerance Distance tolerance for clustering in meters
/// @param min_cluster_size Minimum number of points per cluster
/// @param max_cluster_size Maximum number of points per cluster
/// @param detection Pre-allocated obstacle detection result structure
/// @param cluster_buffer Pre-allocated buffer for clusters
/// @param max_clusters Maximum number of clusters the buffer can hold
/// @param point_buffer Pre-allocated buffer for all clustered points
/// @param max_points Maximum number of points the buffer can hold
/// @return Error code
carla_error_t carla_lidar_detect_obstacles(
    const carla_sensor_data_t *data, float cluster_tolerance,
    size_t min_cluster_size, size_t max_cluster_size,
    carla_lidar_obstacle_detection_t *detection,
    carla_lidar_obstacle_cluster_t *cluster_buffer, size_t max_clusters,
    carla_lidar_detection_t *point_buffer, size_t max_points);

/// Extract the largest cluster (useful for vehicle detection)
/// @param data Input LiDAR sensor data
/// @param cluster_tolerance Distance tolerance for clustering in meters
/// @param min_cluster_size Minimum number of points per cluster
/// @param largest_cluster Pre-allocated buffer for the largest cluster
/// @param max_cluster_points Maximum number of points the cluster buffer can
/// hold
/// @param actual_cluster_points Actual number of points in the largest cluster
/// (output)
/// @return Error code
carla_error_t carla_lidar_extract_largest_cluster(
    const carla_sensor_data_t *data, float cluster_tolerance,
    size_t min_cluster_size, carla_lidar_detection_t *largest_cluster,
    size_t max_cluster_points, size_t *actual_cluster_points);

// ============================================================================
// Data Export Functions
// ============================================================================

/// Save LiDAR point cloud to file
/// @param data Input LiDAR sensor data
/// @param filename Output filename
/// @param format Export format
/// @return Error code
carla_error_t carla_lidar_save_to_file(const carla_sensor_data_t *data,
                                       const char *filename,
                                       carla_lidar_export_format_t format);

/// Save filtered LiDAR points to file
/// @param points Array of LiDAR points to save
/// @param point_count Number of points in the array
/// @param filename Output filename
/// @param format Export format
/// @return Error code
carla_error_t
carla_lidar_save_points_to_file(const carla_lidar_detection_t *points,
                                size_t point_count, const char *filename,
                                carla_lidar_export_format_t format);

/// Get PLY format header for LiDAR data
/// @param point_count Number of points that will be written
/// @param include_intensity Whether to include intensity values
/// @param header_buffer Pre-allocated buffer for header string
/// @param max_header_size Maximum size of header buffer
/// @return Error code
carla_error_t carla_lidar_get_ply_header(size_t point_count,
                                         bool include_intensity,
                                         char *header_buffer,
                                         size_t max_header_size);

// ============================================================================
// Semantic LiDAR Functions
// ============================================================================

/// Filter semantic LiDAR points by semantic tags
/// @param data Input semantic LiDAR sensor data
/// @param filter Semantic filter criteria
/// @param output_points Pre-allocated buffer for filtered points
/// @param max_output_points Maximum number of points the buffer can hold
/// @param actual_output_points Actual number of filtered points (output)
/// @return Error code
carla_error_t carla_semantic_lidar_filter_points(
    const carla_sensor_data_t *data,
    const carla_semantic_lidar_filter_t *filter,
    carla_semantic_lidar_detection_t *output_points, size_t max_output_points,
    size_t *actual_output_points);

/// Extract points belonging to a specific semantic class
/// @param data Input semantic LiDAR sensor data
/// @param semantic_tag Target semantic tag
/// @param output_points Pre-allocated buffer for filtered points
/// @param max_output_points Maximum number of points the buffer can hold
/// @param actual_output_points Actual number of filtered points (output)
/// @return Error code
carla_error_t carla_semantic_lidar_extract_class(
    const carla_sensor_data_t *data, uint32_t semantic_tag,
    carla_semantic_lidar_detection_t *output_points, size_t max_output_points,
    size_t *actual_output_points);

/// Get unique semantic tags present in the point cloud
/// @param data Input semantic LiDAR sensor data
/// @param unique_tags Pre-allocated buffer for unique tags
/// @param max_tags Maximum number of tags the buffer can hold
/// @param actual_tag_count Actual number of unique tags found (output)
/// @return Error code
carla_error_t
carla_semantic_lidar_get_unique_tags(const carla_sensor_data_t *data,
                                     uint32_t *unique_tags, size_t max_tags,
                                     size_t *actual_tag_count);

/// Count points per semantic class
/// @param data Input semantic LiDAR sensor data
/// @param tag_counts Pre-allocated array where index is semantic tag and value
/// is count
/// @param max_tag_value Maximum semantic tag value (array size)
/// @return Error code
carla_error_t
carla_semantic_lidar_count_by_class(const carla_sensor_data_t *data,
                                    size_t *tag_counts, size_t max_tag_value);

// ============================================================================
// Utility Functions
// ============================================================================

/// Initialize a LiDAR filter with default values
/// @param filter Filter structure to initialize
void carla_lidar_filter_init(carla_lidar_filter_t *filter);

/// Initialize a semantic LiDAR filter with default values
/// @param filter Semantic filter structure to initialize
void carla_semantic_lidar_filter_init(carla_semantic_lidar_filter_t *filter);

/// Create a bounding box filter from bounds
/// @param bounds Bounding box to create filter from
/// @param filter Output filter that will accept points within bounds
void carla_lidar_filter_from_bounds(const carla_lidar_bounds_t *bounds,
                                    carla_lidar_filter_t *filter);

/// Check if a point passes the filter criteria
/// @param point Point to test
/// @param filter Filter criteria
/// @return true if point passes filter, false otherwise
bool carla_lidar_point_passes_filter(const carla_lidar_detection_t *point,
                                     const carla_lidar_filter_t *filter);

/// Check if a semantic point passes the filter criteria
/// @param point Semantic point to test
/// @param filter Semantic filter criteria
/// @return true if point passes filter, false otherwise
bool carla_semantic_lidar_point_passes_filter(
    const carla_semantic_lidar_detection_t *point,
    const carla_semantic_lidar_filter_t *filter);

/// Copy LiDAR point data
/// @param src Source point
/// @param dst Destination point
void carla_lidar_point_copy(const carla_lidar_detection_t *src,
                            carla_lidar_detection_t *dst);

/// Copy semantic LiDAR point data
/// @param src Source point
/// @param dst Destination point
void carla_semantic_lidar_point_copy(
    const carla_semantic_lidar_detection_t *src,
    carla_semantic_lidar_detection_t *dst);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_LIDAR_H
