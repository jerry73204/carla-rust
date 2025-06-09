#ifndef CARLA_C_CAMERA_H
#define CARLA_C_CAMERA_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Basic camera data access functions

// Get image dimensions
uint32_t carla_image_get_width(const carla_sensor_data_t *data);
uint32_t carla_image_get_height(const carla_sensor_data_t *data);
uint32_t carla_image_get_fov(const carla_sensor_data_t *data);

// Get raw image data (returns pointer to pixel data)
const uint8_t *carla_image_get_raw_data(const carla_sensor_data_t *data,
                                        size_t *data_size);

// Get specific pixel values (for BGRA images)
carla_color_t carla_image_get_pixel_bgra(const carla_sensor_data_t *data,
                                         uint32_t x, uint32_t y);
bool carla_image_set_pixel_bgra(carla_sensor_data_t *data, uint32_t x,
                                uint32_t y, carla_color_t color);

// Get depth pixel value (for depth images)
float carla_image_get_depth_at(const carla_sensor_data_t *data, uint32_t x,
                               uint32_t y);

// Get semantic class ID (for semantic segmentation images)
uint32_t carla_image_get_semantic_id_at(const carla_sensor_data_t *data,
                                        uint32_t x, uint32_t y);

// Get instance ID (for instance segmentation images)
uint32_t carla_image_get_instance_id_at(const carla_sensor_data_t *data,
                                        uint32_t x, uint32_t y);

// Get optical flow vector (for optical flow images)
carla_optical_flow_pixel_t
carla_image_get_optical_flow_at(const carla_sensor_data_t *data, uint32_t x,
                                uint32_t y);

// Image statistics and analysis functions

// Compute basic image statistics
carla_error_t carla_image_compute_stats(const carla_sensor_data_t *data,
                                        carla_image_stats_t *stats);

// Create image histogram
carla_error_t carla_image_create_histogram(const carla_sensor_data_t *data,
                                           uint32_t bins,
                                           carla_image_histogram_t *histogram);
void carla_image_destroy_histogram(carla_image_histogram_t *histogram);

// Region of interest analysis
carla_error_t carla_image_analyze_roi(const carla_sensor_data_t *data,
                                      const carla_image_roi_t *roi,
                                      carla_image_stats_t *stats);

// Color space conversion functions

// Convert image to different color space
carla_error_t carla_image_convert_color_space(const carla_sensor_data_t *input,
                                              carla_color_space_t target_space,
                                              uint8_t **output_data,
                                              size_t *output_size);

// Convert BGRA to RGB
carla_error_t carla_image_bgra_to_rgb(const carla_sensor_data_t *data,
                                      uint8_t **rgb_data, size_t *rgb_size);

// Convert BGRA to grayscale
carla_error_t carla_image_bgra_to_grayscale(const carla_sensor_data_t *data,
                                            uint8_t **gray_data,
                                            size_t *gray_size);

// Convert BGRA to HSV
carla_error_t carla_image_bgra_to_hsv(const carla_sensor_data_t *data,
                                      uint8_t **hsv_data, size_t *hsv_size);

// Image filtering and processing functions

// Apply image filter
carla_error_t carla_image_apply_filter(const carla_sensor_data_t *input,
                                       carla_image_filter_t filter_type,
                                       float filter_strength,
                                       uint8_t **output_data,
                                       size_t *output_size);

// Gaussian blur
carla_error_t carla_image_gaussian_blur(const carla_sensor_data_t *input,
                                        float sigma, uint8_t **output_data,
                                        size_t *output_size);

// Edge detection
carla_error_t carla_image_edge_detection(const carla_sensor_data_t *input,
                                         float threshold, uint8_t **output_data,
                                         size_t *output_size);

// Image resizing
carla_error_t carla_image_resize(const carla_sensor_data_t *input,
                                 uint32_t new_width, uint32_t new_height,
                                 uint8_t **output_data, size_t *output_size);

// Image cropping
carla_error_t carla_image_crop(const carla_sensor_data_t *input,
                               const carla_image_roi_t *roi,
                               uint8_t **output_data, size_t *output_size);

// Semantic segmentation analysis functions

// Analyze semantic segmentation
carla_error_t
carla_image_analyze_semantic_segmentation(const carla_sensor_data_t *data,
                                          carla_semantic_analysis_t *analysis);

// Get unique semantic classes in image
carla_error_t carla_image_get_semantic_classes(const carla_sensor_data_t *data,
                                               uint32_t **class_ids,
                                               size_t *class_count);

// Count pixels for specific semantic class
uint32_t carla_image_count_semantic_pixels(const carla_sensor_data_t *data,
                                           uint32_t class_id);

// Create semantic class mask
carla_error_t carla_image_create_semantic_mask(const carla_sensor_data_t *data,
                                               uint32_t class_id,
                                               uint8_t **mask_data,
                                               size_t *mask_size);

// Free semantic analysis result
void carla_semantic_analysis_destroy(carla_semantic_analysis_t *analysis);

// Instance segmentation analysis functions

// Analyze instance segmentation
carla_error_t
carla_image_analyze_instance_segmentation(const carla_sensor_data_t *data,
                                          carla_instance_analysis_t *analysis);

// Get instance bounding boxes
carla_error_t carla_image_get_instance_bounding_boxes(
    const carla_sensor_data_t *data, carla_image_roi_t **bounding_boxes,
    uint32_t **instance_ids, size_t *instance_count);

// Create instance mask for specific instance
carla_error_t carla_image_create_instance_mask(const carla_sensor_data_t *data,
                                               uint32_t instance_id,
                                               uint8_t **mask_data,
                                               size_t *mask_size);

// Free instance analysis result
void carla_instance_analysis_destroy(carla_instance_analysis_t *analysis);

// Depth image analysis functions

// Analyze depth image
carla_error_t carla_image_analyze_depth(const carla_sensor_data_t *data,
                                        carla_depth_analysis_t *analysis);

// Convert depth to point cloud
carla_error_t
carla_image_depth_to_point_cloud(const carla_sensor_data_t *depth_data,
                                 const carla_camera_intrinsics_t *intrinsics,
                                 carla_vector3d_t **points,
                                 carla_color_t **colors, size_t *point_count);

// Get world coordinates for pixel (requires camera intrinsics)
carla_error_t
carla_image_pixel_to_world(uint32_t pixel_x, uint32_t pixel_y, float depth,
                           const carla_camera_intrinsics_t *intrinsics,
                           const carla_transform_t *camera_transform,
                           carla_vector3d_t *world_point);

// Project world point to pixel coordinates
carla_error_t
carla_image_world_to_pixel(const carla_vector3d_t *world_point,
                           const carla_camera_intrinsics_t *intrinsics,
                           const carla_transform_t *camera_transform,
                           uint32_t *pixel_x, uint32_t *pixel_y);

// Free depth analysis result
void carla_depth_analysis_destroy(carla_depth_analysis_t *analysis);

// Optical flow analysis functions

// Analyze optical flow
carla_error_t
carla_image_analyze_optical_flow(const carla_sensor_data_t *data,
                                 carla_optical_flow_analysis_t *analysis);

// Get optical flow magnitude image
carla_error_t
carla_image_optical_flow_magnitude(const carla_sensor_data_t *data,
                                   uint8_t **magnitude_data,
                                   size_t *magnitude_size);

// Get optical flow direction image
carla_error_t
carla_image_optical_flow_direction(const carla_sensor_data_t *data,
                                   uint8_t **direction_data,
                                   size_t *direction_size);

// Detect motion regions
carla_error_t carla_image_detect_motion_regions(const carla_sensor_data_t *data,
                                                float motion_threshold,
                                                carla_image_roi_t **regions,
                                                size_t *region_count);

// Free optical flow analysis result
void carla_optical_flow_analysis_destroy(
    carla_optical_flow_analysis_t *analysis);

// DVS (Dynamic Vision Sensor) functions

// Get DVS events from DVS event array data
const carla_dvs_event_t *carla_dvs_get_events(const carla_sensor_data_t *data,
                                              size_t *event_count);

// Analyze DVS events
carla_error_t carla_dvs_analyze_events(const carla_sensor_data_t *data,
                                       carla_dvs_analysis_t *analysis);

// Convert DVS events to image representation
carla_error_t carla_dvs_events_to_image(const carla_sensor_data_t *data,
                                        float time_window, uint8_t **image_data,
                                        size_t *image_size);

// Filter DVS events by time window
carla_error_t carla_dvs_filter_events_by_time(
    const carla_sensor_data_t *data, int64_t start_time, int64_t end_time,
    carla_dvs_event_t **filtered_events, size_t *filtered_count);

// Filter DVS events by polarity
carla_error_t carla_dvs_filter_events_by_polarity(
    const carla_sensor_data_t *data, bool positive_polarity,
    carla_dvs_event_t **filtered_events, size_t *filtered_count);

// Free DVS analysis result
void carla_dvs_analysis_destroy(carla_dvs_analysis_t *analysis);

// Camera calibration and intrinsics functions

// Get camera intrinsics from sensor
carla_error_t
carla_camera_get_intrinsics(const carla_sensor_data_t *data,
                            carla_camera_intrinsics_t *intrinsics);

// Set camera intrinsics (for custom calibration)
carla_error_t
carla_camera_set_intrinsics(carla_sensor_data_t *data,
                            const carla_camera_intrinsics_t *intrinsics);

// Undistort image using camera intrinsics
carla_error_t carla_image_undistort(const carla_sensor_data_t *input,
                                    const carla_camera_intrinsics_t *intrinsics,
                                    uint8_t **output_data, size_t *output_size);

// Computer vision utility functions

// Find contours in binary image
carla_error_t carla_image_find_contours(const uint8_t *binary_image,
                                        uint32_t width, uint32_t height,
                                        carla_vector3d_t **contour_points,
                                        size_t **contour_lengths,
                                        size_t *contour_count);

// Calculate image moments
carla_error_t carla_image_calculate_moments(const uint8_t *image,
                                            uint32_t width, uint32_t height,
                                            float *m00, float *m10, float *m01,
                                            float *m20, float *m11, float *m02);

// Template matching
carla_error_t carla_image_template_match(const carla_sensor_data_t *image,
                                         const uint8_t *template_data,
                                         uint32_t template_width,
                                         uint32_t template_height,
                                         uint32_t *match_x, uint32_t *match_y,
                                         float *match_score);

// Image export functions

// Save image to file
carla_error_t carla_image_save_to_file(const carla_sensor_data_t *data,
                                       const char *filename,
                                       carla_image_export_format_t format);

// Convert image to PNG buffer
carla_error_t carla_image_to_png(const carla_sensor_data_t *data,
                                 uint8_t **png_data, size_t *png_size);

// Convert image to JPEG buffer
carla_error_t carla_image_to_jpeg(const carla_sensor_data_t *data, int quality,
                                  uint8_t **jpeg_data, size_t *jpeg_size);

// Memory management functions

// Free image data buffer
void carla_image_free_data(uint8_t *data);

// Free point cloud data
void carla_image_free_point_cloud(carla_vector3d_t *points,
                                  carla_color_t *colors);

// Free array data
void carla_image_free_array(void *array);

// Free contour data
void carla_image_free_contours(carla_vector3d_t *contour_points,
                               size_t *contour_lengths);

// Multi-camera utilities

// Synchronize multiple camera frames by timestamp
carla_error_t carla_camera_synchronize_frames(carla_sensor_data_t **camera_data,
                                              size_t camera_count,
                                              double time_threshold,
                                              bool *synchronized);

// Stereo vision utilities (for dual camera setups)
carla_error_t carla_stereo_compute_disparity(
    const carla_sensor_data_t *left_image,
    const carla_sensor_data_t *right_image,
    const carla_camera_intrinsics_t *left_intrinsics,
    const carla_camera_intrinsics_t *right_intrinsics, float baseline,
    uint8_t **disparity_data, size_t *disparity_size);

// Rectify stereo image pair
carla_error_t
carla_stereo_rectify_images(const carla_sensor_data_t *left_image,
                            const carla_sensor_data_t *right_image,
                            const carla_camera_intrinsics_t *left_intrinsics,
                            const carla_camera_intrinsics_t *right_intrinsics,
                            uint8_t **left_rectified, uint8_t **right_rectified,
                            size_t *rectified_size);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_CAMERA_H
