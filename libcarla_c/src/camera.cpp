#include "carla_c/camera.h"
#include "internal.h"

#include <carla/sensor/data/Color.h>
#include <carla/sensor/data/DVSEventArray.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/ImageTmpl.h>
#include <carla/sensor/s11n/ImageSerializer.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <vector>

using namespace carla::sensor::data;

// Helper function to get image from sensor data
template <typename ImageType>
static const ImageType *get_image_data(const carla_sensor_data_t *data) {
  if (!data || !data->cpp_data)
    return nullptr;
  return dynamic_cast<const ImageType *>(data->cpp_data.get());
}

// Basic camera data access functions

uint32_t carla_image_get_width(const carla_sensor_data_t *data) {
  auto image = get_image_data<Image>(data);
  return image ? image->GetWidth() : 0;
}

uint32_t carla_image_get_height(const carla_sensor_data_t *data) {
  auto image = get_image_data<Image>(data);
  return image ? image->GetHeight() : 0;
}

uint32_t carla_image_get_fov(const carla_sensor_data_t *data) {
  auto image = get_image_data<Image>(data);
  return image ? static_cast<uint32_t>(image->GetFOVAngle()) : 0;
}

const uint8_t *carla_image_get_raw_data(const carla_sensor_data_t *data,
                                        size_t *data_size) {
  auto image = get_image_data<Image>(data);
  if (!image || !data_size)
    return nullptr;

  *data_size = image->size();
  return reinterpret_cast<const uint8_t *>(image->data());
}

carla_color_t carla_image_get_pixel_bgra(const carla_sensor_data_t *data,
                                         uint32_t x, uint32_t y) {
  carla_color_t result = {0, 0, 0, 0};

  auto image = get_image_data<ImageTmpl<Color>>(data);
  if (!image || x >= image->GetWidth() || y >= image->GetHeight()) {
    return result;
  }

  auto color = image->at(y * image->GetWidth() + x);
  result.b = color.b;
  result.g = color.g;
  result.r = color.r;
  result.a = color.a;

  return result;
}

bool carla_image_set_pixel_bgra(carla_sensor_data_t *data, uint32_t x,
                                uint32_t y, carla_color_t color) {
  auto image =
      const_cast<ImageTmpl<Color> *>(get_image_data<ImageTmpl<Color>>(data));
  if (!image || x >= image->GetWidth() || y >= image->GetHeight()) {
    return false;
  }

  Color carla_color(color.r, color.g, color.b, color.a);
  image->at(y * image->GetWidth() + x) = carla_color;

  return true;
}

float carla_image_get_depth_at(const carla_sensor_data_t *data, uint32_t x,
                               uint32_t y) {
  auto image = get_image_data<ImageTmpl<float>>(data);
  if (!image || x >= image->GetWidth() || y >= image->GetHeight()) {
    return -1.0f;
  }

  return image->at(y * image->GetWidth() + x);
}

uint32_t carla_image_get_semantic_id_at(const carla_sensor_data_t *data,
                                        uint32_t x, uint32_t y) {
  auto image = get_image_data<ImageTmpl<Color>>(data);
  if (!image || x >= image->GetWidth() || y >= image->GetHeight()) {
    return 0;
  }

  auto color = image->at(y * image->GetWidth() + x);
  // Semantic ID is typically encoded in the red channel
  return static_cast<uint32_t>(color.r);
}

uint32_t carla_image_get_instance_id_at(const carla_sensor_data_t *data,
                                        uint32_t x, uint32_t y) {
  auto image = get_image_data<ImageTmpl<Color>>(data);
  if (!image || x >= image->GetWidth() || y >= image->GetHeight()) {
    return 0;
  }

  auto color = image->at(y * image->GetWidth() + x);
  // Instance ID is typically encoded as RGB to uint32
  return (static_cast<uint32_t>(color.r) << 16) |
         (static_cast<uint32_t>(color.g) << 8) | static_cast<uint32_t>(color.b);
}

carla_optical_flow_pixel_t
carla_image_get_optical_flow_at(const carla_sensor_data_t *data, uint32_t x,
                                uint32_t y) {
  carla_optical_flow_pixel_t result = {0.0f, 0.0f};

  if (!data || data->type != CARLA_SENSOR_DATA_IMAGE) {
    return result;
  }

  auto image =
      std::dynamic_pointer_cast<carla::sensor::data::Image>(data->cpp_data);
  if (!image || x >= image->GetWidth() || y >= image->GetHeight()) {
    return result;
  }

  // Optical flow images store flow as RGBA float pixels
  // where R=x_flow, G=y_flow, B=unused, A=unused
  const auto *pixel_data = reinterpret_cast<const float *>(image->data());
  const size_t pixel_index =
      (y * image->GetWidth() + x) * 4; // 4 channels (RGBA)

  result.x = pixel_data[pixel_index];     // R channel = x flow
  result.y = pixel_data[pixel_index + 1]; // G channel = y flow

  return result;
}

// Image statistics functions

carla_error_t carla_image_compute_stats(const carla_sensor_data_t *data,
                                        carla_image_stats_t *stats) {
  if (!data || !stats)
    return CARLA_ERROR_INVALID_ARGUMENT;

  auto image = get_image_data<ImageTmpl<Color>>(data);
  if (!image)
    return CARLA_ERROR_INVALID_ARGUMENT;

  // Initialize basic stats
  stats->width = image->GetWidth();
  stats->height = image->GetHeight();
  stats->channels = 4; // BGRA
  stats->pixel_count = stats->width * stats->height;
  stats->fov_angle = image->GetFOVAngle();
  stats->format = CARLA_IMAGE_FORMAT_COLOR_BGRA;

  // Initialize color stats
  stats->color_stats.min_r = stats->color_stats.min_g =
      stats->color_stats.min_b = 255;
  stats->color_stats.max_r = stats->color_stats.max_g =
      stats->color_stats.max_b = 0;

  uint64_t sum_r = 0, sum_g = 0, sum_b = 0;

  // Analyze all pixels
  for (uint32_t y = 0; y < stats->height; ++y) {
    for (uint32_t x = 0; x < stats->width; ++x) {
      auto color = image->at(y * image->GetWidth() + x);

      // Update min/max
      stats->color_stats.min_r = std::min(stats->color_stats.min_r, color.r);
      stats->color_stats.min_g = std::min(stats->color_stats.min_g, color.g);
      stats->color_stats.min_b = std::min(stats->color_stats.min_b, color.b);

      stats->color_stats.max_r = std::max(stats->color_stats.max_r, color.r);
      stats->color_stats.max_g = std::max(stats->color_stats.max_g, color.g);
      stats->color_stats.max_b = std::max(stats->color_stats.max_b, color.b);

      // Accumulate for average
      sum_r += color.r;
      sum_g += color.g;
      sum_b += color.b;
    }
  }

  // Calculate averages
  stats->color_stats.avg_r = static_cast<float>(sum_r) / stats->pixel_count;
  stats->color_stats.avg_g = static_cast<float>(sum_g) / stats->pixel_count;
  stats->color_stats.avg_b = static_cast<float>(sum_b) / stats->pixel_count;

  // Calculate brightness (average of RGB)
  stats->color_stats.brightness =
      (stats->color_stats.avg_r + stats->color_stats.avg_g +
       stats->color_stats.avg_b) /
      (3.0f * 255.0f);

  // Simple contrast calculation (range / max possible range)
  float r_range = stats->color_stats.max_r - stats->color_stats.min_r;
  float g_range = stats->color_stats.max_g - stats->color_stats.min_g;
  float b_range = stats->color_stats.max_b - stats->color_stats.min_b;
  stats->color_stats.contrast = (r_range + g_range + b_range) / (3.0f * 255.0f);

  return CARLA_ERROR_NONE;
}

carla_error_t carla_image_create_histogram(const carla_sensor_data_t *data,
                                           uint32_t bins,
                                           carla_image_histogram_t *histogram) {
  if (!data || !histogram || bins == 0)
    return CARLA_ERROR_INVALID_ARGUMENT;

  auto image = get_image_data<ImageTmpl<Color>>(data);
  if (!image)
    return CARLA_ERROR_INVALID_ARGUMENT;

  // Allocate histogram arrays
  histogram->bins = bins;
  histogram->red_histogram = new uint32_t[bins]();
  histogram->green_histogram = new uint32_t[bins]();
  histogram->blue_histogram = new uint32_t[bins]();
  histogram->grayscale_histogram = new uint32_t[bins]();

  // Calculate histogram
  for (uint32_t y = 0; y < image->GetHeight(); ++y) {
    for (uint32_t x = 0; x < image->GetWidth(); ++x) {
      auto color = image->at(y * image->GetWidth() + x);

      uint32_t r_bin = (color.r * (bins - 1)) / 255;
      uint32_t g_bin = (color.g * (bins - 1)) / 255;
      uint32_t b_bin = (color.b * (bins - 1)) / 255;
      uint32_t gray = static_cast<uint32_t>(
          0.299f * color.r + 0.587f * color.g + 0.114f * color.b);
      uint32_t gray_bin = (gray * (bins - 1)) / 255;

      histogram->red_histogram[r_bin]++;
      histogram->green_histogram[g_bin]++;
      histogram->blue_histogram[b_bin]++;
      histogram->grayscale_histogram[gray_bin]++;
    }
  }

  return CARLA_ERROR_NONE;
}

void carla_image_destroy_histogram(carla_image_histogram_t *histogram) {
  if (!histogram)
    return;

  delete[] histogram->red_histogram;
  delete[] histogram->green_histogram;
  delete[] histogram->blue_histogram;
  delete[] histogram->grayscale_histogram;

  histogram->red_histogram = nullptr;
  histogram->green_histogram = nullptr;
  histogram->blue_histogram = nullptr;
  histogram->grayscale_histogram = nullptr;
  histogram->bins = 0;
}

carla_error_t carla_image_analyze_roi(const carla_sensor_data_t *data,
                                      const carla_image_roi_t *roi,
                                      carla_image_stats_t *stats) {
  if (!data || !roi || !stats)
    return CARLA_ERROR_INVALID_ARGUMENT;

  auto image = get_image_data<ImageTmpl<Color>>(data);
  if (!image)
    return CARLA_ERROR_INVALID_ARGUMENT;

  // Validate ROI bounds
  if (roi->x + roi->width > image->GetWidth() ||
      roi->y + roi->height > image->GetHeight()) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  // Initialize basic stats
  stats->width = roi->width;
  stats->height = roi->height;
  stats->channels = 4; // BGRA
  stats->pixel_count = roi->width * roi->height;
  stats->fov_angle = image->GetFOVAngle();
  stats->format = CARLA_IMAGE_FORMAT_COLOR_BGRA;

  // Initialize color stats
  stats->color_stats.min_r = stats->color_stats.min_g =
      stats->color_stats.min_b = 255;
  stats->color_stats.max_r = stats->color_stats.max_g =
      stats->color_stats.max_b = 0;

  uint64_t sum_r = 0, sum_g = 0, sum_b = 0;

  // Analyze ROI pixels
  for (uint32_t y = roi->y; y < roi->y + roi->height; ++y) {
    for (uint32_t x = roi->x; x < roi->x + roi->width; ++x) {
      auto color = image->at(y * image->GetWidth() + x);

      // Update min/max
      stats->color_stats.min_r = std::min(stats->color_stats.min_r, color.r);
      stats->color_stats.min_g = std::min(stats->color_stats.min_g, color.g);
      stats->color_stats.min_b = std::min(stats->color_stats.min_b, color.b);

      stats->color_stats.max_r = std::max(stats->color_stats.max_r, color.r);
      stats->color_stats.max_g = std::max(stats->color_stats.max_g, color.g);
      stats->color_stats.max_b = std::max(stats->color_stats.max_b, color.b);

      // Accumulate for average
      sum_r += color.r;
      sum_g += color.g;
      sum_b += color.b;
    }
  }

  // Calculate averages
  stats->color_stats.avg_r = static_cast<float>(sum_r) / stats->pixel_count;
  stats->color_stats.avg_g = static_cast<float>(sum_g) / stats->pixel_count;
  stats->color_stats.avg_b = static_cast<float>(sum_b) / stats->pixel_count;

  // Calculate brightness and contrast
  stats->color_stats.brightness =
      (stats->color_stats.avg_r + stats->color_stats.avg_g +
       stats->color_stats.avg_b) /
      (3.0f * 255.0f);

  float r_range = stats->color_stats.max_r - stats->color_stats.min_r;
  float g_range = stats->color_stats.max_g - stats->color_stats.min_g;
  float b_range = stats->color_stats.max_b - stats->color_stats.min_b;
  stats->color_stats.contrast = (r_range + g_range + b_range) / (3.0f * 255.0f);

  return CARLA_ERROR_NONE;
}

// Color space conversion functions

carla_error_t carla_image_bgra_to_rgb(const carla_sensor_data_t *data,
                                      uint8_t **rgb_data, size_t *rgb_size) {
  if (!data || !rgb_data || !rgb_size)
    return CARLA_ERROR_INVALID_ARGUMENT;

  auto image = get_image_data<ImageTmpl<Color>>(data);
  if (!image)
    return CARLA_ERROR_INVALID_ARGUMENT;

  uint32_t width = image->GetWidth();
  uint32_t height = image->GetHeight();
  *rgb_size = width * height * 3; // RGB = 3 bytes per pixel
  *rgb_data = new uint8_t[*rgb_size];

  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      auto color = image->at(y * image->GetWidth() + x);
      uint32_t rgb_index = (y * width + x) * 3;

      (*rgb_data)[rgb_index + 0] = color.r; // R
      (*rgb_data)[rgb_index + 1] = color.g; // G
      (*rgb_data)[rgb_index + 2] = color.b; // B
    }
  }

  return CARLA_ERROR_NONE;
}

carla_error_t carla_image_bgra_to_grayscale(const carla_sensor_data_t *data,
                                            uint8_t **gray_data,
                                            size_t *gray_size) {
  if (!data || !gray_data || !gray_size)
    return CARLA_ERROR_INVALID_ARGUMENT;

  auto image = get_image_data<ImageTmpl<Color>>(data);
  if (!image)
    return CARLA_ERROR_INVALID_ARGUMENT;

  uint32_t width = image->GetWidth();
  uint32_t height = image->GetHeight();
  *gray_size = width * height; // Grayscale = 1 byte per pixel
  *gray_data = new uint8_t[*gray_size];

  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      auto color = image->at(y * image->GetWidth() + x);
      uint32_t gray_index = y * width + x;

      // Standard grayscale conversion formula
      (*gray_data)[gray_index] = static_cast<uint8_t>(
          0.299f * color.r + 0.587f * color.g + 0.114f * color.b);
    }
  }

  return CARLA_ERROR_NONE;
}

carla_error_t carla_image_bgra_to_hsv(const carla_sensor_data_t *data,
                                      uint8_t **hsv_data, size_t *hsv_size) {
  if (!data || !hsv_data || !hsv_size)
    return CARLA_ERROR_INVALID_ARGUMENT;

  auto image = get_image_data<ImageTmpl<Color>>(data);
  if (!image)
    return CARLA_ERROR_INVALID_ARGUMENT;

  uint32_t width = image->GetWidth();
  uint32_t height = image->GetHeight();
  *hsv_size = width * height * 3; // HSV = 3 bytes per pixel
  *hsv_data = new uint8_t[*hsv_size];

  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      auto color = image->at(y * image->GetWidth() + x);
      uint32_t hsv_index = (y * width + x) * 3;

      // Convert RGB to HSV
      float r = color.r / 255.0f;
      float g = color.g / 255.0f;
      float b = color.b / 255.0f;

      float max_val = std::max({r, g, b});
      float min_val = std::min({r, g, b});
      float delta = max_val - min_val;

      // Hue calculation
      float h = 0.0f;
      if (delta > 0.0f) {
        if (max_val == r) {
          h = 60.0f * (std::fmod((g - b) / delta, 6.0f));
        } else if (max_val == g) {
          h = 60.0f * ((b - r) / delta + 2.0f);
        } else {
          h = 60.0f * ((r - g) / delta + 4.0f);
        }
      }
      if (h < 0.0f)
        h += 360.0f;

      // Saturation calculation
      float s = (max_val == 0.0f) ? 0.0f : (delta / max_val);

      // Value calculation
      float v = max_val;

      (*hsv_data)[hsv_index + 0] = static_cast<uint8_t>(h / 2.0f); // H (0-180)
      (*hsv_data)[hsv_index + 1] =
          static_cast<uint8_t>(s * 255.0f); // S (0-255)
      (*hsv_data)[hsv_index + 2] =
          static_cast<uint8_t>(v * 255.0f); // V (0-255)
    }
  }

  return CARLA_ERROR_NONE;
}

// Semantic segmentation functions

carla_error_t
carla_image_analyze_semantic_segmentation(const carla_sensor_data_t *data,
                                          carla_semantic_analysis_t *analysis) {
  if (!data || !analysis)
    return CARLA_ERROR_INVALID_ARGUMENT;

  auto image = get_image_data<ImageTmpl<Color>>(data);
  if (!image)
    return CARLA_ERROR_INVALID_ARGUMENT;

  // Count unique semantic classes
  std::map<uint32_t, uint32_t> class_counts;
  uint32_t total_pixels = image->GetWidth() * image->GetHeight();

  for (uint32_t y = 0; y < image->GetHeight(); ++y) {
    for (uint32_t x = 0; x < image->GetWidth(); ++x) {
      auto color = image->at(y * image->GetWidth() + x);
      uint32_t class_id =
          static_cast<uint32_t>(color.r); // Semantic ID in red channel
      class_counts[class_id]++;
    }
  }

  // Allocate arrays
  analysis->class_count = class_counts.size();
  analysis->total_pixels = total_pixels;
  analysis->classes = new carla_semantic_class_info_t[analysis->class_count];
  analysis->histogram_size = 256; // Assuming 8-bit semantic IDs
  analysis->class_histogram = new uint32_t[analysis->histogram_size]();

  // Fill class information
  size_t i = 0;
  for (const auto &pair : class_counts) {
    analysis->classes[i].class_id = pair.first;
    analysis->classes[i].class_name = nullptr;   // Would need semantic mapping
    analysis->classes[i].color = {0, 0, 0, 255}; // Default color
    analysis->classes[i].pixel_count = pair.second;
    analysis->classes[i].percentage =
        (float)pair.second / total_pixels * 100.0f;

    // Fill histogram
    if (pair.first < analysis->histogram_size) {
      analysis->class_histogram[pair.first] = pair.second;
    }

    i++;
  }

  return CARLA_ERROR_NONE;
}

void carla_semantic_analysis_destroy(carla_semantic_analysis_t *analysis) {
  if (!analysis)
    return;

  delete[] analysis->classes;
  delete[] analysis->class_histogram;

  analysis->classes = nullptr;
  analysis->class_histogram = nullptr;
  analysis->class_count = 0;
  analysis->histogram_size = 0;
}

// DVS functions

const carla_dvs_event_t *carla_dvs_get_events(const carla_sensor_data_t *data,
                                              size_t *event_count) {
  if (!data || !event_count)
    return nullptr;

  auto dvs_data = get_image_data<DVSEventArray>(data);
  if (!dvs_data)
    return nullptr;

  *event_count = dvs_data->size();
  return reinterpret_cast<const carla_dvs_event_t *>(dvs_data->data());
}

carla_error_t carla_dvs_analyze_events(const carla_sensor_data_t *data,
                                       carla_dvs_analysis_t *analysis) {
  if (!data || !analysis)
    return CARLA_ERROR_INVALID_ARGUMENT;

  if (data->type != CARLA_SENSOR_DATA_DVS_EVENT_ARRAY)
    return CARLA_ERROR_INVALID_ARGUMENT;

  auto dvs_data = std::dynamic_pointer_cast<carla::sensor::data::DVSEventArray>(
      data->cpp_data);
  if (!dvs_data)
    return CARLA_ERROR_INVALID_ARGUMENT;

  analysis->total_events = static_cast<uint32_t>(dvs_data->size());
  analysis->positive_events = 0;
  analysis->negative_events = 0;

  // Analyze events
  for (const auto &event : *dvs_data) {
    if (event.pol) {
      analysis->positive_events++;
    } else {
      analysis->negative_events++;
    }
  }

  // Calculate event rate (simplified - would need timing info)
  analysis->event_rate = static_cast<float>(analysis->total_events);

  // Find active region (simplified)
  analysis->active_region = {0, 0, dvs_data->GetWidth(), dvs_data->GetHeight()};
  analysis->activity_density = static_cast<float>(analysis->total_events) /
                               (dvs_data->GetWidth() * dvs_data->GetHeight());

  return CARLA_ERROR_NONE;
}

void carla_dvs_analysis_destroy(carla_dvs_analysis_t *analysis) {
  // DVS analysis doesn't allocate additional memory currently
  (void)analysis;
}

// Memory management functions

void carla_image_free_data(uint8_t *data) { delete[] data; }

void carla_image_free_array(void *array) {
  delete[] static_cast<uint8_t *>(array);
}

// Simplified implementations for remaining functions
// These would need full image processing library integration for complete
// implementation

carla_error_t carla_image_gaussian_blur(const carla_sensor_data_t *input,
                                        float sigma, uint8_t **output_data,
                                        size_t *output_size) {
  // Placeholder implementation
  (void)input;
  (void)sigma;
  (void)output_data;
  (void)output_size;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t carla_image_save_to_file(const carla_sensor_data_t *data,
                                       const char *filename,
                                       carla_image_export_format_t format) {
  // Placeholder implementation
  (void)data;
  (void)filename;
  (void)format;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

// Additional placeholder implementations for functions requiring complex image
// processing libraries These would be implemented with libraries like OpenCV,
// FREEIMAGE, etc.

carla_error_t carla_image_resize(const carla_sensor_data_t *input,
                                 uint32_t new_width, uint32_t new_height,
                                 uint8_t **output_data, size_t *output_size) {
  (void)input;
  (void)new_width;
  (void)new_height;
  (void)output_data;
  (void)output_size;
  return CARLA_ERROR_NOT_FOUND;
}

carla_error_t carla_image_edge_detection(const carla_sensor_data_t *input,
                                         float threshold, uint8_t **output_data,
                                         size_t *output_size) {
  (void)input;
  (void)threshold;
  (void)output_data;
  (void)output_size;
  return CARLA_ERROR_NOT_FOUND;
}

carla_error_t carla_image_to_png(const carla_sensor_data_t *data,
                                 uint8_t **png_data, size_t *png_size) {
  (void)data;
  (void)png_data;
  (void)png_size;
  return CARLA_ERROR_NOT_FOUND;
}

carla_error_t
carla_image_depth_to_point_cloud(const carla_sensor_data_t *depth_data,
                                 const carla_camera_intrinsics_t *intrinsics,
                                 carla_vector3d_t **points,
                                 carla_color_t **colors, size_t *point_count) {
  (void)depth_data;
  (void)intrinsics;
  (void)points;
  (void)colors;
  (void)point_count;
  return CARLA_ERROR_NOT_FOUND;
}

carla_error_t
carla_image_analyze_instance_segmentation(const carla_sensor_data_t *data,
                                          carla_instance_analysis_t *analysis) {
  (void)data;
  (void)analysis;
  return CARLA_ERROR_NOT_FOUND;
}

carla_error_t carla_image_analyze_depth(const carla_sensor_data_t *data,
                                        carla_depth_analysis_t *analysis) {
  (void)data;
  (void)analysis;
  return CARLA_ERROR_NOT_FOUND;
}

carla_error_t
carla_image_analyze_optical_flow(const carla_sensor_data_t *data,
                                 carla_optical_flow_analysis_t *analysis) {
  if (!data || !analysis)
    return CARLA_ERROR_INVALID_ARGUMENT;

  if (data->type != CARLA_SENSOR_DATA_IMAGE)
    return CARLA_ERROR_INVALID_ARGUMENT;

  auto image =
      std::dynamic_pointer_cast<carla::sensor::data::Image>(data->cpp_data);
  if (!image)
    return CARLA_ERROR_INVALID_ARGUMENT;

  const auto *pixel_data = reinterpret_cast<const float *>(image->data());
  const uint32_t width = image->GetWidth();
  const uint32_t height = image->GetHeight();
  const size_t total_pixels = width * height;

  // Initialize analysis results
  analysis->average_flow = {0, 0, 0};
  analysis->magnitude_avg = 0.0f;
  analysis->magnitude_max = 0.0f;
  analysis->moving_pixels = 0;
  analysis->motion_density = 0.0f;
  analysis->dominant_direction = {0, 0, 0};

  if (total_pixels == 0) {
    return CARLA_ERROR_NONE;
  }

  float total_flow_x = 0.0f;
  float total_flow_y = 0.0f;
  float total_magnitude = 0.0f;
  const float motion_threshold =
      0.5f; // Threshold for considering a pixel as "moving"

  // Process each pixel
  for (size_t i = 0; i < total_pixels; ++i) {
    const size_t pixel_index = i * 4; // 4 channels (RGBA)
    const float flow_x = pixel_data[pixel_index];
    const float flow_y = pixel_data[pixel_index + 1];

    const float magnitude = std::sqrt(flow_x * flow_x + flow_y * flow_y);

    total_flow_x += flow_x;
    total_flow_y += flow_y;
    total_magnitude += magnitude;

    if (magnitude > analysis->magnitude_max) {
      analysis->magnitude_max = magnitude;
    }

    if (magnitude > motion_threshold) {
      analysis->moving_pixels++;
    }
  }

  // Calculate averages
  analysis->average_flow.x = total_flow_x / total_pixels;
  analysis->average_flow.y = total_flow_y / total_pixels;
  analysis->magnitude_avg = total_magnitude / total_pixels;
  analysis->motion_density =
      static_cast<float>(analysis->moving_pixels) / total_pixels;

  // Calculate dominant direction (normalized average flow)
  const float avg_magnitude =
      std::sqrt(analysis->average_flow.x * analysis->average_flow.x +
                analysis->average_flow.y * analysis->average_flow.y);
  if (avg_magnitude > 0) {
    analysis->dominant_direction.x = analysis->average_flow.x / avg_magnitude;
    analysis->dominant_direction.y = analysis->average_flow.y / avg_magnitude;
    analysis->dominant_direction.z = 0; // 2D flow, no Z component
  }

  return CARLA_ERROR_NONE;
}

void carla_instance_analysis_destroy(carla_instance_analysis_t *analysis) {
  (void)analysis;
}

void carla_depth_analysis_destroy(carla_depth_analysis_t *analysis) {
  (void)analysis;
}

void carla_optical_flow_analysis_destroy(
    carla_optical_flow_analysis_t *analysis) {
  (void)analysis;
}
