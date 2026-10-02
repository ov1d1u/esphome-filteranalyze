#pragma once

#include <memory>
#include <vector>

#include "esphome/components/camera/camera.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome::fill_level {

struct TemplateImage {
  const uint8_t *pixels;
  uint16_t width;
  uint16_t height;
};

class FillLevel : public Component, public camera::CameraListener {
 public:
  void set_camera(esp32_camera::ESP32Camera *camera) { this->camera_ = camera; }
  void set_flashlight(light::LightState *flashlight) { this->flashlight_ = flashlight; }
  void set_fill_sensor(sensor::Sensor *sensor) { this->fill_sensor_ = sensor; }
  void set_confidence_sensor(sensor::Sensor *sensor) { this->confidence_sensor_ = sensor; }
  void set_min_confidence(float value) { this->min_confidence_ = value; }
  void set_min_blob_area(uint16_t value) { this->min_blob_area_ = value; }
  void set_interval(uint32_t value) { this->interval_ms_ = value; }
  void set_stabilization_delay(uint32_t value) { this->stabilization_delay_ms_ = value; }
  void add_template(const uint8_t *pixels, uint16_t width, uint16_t height) {
    this->templates_.push_back({pixels, width, height});
  }

  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_camera_image(const std::shared_ptr<camera::CameraImage> &image) override;
  void on_stream_start() override { this->streaming_ = true; }
  void on_stream_stop() override;
  void start_measurement();

 protected:
  struct Match {
    float score{-1.0f};
    int x{0};
    int y{0};
    const TemplateImage *image{nullptr};
  };

  float score_at(const uint8_t *rgb, int frame_width, const TemplateImage &tmpl, int x, int y, int step) const;
  Match find_template(const uint8_t *rgb, int frame_width, int frame_height) const;
  float measure_fill(const uint8_t *rgb, int frame_width, int frame_height, const Match &match) const;
  void finish_measurement();
  void publish_unavailable();

  enum class CaptureState : uint8_t { IDLE, WARMING, REQUEST_NEXT, CAPTURING };

  esp32_camera::ESP32Camera *camera_{nullptr};
  light::LightState *flashlight_{nullptr};
  sensor::Sensor *fill_sensor_{nullptr};
  sensor::Sensor *confidence_sensor_{nullptr};
  std::vector<TemplateImage> templates_;
  float min_confidence_{0.4f};
  uint16_t min_blob_area_{150};
  uint32_t interval_ms_{43200000};
  uint32_t stabilization_delay_ms_{3000};
  uint32_t started_at_ms_{0};
  uint8_t warmup_frames_{0};
  CaptureState state_{CaptureState::IDLE};
  bool started_once_{false};
  bool owns_flashlight_{false};
  bool streaming_{false};
};

}  // namespace esphome::fill_level
