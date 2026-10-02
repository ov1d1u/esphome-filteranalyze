#include "fill_level.h"

#include <algorithm>
#include <cmath>

#include <esp_heap_caps.h>
#include <img_converters.h>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::fill_level {

static const char *const TAG = "fill_level";

void FillLevel::setup() { this->camera_->add_listener(this); }

void FillLevel::loop() {
  const uint32_t now = millis();
  if (this->state_ != CaptureState::IDLE &&
      now - this->started_at_ms_ > this->stabilization_delay_ms_ + 15000) {
    ESP_LOGW(TAG, "Timed out waiting for a camera frame");
    this->publish_unavailable();
    this->finish_measurement();
    return;
  }
  if (this->state_ == CaptureState::REQUEST_NEXT) {
    // The camera can hold a frame captured before the light switched on.
    // Drain it and let auto exposure settle before taking the final frame.
    if (now - this->started_at_ms_ >= this->stabilization_delay_ms_ && this->warmup_frames_ >= 2)
      this->state_ = CaptureState::CAPTURING;
    else
      this->state_ = CaptureState::WARMING;
    this->camera_->request_image(camera::IDLE);
    return;
  }
  if (this->state_ == CaptureState::IDLE &&
      (!this->started_once_ || now - this->started_at_ms_ >= this->interval_ms_))
    this->start_measurement();
}

void FillLevel::start_measurement() {
  if (this->state_ != CaptureState::IDLE || this->camera_->is_failed())
    return;
  this->owns_flashlight_ = !this->flashlight_->remote_values.is_on();
  this->flashlight_->turn_on().perform();
  this->started_at_ms_ = millis();
  this->started_once_ = true;
  this->warmup_frames_ = 0;
  this->state_ = CaptureState::WARMING;
  this->camera_->request_image(camera::IDLE);
  ESP_LOGD(TAG, "Started illuminated measurement");
}

void FillLevel::on_stream_stop() {
  this->streaming_ = false;
  if (this->state_ != CaptureState::IDLE) {
    // The camera's stream-stop automation also switches this light off.
    this->owns_flashlight_ = true;
    this->flashlight_->turn_on().perform();
  }
}

void FillLevel::finish_measurement() {
  this->state_ = CaptureState::IDLE;
  if (this->owns_flashlight_ && !this->streaming_)
    this->flashlight_->turn_off().perform();
  this->owns_flashlight_ = false;
}

void FillLevel::publish_unavailable() {
  this->fill_sensor_->publish_state(NAN);
  if (this->confidence_sensor_ != nullptr)
    this->confidence_sensor_->publish_state(NAN);
}

void FillLevel::dump_config() {
  ESP_LOGCONFIG(TAG, "Fill Level:");
  ESP_LOGCONFIG(TAG, "  Templates: %u", static_cast<unsigned>(this->templates_.size()));
  ESP_LOGCONFIG(TAG, "  Minimum match confidence: %.2f", this->min_confidence_);
  ESP_LOGCONFIG(TAG, "  Interval: %u ms", static_cast<unsigned>(this->interval_ms_));
  ESP_LOGCONFIG(TAG, "  Stabilization delay: %u ms", static_cast<unsigned>(this->stabilization_delay_ms_));
  LOG_SENSOR("  ", "Fill percentage", this->fill_sensor_);
  LOG_SENSOR("  ", "Match confidence", this->confidence_sensor_);
}

// TM_CCOEFF_NORMED on sampled grayscale pixels. The last pass uses every pixel,
// matching OpenCV's metric; earlier passes make the search affordable on ESP32.
float FillLevel::score_at(const uint8_t *rgb, int frame_width, const TemplateImage &tmpl, int x, int y,
                          int step) const {
  uint64_t sum_frame = 0, sum_template = 0, sum_frame2 = 0, sum_template2 = 0, sum_cross = 0;
  uint32_t n = 0;
  for (int ty = 0; ty < tmpl.height; ty += step) {
    const uint8_t *frame_row = rgb + 3 * ((y + ty) * frame_width + x);
    const uint8_t *template_row = tmpl.pixels + ty * tmpl.width;
    for (int tx = 0; tx < tmpl.width; tx += step) {
      const uint8_t *p = frame_row + 3 * tx;
      // cvtColor(BGR2GRAY) rounds these weighted RGB channels to 8 bits.
      const uint32_t a = (77U * p[0] + 150U * p[1] + 29U * p[2] + 128U) >> 8;
      const uint32_t b = template_row[tx];
      sum_frame += a;
      sum_template += b;
      sum_frame2 += a * a;
      sum_template2 += b * b;
      sum_cross += a * b;
      n++;
    }
  }
  const double covariance = double(n) * double(sum_cross) - double(sum_frame) * double(sum_template);
  const double frame_variance = double(n) * double(sum_frame2) - double(sum_frame) * double(sum_frame);
  const double template_variance = double(n) * double(sum_template2) - double(sum_template) * double(sum_template);
  if (frame_variance <= 0 || template_variance <= 0)
    return -1.0f;
  return static_cast<float>(covariance / sqrt(frame_variance * template_variance));
}

FillLevel::Match FillLevel::find_template(const uint8_t *rgb, int frame_width, int frame_height) const {
  Match best;
  for (const auto &tmpl : this->templates_) {
    const int max_x = frame_width - tmpl.width;
    const int max_y = frame_height - tmpl.height;
    if (max_x < 0 || max_y < 0)
      continue;

    Match coarse;
    for (int y = 0; y <= max_y; y += 8) {
      App.feed_wdt();
      for (int x = 0; x <= max_x; x += 8) {
        float score = this->score_at(rgb, frame_width, tmpl, x, y, 8);
        if (score > coarse.score)
          coarse = {score, x, y, &tmpl};
      }
    }
    if (coarse.image == nullptr)
      continue;

    Match medium;
    for (int y = std::max(0, coarse.y - 8); y <= std::min(max_y, coarse.y + 8); y += 2) {
      App.feed_wdt();
      for (int x = std::max(0, coarse.x - 8); x <= std::min(max_x, coarse.x + 8); x += 2) {
        float score = this->score_at(rgb, frame_width, tmpl, x, y, 2);
        if (score > medium.score)
          medium = {score, x, y, &tmpl};
      }
    }
    Match fine;
    for (int y = std::max(0, medium.y - 1); y <= std::min(max_y, medium.y + 1); y++) {
      App.feed_wdt();
      for (int x = std::max(0, medium.x - 1); x <= std::min(max_x, medium.x + 1); x++) {
        float score = this->score_at(rgb, frame_width, tmpl, x, y, 1);
        if (score > fine.score)
          fine = {score, x, y, &tmpl};
      }
    }
    if (fine.score > best.score)
      best = fine;
  }
  return best;
}

static bool is_target_color(const uint8_t *rgb) {
  const int r = rgb[0], g = rgb[1], b = rgb[2];
  const int high = std::max({r, g, b});
  const int low = std::min({r, g, b});
  const int delta = high - low;
  if (high < 65 || high == 0 || delta * 255 < 40 * high)
    return false;
  float hue;
  if (high == r)
    hue = 30.0f * (g - b) / delta;
  else if (high == g)
    hue = 60.0f + 30.0f * (b - r) / delta;
  else
    hue = 120.0f + 30.0f * (r - g) / delta;
  if (hue < 0)
    hue += 180.0f;
  return hue >= 60.0f && hue <= 100.0f;
}

float FillLevel::measure_fill(const uint8_t *rgb, int frame_width, int frame_height, const Match &match) const {
  const int x = match.x + match.image->width / 2 + 15;
  const int y = match.y + 120;
  const int width = 5;
  const int height = match.image->height - 120;
  if (height <= 0 || x < 0 || y < 0 || x + width > frame_width || y + height > frame_height)
    return NAN;

  std::vector<uint8_t> mask(width * height), scratch(width * height);
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++)
      mask[row * width + col] = is_target_color(rgb + 3 * ((y + row) * frame_width + x + col));
  }

  // OpenCV's default morphology border is neutral: outside is filled for
  // erosion and empty for dilation.
  auto morph = [&](bool erode) {
    for (int row = 0; row < height; row++) {
      for (int col = 0; col < width; col++) {
        bool value = erode;
        for (int dy = -2; dy <= 2; dy++) {
          for (int dx = -2; dx <= 2; dx++) {
            int yy = row + dy, xx = col + dx;
            bool sample = yy < 0 || yy >= height || xx < 0 || xx >= width
                              ? erode
                              : mask[yy * width + xx] != 0;
            if (erode)
              value &= sample;
            else
              value |= sample;
          }
        }
        scratch[row * width + col] = value;
      }
    }
    mask.swap(scratch);
  };
  morph(true);
  morph(true);
  morph(false);
  morph(false);
  morph(false);
  morph(true);

  std::vector<uint8_t> occupied_rows(height, 0);
  std::vector<int> queue;
  queue.reserve(width * height);
  for (int start = 0; start < width * height; start++) {
    if (mask[start] != 1)
      continue;
    queue.clear();
    queue.push_back(start);
    mask[start] = 2;
    for (size_t head = 0; head < queue.size(); head++) {
      int pos = queue[head];
      int row = pos / width, col = pos % width;
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          int yy = row + dy, xx = col + dx;
          if (yy >= 0 && yy < height && xx >= 0 && xx < width && mask[yy * width + xx] == 1) {
            mask[yy * width + xx] = 2;
            queue.push_back(yy * width + xx);
          }
        }
      }
    }
    if (queue.size() >= this->min_blob_area_) {
      for (int pos : queue)
        occupied_rows[pos / width] = 1;
    }
  }

  int filled_rows = 0;
  bool found = false;
  for (int row = height - 1; row >= 0; row--) {
    if (occupied_rows[row]) {
      found = true;
      filled_rows++;
    } else if (found) {
      break;
    }
  }
  return 100.0f * filled_rows / height;
}

void FillLevel::on_camera_image(const std::shared_ptr<camera::CameraImage> &image) {
  if (this->state_ == CaptureState::WARMING) {
    this->warmup_frames_++;
    this->state_ = CaptureState::REQUEST_NEXT;
    return;
  }
  if (this->state_ != CaptureState::CAPTURING)
    return;

  auto esp_image = std::static_pointer_cast<esp32_camera::ESP32CameraImage>(image);
  camera_fb_t *frame = esp_image->get_raw_buffer();
  if (frame->format != PIXFORMAT_JPEG || frame->width > 640 || frame->height > 480) {
    ESP_LOGW(TAG, "Expected a JPEG frame no larger than 640x480");
    this->publish_unavailable();
    this->finish_measurement();
    return;
  }
  const size_t rgb_size = size_t(frame->width) * frame->height * 3;
  uint8_t *rgb = static_cast<uint8_t *>(heap_caps_malloc(rgb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (rgb == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate %u bytes in PSRAM", static_cast<unsigned>(rgb_size));
    this->publish_unavailable();
    this->finish_measurement();
    return;
  }
  if (!fmt2rgb888(frame->buf, frame->len, PIXFORMAT_JPEG, rgb)) {
    ESP_LOGE(TAG, "JPEG decoding failed");
    heap_caps_free(rgb);
    this->publish_unavailable();
    this->finish_measurement();
    return;
  }
  Match best = this->find_template(rgb, frame->width, frame->height);
  if (this->confidence_sensor_ != nullptr)
    this->confidence_sensor_->publish_state(best.image == nullptr ? NAN : best.score);
  if (best.image == nullptr || best.score < this->min_confidence_) {
    ESP_LOGW(TAG, "No reliable template match (score %.2f)", best.score);
    this->fill_sensor_->publish_state(NAN);
  } else {
    float fill = this->measure_fill(rgb, frame->width, frame->height, best);
    ESP_LOGD(TAG, "Match %.2f at (%d,%d), fill %.2f%%", best.score, best.x, best.y, fill);
    if (!std::isfinite(fill)) {
      this->fill_sensor_->publish_state(NAN);
    } else {
      const float current = this->fill_sensor_->state;
      if (!this->fill_sensor_->has_state() || !std::isfinite(current) || fill < current || fill - current > 5.0f) {
        this->fill_sensor_->publish_state(fill);
      } else {
        ESP_LOGD(TAG, "Keeping fill %.2f%%; measured increase to %.2f%% is at most 5 points", current, fill);
      }
    }
  }
  heap_caps_free(rgb);
  this->finish_measurement();
}

}  // namespace esphome::fill_level
