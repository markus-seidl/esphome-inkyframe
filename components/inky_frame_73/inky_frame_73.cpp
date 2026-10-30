#include "inky_frame_73.h"

#include <algorithm>
#include <cinttypes>

#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::inky_frame_73 {

static const char *const TAG = "inky_frame_73";

// Controller registers (same names as the Pimoroni driver)
static constexpr uint8_t REG_PSR = 0x00;
static constexpr uint8_t REG_PWR = 0x01;
static constexpr uint8_t REG_POF = 0x02;
static constexpr uint8_t REG_PFS = 0x03;
static constexpr uint8_t REG_PON = 0x04;
static constexpr uint8_t REG_BTST1 = 0x05;
static constexpr uint8_t REG_BTST2 = 0x06;
static constexpr uint8_t REG_BTST3 = 0x08;
static constexpr uint8_t REG_DTM1 = 0x10;
static constexpr uint8_t REG_DRF = 0x12;
static constexpr uint8_t REG_IPC = 0x13;
static constexpr uint8_t REG_PLL = 0x30;
static constexpr uint8_t REG_TSE = 0x41;
static constexpr uint8_t REG_CDI = 0x50;
static constexpr uint8_t REG_TCON = 0x60;
static constexpr uint8_t REG_TRES = 0x61;
static constexpr uint8_t REG_VDCS = 0x82;
static constexpr uint8_t REG_T_VDCS = 0x84;
static constexpr uint8_t REG_AGID = 0x86;
static constexpr uint8_t REG_CMDH = 0xAA;
static constexpr uint8_t REG_CCSET = 0xE0;
static constexpr uint8_t REG_PWS = 0xE3;
static constexpr uint8_t REG_TSSET = 0xE6;

static constexpr uint32_t RESET_PULSE_MS = 10;
static constexpr uint32_t MAX_TRANSFER_TIME_MS = 10;  // transfer in slices so the main loop keeps running
static constexpr size_t TRANSFER_CHUNK_SIZE = 1024;

struct PaletteEntry {
  uint8_t native;  // colour code understood by the controller (4 is unused on Spectra 6 panels)
  uint8_t r, g, b;
};
static constexpr PaletteEntry PALETTE[] = {
    {INKY_BLACK, 0, 0, 0},
    {INKY_WHITE, 255, 255, 255},
    {INKY_YELLOW, 255, 255, 0},
    {INKY_RED, 255, 0, 0},
    {INKY_BLUE, 0, 0, 255},
    {INKY_GREEN, 0, 255, 0},
};

uint8_t InkyFrame73::color_to_native_(Color color) {
  uint32_t best_distance = UINT32_MAX;
  uint8_t best = INKY_BLACK;
  for (const auto &entry : PALETTE) {
    const int dr = (int) color.r - entry.r;
    const int dg = (int) color.g - entry.g;
    const int db = (int) color.b - entry.b;
    const uint32_t distance = dr * dr + dg * dg + db * db;
    if (distance < best_distance) {
      best_distance = distance;
      best = entry.native;
    }
  }
  return best;
}

void InkyFrame73::setup() {
  if (!this->buffer_.init(BUFFER_LENGTH)) {
    this->mark_failed(LOG_STR("Failed to allocate frame buffer"));
    return;
  }
  this->clear();

  this->dc_pin_->setup();
  this->dc_pin_->digital_write(false);
  this->reset_pin_->setup();
  this->reset_pin_->digital_write(true);

  this->sr_clock_pin_->setup();
  this->sr_clock_pin_->digital_write(false);
  this->sr_latch_pin_->setup();
  this->sr_latch_pin_->digital_write(true);
  this->sr_data_pin_->setup();

  this->spi_setup();
}

void InkyFrame73::dump_config() {
  LOG_DISPLAY("", "Pimoroni Inky Frame 7.3\"", this);
  ESP_LOGCONFIG(TAG,
                "  SPI Data Rate: %uMHz\n"
                "  Busy timeout: %" PRIu32 " ms",
                (unsigned) (this->data_rate_ / 1000000), this->busy_timeout_ms_);
  LOG_PIN("  CS Pin: ", this->cs_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  Shift register clock Pin: ", this->sr_clock_pin_);
  LOG_PIN("  Shift register latch Pin: ", this->sr_latch_pin_);
  LOG_PIN("  Shift register data Pin: ", this->sr_data_pin_);
  LOG_UPDATE_INTERVAL(this);
}

uint8_t InkyFrame73::read_shift_register() {
  // Parallel load on the falling latch edge, then shift out MSB first: the first bit is the BUSY line.
  this->sr_latch_pin_->digital_write(false);
  delay_microseconds_safe(1);
  this->sr_latch_pin_->digital_write(true);
  delay_microseconds_safe(1);
  uint8_t out = 0;
  for (uint8_t i = 0; i < 8; i++) {
    out = (out << 1) | (this->sr_data_pin_->digital_read() ? 1 : 0);
    this->sr_clock_pin_->digital_write(true);
    delay_microseconds_safe(1);
    this->sr_clock_pin_->digital_write(false);
    delay_microseconds_safe(1);
  }
  return out;
}

void InkyFrame73::command_(uint8_t reg) { this->command_(reg, nullptr, 0); }

void InkyFrame73::command_(uint8_t reg, const uint8_t *data, size_t length) {
  ESP_LOGV(TAG, "Command: 0x%02X, length %u", reg, (unsigned) length);
  this->dc_pin_->digital_write(false);
  this->enable();
  this->write_byte(reg);
  if (length > 0) {
    this->dc_pin_->digital_write(true);
    this->write_array(data, length);
  }
  this->disable();
}

void InkyFrame73::send_init_sequence_() {
  this->command_(REG_CMDH, {0x49, 0x55, 0x20, 0x08, 0x09, 0x18});
  this->command_(REG_PWR, {0x3F, 0x00, 0x32, 0x2A, 0x0E, 0x2A});
  // 0x53 scans the panel the way the Inky Frame is mounted (the Pimoroni driver's default rotation).
  this->command_(REG_PSR, {0x53, 0x69});
  this->command_(REG_PFS, {0x00, 0x54, 0x00, 0x44});
  this->command_(REG_BTST1, {0x40, 0x1F, 0x1F, 0x2C});
  this->command_(REG_BTST2, {0x6F, 0x1F, 0x16, 0x25});
  this->command_(REG_BTST3, {0x6F, 0x1F, 0x1F, 0x22});
  this->command_(REG_IPC, {0x00, 0x04});
  this->command_(REG_PLL, {0x02});
  this->command_(REG_TSE, {0x00});
  this->command_(REG_CDI, {0x3F});
  this->command_(REG_TCON, {0x02, 0x00});
  this->command_(REG_TRES, {WIDTH >> 8, WIDTH & 0xFF, HEIGHT >> 8, HEIGHT & 0xFF});
  this->command_(REG_VDCS, {0x1E});
  this->command_(REG_T_VDCS, {0x00});
  this->command_(REG_AGID, {0x00});
  this->command_(REG_PWS, {0x2F});
  this->command_(REG_CCSET, {0x00});
  this->command_(REG_TSSET, {0x00});
}

bool InkyFrame73::transfer_data_() {
  const uint32_t start_time = millis();
  if (this->transfer_index_ == 0)
    this->command_(REG_DTM1);

  while (this->transfer_index_ < BUFFER_LENGTH) {
    size_t span_length = 0;
    const uint8_t *data = this->buffer_.get_span(this->transfer_index_, span_length);
    const size_t chunk = std::min({span_length, TRANSFER_CHUNK_SIZE, BUFFER_LENGTH - this->transfer_index_});
    if (data == nullptr || chunk == 0)
      break;
    this->dc_pin_->digital_write(true);
    this->enable();
    this->write_array(data, chunk);
    this->disable();
    this->transfer_index_ += chunk;
    if (this->transfer_index_ < BUFFER_LENGTH && millis() - start_time > MAX_TRANSFER_TIME_MS)
      return false;  // come back next loop
  }
  this->transfer_index_ = 0;
  return true;
}

void InkyFrame73::update() {
  if (this->state_ != UpdateState::UPDATE_STATE_IDLE) {
    ESP_LOGE(TAG, "Display already in state %s", this->state_to_string_());
    return;
  }
  this->set_state_(UpdateState::UPDATE_STATE_UPDATE);
  this->enable_loop();
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  this->update_start_time_ = millis();
#endif
}

void InkyFrame73::loop() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - this->delay_until_) < 0)
    return;
  if (this->waiting_for_idle_) {
    if (this->is_idle_()) {
      ESP_LOGV(TAG, "Panel was busy for %" PRIu32 " ms", now - this->wait_started_);
      this->waiting_for_idle_ = false;
    } else if (now - this->wait_started_ > this->busy_timeout_ms_) {
      ESP_LOGW(TAG, "Panel still busy after %" PRIu32 " ms in state %s, continuing", this->busy_timeout_ms_,
               this->state_to_string_());
      this->waiting_for_idle_ = false;
    } else {
      return;
    }
  }
  this->process_state_();
}

/**
 * IDLE -> UPDATE -> RESET -> RESET_END -> INITIALISE -> TRANSFER_DATA -> POWER_ON -> REFRESH -> POWER_OFF -> FINISH
 * -> IDLE. The states that wait for idle mirror the busy_wait() calls of the Pimoroni driver.
 */
void InkyFrame73::process_state_() {
  switch (this->state_) {
    case UpdateState::UPDATE_STATE_IDLE:
      this->disable_loop();
      break;
    case UpdateState::UPDATE_STATE_UPDATE:
      this->do_update_();
      this->set_state_(UpdateState::UPDATE_STATE_RESET);
      break;
    case UpdateState::UPDATE_STATE_RESET:
      this->reset_pin_->digital_write(false);
      this->set_state_(UpdateState::UPDATE_STATE_RESET_END, RESET_PULSE_MS);
      break;
    case UpdateState::UPDATE_STATE_RESET_END:
      this->reset_pin_->digital_write(true);
      this->set_state_(UpdateState::UPDATE_STATE_INITIALISE, RESET_PULSE_MS);
      break;
    case UpdateState::UPDATE_STATE_INITIALISE:
      this->send_init_sequence_();
      this->set_state_(UpdateState::UPDATE_STATE_TRANSFER_DATA);
      break;
    case UpdateState::UPDATE_STATE_TRANSFER_DATA:
      if (!this->transfer_data_())
        return;
      this->set_state_(UpdateState::UPDATE_STATE_POWER_ON);
      break;
    case UpdateState::UPDATE_STATE_POWER_ON:
      this->command_(REG_PON, {0x00});
      this->set_state_(UpdateState::UPDATE_STATE_REFRESH);
      break;
    case UpdateState::UPDATE_STATE_REFRESH:
      this->command_(REG_DRF, {0x00});
      this->set_state_(UpdateState::UPDATE_STATE_POWER_OFF);
      break;
    case UpdateState::UPDATE_STATE_POWER_OFF:
      this->command_(REG_POF);
      this->set_state_(UpdateState::UPDATE_STATE_FINISH);
      break;
    case UpdateState::UPDATE_STATE_FINISH:
      this->command_(REG_PSR, {0x4F, 0x6E});
      this->set_state_(UpdateState::UPDATE_STATE_IDLE);
      ESP_LOGD(TAG, "Display update took %" PRIu32 " ms", millis() - this->update_start_time_);
      break;
  }
}

void InkyFrame73::set_state_(UpdateState state, uint32_t delay_ms) {
  ESP_LOGV(TAG, "Exit state %s", this->state_to_string_());
  this->state_ = state;
  this->delay_until_ = millis() + delay_ms;
  this->wait_started_ = this->delay_until_;
  switch (state) {
    case UpdateState::UPDATE_STATE_INITIALISE:
    case UpdateState::UPDATE_STATE_POWER_ON:
    case UpdateState::UPDATE_STATE_REFRESH:
    case UpdateState::UPDATE_STATE_POWER_OFF:
    case UpdateState::UPDATE_STATE_FINISH:
      this->waiting_for_idle_ = true;
      break;
    default:
      this->waiting_for_idle_ = false;
      break;
  }
  ESP_LOGV(TAG, "Enter state %s, delay %" PRIu32 ", wait_for_idle=%s", this->state_to_string_(), delay_ms,
           TRUEFALSE(this->waiting_for_idle_));
  if (state == UpdateState::UPDATE_STATE_TRANSFER_DATA) {
    this->high_frequency_loop_.start();
  } else {
    this->high_frequency_loop_.stop();
  }
}

const char *InkyFrame73::state_to_string_() const {
  switch (this->state_) {
    case UpdateState::UPDATE_STATE_IDLE:
      return "IDLE";
    case UpdateState::UPDATE_STATE_UPDATE:
      return "UPDATE";
    case UpdateState::UPDATE_STATE_RESET:
      return "RESET";
    case UpdateState::UPDATE_STATE_RESET_END:
      return "RESET_END";
    case UpdateState::UPDATE_STATE_INITIALISE:
      return "INITIALISE";
    case UpdateState::UPDATE_STATE_TRANSFER_DATA:
      return "TRANSFER_DATA";
    case UpdateState::UPDATE_STATE_POWER_ON:
      return "POWER_ON";
    case UpdateState::UPDATE_STATE_REFRESH:
      return "REFRESH";
    case UpdateState::UPDATE_STATE_POWER_OFF:
      return "POWER_OFF";
    case UpdateState::UPDATE_STATE_FINISH:
      return "FINISH";
  }
  return "UNKNOWN";
}

void InkyFrame73::fill(Color color) {
  if (this->get_clipping().is_set()) {
    Display::fill(color);
    return;
  }
  const uint8_t native = color_to_native_(color);
  this->buffer_.fill(native | (native << 4));
}

bool InkyFrame73::transform_coordinates_(int &x, int &y) const {
  if (this->is_point_clipped(x, y))
    return false;
  switch (this->rotation_) {
    case display::DISPLAY_ROTATION_90_DEGREES: {
      const int native_y = x;
      x = WIDTH - 1 - y;
      y = native_y;
      break;
    }
    case display::DISPLAY_ROTATION_180_DEGREES:
      x = WIDTH - 1 - x;
      y = HEIGHT - 1 - y;
      break;
    case display::DISPLAY_ROTATION_270_DEGREES: {
      const int native_x = y;
      y = HEIGHT - 1 - x;
      x = native_x;
      break;
    }
    default:
      break;
  }
  return x >= 0 && y >= 0 && x < WIDTH && y < HEIGHT;
}

void HOT InkyFrame73::draw_pixel_at(int x, int y, Color color) {
  if (!this->transform_coordinates_(x, y))
    return;
  const uint8_t native = color_to_native_(color);
  const size_t pixel_position = (size_t) y * WIDTH + x;
  const size_t byte_position = pixel_position / 2;
  const uint8_t original = this->buffer_[byte_position];
  if (pixel_position & 1) {
    this->buffer_[byte_position] = (original & 0xF0) | native;
  } else {
    this->buffer_[byte_position] = (original & 0x0F) | (native << 4);
  }
}

}  // namespace esphome::inky_frame_73
