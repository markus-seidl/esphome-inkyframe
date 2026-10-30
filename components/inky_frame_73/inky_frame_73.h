#pragma once

#include <initializer_list>

#include "esphome/components/display/display.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/split_buffer/split_buffer.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

namespace esphome::inky_frame_73 {

enum class UpdateState : uint8_t {
  UPDATE_STATE_IDLE,           // not doing anything
  UPDATE_STATE_UPDATE,         // run the user lambda / pages to fill the buffer
  UPDATE_STATE_RESET,          // drive reset low (active)
  UPDATE_STATE_RESET_END,      // drive reset high (inactive)
  UPDATE_STATE_INITIALISE,     // wait for idle, then send the init sequence
  UPDATE_STATE_TRANSFER_DATA,  // push the frame buffer to the panel
  UPDATE_STATE_POWER_ON,       // wait for idle, then power the panel on
  UPDATE_STATE_REFRESH,        // wait for idle, then start the (slow) refresh
  UPDATE_STATE_POWER_OFF,      // wait for the refresh to finish, then power the panel off
  UPDATE_STATE_FINISH,         // wait for idle, then leave the controller in its resting configuration
};

/// Native colour codes of the Spectra 6 panel (code 4 is missing/non-functional on the panel).
enum InkyColor : uint8_t {
  INKY_BLACK = 0,
  INKY_WHITE = 1,
  INKY_YELLOW = 2,
  INKY_RED = 3,
  INKY_BLUE = 5,
  INKY_GREEN = 6,
};

/// Pimoroni Inky Frame 7.3" (800x480, Spectra 6).
///
/// The panel's BUSY line is not wired to the RP2040/RP2350 directly. It is the first bit clocked out of a 74HC165
/// style parallel-in/serial-out shift register, which also carries the buttons and the RTC alarm / external trigger
/// lines.
class InkyFrame73 : public display::Display,
                    public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                          spi::DATA_RATE_20MHZ> {
 public:
  static constexpr uint16_t WIDTH = 800;
  static constexpr uint16_t HEIGHT = 480;

  // Bit layout of read_shift_register(), identical to the Pimoroni driver
  static constexpr uint8_t BUTTON_A = 1 << 0;
  static constexpr uint8_t BUTTON_B = 1 << 1;
  static constexpr uint8_t BUTTON_C = 1 << 2;
  static constexpr uint8_t BUTTON_D = 1 << 3;
  static constexpr uint8_t BUTTON_E = 1 << 4;
  static constexpr uint8_t RTC_ALARM = 1 << 5;
  static constexpr uint8_t EXTERNAL_TRIGGER = 1 << 6;
  static constexpr uint8_t EINK_IDLE = 1 << 7;  // high when the panel is NOT busy

  void set_dc_pin(GPIOPin *dc_pin) { this->dc_pin_ = dc_pin; }
  void set_reset_pin(GPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }
  void set_sr_clock_pin(GPIOPin *pin) { this->sr_clock_pin_ = pin; }
  void set_sr_latch_pin(GPIOPin *pin) { this->sr_latch_pin_ = pin; }
  void set_sr_data_pin(GPIOPin *pin) { this->sr_data_pin_ = pin; }
  void set_busy_timeout(uint32_t busy_timeout_ms) { this->busy_timeout_ms_ = busy_timeout_ms; }

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  /// Latch and clock out the shift register. See the BUTTON_* / EINK_IDLE masks for the bit layout.
  uint8_t read_shift_register();
  bool is_button_pressed(uint8_t mask) { return (this->read_shift_register() & mask) != 0; }

  /// True from the start of an update until the panel has been refreshed and powered down again.
  bool is_updating() const { return this->state_ != UpdateState::UPDATE_STATE_IDLE; }

  display::DisplayType get_display_type() override { return display::DISPLAY_TYPE_COLOR; }
  void fill(Color color) override;
  void clear() override { this->fill(display::COLOR_ON); }  // white, like paper
  void draw_pixel_at(int x, int y, Color color) override;
  int get_width() override { return this->is_rotated_() ? HEIGHT : WIDTH; }
  int get_height() override { return this->is_rotated_() ? WIDTH : HEIGHT; }

 protected:
  int get_width_internal() override { return WIDTH; }
  int get_height_internal() override { return HEIGHT; }

  static constexpr size_t BUFFER_LENGTH = (size_t) WIDTH * HEIGHT / 2;  // 4 bits per pixel

  bool is_rotated_() const {
    return this->rotation_ == display::DISPLAY_ROTATION_90_DEGREES ||
           this->rotation_ == display::DISPLAY_ROTATION_270_DEGREES;
  }
  bool transform_coordinates_(int &x, int &y) const;
  static uint8_t color_to_native_(Color color);

  bool is_idle_() { return (this->read_shift_register() & EINK_IDLE) != 0; }
  void command_(uint8_t reg);
  void command_(uint8_t reg, const uint8_t *data, size_t length);
  void command_(uint8_t reg, std::initializer_list<uint8_t> data) { this->command_(reg, data.begin(), data.size()); }

  void process_state_();
  void set_state_(UpdateState state, uint32_t delay_ms = 0);
  const char *state_to_string_() const;
  void send_init_sequence_();
  bool transfer_data_();

  GPIOPin *dc_pin_{};
  GPIOPin *reset_pin_{};
  GPIOPin *sr_clock_pin_{};
  GPIOPin *sr_latch_pin_{};
  GPIOPin *sr_data_pin_{};

  split_buffer::SplitBuffer buffer_{};
  size_t transfer_index_{};
  HighFrequencyLoopRequester high_frequency_loop_;

  uint32_t delay_until_{};
  uint32_t wait_started_{};
  uint32_t busy_timeout_ms_{45000};
  bool waiting_for_idle_{};
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  uint32_t update_start_time_{};
#endif
  UpdateState state_{UpdateState::UPDATE_STATE_IDLE};
};

}  // namespace esphome::inky_frame_73
