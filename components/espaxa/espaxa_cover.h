#pragma once

#include <deque>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome::espaxa {

enum class AxaCommand : uint8_t {
  STATUS,
  OPEN,
  CLOSE,
  STOP,
  DEVICE,
  FIRMWARE,
};

// AXA Remote 2.0 response codes
enum class AxaCode : int {
  OK = 200,
  UNLOCKED = 210,
  LOCKED = 211,
  // Assumed to mean 'closed but not fully locked'
  LOCKED_UNSAFE = 212,
  DEVICE = 260,
  FIRMWARE = 261,
  ERROR = 502,
  TIMEOUT = -1,
};

class EspAxaCover : public cover::Cover, public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  cover::CoverTraits get_traits() override;

  void set_unlock_duration(uint32_t ms) { this->unlock_duration_ = ms; }
  void set_open_duration(uint32_t ms) { this->open_duration_ = ms; }
  void set_lock_duration(uint32_t ms) { this->lock_duration_ = ms; }
  void set_poll_interval(uint32_t ms) { this->poll_interval_ = ms; }
  void set_moving_poll_interval(uint32_t ms) { this->moving_poll_interval_ = ms; }
  void set_release_before_close(bool release) { this->release_before_close_ = release; }
  // On = unlocked, as for Home Assistant's lock device class
  void set_lock_sensor(binary_sensor::BinarySensor *sensor) { this->lock_sensor_ = sensor; }

 protected:
  enum class TxState : uint8_t { IDLE, WAKING, WAITING };

  void control(const cover::CoverCall &call) override;

  // Serial transaction handling, never blocks
  void queue_(AxaCommand cmd) { this->commands_.push_back(cmd); }
  void read_uart_();
  void process_tx_(uint32_t now);
  void handle_line_(const char *line);
  void handle_response_(AxaCommand cmd, AxaCode code, const char *line);

  // Motion handling
  void start_move_(float target);
  void start_close_();
  void stop_();
  void lock_failed_();
  void publish_lock_(AxaCode code);
  void finish_move_(float position);
  void recompute_position_(uint32_t now);
  void update_motion_(uint32_t now);

  uint32_t unlock_duration_{10000};
  uint32_t open_duration_{50000};
  uint32_t lock_duration_{45000};
  uint32_t poll_interval_{30000};
  uint32_t moving_poll_interval_{2000};
  bool release_before_close_{true};
  binary_sensor::BinarySensor *lock_sensor_{nullptr};

  std::deque<AxaCommand> commands_;
  TxState tx_state_{TxState::IDLE};
  AxaCommand tx_cmd_{AxaCommand::STATUS};
  uint32_t tx_time_{0};
  uint8_t tx_retries_{0};
  uint32_t last_poll_{0};

  char line_[64]{};
  size_t line_len_{0};

  bool locked_{true};
  // Last code reported by STATUS, used to detect operation with the AXA buttons
  AxaCode last_status_{AxaCode::TIMEOUT};
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};
  float target_{0.0f};
  // Set once the AXA acknowledged the command that starts the motion
  bool motion_started_{false};
  uint32_t move_start_{0};
  float move_start_position_{0.0f};
  // Time between the motion command and the chain moving (unlocking)
  uint32_t move_delay_{0};
  // Time the estimated position reached closed, the AXA then locks
  uint32_t closed_since_{0};
  // The lock wait ran out, waiting for a last STATUS
  bool lock_check_{false};
  // The current close already ran a second time after a failed lock
  bool close_retried_{false};
  uint32_t last_publish_{0};
};

}  // namespace esphome::espaxa
