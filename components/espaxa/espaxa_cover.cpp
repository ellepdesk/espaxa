#include "espaxa_cover.h"

#include <cctype>
#include <cmath>
#include <cstdlib>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::espaxa {

static const char *const TAG = "espaxa";

using namespace esphome::cover;

// The AXA sleeps between commands, a bare newline wakes it up
static const uint32_t WAKE_DELAY_MS = 50;
static const uint32_t RESPONSE_TIMEOUT_MS = 1000;
// Pause between transactions, the AXA ignores commands sent back to back
static const uint32_t COMMAND_GAP_MS = 200;
static const uint8_t MAX_RETRIES = 2;
static const uint32_t PUBLISH_INTERVAL_MS = 1000;
static const float POSITION_EPSILON = 0.01f;

static const char *command_to_str(AxaCommand cmd) {
  switch (cmd) {
    case AxaCommand::STATUS:
      return "STATUS";
    case AxaCommand::OPEN:
      return "OPEN";
    case AxaCommand::CLOSE:
      return "CLOSE";
    case AxaCommand::STOP:
      return "STOP";
    case AxaCommand::DEVICE:
      return "DEVICE";
    case AxaCommand::FIRMWARE:
      return "FIRMWARE";
  }
  return "";
}

void EspAxaCover::setup() {
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    this->position = COVER_CLOSED;
  }
  this->target_ = this->position;
  this->locked_ = this->position == COVER_CLOSED;
  if (this->locked_)
    this->last_status_ = AxaCode::LOCKED;

  this->queue_(AxaCommand::DEVICE);
  this->queue_(AxaCommand::FIRMWARE);
  this->queue_(AxaCommand::STATUS);
}

void EspAxaCover::dump_config() {
  LOG_COVER("", "AXA Remote 2.0", this);
  ESP_LOGCONFIG(TAG,
                "  Unlock Duration: %.1fs\n"
                "  Open Duration: %.1fs\n"
                "  Lock Duration: %.1fs\n"
                "  Poll Interval: %.1fs (moving: %.1fs)\n"
                "  Release Before Close: %s",
                this->unlock_duration_ / 1e3f, this->open_duration_ / 1e3f, this->lock_duration_ / 1e3f,
                this->poll_interval_ / 1e3f, this->moving_poll_interval_ / 1e3f,
                YESNO(this->release_before_close_));
  LOG_BINARY_SENSOR("  ", "Lock", this->lock_sensor_);
}

CoverTraits EspAxaCover::get_traits() {
  auto traits = CoverTraits();
  traits.set_is_assumed_state(false);
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  traits.set_supports_toggle(true);
  traits.set_supports_tilt(false);
  return traits;
}

void EspAxaCover::loop() {
  const uint32_t now = App.get_loop_component_start_time();

  this->read_uart_();
  this->update_motion_(now);

  // Schedule a STATUS poll when the bus is quiet
  bool moving = this->current_operation != COVER_OPERATION_IDLE;
  uint32_t interval = moving ? this->moving_poll_interval_ : this->poll_interval_;
  if (this->tx_state_ == TxState::IDLE && this->commands_.empty() && now - this->last_poll_ >= interval) {
    this->last_poll_ = now;
    this->queue_(AxaCommand::STATUS);
  }

  this->process_tx_(now);
}

void EspAxaCover::control(const CoverCall &call) {
  if (call.get_stop()) {
    this->stop_();
  }
  if (call.get_toggle().has_value()) {
    if (this->current_operation != COVER_OPERATION_IDLE) {
      this->stop_();
    } else if (this->position == COVER_CLOSED || this->last_operation_ == COVER_OPERATION_CLOSING) {
      this->start_move_(COVER_OPEN);
    } else {
      this->start_move_(COVER_CLOSED);
    }
  }
  if (call.get_position().has_value()) {
    this->start_move_(*call.get_position());
  }
}

// ---------------------------------------------------------------------------
// Serial transactions
//
// A transaction is: send "\r\n" to wake the AXA, wait WAKE_DELAY_MS, drop
// whatever it replied, send the command and wait for a line starting with a
// three digit status code. Everything is driven from loop(), so nothing blocks.
// ---------------------------------------------------------------------------

void EspAxaCover::read_uart_() {
  uint8_t c;
  while (this->available() && this->read_byte(&c)) {
    if (c == '\r' || c == '\n') {
      if (this->line_len_ > 0) {
        this->line_[this->line_len_] = '\0';
        this->handle_line_(this->line_);
        this->line_len_ = 0;
      }
    } else if (this->line_len_ < sizeof(this->line_) - 1) {
      this->line_[this->line_len_++] = static_cast<char>(c);
    }
  }
}

void EspAxaCover::process_tx_(uint32_t now) {
  switch (this->tx_state_) {
    case TxState::IDLE:
      if (this->commands_.empty() || now - this->tx_time_ < COMMAND_GAP_MS)
        return;
      this->tx_cmd_ = this->commands_.front();
      this->commands_.pop_front();
      this->tx_retries_ = 0;
      this->write_str("\r\n");
      this->tx_state_ = TxState::WAKING;
      this->tx_time_ = now;
      break;

    case TxState::WAKING:
      if (now - this->tx_time_ < WAKE_DELAY_MS)
        return;
      // Discard the reply to the wake-up newline
      this->line_len_ = 0;
      ESP_LOGV(TAG, "Sending %s", command_to_str(this->tx_cmd_));
      this->write_str(command_to_str(this->tx_cmd_));
      this->write_str("\r\n");
      this->tx_state_ = TxState::WAITING;
      this->tx_time_ = now;
      break;

    case TxState::WAITING:
      if (now - this->tx_time_ < RESPONSE_TIMEOUT_MS)
        return;
      if (this->tx_retries_ < MAX_RETRIES) {
        this->tx_retries_++;
        ESP_LOGD(TAG, "No response to %s, retry %u", command_to_str(this->tx_cmd_), this->tx_retries_);
        this->write_str("\r\n");
        this->tx_state_ = TxState::WAKING;
        this->tx_time_ = now;
        return;
      }
      this->tx_state_ = TxState::IDLE;
      this->tx_time_ = now;
      this->handle_response_(this->tx_cmd_, AxaCode::TIMEOUT, "");
      break;
  }
}

void EspAxaCover::handle_line_(const char *line) {
  if (this->tx_state_ != TxState::WAITING) {
    ESP_LOGV(TAG, "Ignoring '%s'", line);
    return;
  }
  // Responses look like "211 Locked", anything else (echo, help text) is skipped
  if (!(isdigit(line[0]) && isdigit(line[1]) && isdigit(line[2]))) {
    ESP_LOGV(TAG, "Skipping '%s'", line);
    return;
  }
  auto code = static_cast<AxaCode>(atoi(line));
  this->tx_state_ = TxState::IDLE;
  this->tx_time_ = millis();
  this->handle_response_(this->tx_cmd_, code, line);
}

void EspAxaCover::handle_response_(AxaCommand cmd, AxaCode code, const char *line) {
  ESP_LOGD(TAG, "%s -> %d '%s'", command_to_str(cmd), static_cast<int>(code), line);

  switch (cmd) {
    case AxaCommand::DEVICE:
    case AxaCommand::FIRMWARE:
      if (code != AxaCode::TIMEOUT)
        ESP_LOGI(TAG, "%s: %s", command_to_str(cmd), line);
      break;

    case AxaCommand::STOP:
      break;

    case AxaCommand::OPEN:
    case AxaCommand::CLOSE: {
      if (code != AxaCode::OK) {
        ESP_LOGW(TAG, "%s failed (%d)", command_to_str(cmd), static_cast<int>(code));
        this->commands_.clear();
        // The AXA accepted the OPEN that releases it, don't leave it opening
        if (cmd == AxaCommand::CLOSE && this->release_before_close_)
          this->queue_(AxaCommand::STOP);
        this->current_operation = COVER_OPERATION_IDLE;
        this->motion_started_ = false;
        this->queue_(AxaCommand::STATUS);
        this->publish_state();
        break;
      }
      // Closing sends OPEN first, only the CLOSE starts the motion
      bool starts_motion = (cmd == AxaCommand::OPEN && this->current_operation == COVER_OPERATION_OPENING) ||
                           (cmd == AxaCommand::CLOSE && this->current_operation == COVER_OPERATION_CLOSING);
      if (starts_motion && !this->motion_started_) {
        this->motion_started_ = true;
        this->move_start_ = millis();
        this->move_start_position_ = this->position;
        this->closed_since_ = 0;
      }
      break;
    }

    case AxaCommand::STATUS: {
      bool was_locked = this->last_status_ == AxaCode::LOCKED || this->last_status_ == AxaCode::LOCKED_UNSAFE;
      if (code == AxaCode::LOCKED || code == AxaCode::LOCKED_UNSAFE) {
        this->locked_ = true;
        if (this->current_operation == COVER_OPERATION_CLOSING && this->target_ == COVER_CLOSED) {
          // Locking confirmed, no need to wait for the timer
          this->finish_move_(COVER_CLOSED);
        } else if (this->current_operation == COVER_OPERATION_IDLE && this->position != COVER_CLOSED) {
          ESP_LOGI(TAG, "Window was closed manually");
          this->target_ = COVER_CLOSED;
          this->finish_move_(COVER_CLOSED);
        }
      } else if (code == AxaCode::UNLOCKED) {
        this->locked_ = false;
        if (was_locked && this->current_operation == COVER_OPERATION_IDLE && this->position == COVER_CLOSED) {
          // Opened with the button on the AXA, which always opens fully
          ESP_LOGI(TAG, "Window was opened manually");
          this->target_ = COVER_OPEN;
          this->finish_move_(COVER_OPEN);
        }
      }
      if (code == AxaCode::LOCKED || code == AxaCode::LOCKED_UNSAFE || code == AxaCode::UNLOCKED) {
        this->last_status_ = code;
        this->publish_lock_(code);
      }
      if (this->lock_check_) {
        this->lock_check_ = false;
        // Still closing: this last STATUS did not confirm the lock either
        if (this->current_operation == COVER_OPERATION_CLOSING)
          this->lock_failed_();
      }
      break;
    }
  }
}

// ---------------------------------------------------------------------------
// Motion
//
// The AXA only reports locked / unlocked, so the position in between is
// estimated from the time since the motion started.
// ---------------------------------------------------------------------------

void EspAxaCover::start_move_(float target) {
  const uint32_t now = millis();
  this->recompute_position_(now);

  bool locking = this->current_operation == COVER_OPERATION_CLOSING && this->closed_since_ != 0;
  if (locking && target < POSITION_EPSILON) {
    // Already closing, let it lock
    this->publish_state();
    return;
  }
  // Mid-lock, assume a full unlock is needed
  if (locking)
    this->locked_ = true;

  this->commands_.clear();
  this->lock_check_ = false;
  this->close_retried_ = false;

  if (this->current_operation != COVER_OPERATION_IDLE) {
    this->queue_(AxaCommand::STOP);
    this->current_operation = COVER_OPERATION_IDLE;
  }
  this->motion_started_ = false;
  this->target_ = target;

  if (std::fabs(target - this->position) < POSITION_EPSILON) {
    this->publish_state();
    return;
  }

  if (target > this->position) {
    this->move_delay_ = this->locked_ ? this->unlock_duration_ : 0;
    this->current_operation = COVER_OPERATION_OPENING;
    this->queue_(AxaCommand::OPEN);
  } else {
    this->start_close_();
  }
  this->last_operation_ = this->current_operation;
  // Poll soon after starting
  this->last_poll_ = now;
  this->publish_state();
}

void EspAxaCover::start_close_() {
  // The AXA blocks at its end position, a short OPEN releases it
  // (galagaking/espaxa). Seen to make the AXA open instead.
  if (this->release_before_close_)
    this->queue_(AxaCommand::OPEN);
  this->queue_(AxaCommand::CLOSE);
  this->move_delay_ = 0;
  this->motion_started_ = false;
  this->closed_since_ = 0;
  this->current_operation = COVER_OPERATION_CLOSING;
}

// The lock wait ran out. Seen when the AXA opened instead of closing: stop
// it and close once more, from an unknown position.
void EspAxaCover::lock_failed_() {
  this->queue_(AxaCommand::STOP);
  if (!this->close_retried_) {
    ESP_LOGW(TAG, "Lock not confirmed, closing again");
    this->close_retried_ = true;
    // Assume fully open, the worst case; a 211 ends the close early
    this->position = COVER_OPEN;
    this->locked_ = false;
    this->start_close_();
    this->last_poll_ = millis();
    this->publish_state();
    return;
  }
  // Not confirmed twice, the next STATUS will tell
  ESP_LOGW(TAG, "Lock not confirmed");
  this->queue_(AxaCommand::STATUS);
  this->finish_move_(COVER_CLOSED);
}

void EspAxaCover::publish_lock_(AxaCode code) {
  // Only 211 Strong Locked counts, 212 may not be fully locked
  if (this->lock_sensor_ != nullptr)
    this->lock_sensor_->publish_state(code != AxaCode::LOCKED);
}

void EspAxaCover::stop_() {
  this->recompute_position_(millis());
  if (this->current_operation == COVER_OPERATION_CLOSING && this->closed_since_ != 0) {
    // The window is closed already, interrupting the lock gains nothing
    ESP_LOGI(TAG, "Stop ignored while locking");
    this->publish_state();
    return;
  }
  this->commands_.clear();
  this->queue_(AxaCommand::STOP);
  this->target_ = this->position;
  this->current_operation = COVER_OPERATION_IDLE;
  this->motion_started_ = false;
  this->publish_state();
}

void EspAxaCover::finish_move_(float position) {
  this->position = position;
  this->current_operation = COVER_OPERATION_IDLE;
  this->motion_started_ = false;
  this->publish_state();
}

void EspAxaCover::recompute_position_(uint32_t now) {
  if (!this->motion_started_ || this->current_operation == COVER_OPERATION_IDLE)
    return;
  int32_t elapsed = static_cast<int32_t>(now - this->move_start_ - this->move_delay_);
  if (elapsed <= 0)
    return;
  if (this->move_delay_ > 0)
    this->locked_ = false;

  float delta = static_cast<float>(elapsed) / this->open_duration_;
  if (this->current_operation == COVER_OPERATION_OPENING) {
    this->position = clamp(this->move_start_position_ + delta, COVER_CLOSED, COVER_OPEN);
  } else {
    this->position = clamp(this->move_start_position_ - delta, COVER_CLOSED, COVER_OPEN);
  }
}

void EspAxaCover::update_motion_(uint32_t now) {
  if (this->current_operation == COVER_OPERATION_IDLE || !this->motion_started_)
    return;

  this->recompute_position_(now);

  if (this->current_operation == COVER_OPERATION_OPENING && this->position >= this->target_) {
    // The AXA stops by itself when fully open
    if (this->target_ < COVER_OPEN)
      this->queue_(AxaCommand::STOP);
    this->finish_move_(this->target_);
    return;
  }

  if (this->current_operation == COVER_OPERATION_CLOSING && this->position <= this->target_) {
    if (this->target_ > COVER_CLOSED) {
      this->queue_(AxaCommand::STOP);
      this->finish_move_(this->target_);
      return;
    }
    // Fully closed, give the AXA time to lock unless STATUS confirms it first
    if (this->closed_since_ == 0)
      this->closed_since_ = now;
    if (now - this->closed_since_ >= this->lock_duration_ && !this->lock_check_) {
      // The lock may have come just after the last poll, ask once more
      this->lock_check_ = true;
      this->queue_(AxaCommand::STATUS);
    }
  }

  if (now - this->last_publish_ >= PUBLISH_INTERVAL_MS) {
    this->last_publish_ = now;
    this->publish_state(false);
  }
}

}  // namespace esphome::espaxa
