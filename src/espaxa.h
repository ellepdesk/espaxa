#include "esphome.h"
#include <string>

using namespace esphome;

enum class AxaCommand{
  STATUS,
  OPEN,
  CLOSE,
  STOP,
  FIRMWARE,
  DEVICE,
  HELP,
};

// AXA REMOTE 2.0 return strings
enum class AxaCode {
  OK = 200,
  Unlocked = 210,
  Locked = 211,
  Device = 260,
  Firmware = 261,
  Error = 502,
  Invalid = -1,
};

// add OPEN_X positions if needed
enum class AxaPosition{
  LOCKED,
  OPEN_1,
  OPEN_2,
  OPEN_3,
  OPEN_FULL,
  OPEN_MANUAL,
  UNKNOWN,
};

const int unlock_time_ms = 10000;
const int lock_time_ms = 30000;
const int move_time_ms = 50000;
const int move_step_time_ms = move_time_ms / int(AxaPosition::OPEN_FULL);
const int steps = int(AxaPosition::OPEN_FULL);


class EspAxaCover:
  public Component,
  public UARTDevice,
  public Cover {
 private:
  AxaPosition desired_pos;
  AxaPosition current_pos;
  uint32_t last_state_change = 0;

 public:
  EspAxaCover(UARTComponent *parent) :
    Component(),
    UARTDevice(parent),
    Cover()
    {
      auto restored_state = restore_state_();
      if (restored_state.has_value()){
        auto position = restored_state.value().position;
        desired_pos= normalize_position(position);
        current_pos = desired_pos;
        CoverOperation desired_operation = update_operation(desired_pos);
        apply_operation(desired_operation);
      }
      else {
        desired_pos = AxaPosition::LOCKED;
        current_pos = desired_pos;
      }
    }

  void setup() override {
    ESP_LOGCONFIG("espaxa", "Setting up AXA UART...");
  }

  CoverTraits get_traits() override {
    auto traits = CoverTraits();
    traits.set_is_assumed_state(false);
    traits.set_supports_position(true);
    traits.set_supports_tilt(false);
    return traits;
  }

  AxaPosition normalize_position(float pos)
  {
    int p = pos * steps;
    return AxaPosition(p);
  }

  float denormalize_positon(AxaPosition pos){
    if (pos == AxaPosition::OPEN_MANUAL)
      return 1.0;
    float p = int(pos);
    float fp = p / (steps * 1.0);
    return fp;
  }

  const char * axa_command_to_string(AxaCommand cmd){
    switch(cmd){
      case AxaCommand::STATUS:
        return "STATUS\r\n";
      break;
      case AxaCommand::OPEN:
        return "OPEN\r\n";
      break;
      case AxaCommand::CLOSE:
        return "CLOSE\r\n";
      break;
      case AxaCommand::STOP:
        return "STOP\r\n";
      break;
      case AxaCommand::FIRMWARE:
        return "FIRMWARE\r\n";
      break;
      case AxaCommand::DEVICE:
        return "DEVICE\r\n";
      break;
      case AxaCommand::HELP:
        return "?:\r\n";
      break;
    }
    return "";
  }

  AxaCode send_cmd(AxaCommand cmd, uint8_t * buffer = NULL, size_t bufferlen = 0)
  {
    auto return_code = AxaCode::Error;
    write_str("\r\n");
    delay(20);
    for (int i =0; i< 10; i++)
      if(available())
      {
        // Clear buffer
        uint8_t read_buf[128] = {0};
        read_array(read_buf, min(sizeof(read_buf), uint32_t(available())));
      }

    uint8_t read_buf[128] = {0};
    auto cmd_str = axa_command_to_string(cmd);
    write_str(cmd_str);
    delay(200);
    read_array(read_buf, min(sizeof(read_buf), uint32_t(available())));
    size_t len = min(strlen((char*)read_buf), sizeof(read_buf));

    for (int i = 0; i < len; i++)
    {
      uint8_t c = read_buf[i];
      // look for digits
      if (c >= '0' && c <= '9' and len >= i+2)
      {
        int result = 100*(read_buf[i] - '0');
        result += 10*(read_buf[i + 1] - '0');
        result += (read_buf[i + 2] - '0');  // calculate status code from first three digits
        return_code = AxaCode(result);

        // BROKEN! only useful to read device and firmware anyway
        // if (buffer && bufferlen >0)
        // {
        //   memcpy(buffer, read_buf+i+4, min(bufferlen, len-(i+4)));
        // }
        // for (int i = strlen((const char*)buffer); i>=0 ; i--){
        //     if (buffer[i] == '\n' || buffer[i] == '\r'){
        //         buffer[i] = '\0';
        //     }
        // }
        break;
      }
    }
    ESP_LOGCONFIG("espaxa", "send_cmd(%s): %i", axa_command_to_string(cmd), int(return_code));
    return return_code;
  }

  void control(const CoverCall &call) override {
    // This will be called every time the user requests a state change.
    if (call.get_position().has_value()) {
      auto pos = *call.get_position();
      auto position = normalize_position(pos);
      CoverOperation desired_operation = update_operation(position);
      apply_operation(desired_operation);
    }
    if (call.get_stop()) {
      desired_pos = current_pos;
      send_cmd(AxaCommand::STOP);
    }
  }

  CoverOperation update_operation(AxaPosition pos)
  {
    desired_pos = pos;
    CoverOperation desired_operation = COVER_OPERATION_IDLE;
    if (current_pos == desired_pos)
      desired_operation = COVER_OPERATION_IDLE;
    else if (current_pos == AxaPosition::OPEN_MANUAL)
      //close first
      desired_operation = COVER_OPERATION_CLOSING;
    else if (current_pos < desired_pos)
    {
      desired_operation = COVER_OPERATION_OPENING;
    }
    else if (current_pos > desired_pos)
    {
      desired_operation = COVER_OPERATION_CLOSING;
    }
    return desired_operation;
  }

  void apply_operation(CoverOperation desired_operation)
  {
    if (current_operation != CoverOperation::COVER_OPERATION_IDLE)
    {
      send_cmd(AxaCommand::STOP);
      delay(200);
    }

    switch (desired_operation)
    {
      case COVER_OPERATION_OPENING:
      {
        if (send_cmd(AxaCommand::OPEN) != AxaCode::OK)
          break;
        last_state_change = millis();
        break;
      }
      case COVER_OPERATION_CLOSING:
      {
        if (send_cmd(AxaCommand::OPEN) != AxaCode::OK)
          break;
        delay(200);
        if (send_cmd(AxaCommand::CLOSE) != AxaCode::OK)
          break;
        last_state_change = millis();
        break;
      }
      default:
        break;
    }
  }

  AxaPosition poll_position(){
    AxaCode poll_code = send_cmd(AxaCommand::STATUS);

    switch(poll_code)
    {
      // Error responses:
      case AxaCode::Error:
      case AxaCode::Invalid:
      // Invalid returns for STATUS
      case AxaCode::OK:
      case AxaCode::Firmware:
      case AxaCode::Device:
        return AxaPosition::UNKNOWN;
        break;

      case AxaCode::Unlocked:
      {
        if (current_pos == AxaPosition::LOCKED)
          return AxaPosition::OPEN_MANUAL;
        break;
      }
      case AxaCode::Locked:
      {
        return AxaPosition::LOCKED;
        break;
      }
    }
    return current_pos;
  }


  void loop() override {
    uint32_t now = millis();
    int32_t time_since_last_change = now - last_state_change;
    AxaPosition polled_pos = poll_position();
    if (polled_pos == AxaPosition::UNKNOWN)
      return;


    ESP_LOGCONFIG(
      "espaxa",
      "loop: cop(%s), cpos(%i), dpos(%i), ppos(%i)",
      cover_operation_to_str(current_operation),
      int(current_pos),
      int(desired_pos),
      int(polled_pos)
      );

    if (last_state_change == 0)
    {
      current_pos = polled_pos;
      last_state_change = now;
      current_operation = update_operation(desired_pos);
      apply_operation(current_operation);
      return;
    }

    if (current_operation == COVER_OPERATION_OPENING)
    {
      uint32_t step_time_ms = move_step_time_ms;
      if (current_pos == AxaPosition::LOCKED)
          step_time_ms += unlock_time_ms;

      if (time_since_last_change >= step_time_ms)
      {
        last_state_change += move_step_time_ms;
        current_pos = AxaPosition(int(current_pos) + 1);
        publish();
      }
    }
    else if (current_operation == COVER_OPERATION_CLOSING)
    {
      uint32_t step_time_ms = move_step_time_ms;
      if (current_pos == AxaPosition::OPEN_1)
          step_time_ms += lock_time_ms;

      if (time_since_last_change >= step_time_ms)
      {
        last_state_change += move_step_time_ms;
        current_pos = AxaPosition(int(current_pos) - 1);
        publish();
      }
    }

    if (current_operation == COVER_OPERATION_IDLE)
    {
      last_state_change = now;
    }
    else if (current_pos == desired_pos)
    {
      // Only send STOP if the desired position is not and endstop
      if (! (desired_pos == AxaPosition::OPEN_FULL || desired_pos == AxaPosition::LOCKED)){
        send_cmd(AxaCommand::STOP);
      }
      // Check if we are in the state we want to be
      // new command may have arrived
      // or we were moving from a dirty state to closed first
      current_operation = update_operation(desired_pos);
      apply_operation(current_operation);
    }
    current_operation = update_operation(desired_pos);
    publish();
    delay(250);
  }

  void publish()
  {
    if (current_pos == AxaPosition::UNKNOWN)
      return;
    this->position = denormalize_positon(current_pos);
    this->publish_state();
  }
};
