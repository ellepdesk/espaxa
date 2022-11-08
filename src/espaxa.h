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
};
const int poll_delay_ms = 60000;

const int unlock_time_ms = 10000;
const int lock_time_ms = 20000;
const int move_time_ms = 50000;
const int move_step_time_ms = move_time_ms / int(AxaPosition::OPEN_FULL);
const int steps = int(AxaPosition::OPEN_FULL);

class EspAxaCover :
  public Component,
  public UARTDevice,
  public Cover {
  protected:
    uint32_t lastread = millis() - poll_delay_ms;
    uint32_t last_state_change = millis();
    // uint8_t device[64] = {0};
    // uint8_t firmware[64] = {0};
    AxaPosition desired_pos = AxaPosition::LOCKED;
    AxaPosition current_pos = AxaPosition::LOCKED;
    bool state_dirty = true;
 public:
  EspAxaCover(UARTComponent *parent) :
    Component(),
    UARTDevice(parent),
    Cover()
    {}

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
    float p = int(pos);
    float fp = p / (steps * 1.0);
    return fp;
  }

  AxaCode send_cmd(AxaCommand cmd, uint8_t * buffer = NULL, size_t bufferlen = 0)
  {
    auto return_code = AxaCode::Error;
    write_str("\r\n");
    delay(20);
    for (int i =0; i< 10; i++)
      if(available())
      {
        uint8_t read_buf[128] = {0};
        read_array(read_buf, min(sizeof(read_buf), uint32_t(available())));
        // ESP_LOGCONFIG("espaxa", "junk: %s", read_buf);

      }

    uint8_t read_buf[128] = {0};
    const char * cmd_str = "";
    switch(cmd){
      case AxaCommand::STATUS:
        cmd_str = "STATUS\r\n";
      break;
      case AxaCommand::OPEN:
        cmd_str ="OPEN\r\n";
      break;
      case AxaCommand::CLOSE:
        cmd_str = "CLOSE\r\n";
      break;
      case AxaCommand::STOP:
        cmd_str = "STOP\r\n";
      break;
      case AxaCommand::FIRMWARE:
        cmd_str = "FIRMWARE\r\n";
      break;
      case AxaCommand::DEVICE:
        cmd_str = "DEVICE\r\n";
      break;
      case AxaCommand::HELP:
        cmd_str = "?:\r\n";
      break;
    }
    write_str(cmd_str);
    delay(50);
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
    ESP_LOGCONFIG("espaxa", "send_cmd(%i): %i", int(cmd), int(return_code));
    return return_code;
  }

  void control(const CoverCall &call) override {
    // This will be called every time the user requests a state change.
    if (call.get_position().has_value()) {
      auto pos = *call.get_position();
      desired_pos = normalize_position(pos);
    }
    if (call.get_stop()) {
      desired_pos = current_pos;
      // User requested cover stop
    }

    // Only apply new state when idle
    if (current_operation == COVER_OPERATION_IDLE)
    {
      update_desired_operation();
    }
    publish();
  }

  void update_desired_operation()
  {
    CoverOperation desired_operation = COVER_OPERATION_IDLE;
    if (current_pos == desired_pos)
      desired_operation = COVER_OPERATION_IDLE;
    else if (current_pos < desired_pos)
    {
      if (state_dirty && desired_pos!= AxaPosition::OPEN_FULL)
      {
        // close first to get to desired state
        desired_operation = COVER_OPERATION_CLOSING;
      }
      desired_operation = COVER_OPERATION_OPENING;
    }
    else if (current_pos > desired_pos)
    {
      if (state_dirty && desired_pos!= AxaPosition::LOCKED)
      {
        // close first to get to desired state
        desired_operation = COVER_OPERATION_OPENING;
      }
      desired_operation = COVER_OPERATION_CLOSING;
    }

    //apply desired_operation
    switch (desired_operation)
    {
      case COVER_OPERATION_OPENING:
      {
        send_cmd(AxaCommand::OPEN);
        current_operation = COVER_OPERATION_OPENING;
        last_state_change = millis();
        break;
      }
      case COVER_OPERATION_CLOSING:
      {
        send_cmd(AxaCommand::OPEN);
        delay(100);
        send_cmd(AxaCommand::CLOSE);
        current_operation = COVER_OPERATION_CLOSING;
        last_state_change = millis();
        break;
      }
      default:
        break;
    }
  }

  void poll_status(){
    if ((millis() - lastread) < poll_delay_ms){
      return;
    }
    lastread = millis();

    // send STATUS to uart, wait a bit and read response
    AxaCode axa_code = send_cmd(AxaCommand::STATUS);

    if (axa_code == AxaCode::Unlocked and current_pos == AxaPosition::LOCKED){
      desired_pos = current_pos = AxaPosition::OPEN_FULL; // assume fully opened by remote
      state_dirty = true;
    }
    if (axa_code == AxaCode::Locked){
      desired_pos = current_pos = AxaPosition::LOCKED;
    }
    publish();
  }

  void loop() override {
    uint32_t now = millis();
    int32_t time_since_last_change = now - last_state_change;
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
      poll_status();
    }
    else if (current_pos == desired_pos)
    {
      // Only send STOP if the desired position is not and endstop
      if (! (desired_pos == AxaPosition::OPEN_FULL || desired_pos == AxaPosition::LOCKED)){
        send_cmd(AxaCommand::STOP);
      }
      // prevent status call right after state change
      lastread = millis();

      current_operation = COVER_OPERATION_IDLE;
      // Check if we are in the state we want to be
      // new command may have arrived
      // or we were moving from a dirty state to closed first
      update_desired_operation();
      publish();
    }
  }

  void publish()
  {
    this->position = denormalize_positon(current_pos);
    this->publish_state();
  }
};
