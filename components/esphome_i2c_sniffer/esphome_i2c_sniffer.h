// esphome I2C sniffer header for ESP32 ESP-IDF framework
#pragma once

#include <string>
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/automation.h" 
#include "esphome/core/helpers.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"

#ifdef USE_ESP32_FRAMEWORK_ESP_IDF
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace esphome {
namespace esphome_i2c_sniffer {

class EsphomeI2cSniffer : public Component {
 public:
  void set_sda_pin(uint8_t pin) { this->sda_pin_ = (gpio_num_t)pin; }
  void set_scl_pin(uint8_t pin) { this->scl_pin_ = (gpio_num_t)pin; }

  void set_msg_sensor(text_sensor::TextSensor *s) { this->msg_sensor_ = s; }
  void set_last_addr_sensor(sensor::Sensor *s) { this->last_addr_sensor_ = s; }
  void set_last_data_sensor(sensor::Sensor *s) { this->last_data_sensor_ = s; }
  void set_last_byte_sensor(sensor::Sensor *s) { this->last_byte_sensor_ = s; }

  void register_address_trigger(Trigger<uint8_t, bool> *trigger) {
    this->on_address_triggers_.add([trigger](uint8_t addr, bool rw) {
        trigger->trigger(addr, rw);
    });
  }

  void setup() override;
  void loop() override;
  void dump_config() override;

  // ISR logic
  void IRAM_ATTR on_scl_edge_();
  void IRAM_ATTR on_sda_edge_();

 protected:
  void publish_frame_(uint8_t addr, bool rw, const uint8_t *data, uint8_t len);

  // ESP-IDF specific GPIO types
  gpio_num_t sda_pin_;
  gpio_num_t scl_pin_;

  text_sensor::TextSensor *msg_sensor_{nullptr};
  sensor::Sensor *last_addr_sensor_{nullptr};
  sensor::Sensor *last_data_sensor_{nullptr};
  sensor::Sensor *last_byte_sensor_{nullptr};

  CallbackManager<void(uint8_t, bool)> on_address_triggers_;

  // ESP32 Spinlock for ISR/Loop synchronization
  #ifdef USE_ESP32_FRAMEWORK_ESP_IDF
  portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  #endif

  // Protocol State
  volatile bool in_transfer_{false};
  volatile bool last_sda_{true};
  volatile bool last_scl_{true};
  volatile uint8_t bit_idx_{0};
  volatile bool ack_phase_{false};
  volatile uint8_t cur_byte_{0};
  volatile bool have_addr_{false};
  volatile uint8_t addr_{0};
  volatile bool rw_{false};

  // Buffers
  volatile bool new_addr_event_{false};
  volatile uint8_t detected_addr_buf_{0};
  volatile bool detected_rw_buf_{false};

  static constexpr uint8_t MAX_DATA_ = 64;
  uint8_t data_[MAX_DATA_]{};
  volatile uint8_t data_len_{0};
  volatile bool frame_ready_{false};
};

}  // namespace esphome_i2c_sniffer
}  // namespace esphome
