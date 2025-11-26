#pragma once

#include <string>
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/automation.h" 
#include "esphome/core/helpers.h" // Required for CallbackManager
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace esphome_i2c_sniffer {

/**
 * @brief The EsphomeI2cSniffer component captures I2C bus traffic by monitoring 
 * the SDA and SCL pins using interrupts.
 * It inherits from Component to integrate into the ESPHome lifecycle (setup/loop).
 */
class EsphomeI2cSniffer : public Component {
 public:
  // --- Configuration Methods (Setters) ---
  
  /** @brief Sets the SDA (Data) pin number. */
  void set_sda_pin(uint8_t pin) { this->sda_pin_ = pin; }
  
  /** @brief Sets the SCL (Clock) pin number. */
  void set_scl_pin(uint8_t pin) { this->scl_pin_ = pin; }

  // Setters for the output entities
  void set_msg_sensor(text_sensor::TextSensor *s) { this->msg_sensor_ = s; }
  void set_last_address_sensor(sensor::Sensor *s) { this->last_address_sensor_ = s; }
  void set_last_data_sensor(sensor::Sensor *s) { this->last_data_sensor_ = s; }
  void set_last_byte_sensor(sensor::Sensor *s) { this->last_byte_sensor_ = s; }

  /**
   * @brief Registers an automation trigger to be called when an I2C address is detected.
   * This is a critical method where the conversion error was fixed.
   * @param trigger A pointer to the Trigger<uint8_t, bool> instance.
   */
  void register_address_trigger(Trigger<uint8_t, bool> *trigger) {
    // The CallbackManager requires an std::function (or lambda), not a raw Trigger pointer.
    // The lambda wraps the trigger call, adapting the interface.
    this->on_address_triggers_.add([trigger](uint8_t addr, bool rw) {
        // Execute the actual ESPHome trigger with the detected address (uint8) and R/W bit (bool).
        trigger->trigger(addr, rw);
    });
  }

  // --- ESPHome Component Overrides ---
  void setup() override;
  void loop() override;
  void dump_config() override;

  // --- Interrupt Service Routine (ISR) Callbacks ---
  /** * @brief Called by the SCL ISR wrapper. Handles clocking and data/address sampling. 
   * Must be IRAM-safe.
   */
  void on_scl_edge_();
  
  /** * @brief Called by the SDA ISR wrapper. Handles START and STOP condition detection.
   * Must be IRAM-safe.
   */
  void on_sda_edge_();

 protected:
  /**
   * @brief Formats and publishes the captured I2C frame to the ESPHome entities.
   * @note This is called from loop() (main thread) to safely interact with ESPHome publishing.
   */
  void publish_frame_(uint8_t addr, bool rw, const uint8_t *data, uint8_t len);

  // --- Configuration Storage ---
  uint8_t sda_pin_{255}; /**< Configured SDA pin number. */
  uint8_t scl_pin_{255}; /**< Configured SCL pin number. */

  // --- Entity Pointers ---
  text_sensor::TextSensor *msg_sensor_{nullptr}; /**< Full frame output (text). */
  sensor::Sensor *last_address_sensor_{nullptr};   /**< Last detected 7-bit address (numeric). */
  sensor::Sensor *last_data_sensor_{nullptr};   /**< Frame length (number of data bytes). */
  sensor::Sensor *last_byte_sensor_{nullptr};   /**< Value of the last data byte in the frame. */

  // --- Automation Callbacks ---
  /** @brief Manages list of automation actions triggered upon address detection (uint8_t addr, bool rw). */
  CallbackManager<void(uint8_t, bool)> on_address_triggers_;

  // --- Volatile Runtime State (Accessed by ISR) ---
  
  // I2C Protocol State
  volatile bool in_transfer_{false}; /**< True after START, false after STOP. */
  volatile bool last_sda_{true};    /**< Previous state of the SDA pin (for edge detection). */
  volatile bool last_scl_{true};    /**< Previous state of the SCL pin (for edge detection). */

  volatile uint8_t bit_idx_{0};     /**< Current bit index within the 8-bit byte (0-7). */
  volatile bool ack_phase_{false};  /**< True if the next SCL clock is for the ACK/NACK bit (9th bit). */
  volatile uint8_t cur_byte_{0};    /**< Buffer where the current 8 bits are assembled. */

  // Address and Direction
  volatile bool have_addr_{false};  /**< True after the first byte (address) has been sampled. */
  volatile uint8_t addr_{0};        /**< The 7-bit I2C address of the transaction. */
  volatile bool rw_{false};         /**< Read/Write direction bit (true for read, false for write). */

  // ISR -> Loop Communication Buffers (Address Trigger)
  volatile bool new_addr_event_{false};     /**< Flag set by ISR to signal a new address detection to loop(). */
  volatile uint8_t detected_addr_buf_{0};  /**< Address detected by ISR, waiting to be processed by loop(). */
  volatile bool detected_rw_buf_{false};    /**< R/W bit detected by ISR, waiting to be processed by loop(). */

  // Data Frame Buffer
  static constexpr uint8_t MAX_DATA_ = 64; /**< Maximum number of data bytes to capture per frame. */
  uint8_t data_[MAX_DATA_]{};             /**< Buffer to store captured data bytes. */
  volatile uint8_t data_len_{0};           /**< Current number of data bytes in the buffer. */

  // Frame Completion Signal
  volatile bool frame_ready_{false}; /**< Flag set by ISR (on STOP condition) to signal a complete frame. */
};

}  // namespace esphome_i2c_sniffer
}  // namespace esphome
