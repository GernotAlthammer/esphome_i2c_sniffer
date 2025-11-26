#include "Arduino.h"  // attachInterruptArg, pinMode, digitalRead
#include <cstdio>
#include <cstring>
#include <string>
#include "esphome_i2c_sniffer.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esphome_i2c_sniffer {

static const char *const TAG = "i2c_sniffer";

/**
 * @brief IRAM-safe wrapper for the SCL pin interrupt service routine (ISR).
 * This function is executed when the SCL pin changes state (rising or falling edge).
 * It safely calls the non-static member function on the component instance.
 * @param arg A pointer to the EsphomeI2cSniffer instance (passed via attachInterruptArg).
 */
void IRAM_ATTR scl_isr(void *arg) {
  reinterpret_cast<EsphomeI2cSniffer *>(arg)->on_scl_edge_();
}

/**
 * @brief IRAM-safe wrapper for the SDA pin interrupt service routine (ISR).
 * This function is executed when the SDA pin changes state (rising or falling edge).
 * It safely calls the non-static member function on the component instance.
 * @param arg A pointer to the EsphomeI2cSniffer instance.
 */
void IRAM_ATTR sda_isr(void *arg) {
  reinterpret_cast<EsphomeI2cSniffer *>(arg)->on_sda_edge_();
}

/**
 * @brief Initializes the I2C sniffer component.
 * Sets up the pins, reads the initial state, and attaches the interrupt handlers.
 */
void EsphomeI2cSniffer::setup() {
  // Configure both SDA and SCL pins as inputs with internal pull-ups.
  // This is required because I2C is an open-drain bus.
  pinMode(this->scl_pin_, INPUT_PULLUP);
  pinMode(this->sda_pin_, INPUT_PULLUP);

  // Store the initial state of the pins.
  this->last_scl_ = digitalRead(this->scl_pin_);
  this->last_sda_ = digitalRead(this->sda_pin_);

  // Attach interrupts to both pins, triggering on any change (CHANGE).
  // The 'this' pointer is passed as an argument to the ISR wrappers (scl_isr/sda_isr).
  attachInterruptArg(this->scl_pin_, scl_isr, this, CHANGE);
  attachInterruptArg(this->sda_pin_, sda_isr, this, CHANGE);

  ESP_LOGI(TAG, "I2C sniffer initialized on SDA=%u, SCL=%u", this->sda_pin_, this->scl_pin_);
}

/**
 * @brief Dumps the configuration details to the log during boot.
 */
void EsphomeI2cSniffer::dump_config() {
  ESP_LOGCONFIG(TAG, "I2C Sniffer:");
  ESP_LOGCONFIG(TAG, "  SDA Pin: %u", this->sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCL Pin: %u", this->scl_pin_);
}

/**
 * @brief Handles events triggered by changes on the SDA pin. (ISR)
 * This function is primarily responsible for detecting START and STOP conditions.
 */
void IRAM_ATTR EsphomeI2cSniffer::on_sda_edge_() {
  bool sda = digitalRead(this->sda_pin_);
  bool scl = digitalRead(this->scl_pin_);

  // I2C START Condition: SDA transitions from HIGH to LOW while SCL is HIGH.
  if (!sda && this->last_sda_ && scl) {
    this->in_transfer_ = true;      // Mark the start of a transaction.
    this->bit_idx_ = 0;             // Reset bit counter.
    this->ack_phase_ = false;       // Not in the ACK phase yet.
    this->cur_byte_ = 0;            // Reset the current byte assembly buffer.
    this->have_addr_ = false;       // Reset address detection flag.
    this->data_len_ = 0;            // Reset data length counter.
  }
  // I2C STOP Condition: SDA transitions from LOW to HIGH while SCL is HIGH.
  else if (sda && !this->last_sda_ && scl) {
    if (this->in_transfer_) {
      this->in_transfer_ = false;   // Mark the end of the transaction.
      this->frame_ready_ = true;    // Signal the main loop that a complete frame is available.
    }
  }

  // Update the stored state for the next edge detection.
  this->last_sda_ = sda;
}

/**
 * @brief Handles events triggered by changes on the SCL pin. (ISR)
 * This function is responsible for clocking in bits and identifying the address/data bytes.
 */
void IRAM_ATTR EsphomeI2cSniffer::on_scl_edge_() {
  // If we are not currently in an I2C transfer (i.e., no START condition detected),
  // just update the last SCL state and exit.
  if (!this->in_transfer_) {
    this->last_scl_ = digitalRead(this->scl_pin_);
    return;
  }

  bool scl = digitalRead(this->scl_pin_);
  
  // I2C data is sampled on the RISING edge of SCL.
  if (scl) {
    // If we are in the 9th bit (ACK/NACK) phase, we skip data sampling
    // and reset the state machine for the next 8-bit block.
    if (this->ack_phase_) {
      this->ack_phase_ = false;       // Exit ACK phase.
      this->bit_idx_ = 0;             // Reset bit counter.
      this->cur_byte_ = 0;            // Reset current byte buffer.
      return;
    }

    bool sda = digitalRead(this->sda_pin_);
    
    // Sample the SDA line and shift the current byte left, adding the new bit.
    this->cur_byte_ = (this->cur_byte_ << 1) | (sda ? 1 : 0);
    this->bit_idx_++;

    // Check if 8 bits (a full byte) have been sampled.
    if (this->bit_idx_ >= 8) {
      this->ack_phase_ = true;        // The next bit (9th) will be the ACK/NACK.
      this->bit_idx_ = 0;             // Reset bit index for the next byte.

      if (!this->have_addr_) {
        // This is the first byte: the address + R/W bit.
        
        // The 7-bit address is bits 7:1 of the sampled byte.
        this->addr_ = (this->cur_byte_ >> 1) & 0x7F;
        // The Read/Write (R/W) bit is bit 0.
        this->rw_ = (this->cur_byte_ & 0x01) != 0;
        this->have_addr_ = true;
        
        // Signal the main loop that a new address was just detected (for on_address trigger).
        this->detected_addr_buf_ = this->addr_;
        this->detected_rw_buf_ = this->rw_;
        this->new_addr_event_ = true;

      } else {
        // This is a data byte.
        if (this->data_len_ < MAX_DATA_) {
          // Store the data byte if the buffer isn't full.
          this->data_[this->data_len_++] = this->cur_byte_;
        }
      }
      
      this->cur_byte_ = 0; // Clear the byte buffer for the next byte.
    }
  }

  // Update the stored state for the next edge detection.
  this->last_scl_ = scl;
}

/**
 * @brief Publishes the captured frame data to the various ESPHome entities.
 * This function should only be called from the main loop() to ensure thread safety
 * with the ESPHome framework's publishing mechanisms.
 * @param addr The 7-bit I2C address.
 * @param rw The Read/Write bit (true for Read, false for Write).
 * @param data Pointer to the captured data buffer.
 * @param len The length of the data buffer.
 */
void EsphomeI2cSniffer::publish_frame_(uint8_t addr, bool rw, const uint8_t *data, uint8_t len) {
  // Build a human-readable log string containing all frame details.
  char buf[256];
  int pos = snprintf(buf, sizeof(buf), "ADDR 0x%02X %c LEN=%u DATA:", addr, rw ? 'R' : 'W', len);

  // Append data bytes to the string (up to buffer size).
  for (uint8_t i = 0; i < len && pos < sizeof(buf) - 4; i++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X", data[i]);
  }

  std::string out(buf);

  // Publish the full frame string to the text sensor.
  if (this->msg_sensor_ != nullptr) {
    this->msg_sensor_->publish_state(out);
  }

  // Publish numeric sensors.
  if (this->last_address_sensor_ != nullptr) {
    this->last_address_sensor_->publish_state(static_cast<float>(addr));
  }

  if (this->last_data_sensor_ != nullptr) {
    this->last_data_sensor_->publish_state(static_cast<float>(len));
  }

  if (this->last_byte_sensor_ != nullptr) {
    uint8_t last = (len > 0) ? data[len - 1] : 0;
    this->last_byte_sensor_->publish_state(static_cast<float>(last));
  }

  // Log the frame for debugging purposes.
  ESP_LOGD(TAG, "%s", out.c_str());
}

/**
 * @brief Main component loop for handling events from the ISR.
 * This is where interrupts are re-enabled, and the data is processed/published.
 */
void EsphomeI2cSniffer::loop() {
  // 1. Handle Address Triggers (High priority and quick execution)
  if (this->new_addr_event_) {
    uint8_t addr;
    bool rw;
    
    // Disable interrupts temporarily to safely read volatile state variables
    // modified by the ISR, ensuring atomicity.
    noInterrupts(); 
    if (this->new_addr_event_) {
        addr = this->detected_addr_buf_;
        rw = this->detected_rw_buf_;
        this->new_addr_event_ = false; // Reset the event flag
        interrupts(); // Re-enable interrupts immediately after the atomic read
        
        // Execute the user-defined automation triggers (on_address).
        this->on_address_triggers_.call(addr, rw);
    } else {
        interrupts(); // Re-enable interrupts if the check failed
    }
  }

  // 2. Handle Frame Completion
  if (!this->frame_ready_)
    return; // Nothing to process yet.

  // Variables to hold the snapshot of the completed frame data.
  uint8_t addr, len;
  bool rw;
  uint8_t data_snapshot[MAX_DATA_];

  // Atomic snapshot of the frame data collected by the ISR.
  noInterrupts();
  if (!this->frame_ready_) {
    // Re-check after disabling interrupts, in case the ISR reset it immediately before noInterrupts() was called.
    interrupts();
    return;
  }
  
  // Take the snapshot
  this->frame_ready_ = false;
  addr = this->addr_;
  rw = this->rw_;
  len = this->data_len_;
  if (len > MAX_DATA_) len = MAX_DATA_;
  memcpy(data_snapshot, this->data_, len);
  
  interrupts(); // Re-enable interrupts immediately after the critical section.

  // Publish the frame data outside of the critical section.
  this->publish_frame_(addr, rw, data_snapshot, len);
}

}  // namespace esphome_i2c_sniffer
}  // namespace esphome
