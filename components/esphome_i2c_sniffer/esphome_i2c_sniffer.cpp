/**
 * Change code to ESP32 based device with ESP-IDF framework
 */
#include "esphome_i2c_sniffer.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32_FRAMEWORK_ESP_IDF
#include <cstdio>
#include <cstring>
#include "driver/gpio.h"

namespace esphome {
namespace esphome_i2c_sniffer {

static const char *const TAG = "i2c_sniffer";

/**
 * @brief Unified ISR handler for ESP-IDF.
 * ESP-IDF allows specific handlers per pin, but since I2C START/STOP 
 * requires checking both pins simultaneously, we route both to this handler.
 */
static void IRAM_ATTR gpio_isr_handler(void *arg) {
  auto *sniffer = reinterpret_cast<EsphomeI2cSniffer *>(arg);
  // On ESP32, we check both edges to maintain the protocol state machine
  sniffer->on_sda_edge_();
  sniffer->on_scl_edge_();
}

void EsphomeI2cSniffer::setup() {
  ESP_LOGCONFIG(TAG, "Setting up I2C Sniffer (ESP-IDF)...");

  // Configure GPIOs using the ESP-IDF driver
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_ANYEDGE;         // Trigger on both rising and falling edges
  io_conf.mode = GPIO_MODE_INPUT;                // Set as input
  io_conf.pin_bit_mask = (1ULL << this->sda_pin_) | (1ULL << this->scl_pin_);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;       // I2C requires pull-ups
  gpio_config(&io_conf);

  // Install the global ISR service (only needs to be called once for the whole SoC)
  // ESPHome usually handles this, but we use ESP_OK check to be safe.
  gpio_install_isr_service(0);

  // Hook the specific pin interrupts to our handler
  gpio_isr_handler_add(this->sda_pin_, gpio_isr_handler, (void *) this);
  gpio_isr_handler_add(this->scl_pin_, gpio_isr_handler, (void *) this);

  // Initial state capture using ESP-IDF gpio_get_level
  this->last_sda_ = gpio_get_level(this->sda_pin_);
  this->last_scl_ = gpio_get_level(this->scl_pin_);
}

void EsphomeI2cSniffer::on_sda_edge_() {
  bool sda = gpio_get_level(this->sda_pin_);
  bool scl = gpio_get_level(this->scl_pin_);

  // Detection logic for START and STOP conditions
  if (!sda && this->last_sda_ && scl) {
    this->in_transfer_ = true;
    this->bit_idx_ = 0;
    this->ack_phase_ = false;
    this->cur_byte_ = 0;
    this->have_addr_ = false;
    this->last_was_addr_ = false;
    this->addr_ack_ = true;
    this->data_len_ = 0;
    this->frame_time_ms_ = millis();
  } else if (sda && !this->last_sda_ && scl) {
    if (this->in_transfer_) {
      this->in_transfer_ = false;
      this->frame_ready_ = true;
    }
  }
  this->last_sda_ = sda;
}

void EsphomeI2cSniffer::on_scl_edge_() {
  bool scl = gpio_get_level(this->scl_pin_);
  if (!this->in_transfer_) {
    this->last_scl_ = scl;
    return;
  }

  // Sample data on the rising edge of SCL
  if (scl && !this->last_scl_) {
    if (this->ack_phase_) {
      // 9th clock: SDA low = ACK, SDA high = NACK
      bool nack = gpio_get_level(this->sda_pin_);
      if (this->last_was_addr_) {
        this->addr_ack_ = !nack;
      } else if (this->data_len_ > 0 && this->data_len_ <= MAX_DATA_) {
        this->data_ack_[this->data_len_ - 1] = !nack;
      }
      this->ack_phase_ = false;
      this->bit_idx_ = 0;
    } else {
      bool sda = gpio_get_level(this->sda_pin_);
      this->cur_byte_ = (this->cur_byte_ << 1) | (sda ? 1 : 0);
      this->bit_idx_++;

      if (this->bit_idx_ >= 8) {
        this->ack_phase_ = true;
        if (!this->have_addr_) {
          // NOTE: addr_ intentionally stores the *raw* 8-bit byte as seen on
          // the wire (7-bit address shifted left, OR'd with the R/W bit in
          // bit 0) rather than the bare 7-bit address. Many real-world I2C
          // captures/tools report addresses this way (e.g. a write to 7-bit
          // address 0x40 shows up as 0x80), and it lets the low bit double
          // as an inline read/write indicator. This applies consistently to
          // last_addr_sensor, the on_address "address" trigger argument, and
          // the address token in the published message.
          this->addr_ = this->cur_byte_;
          this->rw_ = (this->cur_byte_ & 0x01);
          this->have_addr_ = true;
          this->last_was_addr_ = true;

          // Store detected address for the loop trigger
          this->detected_addr_buf_ = this->addr_;
          this->detected_rw_buf_ = this->rw_;
          this->new_addr_event_ = true;
        } else if (this->data_len_ < MAX_DATA_) {
          this->data_[this->data_len_++] = this->cur_byte_;
          this->last_was_addr_ = false;
        }
        this->cur_byte_ = 0;
      }
    }
  }
  this->last_scl_ = scl;
}

void EsphomeI2cSniffer::loop() {
  // Handle the 'on_address' automation trigger
  if (this->new_addr_event_) {
    uint8_t a; bool r;
    // Enter critical section to read volatile ISR data safely
    portENTER_CRITICAL(&this->lock_);
    a = this->detected_addr_buf_;
    r = this->detected_rw_buf_;
    this->new_addr_event_ = false;
    portEXIT_CRITICAL(&this->lock_);
    
    this->on_address_triggers_.call(a, r);
  }

  // Handle full frame publication
  if (this->frame_ready_) {
    uint8_t a, l; bool r, addr_ack; uint32_t t_ms;
    uint8_t snap[MAX_DATA_];
    bool snap_ack[MAX_DATA_];

    portENTER_CRITICAL(&this->lock_);
    this->frame_ready_ = false;
    a = this->addr_; r = this->rw_; l = this->data_len_; addr_ack = this->addr_ack_;
    t_ms = this->frame_time_ms_;
    if (l > MAX_DATA_) l = MAX_DATA_;
    memcpy(snap, this->data_, l);
    memcpy(snap_ack, this->data_ack_, l);
    portEXIT_CRITICAL(&this->lock_);

    this->publish_frame_(a, r, addr_ack, snap, snap_ack, l, t_ms);
  }
}

void EsphomeI2cSniffer::publish_frame_(uint8_t addr, bool rw, bool addr_ack, const uint8_t *data,
                                        const bool *data_ack, uint8_t len, uint32_t t_ms) {
  // Format: "<t>ms <ADDR> <R|W> <DATA1> <DATA2> ... ACK:<a><d0><d1>..."
  // ADDR and every DATA byte are emitted as a *clean* two-hex-digit token
  // (no "0x" prefix, no suffix) so a simple "iterate whitespace-separated
  // tokens, keep the ones that are exactly 2 hex digits" parser - as used
  // in downstream ESPHome lambdas - reconstructs the address followed by
  // the data bytes in order, unambiguously. ACK/NACK per byte is appended
  // as a separate trailing token so it never breaks that byte-alignment.
  char buf[320];
  int pos = snprintf(buf, sizeof(buf), "%lums %02X %s", (unsigned long) t_ms, addr, rw ? "R" : "W");
  for (int i = 0; i < len && pos < (int) sizeof(buf) - 4; i++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X", data[i]);
  }
  if (pos < (int) sizeof(buf) - 8) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, " ACK:%c", addr_ack ? '+' : '-');
    for (int i = 0; i < len && pos < (int) sizeof(buf) - 2; i++) {
      pos += snprintf(buf + pos, sizeof(buf) - pos, "%c", data_ack[i] ? '+' : '-');
    }
  }

  if (this->msg_sensor_) this->msg_sensor_->publish_state(buf);
  if (this->last_addr_sensor_) this->last_addr_sensor_->publish_state(addr);
  if (this->last_data_sensor_) this->last_data_sensor_->publish_state(len);
  if (this->last_byte_sensor_ && len > 0) this->last_byte_sensor_->publish_state(data[len - 1]);
}

void EsphomeI2cSniffer::dump_config() {
  ESP_LOGCONFIG(TAG, "I2C Sniffer (ESP-IDF Framework)");
  ESP_LOGCONFIG(TAG, "  SDA Pin: %d", (int)this->sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCL Pin: %d", (int)this->scl_pin_);
}

} // namespace esphome_i2c_sniffer
} // namespace esphome

#endif
