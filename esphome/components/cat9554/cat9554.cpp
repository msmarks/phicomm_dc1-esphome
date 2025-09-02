#include "cat9554.h"
#include "esphome/core/log.h"

namespace esphome {
namespace cat9554 {

static const char *const TAG = "cat9554";

void CAT9554Component::setup() {
  // Test to see if device exists
  if (!this->read_inputs_()) {
    ESP_LOGE(TAG, "CAT9554 not detected at 0x%02X", this->address_);
    this->mark_failed();
    return;
  }

  // Set up IRQ pin if configured
  if (this->enable_irq_) {
    this->irq_pin_->setup();
    this->irq_pin_->pin_mode(gpio::FLAG_INPUT);
    this->update_gpio_ = false;
  }

  // All inputs at initialization
  this->config_mask_ = 0xFF;  // 1 means input, 0 means output for CAT9554
  this->config_gpio_();
  // All outputs low
  this->output_mask_ = 0;
  this->write_gpio_();
  // Read the inputs
  this->read_inputs_();
  ESP_LOGD(TAG, "Initialization complete. Warning: %d, Error: %d", this->status_has_warning(),
           this->status_has_error());
}

void CAT9554Component::loop() {
  bool need_update = false;
  
  // Check IRQ pin if enabled
  if (this->enable_irq_ && !this->irq_pin_->digital_read()) {
    need_update = true;
    this->update_gpio_ = false;
  }
  
  // Update inputs if needed or no IRQ
  if (!this->enable_irq_ || need_update) {
    this->read_inputs_();
  }
  
  // Clear all the previously read flags.
  this->was_previously_read_ = 0x00;
}

void CAT9554Component::dump_config() {
  ESP_LOGCONFIG(TAG, "CAT9554:");
  LOG_I2C_DEVICE(this)
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
  }
}

bool CAT9554Component::digital_read(uint8_t pin) {
  // Note: We want to try and avoid doing any I2C bus read transactions here
  // to conserve I2C bus bandwidth. So what we do is check to see if we
  // have seen a read during the time esphome is running this loop. If we have,
  // we do an I2C bus transaction to get the latest value. If we haven't
  // we return a cached value which was read at the time loop() was called.
  if (this->was_previously_read_ & (1 << pin))
    this->read_inputs_();  // Force a read of a new value
  // Indicate we saw a read request for this pin in case a
  // read happens later in the same loop.
  this->was_previously_read_ |= (1 << pin);
  return this->input_mask_ & (1 << pin);
}

void CAT9554Component::digital_write(uint8_t pin, bool value) {
  if (value) {
    this->output_mask_ |= (1 << pin);
  } else {
    this->output_mask_ &= ~(1 << pin);
  }
  this->write_gpio_();
}

void CAT9554Component::pin_mode(uint8_t pin, gpio::Flags flags) {
  if (flags == gpio::FLAG_INPUT) {
    // Set mode mask bit (1 means input for CAT9554)
    this->config_mask_ |= (1 << pin);
  } else if (flags == gpio::FLAG_OUTPUT) {
    // Clear mode mask bit (0 means output for CAT9554)
    this->config_mask_ &= ~(1 << pin);
  }
  this->config_gpio_();
}

bool CAT9554Component::read_inputs_() {
  if (this->is_failed()) {
    ESP_LOGD(TAG, "Device marked failed");
    return false;
  }

  uint8_t data;
  this->last_error_ = this->read_register(INPUT_REG, &data, 1);
  if (this->last_error_ != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGE(TAG, "read_inputs_(): I2C I/O error: %d", (int) this->last_error_);
    return false;
  }
  
  this->status_clear_warning();
  this->input_mask_ = data;
  return true;
}

bool CAT9554Component::write_gpio_() {
  if (this->is_failed())
    return false;

  this->last_error_ = this->write_register(OUTPUT_REG, &this->output_mask_, 1);
  if (this->last_error_ != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGE(TAG, "write_gpio_(): I2C I/O error: %d", (int) this->last_error_);
    return false;
  }

  this->status_clear_warning();
  return true;
}

bool CAT9554Component::config_gpio_() {
  if (this->is_failed())
    return false;

  this->last_error_ = this->write_register(CONFIG_REG, &this->config_mask_, 1);
  if (this->last_error_ != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGE(TAG, "config_gpio_(): I2C I/O error: %d", (int) this->last_error_);
    return false;
  }

  this->status_clear_warning();
  return true;
}

bool CAT9554Component::read_config_() {
  if (this->is_failed())
    return false;

  uint8_t data;
  this->last_error_ = this->read_register(CONFIG_REG, &data, 1);
  if (this->last_error_ != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGE(TAG, "read_config_(): I2C I/O error: %d", (int) this->last_error_);
    return false;
  }
  
  this->config_mask_ = data;
  this->status_clear_warning();
  return true;
}

float CAT9554Component::get_setup_priority() const { return setup_priority::IO; }

// Run our loop() method very early in the loop, so that we cache read values before
// before other components call our digital_read() method.
float CAT9554Component::get_loop_priority() const { return 9.0f; }  // Just after WIFI

void CAT9554GPIOPin::setup() { pin_mode(flags_); }
void CAT9554GPIOPin::pin_mode(gpio::Flags flags) { this->parent_->pin_mode(this->pin_, flags); }
bool CAT9554GPIOPin::digital_read() { return this->parent_->digital_read(this->pin_) != this->inverted_; }
void CAT9554GPIOPin::digital_write(bool value) { this->parent_->digital_write(this->pin_, value != this->inverted_); }
std::string CAT9554GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via CAT9554", pin_);
  return buffer;
}

}  // namespace cat9554
}  // namespace esphome
