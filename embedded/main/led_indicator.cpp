#include "led_indicator.h"

#include "driver/gpio.h"

// LED RGB de cátodo común:
//   GPIO25 = rojo   (Decaida)
//   GPIO26 = amarillo/naranja (Estable) — conectar LED amarillo entre GPIO26 y GND
//   GPIO33 = verde  (Ideal)
namespace {
constexpr gpio_num_t kPinRed    = GPIO_NUM_25;
constexpr gpio_num_t kPinYellow = GPIO_NUM_26;
constexpr gpio_num_t kPinGreen  = GPIO_NUM_33;

void set_pin(gpio_num_t pin, bool on) {
    gpio_set_level(pin, on ? 1 : 0);
}
}  // namespace

namespace led_indicator {

void init() {
    const gpio_num_t pins[] = {kPinRed, kPinYellow, kPinGreen};
    for (gpio_num_t pin : pins) {
        gpio_config_t cfg = {};
        cfg.intr_type    = GPIO_INTR_DISABLE;
        cfg.mode         = GPIO_MODE_OUTPUT;
        cfg.pin_bit_mask = (1ULL << pin);
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        gpio_config(&cfg);
        gpio_set_level(pin, 0);
    }
}

void set_state(PlantState state) {
    set_pin(kPinRed,    state == PlantState::Decaida);
    set_pin(kPinYellow, state == PlantState::Estable);
    set_pin(kPinGreen,  state == PlantState::Ideal);
}

}  // namespace led_indicator
