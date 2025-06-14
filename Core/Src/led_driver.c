#include "led_driver.h"
/**
 * @brief LED driver initialization.
 */
void led_init(led_handle_t *led) {
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

/**
 * @brief Turn on the LED.
 * @param led Pointer to the LED handle.
 */
void led_on(led_handle_t *led) {
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}

/**
 * @brief Turn off the LED.
 * @param led Pointer to the LED handle.
 */
void led_off(led_handle_t *led) {
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

/**
 * @brief Toggle the LED state.
 * @param led Pointer to the LED handle.
 */
void led_toggle(led_handle_t *led) {
    HAL_GPIO_TogglePin(led->port, led->pin);
}
