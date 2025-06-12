#include "keypad_driver.h"

static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

/**
 * @brief Inicializa el controlador del teclado.
 * @param keypad: Puntero al manejador del teclado.
 * Rows are outputs and columns are inputs with external interrupts
 */
void keypad_init(keypad_handle_t* keypad) {
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        // Configurar las filas como salidas
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET);
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = keypad->row_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(keypad->row_ports[i], &GPIO_InitStruct);
    }

    for (int i = 0; i < KEYPAD_COLS; i++) {
        // Configurar las columnas como entradas con interrupciones externas y pull-up
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = keypad->col_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(keypad->col_ports[i], &GPIO_InitStruct);
    }
}
/**
 * @brief Escanea el teclado y devuelve la tecla presionada.
 * @param keypad: Puntero al manejador del teclado.
 * @param col_pin: Pin de la columna a escanear.
 * @return La tecla presionada o '\0' si no hay ninguna tecla presionada.
 */
char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin) {
    char key_pressed = '\0';
    static uint32_t last_press_time = 0;
    if (HAL_GetTick() - last_press_time < 100) {
        // Anti-rebote: espera 100 ms antes de procesar la siguiente tecla
        return key_pressed;
    }
    last_press_time = HAL_GetTick();
    switch (col_pin)
    {
    case KEYPAD_C1_Pin:
        for (int i = 0; i < KEYPAD_ROWS; i++) {
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET);
            if (HAL_GPIO_ReadPin(keypad->col_ports[0], KEYPAD_C1_Pin) == GPIO_PIN_RESET) {
                key_pressed = keypad_map[i][0];
            }
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET);
        }
        break;
    case KEYPAD_C2_Pin:
        for (int i = 0; i < KEYPAD_ROWS; i++) {
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET);
            if (HAL_GPIO_ReadPin(keypad->col_ports[1], KEYPAD_C2_Pin) == GPIO_PIN_RESET) {
                key_pressed = keypad_map[i][1];
            }
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET);
        }
        break;
    case KEYPAD_C3_Pin:
        for (int i = 0; i < KEYPAD_ROWS; i++) {
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET);
            if (HAL_GPIO_ReadPin(keypad->col_ports[2], KEYPAD_C3_Pin) == GPIO_PIN_RESET) {
                key_pressed = keypad_map[i][2];
            }
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET);
        }
        break;
    case KEYPAD_C4_Pin:
        for (int i = 0; i < KEYPAD_ROWS; i++) {
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET);
            if (HAL_GPIO_ReadPin(keypad->col_ports[3], KEYPAD_C4_Pin) == GPIO_PIN_RESET) {
                key_pressed = keypad_map[i][3];
            }
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET);
        }
        break;
    
    default:
        // Si no se reconoce el pin de la columna, no se presiona ninguna tecla
        return key_pressed;
    }
    return key_pressed;
}