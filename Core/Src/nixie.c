#include "nixie.h"

GPIO_TypeDef *_nixie_a_ports[] = {A1_GPIO_Port, A2_GPIO_Port, A3_GPIO_Port, A4_GPIO_Port, A5_GPIO_Port, A6_GPIO_Port};
uint16_t _nixie_a_pins[] = {A1_Pin, A2_Pin, A3_Pin, A4_Pin, A5_Pin, A6_Pin};

GPIO_TypeDef *_nixie_d_ports[] = {D0_GPIO_Port, D1_GPIO_Port, D2_GPIO_Port, D3_GPIO_Port, D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port, D8_GPIO_Port, D9_GPIO_Port};
uint16_t _nixie_d_pins[] = {D0_Pin, D1_Pin, D2_Pin, D3_Pin, D4_Pin, D5_Pin, D6_Pin, D7_Pin, D8_Pin, D9_Pin};

uint8_t _displayed_digits[DIGITS_NUM]; // Array to hold the digits for the Nixie tube

uint8_t _current_displayed_index = 0; // Index of the currently displayed digit

void nixie_setDigits(uint8_t *digit)
{
    for (int i = 0; i < DIGITS_NUM; i++)
    {
        _displayed_digits[i] = digit[i];
    }
}

void nixie_refresh(void)
{
    for (int i = 0; i < 10; i++)
    {
        HAL_GPIO_WritePin(_nixie_d_ports[i], _nixie_d_pins[i], GPIO_PIN_RESET);
    }

    for (int i = 0; i < DIGITS_NUM; i++)
    {
        HAL_GPIO_WritePin(_nixie_a_ports[i], _nixie_a_pins[i], (i == _current_displayed_index) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    uint8_t dispDig = _displayed_digits[_current_displayed_index];

    if (dispDig < EMPTY_DIGIT)
        HAL_GPIO_WritePin(_nixie_d_ports[dispDig], _nixie_d_pins[dispDig], GPIO_PIN_SET);

    _current_displayed_index = (_current_displayed_index + 1) % DIGITS_NUM;
}

uint8_t nixie_blink(uint8_t digit)
{
    if (HAL_GetTick() % 1000 >= 500) // Toggle every 500 ms
    {
        return digit;
    }
    else
    {
        return EMPTY_DIGIT;
    }
}