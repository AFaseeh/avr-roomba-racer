#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

typedef enum {
    PORT_B,
    PORT_C,
    PORT_D
} GPIO_Port_t;

typedef enum { GPIO_DIR_INPUT, GPIO_DIR_OUTPUT } GPIO_Dir_t;
typedef enum { GPIO_PIN_LOW, GPIO_PIN_HIGH } GPIO_State_t;

void gpio_set_pin_direction(GPIO_Port_t port, uint8_t pin, GPIO_Dir_t direction);
void gpio_write_pin(GPIO_Port_t port, uint8_t pin, GPIO_State_t state);
GPIO_State_t gpio_read_pin(GPIO_Port_t port, uint8_t pin);

#endif /* GPIO_H */