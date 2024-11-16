#include "sim800l.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"

void sim800l_init(){
    uart_init(uart0, 9600);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
}

uint8_t sim800l_get_command(int32_t num){
    uint8_t command = 5;
    return command;
}