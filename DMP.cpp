// #include "bmp280.hpp"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pico/util/datetime.h"
// #include "sim800l.h"
#include <cstdint>
#include <hardware/gpio.h>
#include <pico.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdio.h>
// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for
// information on GPIO assignments

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart0
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for
// information on GPIO assignments

// RTC
//******************************* */
datetime_t t = {.year = 2020,
                .month = 06,
                .day = 05,
                .dotw = 5, // 0 is Sunday, so 5 is Friday
                .hour = 15,
                .min = 45,
                .sec = 00};

// BMP280
//***************************** */
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define ADDR _u(0x76)

// hardware registers
#define REG_CONFIG _u(0xF5)
#define REG_CTRL_MEAS _u(0xF4)
#define REG_RESET _u(0xE0)

#define REG_TEMP_XLSB _u(0xFC)
#define REG_TEMP_LSB _u(0xFB)
#define REG_TEMP_MSB _u(0xFA)

#define REG_PRESSURE_XLSB _u(0xF9)
#define REG_PRESSURE_LSB _u(0xF8)
#define REG_PRESSURE_MSB _u(0xF7)

// calibration registers
#define REG_DIG_T1_LSB _u(0x88)
#define REG_DIG_T1_MSB _u(0x89)
#define REG_DIG_T2_LSB _u(0x8A)
#define REG_DIG_T2_MSB _u(0x8B)
#define REG_DIG_T3_LSB _u(0x8C)
#define REG_DIG_T3_MSB _u(0x8D)
#define REG_DIG_P1_LSB _u(0x8E)
#define REG_DIG_P1_MSB _u(0x8F)
#define REG_DIG_P2_LSB _u(0x90)
#define REG_DIG_P2_MSB _u(0x91)
#define REG_DIG_P3_LSB _u(0x92)
#define REG_DIG_P3_MSB _u(0x93)
#define REG_DIG_P4_LSB _u(0x94)
#define REG_DIG_P4_MSB _u(0x95)
#define REG_DIG_P5_LSB _u(0x96)
#define REG_DIG_P5_MSB _u(0x97)
#define REG_DIG_P6_LSB _u(0x98)
#define REG_DIG_P6_MSB _u(0x99)
#define REG_DIG_P7_LSB _u(0x9A)
#define REG_DIG_P7_MSB _u(0x9B)
#define REG_DIG_P8_LSB _u(0x9C)
#define REG_DIG_P8_MSB _u(0x9D)
#define REG_DIG_P9_LSB _u(0x9E)
#define REG_DIG_P9_MSB _u(0x9F)

// number of calibration registers to be read
#define NUM_CALIB_PARAMS 24

struct bmp280_calib_param {
  // temperature params
  uint16_t dig_t1;
  int16_t dig_t2;
  int16_t dig_t3;

  // pressure params
  uint16_t dig_p1;
  int16_t dig_p2;
  int16_t dig_p3;
  int16_t dig_p4;
  int16_t dig_p5;
  int16_t dig_p6;
  int16_t dig_p7;
  int16_t dig_p8;
  int16_t dig_p9;
};

#ifdef i2c_default
void bmp280_init() {
  // use the "handheld device dynamic" optimal setting (see datasheet)
  uint8_t buf[2];

  // 500ms sampling time, x16 filter
  const uint8_t reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;

  // send register number followed by its corresponding value
  buf[0] = REG_CONFIG;
  buf[1] = reg_config_val;
  i2c_write_blocking(i2c_default, ADDR, buf, 2, false);

  // osrs_t x1, osrs_p x4, normal mode operation
  const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | (0x03);
  buf[0] = REG_CTRL_MEAS;
  buf[1] = reg_ctrl_meas_val;
  i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
}

void bmp280_read_raw(int32_t *temp, int32_t *pressure) {
  // BMP280 data registers are auto-incrementing and we have 3 temperature and
  // pressure registers each, so we start at 0xF7 and read 6 bytes to 0xFC
  // note: normal mode does not require further ctrl_meas and config register
  // writes

  uint8_t buf[6];
  uint8_t reg = REG_PRESSURE_MSB;
  i2c_write_blocking(i2c_default, ADDR, &reg, 1,
                     true); // true to keep master control of bus
  i2c_read_blocking(i2c_default, ADDR, buf, 6,
                    false); // false - finished with bus

  // store the 20 bit read in a 32 bit signed integer for conversion
  *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
  *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
}

void bmp280_reset() {
  // reset the device with the power-on-reset procedure
  uint8_t buf[2] = {REG_RESET, 0xB6};
  i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
}

// intermediate function that calculates the fine resolution temperature
// used for both pressure and temperature conversions
int32_t bmp280_convert(int32_t temp, struct bmp280_calib_param *params) {
  // use the 32-bit fixed point compensation implementation given in the
  // datasheet

  int32_t var1, var2;
  var1 = ((((temp >> 3) - ((int32_t)params->dig_t1 << 1))) *
          ((int32_t)params->dig_t2)) >>
         11;
  var2 = (((((temp >> 4) - ((int32_t)params->dig_t1)) *
            ((temp >> 4) - ((int32_t)params->dig_t1))) >>
           12) *
          ((int32_t)params->dig_t3)) >>
         14;
  return var1 + var2;
}

int32_t bmp280_convert_temp(int32_t temp, struct bmp280_calib_param *params) {
  // uses the BMP280 calibration parameters to compensate the temperature value
  // read from its registers
  int32_t t_fine = bmp280_convert(temp, params);
  return (t_fine * 5 + 128) >> 8;
}

int32_t bmp280_convert_pressure(int32_t pressure, int32_t temp,
                                struct bmp280_calib_param *params) {
  // uses the BMP280 calibration parameters to compensate the pressure value
  // read from its registers

  int32_t t_fine = bmp280_convert(temp, params);

  int32_t var1, var2;
  uint32_t converted = 0.0;
  var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)params->dig_p6);
  var2 += ((var1 * ((int32_t)params->dig_p5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)params->dig_p4) << 16);
  var1 = (((params->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
          ((((int32_t)params->dig_p2) * var1) >> 1)) >>
         18;
  var1 = ((((32768 + var1)) * ((int32_t)params->dig_p1)) >> 15);
  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  converted =
      (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
  if (converted < 0x80000000) {
    converted = (converted << 1) / ((uint32_t)var1);
  } else {
    converted = (converted / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)params->dig_p9) *
          ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >>
         12;
  var2 = (((int32_t)(converted >> 2)) * ((int32_t)params->dig_p8)) >> 13;
  converted =
      (uint32_t)((int32_t)converted + ((var1 + var2 + params->dig_p7) >> 4));
  return converted;
}

void bmp280_get_calib_params(struct bmp280_calib_param *params) {
  // raw temp and pressure values need to be calibrated according to
  // parameters generated during the manufacturing of the sensor
  // there are 3 temperature params, and 9 pressure params, each with a LSB
  // and MSB register, so we read from 24 registers

  uint8_t buf[NUM_CALIB_PARAMS] = {0};
  uint8_t reg = REG_DIG_T1_LSB;
  i2c_write_blocking(i2c_default, ADDR, &reg, 1,
                     true); // true to keep master control of bus
  // read in one go as register addresses auto-increment
  i2c_read_blocking(i2c_default, ADDR, buf, NUM_CALIB_PARAMS,
                    false); // false, we're done reading

  // store these in a struct for later use
  params->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
  params->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
  params->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

  params->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
  params->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
  params->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
  params->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
  params->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
  params->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
  params->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
  params->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
  params->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];
}

void bmp280_senzor_init() {
  i2c_init(i2c_default, 100 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  // configure BMP280
  bmp280_init();
}

#endif

void init_all() {
  gpio_init(15); // Door End Stop
  gpio_set_dir(15, GPIO_IN);
  gpio_pull_up(15);

  gpio_init(25); // Build-in LED
  gpio_set_dir(25, GPIO_OUT);

  gpio_init(16);
  gpio_init(17);
  gpio_set_dir(16, GPIO_OUT);
  gpio_set_dir(17, GPIO_OUT);

  adc_init();
  adc_gpio_init(27);
  adc_gpio_init(28);
}
uint32_t get_current(uint adc_channel) {
  // const uint32_t conversion_factor = 3300 / (1 << 12);
  const float adc_const = 3.3 / (1 << 12);
  const int amp_const = 123;
  float voltage_33;
  uint voltage_5;
  float current;
  adc_select_input(adc_channel);
  voltage_33 = adc_read() * adc_const;
  current = (voltage_33 - 1680) * amp_const;
  // return current;
  return voltage_33;
}

void door_control(bool state) {
  // 0-CLOSE | 1-OPEN
  uint16_t timeout = 0;
  if (state) {
    gpio_put(16, 1);
    timeout = 2800;
  } else {
    gpio_put(17, 1);
    timeout = 2800;
  }

  while (timeout > 0) {
    timeout--;
    // Check current!
    // Watch Light Gate!

    sleep_ms(1);
  }

  gpio_put(16, 0);
  gpio_put(17, 0);
}
/*
  M     M      A      II  N    N
  MMM MMM     A A     II  NN   N
  M  M  M    A   A    II  N N  N
  M     M   AAAAAAA   II  N  N N
  M     M  A       A  II  N   NN
  M     M  A       A  II  N    N
 */

int main() {
  const int amp_offset = 1680;
  const float conversion_factor = 3.3f / (1 << 12);

  char datetime_buf[256];
  char *datetime_str = &datetime_buf[0];

  rtc_init();

  rtc_set_datetime(&t);

  stdio_init_all();

  init_all();

  gpio_put(25, 1);

  bmp280_senzor_init();

  // retrieve fixed compensation params
  struct bmp280_calib_param params;
  bmp280_get_calib_params(&params);

  int32_t raw_temperature;
  int32_t raw_pressure;

  sleep_ms(250); // sleep so that data polling and register update don't collide

  // bmp280_init();
  // sim800l_init();
  //  I2C Initialisation. Using it at 400Khz.
  //  For more examples of I2C use see
  //  https://github.com/raspberrypi/pico-examples/tree/master/i2c

  // Set up our UART
  uart_init(UART_ID, BAUD_RATE);

  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  // Use some the various UART functions to send out data
  // In a default system, printf will also output via the default UART

  // Send out a string, with CR/LF conversions
  uart_puts(UART_ID, " o - open; c - close; h - help\n");

  // For more examples of UART use see
  // https: //github.com/raspberrypi/pico-examples/tree/master/uart

  while (0) {
    bmp280_read_raw(&raw_temperature, &raw_pressure);
    int32_t temperature = bmp280_convert_temp(raw_temperature, &params);
    int32_t pressure =
        bmp280_convert_pressure(raw_pressure, raw_temperature, &params);
    char x = temperature;
    uart_putc(UART_ID, x);
    uart_putc(UART_ID, '\n');
    printf("Pressure = %.3f kPa\n", pressure / 1000.f);
    printf("Temp. = %.2f C\n", temperature / 100.f);
    // poll every 500ms
    sleep_ms(1000);
  }

  while (0) {
    if (gpio_get(15)) {
      gpio_put(25, 1);
    } else {
      gpio_put(25, 0);
    }
  }

  uint32_t x = get_current(1);
  char buff[20];
  sprintf(buff, "%u \n", x);
  while (1) {
    switch (uart_getc(UART_ID)) {
    case 'h':
      uart_puts(UART_ID, "\ro - Open door; c - Close door; h - Show this "
                         "page; t - Prints system time; ");
      uart_putc(UART_ID, 0x0A);
      // uart_putc(UART_ID, '\n');
      break;
    case 'o':
      gpio_put(25, 1);
      door_control(1);
      break;
    case 'c':
      gpio_put(25, 0);
      door_control(0);
      break;
    case 't':
      rtc_get_datetime(&t);
      datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
      sprintf(buff, "\r%s", datetime_str);
      uart_puts(UART_ID, buff);
    }
  }
}
