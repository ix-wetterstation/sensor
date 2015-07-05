#ifndef HTU21_H
#define HTU21_H

#include <stdint.h>
#include <stdbool.h>

#define HTU21_ADDRESS 0x80

enum htu21_command_t {
    HTU21_READ_TEMPERATURE_BLOCKING = 0xE3,
    HTU21_READ_HUMIDITY_BLOCKING = 0xE5,
    HTU21_READ_TEMPERATURE = 0xF3,
    HTU21_READ_HUMIDITY = 0xF5,
    HTU21_WRITE_USER_REG = 0xE6,
    HTU21_READ_USER_REG = 0xE7,
    HTU21_SOFT_RESET = 0xFE,
};

struct htu21_user_register_t {
    union {
        struct {
            enum {
                HTU21_RES_RH12_T14 = 0x0, /* default */
                HTU21_RES_RH08_T12 = 0x1,
                HTU21_RES_RH10_T13 = 0x2,
                HTU21_RES_RH11_T11 = 0x3,
            } resolution : 2;
            uint8_t end_of_batt : 1;
            uint8_t pad : 3;
            uint8_t enable_onchip_heater : 1;
            uint8_t disable_otp_reload : 1;
        };
        uint8_t raw;
    };
};

/* note: takes less than 15ms */
void htu21_reset();
bool htu21_read_temperature    (int16_t *value);
bool htu21_read_humidity       (uint8_t *value);
bool htu21_read_user_register  (struct htu21_user_register_t* user_reg);
bool htu21_write_user_register (struct htu21_user_register_t* user_reg);

#endif /* HTU21_H */
