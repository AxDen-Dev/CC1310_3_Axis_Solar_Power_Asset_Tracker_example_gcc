#ifndef BOARD_DEFINE_H_
#define BOARD_DEFINE_H_

#include <ti/drivers/Board.h>

#define LED_RED_GPIO IOID_1
#define LED_BLUE_GPIO IOID_0

#define HALL_SENSOR_GPIO IOID_2

#define BAT_EN_GPIO IOID_7
#define BAT_LEVEL_ADC IOID_6

#define GPS_POWER_EN_GPIO IOID_5

#define I2C_SDA IOID_8
#define I2C_SCL IOID_9

#define UART_RX IOID_4
#define UART_TX IOID_3

#endif /* BOARD_DEFINE_H_ */
