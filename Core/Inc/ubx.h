/*
 * ubx.h
 *
 *  Created on: 2022?8?15?
 *      Author: abc
 */

#ifndef SRC_UBX_H_
#define SRC_UBX_H_

#define UBX_SYNC_CHAR1             0xB5
#define UBX_SYNC_CHAR2             0x62
#define UBX_CFG_CLASS_ID           0x06
#define UBX_CFG_TP5_MSG_ID         0x31

#define GPS_UART USART1

#define GPS_PWR_EN GPIO_PIN_12  	/*PB12*/
#define GPS_RST_PIN GPIO_PIN_13 	/*PB13*/
#define GPS_1PPS_PIN GPIO_PIN_2 	/*PD2*/
#define GPS_SYNC_PIN GPIO_PIN_12	/*PC12*/

#define ARRAYNUM(arr_name)     (uint32_t)(sizeof(arr_name) / sizeof(*(arr_name)))
#define USART1_DATA_ADDRESS    ((uint32_t)&USART_DATA(USART1))


/** Config Timepulse0 pin
        Unlock to GPS: Period 1000000us, Length 0
        Lock to GPS: Period 1000000us, Length 500000us
*/
const unsigned char UBX_MSG_DATA_SET_TIMEPULSE_0[] = {
    0x00,                   // Timepulse selection (0 = TIMEPULSE, 1 =TIMEPULSE2)
    0x01,                   // Reserved
    0x00, 0x00,             // Reserved
    0x00, 0x00,             // Antenna cable delay
    0x00, 0x00,             // RF group delay
    0x00, 0x00, 0x00, 0x00, // Frequency or period time, depending on setting of bit ’isFreq’
    0x01, 0x00, 0x00, 0x00, // Frequency or period time when locked to GPS time, only used if ’lockedOtherSet’ is set
    0x00, 0x00, 0x00, 0x00, // Pulse length or duty cycle, depending on ’isLength’
    0x00, 0x00, 0X00, 0X80, // Pulse length or duty cycle when locked to GPS time, only used if ’lockedOtherSet’ is set
    0x00, 0x00, 0x00, 0x00, // User configurable timepulse delay
    0xef, 0x00, 0x00, 0x00  // configuration flags (see graphic below)
};
/** Config Timepulse1 pin
        Unlock to GPS: Period 1000000us, Length 0
        Lock to GPS: Frequency 1024000Hz
*/
unsigned char UBX_MSG_DATA_SET_TIMEPULSE_1[] = {
    0x01,                   // Timepulse selection (0 = TIMEPULSE, 1 =TIMEPULSE2)
    0x01,                   // Reserved
    0x00, 0x00,             // Reserved
    0x00, 0x00,             // Antenna cable delay
    0x00, 0x00,             // RF group delay
    0x00, 0xA0, 0x0F, 0x00, // Frequency or period time, depending on setting of bit ’isFreq’
    0x00, 0xA0, 0x0F, 0x00, // 512kHz=0x0007D000 Frequency or period time when locked to GPS time, only used if ’lockedOtherSet’ is set
    0x00, 0x00, 0x00, 0x80, // Pulse length or duty cycle, depending on ’isLength’
    0x00, 0x00, 0X00, 0X80, // Pulse length or duty cycle when locked to GPS time, only used if ’lockedOtherSet’ is set
    0x00, 0x00, 0x00, 0x00, // User configurable timepulse delay
    0xef, 0x00, 0x00, 0x00  // configuration flags (see graphic below)
};

#endif
