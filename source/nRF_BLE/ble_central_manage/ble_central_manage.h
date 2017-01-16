/**
  ****************************************************************************************
  * @file    ble_central_manage.h
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   header of ble_central_manage.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_CENTRAL_MANAGE_H__
#define __BLE_CENTRAL_MANAGE_H__

#include "main.h"


/* Central related define  */

#define APP_TIMER_PRESCALER         0                                             /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE     2                                             /**< Size of timer operation queues. */


#define SCAN_INTERVAL               0x0140                                        /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0100                                        /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT                (uint16_t) MSEC_TO_UNITS(1000, UNIT_1_25_MS)

#define MIN_CONNECTION_INTERVAL     (uint16_t) MSEC_TO_UNITS(100, UNIT_1_25_MS)   /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL     (uint16_t) MSEC_TO_UNITS(200, UNIT_1_25_MS)    /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY               0                                             /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT         (uint16_t) MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 milliseconds. */


/* self define service UUID */
#define YWK_DEVICE_CONFIRM_UUID     0x2311      //云卫康设备认证服务UUID，在广播包中包含改服务UUID的即认为是云卫康设备


ret_code_t scan_start(void);
ret_code_t scan_stop(void);
void on_ble_central_evt(const ble_evt_t * const p_ble_evt);





#endif // __BLE_CENTRAL_MANAGE_H__

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




