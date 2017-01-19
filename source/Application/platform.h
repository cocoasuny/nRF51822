/**
  ****************************************************************************************
  * @file    platform.h
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   platform define 
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_H_
#define __PLATFORM_H_

/* Includes ------------------------------------------------------------------*/


/* debug switch --------------------------------------------------------------*/
#define HARDFAULT_HANDLER_ENABLED	1
#define DEBUG_BLE_CONNECT
#define DEBUG_BLE_EVENT
#define DEBUG_BLE_SCAN

/* shell debug switch --------------------------------------------------------*/
#define SHELL_ENABLE
#define SHELL_BLE_ENABLE


/* gpio and buffer define for uart */
#define APP_UART_ENABLED	1
#define RETARGET_ENABLED	1
#ifndef UART_TX_BUF_SIZE
    #define UART_TX_BUF_SIZE 512         /**< UART TX buffer size. */
#endif
#ifndef UART_RX_BUF_SIZE
    #define UART_RX_BUF_SIZE 1            /**< UART RX buffer size. */
#endif

#define RX_PIN_NUMBER  				11    // UART RX pin number.
#define TX_PIN_NUMBER  				9     // UART TX pin number.
#define CTS_PIN_NUMBER 				10    // UART Clear To Send pin number. Not used if HWFC is set to false
#define RTS_PIN_NUMBER 				8     // Not used if HWFC is set to false

/* gpio define for leds */
#define LED_1          21
#define LED_2          22
#define LED_3          23
#define LED_4          24

/* 停止扫描定时器定义 */
#define STOP_SCAN_TIME      3000 / portTICK_PERIOD_MS  //扫描5s后停止扫描
#define DEFAULT_REF_RSSI    -70  //默认参考蓝牙信号强度值，大于改信号质量的设备才进入扫描队列
#define MAX_SCAN_LIST_NUM   10   //最大支持扫描列表数量
#define SN_NUM_LEN          4    //序列号长度：4字节

/*  Define the max event size and event queue size      */
#define SCHED_MAX_EVENT_DATA_SIZE           50
#define SCHED_QUEUE_SIZE                    40  /**< Maximum number of events in the scheduler queue. */

#endif /* __PLATFORM_H_ */


/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/


