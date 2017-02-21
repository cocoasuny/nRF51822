/**
  ****************************************************************************************
  * @file    platform.h
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   header of main.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN__
#define __MAIN__

#include "include.h"



/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
  uint8_t * p_data; /**< Pointer to data. */
  uint16_t data_len; /**< Length of data. */
} data_t;

/* type struct define */
typedef struct
{
	uint8_t                             DevSupportCap;
	uint8_t                             DevREV[3];
	uint8_t                             FWREV[3];
	uint8_t                             Flag_ReceviedDevInfor;
    uint8_t                             STMBootVersion[3];
    uint8_t                             protocolVersion;
    int8_t                              rssi;
    uint8_t                             sn[SN_NUM_LEN];
    ble_gap_addr_t                      MACaddr;
    bool                                isValid;
    uint16_t                            conn_handle;
    uint32_t                            char_find_manage;
    bonding_service_t                   bonding_service;
    devinfo_manage_service_t            devinfo_manage_service;
    monitor_template_service_t          monitor_template_service;
}DeviceInfomation_t;

/*type struct define for ble scan list */
typedef struct
{
    int8_t              rssi;
    uint8_t             sn[SN_NUM_LEN];
    ble_gap_addr_t      MACaddr;
    bool                isValid;
}BLE_SCAN_LIST_T;

/* type struct define for ble handler task */
typedef enum
{
    EVENT_APP_BLE_DEFAULT =0,
    EVENT_APP_BLE_START_SCAN,
    EVENT_APP_BLE_STOP_SCAN,
    EVENT_APP_BLE_CONNECT,
    EVENT_APP_BLE_DISCONNECT,
    EVENT_APP_BLE_PASSKEY_WRITE
}BLE_EVENT_ID_T;

typedef struct
{
    BLE_EVENT_ID_T      eventID;
}BLE_MSG_T;


/* Queue size define */
#define BLE_EVENT_QUEUE_SIZE        10


/* gloable variables declare */
extern DeviceInfomation_t  			g_DeviceInformation;                //硬件设备信息
extern BLE_SCAN_LIST_T              gScanList[];  
extern ble_db_discovery_t           g_ble_db_discovery[]; /**< list of DB structures used by the database discovery module. */


#endif // __MAIN__

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




