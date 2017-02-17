/**
  ****************************************************************************************
  * @file    ble_central_service_bonding.h
  * @author  Jason
  * @version V1.0.0
  * @date    2017-2-9
  * @brief   header of ble_central_service_bonding.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_CENTRAL_SERVICE_BONDING_H_
#define __BLE_CENTRAL_SERVICE_BONDING_H_

/* Includes ------------------------------------------------------------------*/
#include "common.h"


/* function declare */
uint32_t ble_central_service_bonding_init(bonding_service_t *p_bondinf_service);
void reset_ble_central_bonding_service(bonding_service_t *p_bondinf_service);
void ble_bonding_db_discovery_evt_handler(DeviceInfomation_t *p_dev_info, ble_db_discovery_evt_t * p_evt);
void ble_bonding_ble_evt_handler(ble_evt_t * p_ble_evt);
void ble_central_passkey_write(DeviceInfomation_t *p_dev);


#endif /* __BLE_CENTRAL_SERVICE_BONDING_H_ */


/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/



