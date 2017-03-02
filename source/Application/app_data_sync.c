/**
  ****************************************************************************************
  * @file    app_data_sync.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-2
  * @brief   the sync data for app,include:
  *             - start the data sync
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "app_data_sync.h"


uint8_t              g_data_syncbuf[MAX_LEN_ONE_PACKET] = {0};
uint16_t             current_data_len = 0;
uint16_t             need_rx_data_len = 0;
uint16_t             rx_data_cnt = 0;
bool                 flag_get_rx_data_len = true; 

/**
  * @brief  reset the data sync variables
  * @param  None
  * @retval None
  */
void reset_data_sync(void)
{
    current_data_len = 0;
    need_rx_data_len = 0;
    rx_data_cnt = 0;
    memset(g_data_syncbuf,0,MAX_LEN_ONE_PACKET);
    flag_get_rx_data_len = true;
}







/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/



