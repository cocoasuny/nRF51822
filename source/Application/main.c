/**
  ****************************************************************************************
  * @file    bsp.c
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   main program for nRF51822
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
#include "main.h"

/* gloable variables define */
DeviceInfomation_t  		g_DeviceInformation;                            //硬件设备信息
BLE_SCAN_LIST_T             gScanList[MAX_SCAN_LIST_NUM];                   //扫描列表

/* private variables define */


/* private function declare*/
static void scheduler_init (void);
static void power_manage(void);



/**@brief Function for application main entry.
 */
int main(void)
{
	/* clock config */
	nrf_drv_clock_init();
	
    /* app scheduler init */
    scheduler_init();
    
	/* bsp init */
	bsp_init();
	    
    /* ble init */
    ble_init();

	while(1)
	{
        app_sched_execute();
        power_manage();		
	}
}

/**@brief Function for initializing the Event Scheduler.
 */
static void scheduler_init (void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}



/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




