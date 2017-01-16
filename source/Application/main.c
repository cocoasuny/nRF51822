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
SemaphoreHandle_t 			g_semaphore_ble_event_ready = NULL;				/**< Semaphore raised if there is a new event to be 
                                                                                processed in the BLE thread. */
DeviceInfomation_t  		g_DeviceInformation;                            //硬件设备信息
QueueHandle_t               g_bleEventQueue = NULL;                         //event queue for ble
BLE_SCAN_LIST_T             gScanList[MAX_SCAN_LIST_NUM];                   //扫描列表

/* private variables define */
static TaskHandle_t 				m_shell_thread;					  	    /**< Definition of Logger thread. */
static TaskHandle_t 				m_ble_top_implementation_thread;      	/**< Definition of BLE stack thread. */
static TaskHandle_t 				m_ble_event_handler_thread;      	    /**< Definition of ble event handler thread. */


/* private function declare*/


/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{

}
/**@brief Function for application main entry.
 */
int main(void)
{
	/* clock config */
	nrf_drv_clock_init();
	
	/* bsp init */
	bsp_init();
	
	system_info_test_timer_init(); 
    
	/* Init a semaphore for the BLE thread. */
    g_semaphore_ble_event_ready = xSemaphoreCreateBinary();
    if (NULL == g_semaphore_ble_event_ready)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }	
	
	/* creat a thread for ble top level implementation*/
	if(
		pdPASS != xTaskCreate(ble_top_implementation_thread,"ble",BLE_TOP_IMPLEMENTATION_STACK,NULL,
							  	BLE_TOP_IMPLEMENTATION_PRIORITY,&m_ble_top_implementation_thread)
	)
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
    
	/* creat a thread for ble event handler thread*/
	if(
		pdPASS != xTaskCreate(ble_event_handler_thread,"ble",BLE_EVENT_HANDLER_STACK,NULL,
							  	BLE_EVENT_HANDLER_PRIORITY,&m_ble_event_handler_thread)
	)
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}    
#ifdef SHELL_ENABLE	
	/* creat a thread for logger */	
    if (pdPASS != xTaskCreate(shellCtlTask, "shell", TASK_SHELLCTL_STACK, NULL, TASK_SHELLCTL_PRIORITY, &m_shell_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif  

	// Start FreeRTOS scheduler.
    vTaskStartScheduler();
	
    printf("OS Scheduler Err\r\n");
	while(1)
	{
		
	}
}

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




