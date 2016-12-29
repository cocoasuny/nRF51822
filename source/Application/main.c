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
DeviceInfomation_t  		g_DeviceInformation; //硬件设备信息

/* private variables define */
static TaskHandle_t 				m_shell_thread;					  	/**< Definition of Logger thread. */
static TaskHandle_t 				m_ble_top_implementation_thread;      	/**< Definition of BLE stack thread. */


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
#ifdef SHELL_ENABLE	
	/* creat a thread for logger */	
    if (pdPASS != xTaskCreate(shellCtlTask, "shell", TASK_SHELLCTL_STACK, NULL, TASK_SHELLCTL_PRIORITY, &m_shell_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif    
	
	// Start FreeRTOS scheduler.
    vTaskStartScheduler();
	
	while(1)
	{
		APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
	}
}

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




