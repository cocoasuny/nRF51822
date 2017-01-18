/**
  ****************************************************************************************
  * @file    system_info_test.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-1-16
  * @brief   system information test
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "system_info_test.h"
#include "nrf_drv_timer.h"

/* private function declare */
static void timer_system_test(nrf_timer_event_t event_type, void * p_context);

/* private variabled declare */
static nrf_drv_timer_t 			m_timer_system_test = NRF_DRV_TIMER_INSTANCE(1);

volatile uint32_t ulHighFrequencyTimerTicks = 0UL;

/**
  * @brief  system_info_test_timer_init
  * @note   用于获取FreeRTOS CPU使用信息 
  * @param  None
  * @retval None
  */
void system_info_test_timer_init(void)
{
 	nrf_drv_timer_config_t 				timer_cfg;
	uint32_t 							err_code = NRF_ERROR_NULL;
    uint32_t                            time_ticks;
    uint32_t                            time_ms = 10; //Time(in miliseconds) between consecutive compare events.
    
	/* set the timer cfg */
	timer_cfg.frequency		= NRF_TIMER_FREQ_31250Hz;  	//prescaler 
	timer_cfg.mode			= NRF_TIMER_MODE_TIMER;	    //timer mode
	timer_cfg.bit_width		= NRF_TIMER_BIT_WIDTH_16;	//The TIMER's maximum value is configured by 
														//changing the bit-width of the timer 
	timer_cfg.p_context     = NULL;   

	/* init the timer */
	err_code = nrf_drv_timer_init(&m_timer_system_test, &timer_cfg, timer_system_test);
    APP_ERROR_CHECK(err_code);
    
    time_ticks = nrf_drv_timer_ms_to_ticks(&m_timer_system_test, time_ms);
    nrf_drv_timer_extended_compare(
                                    &m_timer_system_test, 
                                    NRF_TIMER_CC_CHANNEL0, 
                                    time_ticks, 
                                    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, 
                                    true);

    /* enable timer */
    nrf_drv_timer_enable(&m_timer_system_test);        
}
/**
  * @brief  timer_system_test
  * @param  event_type,p_context
  * @retval None
  */
static void timer_system_test(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
        {
            ulHighFrequencyTimerTicks++;
        }
        break;
        default:break;
    }  
}
/**
  * @brief  timer_system_test
  * @param  event_type,p_context
  * @retval None
  */
void get_system_run_status(void)
{
    uint8_t pcWriteBuffer[500];
    
    printf("=================================================\r\n");
    printf("task         status    pri     stack   taskID\r\n");
    vTaskList((char *)&pcWriteBuffer);
    printf("%s\r\n", pcWriteBuffer);
    printf("\r\ntask          run count        usage\r\n");
    vTaskGetRunTimeStats((char *)&pcWriteBuffer);
    printf("%s\r\n", pcWriteBuffer);    
}


/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

