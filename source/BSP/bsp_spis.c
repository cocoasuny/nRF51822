/**
  ****************************************************************************************
  * @file    bsp_spis.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-14
  * @brief   bsp spi slave
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bsp_spis.h"

#define SPIS_INSTANCE 1 /**< SPIS instance index. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
static void spis_event_handler(nrf_drv_spis_event_t event);

static uint8_t       m_tx_buf[50] = {0};                 /**< TX buffer. */
static uint8_t       m_rx_buf[51] = {0};                 /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/**
  * @brief  bsp_spi_slave_config  
  * @param  None
  * @retval None
  */
void bsp_spi_slave_config(void)
{
    // Enable the constant latency sub power mode to minimize the time it takes
    // for the SPIS peripheral to become active after the CSN line is asserted
    // (when the CPU is in sleep mode).
    NRF_POWER->TASKS_CONSTLAT = 1;

    nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
    spis_config.csn_pin               = SPIS_CSN_PIN;
    spis_config.miso_pin              = SPIS_MISO_PIN;
    spis_config.mosi_pin              = SPIS_MOSI_PIN;
    spis_config.sck_pin               = SPIS_SCK_PIN;

    APP_ERROR_CHECK(nrf_drv_spis_init(&spis, &spis_config, spis_event_handler)); 

    nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length);    
}
/**
  * @brief  spis_event_handler  
  * @param  event
  * @retval None
  */
static void spis_event_handler(nrf_drv_spis_event_t event)
{
    uint8_t i = 0;
    
    if(event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {
        printf("Rx_len%d:",event.rx_amount);
        for(i=0;i<event.rx_amount;i++)
        {
            printf("0x%x,",m_rx_buf[i]);
        }
        printf("\r\n");
        memset(m_rx_buf,0,5);
        //printf("Transfer completed. Received: %s\r\n",(uint32_t)m_rx_buf);
        APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));
    }
}






/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/


