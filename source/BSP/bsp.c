/**
  ****************************************************************************************
  * @file    bsp.c
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   bsp
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
#include "bsp.h"

/* Includes ------------------------------------------------------------------*/

/**
  * @brief  bsp init 
  * @note   This function is called  automatically at the beginning of program 
  * @param  None
  * @retval None
  */
void bsp_init(void)
{
	/* uart init */
	uart_init();
}



/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

