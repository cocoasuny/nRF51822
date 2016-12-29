/*********************************Copyright (c)********************************* 
**                                
**                                 FIVE工作组 
** 
**---------------------------------File Info------------------------------------ 
** File Name:               shell.h 
** Last modified Date:      2014/3/5 15:42:05 
** Last Version:            V2    
** Description:             none 
** 
**------------------------------------------------------------------------------ 
** Created By:              wanxuncpx 
** Created date:            2014/3/5 15:42:11 
** Version:                 V2 
** Descriptions:            none 
**------------------------------------------------------------------------------ 
** Libraries:               无关 
** version                  无关 
*******************************************************************************/  
  
/****************************************************************************** 
更新说明: 
******************************************************************************/  
  
/****************************************************************************** 
*********************************  应 用 资 料 ******************************** 
******************************************************************************/  
  
#ifndef _SHELL_H_  
#define _SHELL_H_  
/****************************************************************************** 
********************************* 文件引用部分 ******************************** 
******************************************************************************/  
#include "stdint.h"         //包含uint8_t等数据类型  
#include "stdbool.h"        //包含Bool类型  
#include "stdio.h"          //包含printf支持  
#include "platform.h"
#include "bsp_uart.h"
#include "main.h"
  
/****************************************************************************** 
********************************* 参数宏定义 ********************************* 
******************************************************************************/  
//版本定义  
#define SHELL_VER           2       //Shell版本  
  
//缓冲大小配置  
#define SHELL_RX_MAX        (256+32)        //shell指令接收缓冲大小  
#define SHELL_TX_MAX        (512)           //shell指令发送缓冲大小  
  
/****************************************************************************** 
********************************* 数 据 声 明 ********************************* 
******************************************************************************/  
/*---------------------*  
*     Shell接收 
*----------------------*/  
//接收数据  
extern volatile uint16_t   shell_rx_rdy;                    //0:空闲,非零:忙,用户读为非零后清零  
extern volatile uint8_t    shell_rx_buff[SHELL_RX_MAX+1];   //接收缓冲  
  
/****************************************************************************** 
********************************* 函 数 声 明 ********************************* 
******************************************************************************/  
/*---------------------*  
*    输出函数 
*----------------------*/  
   
//初始化Shell  
void shell_Init(uint32_t baud);          //模块初始化 
void HAL_UART_RxCpltCallback(uint8_t cr);
    
/*---------------------*  
*     辅助判断指令 
*----------------------*/  
extern bool StrComp(void * buffer,void * StrCmd);   //字符串匹配比较函数  
  
/*---------------------*  
*       Shell服务 
*----------------------*/  
//在main.c函数while()中判断shell_rx_rdy是否为非零,为非零才执行以下程序
extern void shellCtlTask(void *pvParameters);
extern void Shell_ProcessorHandler(void);
extern void Shell_Invalid_Service(void); //指令未处理服务(会处理shell_rx_rdy信号)  

 
#endif  
