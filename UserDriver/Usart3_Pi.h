/**
 * @file Usart3_Pi.h
 * @brief USART3 驱动头文件 —— 树莓派(Raspberry Pi)通信
 *
 * 硬件连接: 树莓派 via UART
 * 主要用途: 接收树莓派通过 SLAM 算法解算的 N10P 雷达定位数据
 * 核心功能: 为无人机提供外部定位信息，辅助飞控进行位置估计与导航
 */

#ifndef __USART3_PI_H
#define __USART3_PI_H

#include "McuConfig.h"

/**
 * @brief 树莓派定位数据逐字节解析（状态机）
 * @param com_data 从串口3接收到的单字节数据
 * @note  由 Drv_Uart.c 的 drvU3DataCheck() 在 ANO_LX_Task() 1ms周期中逐字节调用。
 *        帧格式：0x45(头) + 4字节有效数据 + 0x46(尾)。
 *        解析完成后置位内部标志，供 Pi_GetData_Flag 查询。
 */
void Pi_DataAnl(u8 com_data);

/**
 * @brief 查询树莓派数据接收完成标志
 * @return SET(1) 有新帧已就绪，同时自动清零；RESET(0) 暂无新数据
 * @note  典型调用者：Ano_Scheduler.c 的 Loop_50Hz（20ms周期任务）
 */
u8 Pi_GetData_Flag(void);

/**
 * @brief 拷贝最新接收到的有效数据到外部缓冲区
 * @param store_array 外部缓冲区指针，长度至少 4 字节
 * @note  典型调用者：Ano_Scheduler.c 的 Loop_50Hz，在 Pi_GetData_Flag 返回 SET 后调用。
 *        数据映射：
 *        [0][1] -> now_x (高8位在前，低8位在后)，[2][3] -> now_y (高8位在前，低8位在后)
 */
void Pi_GetData(u8* store_array);

#endif
