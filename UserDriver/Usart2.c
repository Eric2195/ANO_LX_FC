/**
 * @file Usart2.c
 * @brief USART2 驱动 —— 无线串口 DL20（地面站通信）
 *
 * 硬件连接: 无线串口模块 DL20
 * 主要用途: 地面站(GCS)与飞控(FC)之间的通信
 * 核心功能: 接收地面站指令，主要用于设置禁飞区参数
 *
 * 数据协议: 帧头 0x45，帧尾 0x46，有效数据长度 GS_VALID_BYTE_LENGTH (2字节)
 */

#include "Usart2.h"
#include "Drv_Uart.h"

/* 地面站一帧有效数据长度：2字节 = 1组障碍物坐标(x,y) */
#define GS_VALID_BYTE_LENGTH 2

/* 一帧数据接收完成标志（由 GS_DataAnl 置位，由 GS_GetData_Flag 清零） */
static volatile u8 g_GS_dataAnlScs_flag = RESET;
/* 接收缓存区，大小需 >= GS_VALID_BYTE_LENGTH */
static u8 g_GS_val_data[20];

/**
 * @brief 地面站数据逐字节解析（状态机）
 * @note  调用时机：由 Drv_Uart.c 的 drvU2DataCheck() 在串口接收中断上下文外逐字节调用，
 *        最终在 ANO_LX_Task() -> DrvUartDataCheck() -> drvU2DataCheck() 流程中被周期执行。
 * @note  数据格式：帧头 0x45 + 2字节有效数据 + 帧尾 0x46
 *        有效数据含义（由 Ano_Scheduler.c 的 Loop_50Hz 消费）：
 *        [0]:x(1~9), [1]:y(1~7)，均为1-based坐标
 */
void GS_DataAnl(u8 com_data)
{
	static u8 rx_state = 0;
	static u8 check_sum = 0;
	static u8 pack_data_pointer = 0;

	/* 若上一帧尚未被取走，则丢弃新数据，防止覆盖 */
	if (!g_GS_dataAnlScs_flag)
	{
		/* ---- state 0: 等待帧头 0x45 ---- */
		if (rx_state == 0)
		{
			check_sum = 0;
			if (com_data == 0x45)
			{
				rx_state = 1;
				check_sum += com_data;
			}
			else
			{
				rx_state = 0;
			}
		}
		/* ---- state 1: 接收有效数据区 ---- */
		else if (rx_state == 1)
		{
			*(g_GS_val_data + pack_data_pointer) = com_data;
			pack_data_pointer++;
			check_sum += com_data;
			if (pack_data_pointer >= GS_VALID_BYTE_LENGTH)
			{
				rx_state = 2;      /* 数据收满，转去等待帧尾 */
				pack_data_pointer = 0;
			}
		}
		/* ---- state 2: 等待帧尾 0x46 ---- */
		else if (rx_state == 2)
		{
			if (com_data == 0x46)
			{
				rx_state = 0;
				g_GS_dataAnlScs_flag = SET; /* 标记一帧接收完成 */
			}
			else
			{
				rx_state = 0; /* 帧尾错误，重新同步 */
			}
		}
		else
		{
			rx_state = 0;
			check_sum = 0;
			pack_data_pointer = 0;
		}
	}
}

/**
 * @brief 查询地面站数据接收完成标志
 * @return SET(1) 表示新帧已就绪，同时自动清零标志；RESET(0) 表示暂无新数据
 * @note  调用者：Ano_Scheduler.c 的 Loop_50Hz
 */
u8 GS_GetData_Flag(void)
{
	if (g_GS_dataAnlScs_flag)
	{
		g_GS_dataAnlScs_flag = RESET;
		return SET;
	}
	return RESET;
}

/**
 * @brief 拷贝最新接收到的有效数据到外部缓冲区
 * @param store_array 外部接收缓冲区，长度至少 GS_VALID_BYTE_LENGTH
 * @note  调用者：Ano_Scheduler.c 的 Loop_50Hz，在 GS_GetData_Flag 返回 SET 后调用
 */
void GS_GetData(u8* store_array)
{
	for (u8 i = 0; i < GS_VALID_BYTE_LENGTH; i++)
	{
		*(store_array++) = *(g_GS_val_data + i);
	}
}
