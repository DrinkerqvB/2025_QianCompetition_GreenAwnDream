#ifndef __OLED_H__
#define __OLED_H__


#include "stm32f4xx.h"
#include "OLED_Data.h"
#include "delay.h"
#include <stdarg.h>  //格式化显示


/*上层协议*/
void OLED_WriteCommand(uint8_t cmmand);  //在控制位选项上，选择不连续模式，并且片选为命令（DC为0）
void OLED_WriteData(uint8_t Data);  //在控制位选项上，选择不连续模式，并且片选为数据（DC为1）

/*x:横坐标 page:指定页*/
void OLED_SetCursor(uint8_t x , uint8_t page);  //设置光标

/*功能接口*/
void OLED_ManualClear(void);
void IIC_init(void);//PB6-->SCL || PB7-->SDA 引脚初始化
void OLED_init(void); //初始化
void OLED_Clear(void); //清屏
void OLED_AreaClear(int16_t X , int16_t Y , int16_t X1 , int16_t Y1); //指定区域清除函数(0-127 , 0-63)
void OLED_AreaReversal(int16_t X , int16_t Y , int16_t X1 , int16_t Y1);  //指定区域反色显示
void OLED_AreaRefresh(int16_t X , int16_t Y , int16_t X1 , int16_t Y1);  //指定区域刷新显示,Y轴只能是8的倍数
void OLED_Update(void);  //全屏刷新

/*显示接口*/
void OLED_ShowChar(int16_t X, int16_t Y, uint32_t Number); //基本显示代码，其余代码都基于此
void OLED_ShowString(int16_t X, int16_t Y, char *String); //8*16-->宽*高,配合sprintf使用
void OLED_Printf(int16_t X, int16_t Y,const char *format, ...);  //格式化显示
/*
	显示图片，长宽只能为8的倍数，纵坐标也只能为8的倍数
*/
void OLED_ShowImage(uint8_t X , uint8_t Y , uint8_t Lenth , uint8_t With , char Str[]);

#endif

