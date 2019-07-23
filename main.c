#include "stm32f10x.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void SendIO(uint8_t send);
void ReceiveIO(void);
//extern u8 recvData ;
void Delay(uint32_t nCount);
  
enum stat{
COM_START_BIT,	//起始位
COM_D0_BIT,	//bit0
COM_D1_BIT,	//bit1
COM_D2_BIT,	//bit2
COM_D3_BIT,	//bit3
COM_D4_BIT,	//bit4
COM_D5_BIT,	//bit5
COM_D6_BIT,	//bit6
COM_D7_BIT,	//bit7
COM_STOP_BIT,	//bit8
};

u8 recvData = 0; //接收的数据
enum stat recvStat = COM_STOP_BIT;	//定义初始为停止位

void Delay(uint32_t nCount)
{
   for(; nCount != 0; nCount--);
}


void RCC_Configuration(void)
{
  SystemInit();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}//初始化时钟


// 发送函数
void SendIO(uint8_t send)
{
  uint8_t i = 0;
  GPIO_ResetBits(GPIOA,GPIO_Pin_9); //起始位 低电平
  //Delay(0x2E5);	 //发送完起始位，延时	  1000000 / 9600  = 104   432 是试出来的延时
  Delay(0x33A);
  for(i=0; i<8;i++)	  //8位数据位	一位停止位
  {
    if(send & 0x01)
    {
      GPIO_SetBits(GPIOA,GPIO_Pin_9);//当前数据位是1 ，就拉高
    }
    else
    {
      GPIO_ResetBits(GPIOA,GPIO_Pin_9);//当前数据位是0，就置零
    }
    
    send >>= 1 ;
    Delay(0x432);	// 发送每一位数据位延时
  }
  
  //Delay(0x212);		// 	 发送完数据位延时
  GPIO_SetBits(GPIOA,GPIO_Pin_9); //停止位
  Delay(0x33A);
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    //上拉输入 ，IO口默认是高电平
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA,GPIO_Pin_10);//保持高电平
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  EXTI_InitStruct.EXTI_Line=EXTI_Line10;	
  EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling;    //下降沿中断
  EXTI_InitStruct.EXTI_LineCmd=ENABLE;
  EXTI_Init(&EXTI_InitStruct);
  NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn; //外部中断，边沿触发
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}//配置外部中断


int main(void)
{
  RCC_Configuration();
  GPIO_Configuration();
  GPIO_SetBits(GPIOA,GPIO_Pin_9);//先拉高 ，第一次数据没有之前没有停止位
  while(1)
  {	
    SendIO(recvData);
    Delay(0xFFFFFF);
  }
}

void EXTI15_10_IRQHandler(void)	
{
  if(EXTI_GetITStatus(EXTI_Line10)!=RESET)	  //检查外部中断是否产生了
  {	
    if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)) //检测引脚高低电平，如果是低电平，则说明检测到起始位
    {
      GPIO_SetBits(GPIOA,GPIO_Pin_2);//LED 代表检测到了起始位
      Delay(0x432);	//延时 0x432 约等于104us
      if(recvStat == COM_STOP_BIT)
      {
        recvStat = COM_START_BIT;	//此时的状态是起始
        while(recvStat!= COM_STOP_BIT) // 循环到停止位
        {
          recvStat++; // 改变状态
          if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)) //‘1’
          {
            recvData |= (1 <<(recvStat -1));
          }
          else
          {
            recvData &= ~(1 << (recvStat -1));
          }	
          Delay(0x432);
          
        }
      }
    }
  EXTI_ClearITPendingBit(EXTI_Line10);	//清除EXTI_Line10中断挂起标志位
  } 
}
