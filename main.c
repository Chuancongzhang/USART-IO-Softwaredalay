#include "stm32f10x.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void SendIO(uint8_t send);
void ReceiveIO(void);
//extern u8 recvData ;
void Delay(uint32_t nCount);
  
enum stat{
COM_START_BIT,	//��ʼλ
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

u8 recvData = 0; //���յ�����
enum stat recvStat = COM_STOP_BIT;	//�����ʼΪֹͣλ

void Delay(uint32_t nCount)
{
   for(; nCount != 0; nCount--);
}


void RCC_Configuration(void)
{
  SystemInit();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}//��ʼ��ʱ��


// ���ͺ���
void SendIO(uint8_t send)
{
  uint8_t i = 0;
  GPIO_ResetBits(GPIOA,GPIO_Pin_9); //��ʼλ �͵�ƽ
  //Delay(0x2E5);	 //��������ʼλ����ʱ	  1000000 / 9600  = 104   432 ���Գ�������ʱ
  Delay(0x33A);
  for(i=0; i<8;i++)	  //8λ����λ	һλֹͣλ
  {
    if(send & 0x01)
    {
      GPIO_SetBits(GPIOA,GPIO_Pin_9);//��ǰ����λ��1 ��������
    }
    else
    {
      GPIO_ResetBits(GPIOA,GPIO_Pin_9);//��ǰ����λ��0��������
    }
    
    send >>= 1 ;
    Delay(0x432);	// ����ÿһλ����λ��ʱ
  }
  
  //Delay(0x212);		// 	 ����������λ��ʱ
  GPIO_SetBits(GPIOA,GPIO_Pin_9); //ֹͣλ
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    //�������� ��IO��Ĭ���Ǹߵ�ƽ
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA,GPIO_Pin_10);//���ָߵ�ƽ
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  EXTI_InitStruct.EXTI_Line=EXTI_Line10;	
  EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling;    //�½����ж�
  EXTI_InitStruct.EXTI_LineCmd=ENABLE;
  EXTI_Init(&EXTI_InitStruct);
  NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn; //�ⲿ�жϣ����ش���
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}//�����ⲿ�ж�


int main(void)
{
  RCC_Configuration();
  GPIO_Configuration();
  GPIO_SetBits(GPIOA,GPIO_Pin_9);//������ ����һ������û��֮ǰû��ֹͣλ
  while(1)
  {	
    SendIO(recvData);
    Delay(0xFFFFFF);
  }
}

void EXTI15_10_IRQHandler(void)	
{
  if(EXTI_GetITStatus(EXTI_Line10)!=RESET)	  //����ⲿ�ж��Ƿ������
  {	
    if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)) //������Ÿߵ͵�ƽ������ǵ͵�ƽ����˵����⵽��ʼλ
    {
      GPIO_SetBits(GPIOA,GPIO_Pin_2);//LED �����⵽����ʼλ
      Delay(0x432);	//��ʱ 0x432 Լ����104us
      if(recvStat == COM_STOP_BIT)
      {
        recvStat = COM_START_BIT;	//��ʱ��״̬����ʼ
        while(recvStat!= COM_STOP_BIT) // ѭ����ֹͣλ
        {
          recvStat++; // �ı�״̬
          if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)) //��1��
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
  EXTI_ClearITPendingBit(EXTI_Line10);	//���EXTI_Line10�жϹ����־λ
  } 
}
