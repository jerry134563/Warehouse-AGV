#ifndef __MYCAN_H
#define __MYCAN_H
#define 	u8 		uint8_t
#define 	u16 	uint16_t 
#define 	u32 	uint32_t 
#define 	u64 	uint64_t 

#include "main.h"


typedef struct
{
	u8 message1[8];
	u8 message2[8];
	u8 message3[8];
	u8 message4[8]; 
	u8 message5[8]; 
	u8 message6[8]; 
	u8 message7[8]; 
	u8 message8[8]; 
    u8 message9[8]; 
    u8 message10[8]; 
    u8 message11[8]; 
    u8 message12[8]; 
}StepMessages;

typedef struct
{
    volatile int64_t Integral_bias_speed;
    int16_t Speed_RPM;
    u16     angle;
    u8      state;
}Step_Motor;

extern FDCAN_TxHeaderTypeDef drive1_tx;
extern FDCAN_TxHeaderTypeDef drive2_tx;
extern FDCAN_TxHeaderTypeDef drive3_tx;


void CAN_Init(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void Step_Set_Zero(FDCAN_TxHeaderTypeDef drive);
void Step_Reset(FDCAN_TxHeaderTypeDef drive);
void Step_Run(FDCAN_TxHeaderTypeDef drive, int32_t Speed);
void Step_Pos(FDCAN_TxHeaderTypeDef drive, int32_t Pos);
void Step_Servo_1(FDCAN_TxHeaderTypeDef drive, int32_t Pos);
void Step_Servo_2(FDCAN_TxHeaderTypeDef drive, int32_t Pos);
void Step_RPM(FDCAN_TxHeaderTypeDef drive, int32_t Speed);
void Step_Time_Limit(FDCAN_TxHeaderTypeDef drive, int32_t Speed, u16 Time_Limit);
void Step_LowSpeed(FDCAN_TxHeaderTypeDef drive, int32_t Speed);
void Step_Set_MaxSpeed(FDCAN_TxHeaderTypeDef drive, u16 Speed);
void Step_Set_Points(FDCAN_TxHeaderTypeDef drive, u8 num);
void Step_Goto_Points(FDCAN_TxHeaderTypeDef drive, u8 num);



#endif
