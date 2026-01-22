#include "main.h"
#include "fdcan.h"
#include "mycan.h"
StepMessages Messages = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //复位
    {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //速度
    {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //位置
    {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //舵机（0-360°）
    {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //低速
    {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //限速位置
    {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //速度（RPM）
    {0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //舵机（0-16384）
    {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //设置零点
    {0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //设置运动最大转速
    {0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //设置运动最大转速
    {0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //设置运动最大转速
};

FDCAN_TxHeaderTypeDef drive1_tx;
FDCAN_TxHeaderTypeDef drive2_tx;
FDCAN_TxHeaderTypeDef drive3_tx;
FDCAN_TxHeaderTypeDef drive4_tx;
FDCAN_FilterTypeDef can_fillter;
  void CAN_Init(void) //500k bit/s
{
    drive1_tx.Identifier = 0x01;
    drive1_tx.IdType = FDCAN_STANDARD_ID;
    drive1_tx.TxFrameType = FDCAN_DATA_FRAME;
    drive1_tx.DataLength = FDCAN_DLC_BYTES_8;
    drive1_tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    drive1_tx.BitRateSwitch = FDCAN_BRS_OFF;
    drive1_tx.FDFormat = FDCAN_CLASSIC_CAN;
    drive1_tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	
    
	drive2_tx.Identifier = 0x02; //第二台驱动器id
    drive2_tx.IdType = FDCAN_STANDARD_ID;
    drive2_tx.TxFrameType = FDCAN_DATA_FRAME;
    drive2_tx.DataLength = FDCAN_DLC_BYTES_8;
    drive2_tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    drive2_tx.BitRateSwitch = FDCAN_BRS_OFF;
    drive2_tx.FDFormat = FDCAN_CLASSIC_CAN;
    drive2_tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	
	
	drive3_tx.Identifier = 0x03; //第三台驱动器id
    drive3_tx.IdType = FDCAN_STANDARD_ID;
    drive3_tx.TxFrameType = FDCAN_DATA_FRAME;
    drive3_tx.DataLength = FDCAN_DLC_BYTES_8;
    drive3_tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    drive3_tx.BitRateSwitch = FDCAN_BRS_OFF;
    drive3_tx.FDFormat = FDCAN_CLASSIC_CAN;
    drive3_tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	
	drive4_tx.Identifier = 0x04; //第4台驱动器id
    drive4_tx.IdType = FDCAN_STANDARD_ID;
    drive4_tx.TxFrameType = FDCAN_DATA_FRAME;
    drive4_tx.DataLength = FDCAN_DLC_BYTES_8;
    drive4_tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    drive4_tx.BitRateSwitch = FDCAN_BRS_OFF;
    drive4_tx.FDFormat = FDCAN_CLASSIC_CAN;
    drive4_tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    
    // 过滤器配置
    can_fillter.IdType = FDCAN_STANDARD_ID;
    can_fillter.FilterType = FDCAN_FILTER_DUAL; //FDCAN_FILTER_RANGE
    can_fillter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_fillter.FilterIndex = 0;
    can_fillter.FilterID1 = 0x01;  // ID1范围起始
    can_fillter.FilterID2 = 0x02;  // ID2范围结束
    HAL_FDCAN_ConfigFilter(&hfdcan1, &can_fillter);

    can_fillter.FilterIndex = 1;
    can_fillter.FilterID1 = 0x03;  // ID1范围起始
    can_fillter.FilterID2 = 0x04;  // ID2范围结束
    HAL_FDCAN_ConfigFilter(&hfdcan1, &can_fillter);

    // 激活通知
    HAL_FDCAN_ActivateNotification(
        &hfdcan1, 
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
        0);

    while(HAL_FDCAN_Start(&hfdcan1));
}

/**
 * @brief       复位，回到设定的零点位置
 * @param       对应电机的结构体
 * @retval      无
 */
void Step_Set_Zero(FDCAN_TxHeaderTypeDef drive)
{
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message9);
}

/**
 * @brief       复位，回到设定的零点位置
 * @param       对应电机的结构体
 * @retval      无
 */
void Step_Reset(FDCAN_TxHeaderTypeDef drive)
{
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message1);
}

/**
 * @brief       设置电机速度
 * @param       速度 单位为每1ms的编码器单位
 * @retval      无
 */
void Step_Run(FDCAN_TxHeaderTypeDef drive, int32_t Speed)
{
    u8 buf[4];
    u8 flag;
    uint32_t var;
    if(Speed>=0) flag = 1;
    else flag = 0;
    var = (Speed>=0)? Speed:-Speed;
    
    buf[0] = var>>24;
    buf[1] = (var>>16)&0xff;
    buf[2] = (var>>8)&0xff;
    buf[3] = (var)&0xff;
    Messages.message2[0] = 0x01;
    Messages.message2[1] = buf[0];
    Messages.message2[2] = buf[1];
    Messages.message2[3] = buf[2];
    Messages.message2[4] = buf[3];
    Messages.message2[5] = flag;
    Messages.message2[6] = 0x00;
    Messages.message2[7] = 0x00;
    
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message2);
}

/**
 * @brief       设置电机位置
 * @param       位置 单位为编码器单位
 * @retval      无
 */
void Step_Pos(FDCAN_TxHeaderTypeDef drive, int32_t Pos)
{
    u8 buf[4];
    u8 pos_flag;
    uint32_t var;
    if(Pos>=0) pos_flag = 1;
    else pos_flag = 0;
    var = (Pos>=0)? Pos:-Pos;
    buf[0] = var>>24;
    buf[1] = (var>>16)&0xff;
    buf[2] = (var>>8)&0xff;
    buf[3] = (var)&0xff;
    Messages.message3[0] = 0x02;
    Messages.message3[1] = buf[0];
    Messages.message3[2] = buf[1];
    Messages.message3[3] = buf[2];
    Messages.message3[4] = buf[3];
    Messages.message3[5] = pos_flag;
    Messages.message3[6] = 0x00;
    Messages.message3[7] = 0x00;
    
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message3);
}

/**
 * @brief       设置电机位置
 * @param       位置 单位为0-360
 * @retval      无
 */
void Step_Servo_1(FDCAN_TxHeaderTypeDef drive, int32_t Pos)
{
    u8 buf[4];
    Pos = (Pos>=360)?360:Pos;
    Pos = (Pos<=-360)?-360:Pos;
    Pos = (Pos % 360 + 360) % 360;
    buf[0] = Pos>>24;
    buf[1] = (Pos>>16)&0xff;
    buf[2] = (Pos>>8)&0xff;
    buf[3] = (Pos)&0xff;
    Messages.message4[0] = 0x03;
    Messages.message4[1] = buf[0];
    Messages.message4[2] = buf[1];
    Messages.message4[3] = buf[2];
    Messages.message4[4] = buf[3];
    Messages.message4[5] = 0x00;
    Messages.message4[6] = 0x00;
    Messages.message4[7] = 0x00;
    
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message4);
}

/**
 * @brief       设置电机位置
 * @param       位置 单位为编码器单位
 * @retval      无
 */
void Step_Servo_2(FDCAN_TxHeaderTypeDef drive, int32_t Pos)
{
    u8 buf[4];
    Pos = (Pos>=16384)?16384:Pos;
    Pos = (Pos<=-16384)?-16384:Pos;
    Pos = (Pos % 16384 + 16384) % 16384;
    buf[0] = Pos>>24;
    buf[1] = (Pos>>16)&0xff;
    buf[2] = (Pos>>8)&0xff;
    buf[3] = (Pos)&0xff;
    Messages.message8[0] = 0x07;
    Messages.message8[1] = buf[0];
    Messages.message8[2] = buf[1];
    Messages.message8[3] = buf[2];
    Messages.message8[4] = buf[3];
    Messages.message8[5] = 0x00;
    Messages.message8[6] = 0x00;
    Messages.message8[7] = 0x00;
    
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message8);
}

/**
 * @brief       设置电机速度
 * @param       速度 单位为RPM(0-3500),最高大概在4500，但是速度越高力矩会有所下降
 * @retval      无
 */
void Step_RPM(FDCAN_TxHeaderTypeDef drive, int32_t Speed)
{
    u8 buf[4];
    u8 flag;
    uint32_t var;
    if(Speed>=0) flag = 1;
    else flag = 0;
    var = (Speed>=0)? Speed:-Speed;
    
    buf[0] = var>>24;
    buf[1] = (var>>16)&0xff;
    buf[2] = (var>>8)&0xff;
    buf[3] = (var)&0xff;
    Messages.message7[0] = 0x06;
    Messages.message7[1] = buf[0];
    Messages.message7[2] = buf[1];
    Messages.message7[3] = buf[2];
    Messages.message7[4] = buf[3];
    Messages.message7[5] = flag;
    Messages.message7[6] = 0x00;
    Messages.message7[7] = 0x00;
    
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message7);
}

/**
 * @brief       设置电机速度
 * @param       位置（编码器单位），限定的时间（ms）
 * @retval      无
 */
void Step_Time_Limit(FDCAN_TxHeaderTypeDef drive, int32_t Speed, u16 Time_Limit)
{
    u8 time[2];
    u8 buf[4];
    u8 flag;
    uint32_t var;
    if(Speed>=0) flag = 1;
    else flag = 0;
    var = (Speed>=0)? Speed:-Speed;

    time[0] = Time_Limit>>8;
    time[1] = Time_Limit&0xff;
    buf[0] = var>>24;
    buf[1] = (var>>16)&0xff;
    buf[2] = (var>>8)&0xff;
    buf[3] = (var)&0xff;
    Messages.message6[0] = 0x05;
    Messages.message6[1] = buf[0];
    Messages.message6[2] = buf[1];
    Messages.message6[3] = buf[2];
    Messages.message6[4] = buf[3];
    Messages.message6[5] = time[0];
    Messages.message6[6] = time[1];
    Messages.message6[7] = flag;
    
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message6);
}

/**
 * @brief       设置电机速度
 * @param       速度 单位为每25ms的编码器单位
 * @retval      无
 */
void Step_LowSpeed(FDCAN_TxHeaderTypeDef drive, int32_t Speed)
{
    u8 buf[4];
    u8 flag;
    uint32_t var;
    if(Speed>=0) flag = 1;
    else flag = 0;
    var = (Speed>=0)? Speed:-Speed;
    
    buf[0] = var>>24;
    buf[1] = (var>>16)&0xff;
    buf[2] = (var>>8)&0xff;
    buf[3] = (var)&0xff;
    Messages.message5[0] = 0x04;
    Messages.message5[1] = buf[0];
    Messages.message5[2] = buf[1];
    Messages.message5[3] = buf[2];
    Messages.message5[4] = buf[3];
    Messages.message5[5] = flag;
    Messages.message5[6] = 0x00;
    Messages.message5[7] = 0x00;
    
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message5);
}

/**
 * @brief       设置电机最大运动转速
 * @param       最大速度值，80为3000rpm，目前限制在最大为80
 * @retval      无
 */
void Step_Set_MaxSpeed(FDCAN_TxHeaderTypeDef drive, u16 Speed)
{
    u8 buf[2];
    
    buf[0] = (Speed>>8)&0xff;
    buf[1] = (Speed)&0xff;
    Messages.message10[0] = 0x09;
    Messages.message10[1] = buf[0];
    Messages.message10[2] = buf[1];
    Messages.message10[3] = 0x00;
    Messages.message10[4] = 0x00;
    Messages.message10[5] = 0x00;
    Messages.message10[6] = 0x00;
    Messages.message10[7] = 0x00;
    
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message10);
}

/**
 * @brief       设置电机设置点的位置（相对于零点的位置）
 * @param       可以设置10个点
 * @retval      无
 */
void Step_Set_Points(FDCAN_TxHeaderTypeDef drive, u8 num)
{
    u8 buf[2];
    
    buf[0] = num;

    Messages.message11[0] = 0x0A;
    Messages.message11[1] = buf[0];
    Messages.message11[2] = 0x00;
    Messages.message11[3] = 0x00;
    Messages.message11[4] = 0x00;
    Messages.message11[5] = 0x00;
    Messages.message11[6] = 0x00;
    Messages.message11[7] = 0x00;
    
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message11);
}

/**
 * @brief       运动到电机设置点的位置（相对于零点的位置）
 * @param       可以设置10个点
 * @retval      无
 */
void Step_Goto_Points(FDCAN_TxHeaderTypeDef drive, u8 num)
{
    u8 buf[2];
    
    buf[0] = num;

    Messages.message12[0] = 0x0B;
    Messages.message12[1] = buf[0];
    Messages.message12[2] = 0x00;
    Messages.message12[3] = 0x00;
    Messages.message12[4] = 0x00;
    Messages.message12[5] = 0x00;
    Messages.message12[6] = 0x00;
    Messages.message12[7] = 0x00;
    
    HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1, 
        &drive,
        Messages.message12);
}

FDCAN_RxHeaderTypeDef can_rx;
Step_Motor Step[4] = {0};
u8 canBuf[8];
/**
 * @brief       can1 FIFO0接收中断函数
 * @param       无
 * @retval      无
 */	
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    u32 var1 = 0, var2 = 0;
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        HAL_FDCAN_GetRxMessage(
            hfdcan, 
            FDCAN_RX_FIFO0,
            &can_rx,
            canBuf);
	
        if(can_rx.Identifier == 0x02) //第一台驱动器
        {
            if(canBuf[0] == 0x00)
            {
                if(canBuf[5] == 0) //0为状态正常，1为不正常
                {
                    Step[1].state     = canBuf[5];
                    Step[1].Speed_RPM = ((canBuf[1]<<8) | (canBuf[2]));
                    Step[1].angle     = ((canBuf[3]<<8) | (canBuf[4]));
                }		
                else //状态异常
                {
                    Step[1].state     = canBuf[5];
                }
            }
            if(canBuf[0] == 0x01) //总位移
            {
                var1 = (u32)((canBuf[1]<<24)|(canBuf[2]<<16)|(canBuf[3]<<8)|(canBuf[4]));
            }
            if(canBuf[0] == 0x02)
            {
                var2 = (u32)((canBuf[1]<<24)|(canBuf[2]<<16)|(canBuf[3]<<8)|(canBuf[4]));
                if(canBuf[5] == 0x01) //正
                {
                    Step[1].Integral_bias_speed = (u64)((((u64)var1)<<32) | var2);
                }
                else if(canBuf[5] == 0x00) //负
                {
                    Step[1].Integral_bias_speed = -(u64)((((u64)var1)<<32) | var2);
                }
            }
        }	
        if(can_rx.Identifier == 0x01) //第一台驱动器（右边）
        {
            if(canBuf[0] == 0x00)
            {
                if(canBuf[5] == 0) //0为状态正常，1为不正常
                {
                    Step[0].state     = canBuf[5];
                    Step[0].Speed_RPM = ((canBuf[1]<<8) | (canBuf[2]));
                    Step[0].angle     = ((canBuf[3]<<8) | (canBuf[4]));
                }		
                else //状态异常
                {
                    Step[0].state     = canBuf[5];
                }
            }
            if(canBuf[0] == 0x01) //总位移
            {
                var1 = (u32)((canBuf[1]<<24)|(canBuf[2]<<16)|(canBuf[3]<<8)|(canBuf[4]));
            }
            if(canBuf[0] == 0x02)
            {
                var2 = (u32)((canBuf[1]<<24)|(canBuf[2]<<16)|(canBuf[3]<<8)|(canBuf[4]));
                if(canBuf[5] == 0x01) //正
                {
                    Step[0].Integral_bias_speed = (u64)((((u64)var1)<<32) | var2);
                }
                else if(canBuf[5] == 0x00) //负
                {
                    Step[0].Integral_bias_speed = -(u64)((((u64)var1)<<32) | var2);
                }
            }
        }			
    }
}














