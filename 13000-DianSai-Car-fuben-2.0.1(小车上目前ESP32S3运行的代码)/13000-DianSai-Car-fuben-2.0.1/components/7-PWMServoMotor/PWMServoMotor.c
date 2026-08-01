// #include "PWMServoMotor.h"
// #include "PWM.h"

// void Set_Angle_180(uint8_t Angle)//Duty = ((1/1800) * Angle + 0.025) *1024
// {
//     if(Angle > 180)
//     {
//         Angle = 180;
//     }
//     uint16_t Duty = ( (float)Angle/1800 + 0.025 ) * 1024;
//     Duty_Set(Duty);
// }
// void PWMServoMotor_Init(void)
// {
//     PWM_Init();
// }