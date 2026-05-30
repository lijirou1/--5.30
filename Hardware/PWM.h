#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare);	//PWM通道1，PA0
void PWM_SetCompare2(uint16_t Compare);	//PWM通道2，PA1

#endif
