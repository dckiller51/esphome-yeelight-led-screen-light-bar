#pragma once
#include "../../drivers.h"

void PWM_MaxFqeSet(PWM_TypeDef pwmNumber,unsigned short MaxValue,unsigned short Fqe);
void PWM_DutyValueSet(PWM_TypeDef pwmNumber,unsigned short value);
void user_PWMInit(PWM_TypeDef pwmNumber,unsigned short MaxValue,unsigned short Fqe);
