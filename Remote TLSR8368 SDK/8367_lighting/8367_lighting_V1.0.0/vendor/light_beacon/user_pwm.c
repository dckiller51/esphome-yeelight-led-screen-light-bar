//#include "../common.h"
#include "app_config.h"
#include "../../drivers.h"

unsigned short pwm_maxvalue[5]={0xffff,0xffff,0xffff,0xffff,0xffff};

void PWM_MaxFqeSet(PWM_TypeDef pwmNumber,unsigned short MaxValue,unsigned short Fqe)
{
	unsigned int Pwm_clk=CLOCK_SYS_CLOCK_1US*1000*1000/(reg_pwm_clk+1);
	unsigned int cycValue=Pwm_clk/Fqe;
//	WRITE_REG16((CYC_VALUE_BASE + pwmNumber*4),cycValue);
	reg_pwm_max(pwmNumber)=cycValue;
	pwm_maxvalue[pwmNumber]=MaxValue;
}

void PWM_DutyValueSet(PWM_TypeDef pwmNumber,unsigned short value)
{
	unsigned short cycValue=reg_pwm_max(pwmNumber);//READ_REG16(CYC_VALUE_BASE + pwmNumber*4);
	unsigned short csValue=cycValue*value/pwm_maxvalue[pwmNumber];
//	WRITE_REG16((CSC_VALUE_BASE + pwmNumber*4),csValue);
	reg_pwm_cmp(pwmNumber)=csValue;
}

void user_PWMInit(PWM_TypeDef pwmNumber,unsigned short MaxValue,unsigned short Fqe)
{
	if (pwmNumber<1) {
//		SET_PWMMODE(pwmNumber,NORMAL);
		WRITE_REG8(0x783,0);
	}
	PWM_MaxFqeSet(pwmNumber,MaxValue,Fqe);
}
