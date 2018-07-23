// ---------------------------------------------------------------------------------
// libSwitch.h
//
// Created on: 12-Feb-2010
// Author: nbk
// ---------------------------------------------------------------------------------

#ifndef LIBSWITCH_H_
#define LIBSWITCH_H_

#define USER_SW_PORT	GPIOA			// Change port pin here!
#define USER_SW_PIN		GPIO_Pin_14
#define USER_SW		GPIO_ReadInputDataBit(USER_SW_PORT,USER_SW_PIN)

extern void InitSwitch(void);
extern int16_t ReadSWSpecial(void);
extern void WaitSW(void);
extern void User_sw_service(void);
#endif /* LIBSWITCH_H_ */
