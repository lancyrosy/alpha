// ---------------------------------------------------------------------------------
//   File        :libSysTick.h
//   Date        :2009/06/19
//   Author      :nbk
//   Description :Wait routine Library Header
// ---------------------------------------------------------------------------------

#ifndef LIBSYSTICK_H_
#define LIBSYSTICK_H_

extern void DelaymSec(uint32_t time_ms);
extern void SysTick_Configuration(void);
extern void Handler_SysTick(void);

#endif
