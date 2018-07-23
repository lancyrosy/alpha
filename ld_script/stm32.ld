/*
Linker script for STM32F10x
Configure memory sizes, end of stack and boot mode for your project here.
Modified : nbk
*/

/* include the common STM32F10x sub-script */
INCLUDE "../ld_script/STM32_COMMON.ld"

/* Memory Spaces Definitions */
MEMORY
{
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K  /* also change _estack below */
/*FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 128K*/
/*  FLASH (rx) : ORIGIN = 0x8003000, LENGTH = 512K - 12K*/
  FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 512K
  FLASHB1 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB0 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB1 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB2 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB3 (rx) : ORIGIN = 0x00000000, LENGTH = 0
}

/* highest address of the user mode stack (64K RAM)*/
_estack = 0x20010000;

/* include the section management sub-script */
/* (either "STM32_SEC_FLASH.ld" or "STM32_SEC_RAM.ld") */
INCLUDE "../ld_script/STM32_SEC_FLASH.ld"
