// ---------------------------------------------------------------------------------
// libFlash.c
// Created on: 30-Mar-2010
// Author: nbk
// ReadDataFromFlash() - Read the flash data
// WriteDataToFlash() - Write data to flash
// CheckFlashData() - Check that flash data is valid
// ---------------------------------------------------------------------------------
#include "project.h"
#include <string.h>

uint32_t flashDataAddress = 0x807F800;	// last page of high density device
char flashDataTestStrg[] = "AlphaProject";

typedef struct {
	char flashDataTestStrg[16];

} sFlashData;


sFlashData flashData;

// Return False if flash data is not valid
bool CheckFlashData() {
	return strncmp(flashDataTestStrg, (char*)flashDataAddress, strlen(flashDataTestStrg));
}

bool ReadDataFromFlash() {
	// Check that flash data is valid
	// Just use a simple string to check
	sFlashData *ptr;

	if (CheckFlashData()) {
		DispDotMatrixWait("FlashData not initialised!");
		return FALSE;
	}

	ptr = (sFlashData *)flashDataAddress;
	flashData = *ptr;

	//DispDotMatrixWait("OK !");
    return TRUE;
}

void WriteDataToFlash() {
	int i;
	FLASH_Status status;
	uint16_t *ptr, *dest;

	strcpy((char*)&flashData, (const char*)flashDataTestStrg);

    FLASH_Unlock();
    status = FLASH_ErasePage(flashDataAddress);
    switch(status) {
    case FLASH_BUSY:
    	DispDotMatrixWait("EraseBusy");
    	break;
    case FLASH_ERROR_PG:
    	DispDotMatrixWait("EraseError page");
    	break;
    case FLASH_ERROR_WRP:
    	DispDotMatrixWait("EraseError write");
    	break;
    case FLASH_COMPLETE:
    	//DispDotMatrixWait("EraseOK!!");
    	break;
    case FLASH_TIMEOUT:
    	DispDotMatrixWait("EraseTimeout");
    	break;
    default:
    	break;

    }
    ptr = (uint16_t*)&flashData;
    dest = (uint16_t*)flashDataAddress;
    for (i=0; i<sizeof(flashData)/sizeof(*ptr); i++) {
    	status = FLASH_ProgramHalfWord((uint32_t)dest,*ptr);
        switch(status) {
        case FLASH_BUSY:
        	DispDotMatrixWait("WriteBusy");
        	break;
        case FLASH_ERROR_PG:
        	DispDotMatrixWait("WriteError page");
        	break;
        case FLASH_ERROR_WRP:
        	DispDotMatrixWait("WriteError write");
        	break;
        case FLASH_COMPLETE:
        	//DispDotMatrixWait("EraseOK!!");
        	break;
        case FLASH_TIMEOUT:
        	DispDotMatrixWait("WriteTimeout");
        	break;
        default:
        	break;

        }   	ptr++;
    	dest++;
    }

    FLASH_Lock();
	DispDotMatrixWait("Done!");
}


