// ---------------------------------------------------------------------------------
// libCMUCAM.h
// Created on: 12-Feb-2010
// Author: nbk
//  desc :
// ---------------------------------------------------------------------------------
#ifndef LIBCMUCAM_H_
#define LIBCMUCAM_H_

// CAM3 image XSIZE and YSIZE depends on resolution setting.
// Default is 88x72.
// Next size is 176x144. Send command (DS 1 1)
// Highest resolution is 352x288. Send command (HR 1) & (DS 1 1)
// Currently, the software processing limits the maximum x&y size to 255 as data
// are stored in bytes rather than 16-bit words

#define CAM3_XSIZE			88
#define CAM3_YSIZE			72

#define CAM3_NUMOFCOLOR		3
#define CAM3_NUMOFPARA		11
#define CAM3_CX				0		// centroid x
#define CAM3_CY				1		// centroid y
#define CAM3_X0				2		// Bounding box topLeft coordinate, x0,y0
#define CAM3_Y0				3
#define CAM3_X1				4		// Bounding box bottomRight coordinate, x1,y1
#define CAM3_Y1				5
#define CAM3_AREA			6		// Area in pixel of color blob
#define CAM3_FILL			7		// percentage of color area over bounding box area
#define CAM3_XLEFT			8		// left edge pixel centroid
#define CAM3_XRIGHT			9		// right edge pixel centroid
#define CAM3_HANGLE			10		// horizontal angle of line

volatile extern char camDataChar[CAM3_NUMOFCOLOR][50];
volatile extern int camData[CAM3_NUMOFCOLOR][CAM3_NUMOFPARA];
volatile extern bool camDataFlag[CAM3_NUMOFCOLOR];
extern int alignColorPara;
extern int alignColorNum, alignXref;
extern bool bColorTrackFlag;

void PrintCamValues();	// To print CMUCAM3 values for debugging purposes

void StartCMUCAM(int16_t uartNum, int baudrate);
void PrintCamValues();
void CAMRxISR();
#endif
