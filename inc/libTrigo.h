/*****************************************************************************
  File : libTrigo.h
  Date : 10-Dec-2007
  Name : NBK
  Desc : Include file for libTrigo.c
*****************************************************************************/
#ifndef _LIBTRIGO_H
#define _LIBTRIGO_H


/***************************************************************************
 Data type definition
***************************************************************************/

/*****************************************************************************
  Macros
*****************************************************************************/
#define Degree180			1800
#define Degree90			900
#define SineTabSize			900
#define SineValueScale		10000
#define TangentTabSize		450
#define TangentValueScale	10000
#define Limit180Deg(angle)	(angle=(angle>1800)?(-3600+angle):((angle<-1800)?(3600+angle):angle))

/*****************************************************************************
  Global variables
*****************************************************************************/
extern  const int16_t sineTab[];
extern const int16_t tangentTab[];

/*****************************************************************************
  Function prototypes
*****************************************************************************/
// return the value of sine function.
// Input deg are in terms of 0.1 degrees. Range from -3600 to 3600
// Output is from -10000 to 10000, i.e. values are scaled up 10000!!!
int16_t Sine(int16_t deg);
int16_t Cosine(int16_t deg);
int16_t ArcSine(int16_t sine);
int16_t ArcTangent(int16_t y, int16_t x);

#endif
