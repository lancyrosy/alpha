// ---------------------------------------------------------------------------------
// libDebug.h
// Created on: 12-Feb-2010
// Author: nbk
//  desc : header file containing all debug routines for trashbin
//         and uMouse simulation
// ---------------------------------------------------------------------------------
#ifndef LIBDEBUG_H_
#define LIBDEBUG_H_

void _PRINTPATH(uint8_t *path, int16_t pathLen);

void _PRINTFASTPATH(uint8_t *path);

void _PRINTBINNUM();
void _PRINTNAME(uint8_t *strg);
void _PRINTMSG(char *strg);
void _PRINTERROR(char *strg);
void _PRINTROBOTPOS();
void _PRINTDOTMATRIX(char *strg);

#if defined(DEBUG) || defined(SIMULATION)

#define PRINTPATH(path, pathLen) 	_PRINTPATH(path, pathLen)
#define PRINTFASTPATH(path) 		_PRINTFASTPATH(path)
#define PRINTBINNUM() 				_PRINTBINNUM()
#define PRINTDOTMATRIX(strg) 		   _PRINTDOTMATRIX(strg)
#define PRINTMSG(strg) 			_PRINTMSG(strg)
#define PRINTERROR(strg) 		_PRINTERROR(strg)
#define PRINTROBOTPOS()			_PRINTROBOTPOS()
#define PUTCHAR(c)				PutChar(c)
#define GETCHAR(c)				GetChar()
#define	DEBUGCODE(n)			(debugCode=n)
#define	DEBUGCODE1(n)			(debugCode1=n)
#define	DEBUGCODE2(n)			(debugCode2=n)
#else

#define PRINTPATH(path, pathLen)
#define PRINTFASTPATH(path)
#define PRINTBINNUM()
#define PRINTNAME(strg)
#define PRINTMSG(strg)
#define PRINTERROR(strg)
#define PRINTDUMPSITE( lowest )
#define PRINTROBOTPOS()
#define PUTCHAR(c)
#define GETCHAR()
#define	DEBUGCODE(n)
#define	DEBUGCODE1(n)
#define	DEBUGCODE2(n)
#endif


#endif /* LIBDEBUG_H_ */
