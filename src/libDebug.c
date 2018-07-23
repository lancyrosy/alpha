// ---------------------------------------------------------------------------------
//  file : libdebug.c
//  name : nbk
//  date : 26/9/96
//  desc : file containing all debug routines for trashbin and uMouse simulation
// ---------------------------------------------------------------------------------

#include "Project.h"
#include "libdebug.h"
#define DEBUG
#ifdef DEBUG

#define FIRSTROW (1)

char *dirSymbol[]={"N","ne","E","se","S","sw","W","nw"};
static char buf[256];
void _PRINTPATH(uint8_t *path, int16_t pathLen) {

	register int i, j;

	for (i=0, j=0; i<pathLen; i++) {
		j+=sprintf( &buf[j], "%s", dirSymbol[path[i]] );
	}
	sprintf( &buf[j], ".  ");

	gotoxy(1, FIRSTROW+1);
	printf("PATH=%s", buf);
}

void _PRINTFASTPATH(uint8_t *path) {

	register int i, j;

	for (i=0, j=0; path[i+1]!=0; i+=2) {
		j+=sprintf( &buf[j], "%s%1d", dirSymbol[path[i]], path[i+1] );
	}
	sprintf( &buf[j], ".  ");

	gotoxy(1, FIRSTROW+1);
	printf("PATH=%s", buf);
}

void _PRINTDOTMATRIX(char *strg) {
	gotoxy(1, 1);
	printf("%s.", strg);
}

void _PRINTMSG(char *strg) {
	gotoxy(1, FIRSTROW);
	printf("%s.  ", strg);
}

void _PRINTERROR(char *strg) {
	gotoxy(1, FIRSTROW+4);
	printf( "%s.  ", strg);
}


void _PRINTROBOTPOS() {
	gotoxy(50, FIRSTROW+2);
//	printf("%02d,%02d%2s-", robotStatus.myRowPos, robotStatus.myColPos, dirSymbol[robotStatus.myDir]);
}

#endif
