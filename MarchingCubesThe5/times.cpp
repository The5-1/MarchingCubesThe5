/*
_CRT_SECURE_NO_DEPRECATE allows the use of fopen, _ftime, ...
*/
#define _CRT_SECURE_NO_DEPRECATE
#include "times.h"

/* **************************************************************************************
0. Convert the Windows time-Commands to Linux standards
************************************************************************************** */
int gettimeofday(timeval* t, void* timezone)
{
	struct _timeb timebuffer;
	_ftime(&timebuffer);
	t->tv_sec = timebuffer.time;
	t->tv_usec = 1000 * timebuffer.millitm;
	return 0;
}

clock_t times(tms *__buffer) {

	__buffer->tms_utime = clock();
	__buffer->tms_stime = 0;
	__buffer->tms_cstime = 0;
	__buffer->tms_cutime = 0;
	return __buffer->tms_utime;
}

/* **************************************************************************************
1. Timer Class to give current time and time steps
************************************************************************************** */
Timer::Timer() {
	gettimeofday(&tv, 0);
	startTime = (double)tv.tv_sec + (double)tv.tv_usec / 1000.0;
}

void Timer::update() {
	gettimeofday(&tv, 0);
	double stoppedAt = (double)tv.tv_sec + (double)tv.tv_usec / (1000.0 * 1000.0);
	intervall = stoppedAt - startTime;
	startTime = stoppedAt;
	currentTime += intervall;
}