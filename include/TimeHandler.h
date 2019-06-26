//All the functions that handles timer

#ifndef TIMEHANDLER_H
#define TIMEHANDLER_H

//#include <fcntl.h>
//#include <unistd.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <time.h>
#include <sys/time.h>
#include <math.h>

//__inline__ unsigned long long int rdtsc()
inline unsigned long long int rdtsc()
{
     unsigned long long int x;
     __asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
     return x;
}

inline long long int clickPerSecond()
{
	//Get the clicks per second
	long long int curr_time = rdtsc();
	sleep(1);
	
	long long int click_sec = rdtsc() - curr_time;
	return click_sec;
}

//Should change it to a class, it would be easier

inline double currTimeMS(long long int click_sec)
{
	long long int curr_time = rdtsc() ;
	double curr_time_MS = ( (double)  curr_time ) / ( (double) click_sec ) * 1000.0;//In millsecond
	return curr_time_MS;
}


#endif

