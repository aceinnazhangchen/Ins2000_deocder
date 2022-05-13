#pragma once
#include <time.h>

#define   LEAPS   18

typedef struct {        /* time struct */
	time_t time;        /* time (s) expressed by standard time_t */
	double sec;         /* fraction of second under 1 s */
} gtime_t;

gtime_t epoch2time(const double *ep);
gtime_t gpst2time(int week, double sec);
gtime_t timeadd(gtime_t t, double sec);
gtime_t gpst2utc(gtime_t t);
void time2epoch(gtime_t t, double *ep);