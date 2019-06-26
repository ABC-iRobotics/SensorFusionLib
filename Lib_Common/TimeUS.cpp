#include "TimeUS.h"

#include <windows.h>

TimeUS::TimeUS() {
	SYSTEMTIME time;
	GetSystemTime(&time);
	time_in_us = time.wYear;
	time_in_us *= 12;
	time_in_us += time.wDay;
	time_in_us *= 24;
	time_in_us += time.wHour;
	time_in_us *= 60;
	time_in_us += time.wMinute;
	time_in_us *= 60;
	time_in_us += time.wSecond;
	time_in_us *= 1000;
	time_in_us += time.wMilliseconds;
	time_in_us *= 1000;
}

TimeUS TimeUS::operator-(const TimeUS& time) const { return TimeUS(time_in_us - time.time_in_us); }

unsigned long TimeUS::TimeInUS() const { return time_in_us; }

double TimeUS::TimeInMS() const { return double(time_in_us)*1e-3; }

double TimeUS::TimeInS() const { return double(time_in_us)*1e-6; }

TimeUS::TimeUS(unsigned long timeinUS) : time_in_us(timeinUS) {}

TimeUS::~TimeUS()
{
}
