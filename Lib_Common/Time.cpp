#include "Time.h"

#include <windows.h>

Time::Time() {
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

Time Time::operator-(const Time& time) const { return Time(time_in_us - time.time_in_us); }

unsigned long Time::TimeInUS() const { return time_in_us; }

double Time::TimeInMS() const { return double(time_in_us)*1e-3; }

double Time::TimeInS() const { return double(time_in_us)*1e-6; }

Time::Time(unsigned long timeinUS) : time_in_us(timeinUS) {}

Time::~Time()
{
}
