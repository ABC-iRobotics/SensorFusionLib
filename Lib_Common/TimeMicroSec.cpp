#include "TimeMicroSec.h"

#include <windows.h>

TimeMicroSec::TimeMicroSec() {
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

TimeMicroSec TimeMicroSec::operator-(const TimeMicroSec& time) const { return TimeMicroSec(time_in_us - time.time_in_us); }

unsigned long TimeMicroSec::TimeInUS() const { return time_in_us; }

double TimeMicroSec::TimeInMS() const { return double(time_in_us)*1e-3; }

double TimeMicroSec::TimeInS() const { return double(time_in_us)*1e-6; }

TimeMicroSec::TimeMicroSec(unsigned long timeinUS) : time_in_us(timeinUS) {}

TimeMicroSec::~TimeMicroSec()
{
}

#include <thread>

void TimeMicroSec::Sleep_for() const {
	std::this_thread::sleep_for(std::chrono::duration<unsigned long, std::micro>(time_in_us));
}

bool TimeMicroSec::operator>=(const TimeMicroSec& time) const {
	return time_in_us >= time.time_in_us;
}

bool TimeMicroSec::operator<=(const TimeMicroSec& time) const {
	return time_in_us <= time.time_in_us;
}

TimeMicroSec TimeMicroSec::operator+(const TimeMicroSec& time) const {
	return TimeMicroSec(time_in_us + time.time_in_us);
}

void TimeMicroSec::operator+=(const TimeMicroSec& time) {
	time_in_us += time.time_in_us;
}

TimeMicroSec TimeMicroSec::operator*(double r) const {
	return TimeMicroSec((unsigned long)(time_in_us*r));
}