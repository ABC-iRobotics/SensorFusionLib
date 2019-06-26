#pragma once

class Time {
	unsigned long time_in_us;

public:
	Time();

	Time(unsigned long timeinUS);

	unsigned long TimeInUS() const;

	double TimeInMS() const;

	double TimeInS() const;

	Time operator-(const Time& time) const;

	~Time();
};



