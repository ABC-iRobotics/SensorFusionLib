#pragma once

class TimeUS {
	unsigned long time_in_us;

public:
	TimeUS();

	TimeUS(unsigned long timeinUS);

	unsigned long TimeInUS() const;

	double TimeInMS() const;

	double TimeInS() const;

	TimeUS operator-(const TimeUS& time) const;

	~TimeUS();
};



