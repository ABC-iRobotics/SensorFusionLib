#pragma once

class TimeMicroSec {
	unsigned long time_in_us;

public:
	TimeMicroSec();

	TimeMicroSec(unsigned long timeinUS);

	unsigned long TimeInUS() const;

	double TimeInMS() const;

	double TimeInS() const;

	TimeMicroSec operator-(const TimeMicroSec& time) const;

	bool operator>=(const TimeMicroSec& time) const;

	bool operator<=(const TimeMicroSec& time) const;

	TimeMicroSec operator+(const TimeMicroSec& time) const;

	void operator+=(const TimeMicroSec& time);

	TimeMicroSec operator*(double r) const;

	~TimeMicroSec();

	void Sleep_for() const;
};



