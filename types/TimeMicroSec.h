#pragma once

/*! \brief Simple class to wrap time related operations allowing micro sec precision
*
* Implemented now only for windows
*/
class TimeMicroSec {
	unsigned long time_in_us;

public:
	TimeMicroSec(); /*!< Constructor with the current time */

	TimeMicroSec(unsigned long timeinUS); /*!< Constructor */

	unsigned long TimeInUS() const; /*!< Getter in micro sec */

	double TimeInMS() const; /*!< Getter in mili sec */

	double TimeInS() const; /*!< Getter in sec */

	TimeMicroSec operator-(const TimeMicroSec& time) const; /*!< Difference operator */

	bool operator>=(const TimeMicroSec& time) const; /*!< Is bigger or equal operator */

	bool operator<=(const TimeMicroSec& time) const; /*!< Is smaller or equal operator */

	TimeMicroSec operator+(const TimeMicroSec& time) const; /*!< Sum operator */

	void operator+=(const TimeMicroSec& time);  /*!< Increasing operator */

	TimeMicroSec operator*(double r) const; /*!< Product with a scalar operator */

	~TimeMicroSec();

	void Sleep_for() const; /*!< Sleep for the stored value */
};



