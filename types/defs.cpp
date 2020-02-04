#include "defs.h"

using namespace SF;

DTime SF::duration_since_epoch(const Time & t) {
	return std::chrono::duration_cast<DTime>(t.time_since_epoch());
}

Time SF::InitFromDurationSinceEpochInMicroSec(const long long & value) {
	return Time(DTime(value));
}

DTime SF::InitFromDurationInMicroSec(const long long & value) {
	return DTime(value);
}

Time SF::Now() {
	return std::chrono::system_clock::now();
}
