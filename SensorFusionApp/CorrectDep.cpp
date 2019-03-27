#include "pch.h"
#include "CorrectDep.h"


Eigen::VectorXi CorrectLength(unsigned int newsize, const Eigen::RowVectorXi& in) {
	unsigned int oldsize = in.size();
	Eigen::VectorXi out = in;
	out.conservativeResize(newsize);
	for (unsigned int i = oldsize; i < newsize; i++) out(i) = 0;
	return out;
}
