#pragma once
#include "defs.h"

namespace SF {
	enum MsgType { DATAMSG, TEXT, NOTHING };

	const DTime tWaitNextMsg = DTime(150);
}
