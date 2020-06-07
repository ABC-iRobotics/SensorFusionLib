#pragma once
#include "defs.h"

namespace SF {
	enum MsgType { DATAMSG, TEXT, NOTHING };

	const DTime tWaitNextMsg = DTime(150); // If no msg got within 150 us, msgqueueempty is called
}
