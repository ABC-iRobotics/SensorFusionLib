#pragma once

#include "DataMsg.h"

namespace SF {

	bool VerifyDataMsgContent(void* buf, int length);

	DataMsg InitDataMsg(void* buf, OperationType source,
		unsigned char ID, DataType type);

	void SerializeDataMsg(const DataMsg& msg, void*& buf, int& length);

}
