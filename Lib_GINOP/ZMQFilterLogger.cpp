#include "ZMQFilterLogger.h"



ZMQFilterLogger::ZMQFilterLogger(SystemManager & filter, int port) :
	FilterLog(filter), zmqPub(port) {}

ZMQFilterLogger::~ZMQFilterLogger()
{
}
