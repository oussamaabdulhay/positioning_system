#pragma once 

#include "DataMessage.hpp"
#include "common_types.hpp"

class PositionMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();

	double x, y, z;
};