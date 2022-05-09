#ifndef CAN_COM_H
#define CAN_COM_H

#include "../structs.h"
#include "user_config.h"
#include "mbed.h"
#include "../math_ops.h"

void pack_reply(CANMessage *msg, float p, float v, float t);		//CAN的答应包结构 
void unpack_cmd(CANMessage msg, ControllerStruct * controller);	//CAN接口的指令

#endif
