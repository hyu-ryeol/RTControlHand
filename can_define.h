/*
 * can_define.h
 *
 *  Created on: 2018. 7. 12.
 *      Author: Administrator
 */

#ifndef CAN_DEFINE_H_
#define CAN_DEFINE_H_

#define NUM_CLIENT 10

#define OBJ_READ 	1
#define OBJ_WRITE 	2

#define READ_REQUEST 			0x40
#define READ_1BYTE_RESPONSE 	0x4F
#define READ_2BYTE_RESPONSE 	0x4B
#define READ_4BYTE_RESPONSE 	0x43

#define WRITE_REQUEST_1BYTE 	0x2F
#define WRITE_REQUEST_2BYTE 	0x2B
#define WRITE_REQUEST_4BYTE 	0x23
#define WRITE_REQUEST_RESPONSE 	0x60

#define COB_NMT 0x000
#define COB_SYNC 0x80
#define COB_TPDO1 0x180
#define COB_RPDO1 0x200
#define COB_TPDO2 0x280
#define COB_RPDO2 0x300
#define COB_TPDO3 0x380
#define COB_RPDO3 0x400
#define COB_TPDO4 0x480
#define COB_RPDO4 0x500
#define COB_SDO 0x600

#define OBJ_CONTROLWORD 			0x6040U
#define SUB_OBJ_SHUTDOWN 			0x06
#define SUB_OBJ_SWITCHON 			0x07
#define SUB_OBJ_ENABLE_OPERATION 	0x0F
#define SUB_OBJ_ENABLE_HOMMING 		0x1F
#define SUB_OBJ_DISABLE_VOLTAGE 	0x02

#define OBJ_STATUSWORD 	0x6041U

#define OBJ_TARGET_TORQUE 0x6071U

#define NMT_START_NODE 	0x01
#define NMT_STOP_NODE 	0x02
#define NMT_PREOP_MODE 	0x80
#define NMT_RESET_NODE 	0x81
#define NMT_RESET_COMMU 0x82

#define OBJ_MODES_OPERATION 0x6060U
#define OBJ_MODES_OPERATION_DISPLAY 0x6061U
#define NO_MODE 		0xFF
#define	POSITION_MODE 	0x01
#define VELOCITY_MODE 	0x03
#define TORQUE_MODE 	0x04
#define HOMING_MODE 	0x06
#define CYNC_TORQUE_MODE 0x0a

//Homming
#define OBJ_HOMING_METHOD 		0x6098U
#define OBJ_HOMING_SPEED 		0x6099U
#define SUB_OBJ_SPEED_SWITCH 	0x01
#define SUB_OBJ_SPEED_ZERO 		0x02
#define OBJ_CURRENT_THRESHOLD 	0x30B2U
#define OBJ_HOME_OFFSET 		0x30B1U
#define OBJ_HOME_ZERO			0x30B0U

#define HOMING_ACTUALPOSITION 0x25
#define HOMING_CURRENT_POSTIVE 0xFD
#define HOMING_CURRENT_NEGATIVE 0xFC

#define OBJ_MODES_DISPLAY	0x6061U
#define OBJ_POSITION_ACTUAL 0x6063U
#define OBJ_VELOCITY_ACTUAL 0x6069U
#define OBJ_CURRENT_ACTUAL 	0x6078U

#define OBJ_SUBINDEX_NULL 0x00U

#define OBJ_RATETORQUE 0x6076U
#define OBJ_PROFILE_VELOCITY 0x6081U

#define PDO_4BYTE 0x20
#define PDO_2BYTE 0x10

#define TPDO1_INDEX 0x1A00U
#define TPDO2_INDEX 0x1A01U
#define TPDO2_INDEX2 0x1801U
#define TPDO3_INDEX 0x1A02U
#define TPDO4_INDEX 0x1A03U

#define RPDO1_INDEX 0x1600U
#define RPDO2_INDEX 0x1601U

typedef union{
	struct{
		unsigned char 	type;
		unsigned char 	index_low;
		unsigned char 	index_high;
		unsigned char 	subindex;
		unsigned char 	data[4];
	}info;
	unsigned char value[8];
}SDO_PACKET;

typedef union{
	unsigned char 	uint8Value[4];
	unsigned short 	uint16Value[2];
	unsigned long 	uint32Value;
}DATA_OBJECT;

typedef union {
	struct {
		unsigned char bit0 : 1;
		unsigned char bit1 : 1;
		unsigned char bit2 : 1;
		unsigned char bit3 : 1;
		unsigned char bit4 : 1;
		unsigned char bit5 : 1;
		unsigned char bit6 : 1;
		unsigned char bit7 : 1;
		unsigned char bit8 : 1;
		unsigned char bit9 : 1;
		unsigned char bit10 : 1;
		unsigned char bit11 : 1;
		unsigned char bit12 : 1;
		unsigned char bit13 : 1;
		unsigned char bit14 : 1;
		unsigned char bit15 : 1;
	}bit;
	unsigned char value[2];
	unsigned short uint16_value;
}StatusInfo;

typedef union {
	struct {
		unsigned char bit0 : 1; //switch on
		unsigned char bit1 : 1;	//Enable voltage
		unsigned char bit2 : 1;	//Quick stop
		unsigned char bit3 : 1;	//Enable operation
		unsigned char bit4 : 1;	//operating mode specific(new setpoint)
		unsigned char bit5 : 1; //operating mode specific(change set immediately)
		unsigned char bit6 : 1; //operating mode specific(abs/rel)
		unsigned char bit7 : 1;
		unsigned char bit8 : 1;
		unsigned char bit9 : 1;
		unsigned char bit10 : 1;
		unsigned char bit11 : 1;
		unsigned char bit12 : 1;
		unsigned char bit13 : 1;
		unsigned char bit14 : 1;
		unsigned char bit15 : 1;
	}bit;
	unsigned char uint8_value[2];
	unsigned short uint16_value;
}ControlInfo;

#endif /* CAN_DEFINE_H_ */
