/* NRMKFoundation, Copyright 2016- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */

/* NRMKFoundation, Copyright 2016- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */

#ifndef __XENO__
#define __XENO__
#endif

#include "main.h"

TP_PORT test_port = port1;

int rtsercan_fd  = -1;
CAN_FRAME txframe, rxframe;

RT_QUEUE msg_print;
RT_TASK control_task;
RT_TASK print_task;
RT_TASK can_recv_task;

static int working;
int system_flag = 1;
int deactivate_flag = 1;
int demo_step=0;
int demo_joint=0;
int demo_delay=0;
int demo_type=0;

void activate(int id);
void deactivate(int id);
void Homing(int id);
int NMT_state(int id, unsigned char state);
int modeofoperation(int id, unsigned char mode);
int controlword(int id, unsigned char data);
int ratetorque(int id, float *data);
int statusword(int id, unsigned char *data);

int RPDO2_send(int id, short RPDO_VAL);
int RPDO3_send(int id, int controlword, int despos);
int TPDO2_read(int *d1, int *d2);
int TPDO2_read(int *d1, int *d2, int *d3);
int epos_sync(void);
void InputSaturation(StateInfo *joint);

void demo1(StateInfo *joint);
void demo2(StateInfo *joint);

KeyboardHandler hKey;
int KeyCh;

TCPClientHandler sClient;
int CommandState=0;

void CleanUp(void)
{
	sClient.CloseClient();
	hKey.CloseKeyboard();
	for(int j=0;j< NUM_CLIENT; ++j )
	{
		deactivate(j+1);
	}
	close(rtsercan_fd);
}

void catch_signal(int sig)
{
	system_flag = 2;
	CleanUp();
	rt_queue_unbind(&msg_print);
	rt_task_delete(&control_task);
	rt_task_delete(&print_task);

	rt_printf("\n System Deactivate! \n");
	return;
}

void control_thread(void* arg)
{
	int res;
	int count = 1;
	int print_count = 0, test_count = 0;
	void *msg;
	double demo_time=0;

	double init_time=0;

	Controller controller;
	StateInfo Jinfo;
	ControlInfo cInfo;
	MotorInfo mInfo;

	hKey.InitializeKeyboard();

	int len =sizeof(StateInfo);
	memset(&Jinfo, 0, len);
	memset(&mInfo, 0, sizeof(MotorInfo));

	res = NMT_state(0, NMT_PREOP_MODE);

	rt_print_CANFrame(txframe);
	do{
		usleep(5000);
		res = SERCAN_read(rtsercan_fd, &rxframe);
		if(res == SERCAN_ERR_FREE){
			rt_print_CANFrame(rxframe);
			count++;
		}

	}while(count >=3);


	for(int j=0;j< NUM_CLIENT; ++j )
	{
		activate(j+1);
	}

	count = 0;

	rt_task_set_periodic(NULL, TM_NOW, 5e6);
	Jinfo.point1 = rt_timer_read();
	while (working)
	{
		rt_task_wait_period(NULL);

		if(system_flag == 1)
		{
			Jinfo.point2 = rt_timer_read();

			epos_sync();
			for(int j=0;j<NUM_CLIENT; ++j ){
				res = TPDO2_read(Jinfo.status, Jinfo.toq, Jinfo.enc_pos);
				Jinfo.StatusWord[j].uint16_value = Jinfo.status[j];
			}

			if(hKey._kbhit())
			{
				KeyCh = hKey._getch();
				hKey._putch(KeyCh);
				CommandState = KeyCh;
			}

			controller.PDcontroller(&Jinfo, &mInfo, CommandState);

			for(int j=0;j< NUM_CLIENT; ++j ){
				//res = RPDO2_send(j+1, 0);
				res = RPDO2_send(j+1, Jinfo.int16_val[j]);
			}

			Jinfo.point1 = Jinfo.point3;
			Jinfo.point3 = rt_timer_read();

			Jinfo.dt = ((double)(Jinfo.point3-Jinfo.point1))*1e-9;
			Jinfo.control_dt = ((double)(Jinfo.point3-Jinfo.point2))*1e-9;
			if(Jinfo.worst_dt <= Jinfo.control_dt)
				Jinfo.worst_dt = Jinfo.control_dt;

			Jinfo.systime += Jinfo.dt;
			if((Jinfo.systime - 60.0*Jinfo.systime_min) >= 60)
			{
				Jinfo.systime_min++;
			}

			if(print_count >= PRINT_COUNT)
			{
				msg = rt_queue_alloc(&msg_print, len);
				if(msg == NULL)
					rt_printf("rt_queue_alloc failed to allocate, NULL pointer received\n");
				memcpy(msg, &Jinfo, len);
				rt_queue_send(&msg_print, msg, len, Q_NORMAL);
				print_count = 1;
			}
			else
			{
				if(print_count == 0)
				{
					Jinfo.control_dt = 0;
					Jinfo.worst_dt = 0;
					Jinfo.systime = 0;
					Jinfo.systime_min = 0;
				}
				print_count++;
			}

		}
	}
}

void recv_thread(void *arg)
{

	int tcpcommand;
	rt_task_set_periodic(NULL, TM_NOW, 5e6);
	while (1)
	{
		rt_task_wait_period(NULL);
		try{
			sClient.InitializeClient();

			tcpcommand= sClient.Update();
			if(tcpcommand == 0x00)
			{
				CommandState = 53;
			}
			else if(tcpcommand == 0x01)
			{
				CommandState = 50;
			}
			else if(tcpcommand == 0x11)
			{
				CommandState = 54;
			}
			else if(tcpcommand == 0x10)
			{
				CommandState = 52;
			}
		}
		catch(Poco::Exception& exc)
		{

		}

	}
}

void print_task_proc(void *arg)
{

	ssize_t len;
	void *msg;
	StateInfo info;
	memset(&info, 0, sizeof(StateInfo));

	int err = rt_queue_bind(&msg_print, "PRINT_QUEUE", TM_INFINITE);
	if(err)
		fprintf(stderr, "failed to queue bind, code %d\n", err);

	while ((len = rt_queue_receive(&msg_print, &msg, TM_INFINITE)) > 0)
	{
		memcpy(&info, msg, sizeof(StateInfo));
		rt_printf("System Time: [%d min, %0.3lf sec]\n",
				info.systime_min, info.systime-60.0*info.systime_min );
		rt_printf("Task: [%0.2f]ms, Control: [%0.3f]ms, Worst:[%0.2f]ms\n",
				info.dt*1000.0, info.control_dt*1000.0,info.worst_dt*1000.0);
		for(int i=0; i<NUM_CLIENT; ++i)
		{
			//rt_printf("Joint %d: status:[0x%04x] Pos: %d, Toq: %d\n",
			//					i, info.StatusWord[i].uint16_value, info.enc_pos[i], info.toq[i]);
			rt_printf("Joint %d: status:[0x%04x] Pos: %0.1finc, Toq: %d%\t",
											i, info.StatusWord[i].uint16_value, info.aq[i], info.toq[i]);
			rt_printf("c_torq: %d\n", info.int16_val[i]);
		}
		rt_printf("received Key >> %d\n", KeyCh);
		rt_printf("received message >> len=%d bytes, ptr=%p\n\n", len, msg);
		rt_queue_free(&msg_print, msg);
	}
}


int main(int argc, char* argv[])
{

	int res;

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* no memory-swapping for this programm */
	mlockall(MCL_CURRENT | MCL_FUTURE);

	// Perform auto-init of rt_print buffers if the task doesn't do so
	rt_print_auto_init(1);

	// open rtsercan*******************
	rtsercan_fd=SERCAN_open();
	rt_printf("Sercan_fd = %d\n", rtsercan_fd);
	if (rtsercan_fd < 0)
		return 1;
	else
		working = 1;
	//********************************

	if(argc > 1)
	{
		demo_type = atoi(argv[1]);
		switch(atoi(argv[1]))
		{
		case 1:
			demo_type = 1;
			break;
		case 2:
			demo_type = 2;
			break;
		default:
			demo_type = 0;
			break;
		}
	}
	else{
		demo_type = 0;
	}


	res = rt_queue_create(&msg_print, "PRINT_QUEUE", sizeof(StateInfo)*10, 10, Q_FIFO|Q_SHARED);
	if(res < 0){
		rt_printf("rt_queue create failed: %d\n", res);
		return 1;
	}

	//-------------------------------------------------------

	rt_task_create(&can_recv_task, "can_recv_task", 0, 99, 0);
	rt_task_start(&can_recv_task, &recv_thread, NULL);

	rt_task_create(&control_task, "control_task", 0, 95, T_FPU);
	rt_task_start(&control_task, &control_thread, NULL);

	rt_task_create(&print_task, "print_task", 0, 70, 0);
	rt_task_start(&print_task, &print_task_proc, NULL);

	pause();
	CleanUp();
	return 0;

}

void activate(int id)
{
	modeofoperation(id, TORQUE_MODE);

	controlword(id, SUB_OBJ_SHUTDOWN);
	controlword(id, SUB_OBJ_SWITCHON);
	controlword(id, SUB_OBJ_ENABLE_OPERATION);
	NMT_state(id, NMT_START_NODE);
}

void deactivate(int id)
{
	controlword(id, SUB_OBJ_SHUTDOWN);
	//controlword(id, SUB_OBJ_DISABLE_VOLTAGE);
	//NMT_state(id, NMT_RESET_NODE);
}

void Homing(int id)
{
	int res;

	modeofoperation(id, HOMING_MODE);
	controlword(id, SUB_OBJ_SHUTDOWN);
	controlword(id, SUB_OBJ_SWITCHON);
	controlword(id, SUB_OBJ_ENABLE_OPERATION);

	////////// 1. homing method
	memset(txframe.data, 0, sizeof(txframe.data));
	memset(rxframe.data, 0, sizeof(rxframe.data));

	SDO_PACKET tx_obj;
	DATA_OBJECT tx_data;

	if(id == 1)
	{
		txframe.can_id = COB_SDO + id;
		tx_data.uint16Value[0] = OBJ_HOMING_METHOD;
		tx_obj.info.type = WRITE_REQUEST_1BYTE;
		tx_obj.info.index_low = tx_data.uint8Value[0];
		tx_obj.info.index_high = tx_data.uint8Value[1];
		tx_obj.info.subindex = OBJ_SUBINDEX_NULL;
		tx_obj.info.data[0] = HOMING_CURRENT_NEGATIVE;

	}
	else if(id == 2)
	{
		txframe.can_id = COB_SDO + id;
		tx_data.uint16Value[0] = OBJ_HOMING_METHOD;
		tx_obj.info.type = WRITE_REQUEST_1BYTE;
		tx_obj.info.index_low = tx_data.uint8Value[0];
		tx_obj.info.index_high = tx_data.uint8Value[1];
		tx_obj.info.subindex = OBJ_SUBINDEX_NULL;
		tx_obj.info.data[0] = HOMING_CURRENT_POSTIVE;

	}
	else if(id == 4)
	{
		txframe.can_id = COB_SDO + id;
		tx_data.uint16Value[0] = OBJ_HOMING_METHOD;
		tx_obj.info.type = WRITE_REQUEST_1BYTE;
		tx_obj.info.index_low = tx_data.uint8Value[0];
		tx_obj.info.index_high = tx_data.uint8Value[1];
		tx_obj.info.subindex = OBJ_SUBINDEX_NULL;
		tx_obj.info.data[0] = HOMING_CURRENT_NEGATIVE;

	}
	else
	{
		txframe.can_id = COB_SDO + id;
		tx_data.uint16Value[0] = OBJ_HOMING_METHOD;
		tx_obj.info.type = WRITE_REQUEST_1BYTE;
		tx_obj.info.index_low = tx_data.uint8Value[0];
		tx_obj.info.index_high = tx_data.uint8Value[1];
		tx_obj.info.subindex = OBJ_SUBINDEX_NULL;
		tx_obj.info.data[0] = HOMING_CURRENT_POSTIVE;

	}

	memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
	txframe.can_dlc = 5;
	SERCAN_write(rtsercan_fd, txframe);
	rt_print_CANFrame(txframe);
	usleep(1000);
	do{
		res = SERCAN_read(rtsercan_fd, &rxframe);
	}while(res == SERCAN_ERR_FREE);
	rt_print_CANFrame(rxframe);

	////////// 2. homing offset
	tx_data.uint16Value[0] = OBJ_HOME_OFFSET;
	tx_obj.info.type = WRITE_REQUEST_4BYTE;
	tx_obj.info.index_low = tx_data.uint8Value[0];
	tx_obj.info.index_high = tx_data.uint8Value[1];
	tx_obj.info.subindex = OBJ_SUBINDEX_NULL;
	if(id == 1)
	{	//-519797
		unsigned int int32_tmp = (unsigned int)519797;
		tx_obj.info.data[0] = int32_tmp & 0x000000ff;
		tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
		tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
		tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;
	}
	else if(id == 2)
	{	//-512048
		unsigned int int32_tmp = (unsigned int)-512048;
		tx_obj.info.data[0] = int32_tmp & 0x000000ff;
		tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
		tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
		tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;
	}
	else if(id == 3)
	{	//-121415
		unsigned int int32_tmp = (unsigned int)-124124;
		tx_obj.info.data[0] = int32_tmp & 0x000000ff;
		tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
		tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
		tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;
	}
	else if(id == 4)
	{	//-121415
		unsigned int int32_tmp = (unsigned int)16459;
		tx_obj.info.data[0] = int32_tmp & 0x000000ff;
		tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
		tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
		tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;
	}

	memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
	txframe.can_dlc = 8;
	SERCAN_write(rtsercan_fd, txframe);
	rt_print_CANFrame(txframe);
	usleep(1000);
	do{
		res = SERCAN_read(rtsercan_fd, &rxframe);
	}while(res == SERCAN_ERR_FREE);
	rt_print_CANFrame(rxframe);


	////////// 2.5 homing zero
	memset(txframe.data, 0, sizeof(txframe.data));

	tx_data.uint16Value[0] = OBJ_HOME_ZERO;
	tx_obj.info.type = WRITE_REQUEST_4BYTE;
	tx_obj.info.index_low = tx_data.uint8Value[0];
	tx_obj.info.index_high = tx_data.uint8Value[1];
	tx_obj.info.subindex = OBJ_SUBINDEX_NULL;

	unsigned int int32_tmp = (unsigned int)0;
	tx_obj.info.data[0] = int32_tmp & 0x000000ff;
	tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
	tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
	tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;

	memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
	txframe.can_dlc = 8;
	SERCAN_write(rtsercan_fd, txframe);
	rt_print_CANFrame(txframe);
	usleep(1000);
	do{
		res = SERCAN_read(rtsercan_fd, &rxframe);
	}while(res == SERCAN_ERR_FREE);
	rt_print_CANFrame(rxframe);

	//////////3.homing speed switch
	tx_data.uint16Value[0] = OBJ_HOMING_SPEED;
	tx_obj.info.type = WRITE_REQUEST_4BYTE;
	tx_obj.info.index_low = tx_data.uint8Value[0];
	tx_obj.info.index_high = tx_data.uint8Value[1];
	tx_obj.info.subindex = SUB_OBJ_SPEED_SWITCH;

	if(id == 1)
	{
		//2000 rpm
		unsigned int int32_tmp = (unsigned int)2000;
		tx_obj.info.data[0] = int32_tmp & 0x000000ff;
		tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
		tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
		tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;

		memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
		txframe.can_dlc = 8;
		SERCAN_write(rtsercan_fd, txframe);
		rt_print_CANFrame(txframe);
		usleep(1000);
		do{
			res = SERCAN_read(rtsercan_fd, &rxframe);
		}while(res == SERCAN_ERR_FREE);
		rt_print_CANFrame(rxframe);

	}
	else if(id == 2)
	{
		//2000 rpm
		unsigned int int32_tmp = (unsigned int)2000;
		tx_obj.info.data[0] = int32_tmp & 0x000000ff;
		tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
		tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
		tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;

		memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
		txframe.can_dlc = 8;
		SERCAN_write(rtsercan_fd, txframe);
		rt_print_CANFrame(txframe);
		usleep(1000);
		do{
			res = SERCAN_read(rtsercan_fd, &rxframe);
		}while(res == SERCAN_ERR_FREE);
		rt_print_CANFrame(rxframe);
	}
	else if(id == 3)
	{

	}
	else if(id == 4)
	{
		//2000 rpm
		unsigned int int32_tmp = (unsigned int)80;
		tx_obj.info.data[0] = int32_tmp & 0x000000ff;
		tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
		tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
		tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;

		memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
		txframe.can_dlc = 8;
		SERCAN_write(rtsercan_fd, txframe);
		rt_print_CANFrame(txframe);
		usleep(1000);
		do{
			res = SERCAN_read(rtsercan_fd, &rxframe);
		}while(res == SERCAN_ERR_FREE);
		rt_print_CANFrame(rxframe);
	}
	//////////4.homing speed zero
	tx_data.uint16Value[0] = OBJ_HOMING_SPEED;
	tx_obj.info.type = WRITE_REQUEST_4BYTE;
	tx_obj.info.index_low = tx_data.uint8Value[0];
	tx_obj.info.index_high = tx_data.uint8Value[1];
	tx_obj.info.subindex = SUB_OBJ_SPEED_ZERO;
	if(id == 1)
	{
		//2000 rpm
		unsigned int int32_tmp = (unsigned int)2000;
		tx_obj.info.data[0] = int32_tmp & 0x000000ff;
		tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
		tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
		tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;

		memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
		txframe.can_dlc = 8;
		SERCAN_write(rtsercan_fd, txframe);
		rt_print_CANFrame(txframe);
		usleep(1000);
		do{
			res = SERCAN_read(rtsercan_fd, &rxframe);
		}while(res == SERCAN_ERR_FREE);
		rt_print_CANFrame(rxframe);

	}
	else if(id == 2)
	{
		//2000 rpm
		unsigned int int32_tmp = (unsigned int)2000;
		tx_obj.info.data[0] = int32_tmp & 0x000000ff;
		tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
		tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
		tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;

		memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
		txframe.can_dlc = 8;
		SERCAN_write(rtsercan_fd, txframe);
		rt_print_CANFrame(txframe);
		usleep(1000);
		do{
			res = SERCAN_read(rtsercan_fd, &rxframe);
		}while(res == SERCAN_ERR_FREE);
		rt_print_CANFrame(rxframe);
	}
	else if(id == 3)
	{

	}
	else if(id == 4)
	{
		//2000 rpm
		unsigned int int32_tmp = (unsigned int)80;
		tx_obj.info.data[0] = int32_tmp & 0x000000ff;
		tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
		tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
		tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;

		memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
		txframe.can_dlc = 8;
		SERCAN_write(rtsercan_fd, txframe);
		rt_print_CANFrame(txframe);
		usleep(1000);
		do{
			res = SERCAN_read(rtsercan_fd, &rxframe);
		}while(res == SERCAN_ERR_FREE);
		rt_print_CANFrame(rxframe);
	}


	//////////5.current threshold positive speed
	tx_data.uint16Value[0] = OBJ_CURRENT_THRESHOLD;
	tx_obj.info.type = WRITE_REQUEST_2BYTE;
	tx_obj.info.index_low = tx_data.uint8Value[0];
	tx_obj.info.index_high = tx_data.uint8Value[1];
	tx_obj.info.subindex = 0x00;

	if(id == 1)
	{
		//1500 mA
		unsigned short int16_tmp = (unsigned short)1500;
		tx_obj.info.data[0] = int16_tmp & 0x00ff;
		tx_obj.info.data[1] = (int16_tmp & 0xff00) >> 8;
	}
	else if(id == 2)
	{
		//1500 mA
		unsigned short int16_tmp = (unsigned short)1500;
		tx_obj.info.data[0] = int16_tmp & 0x00ff;
		tx_obj.info.data[1] = (int16_tmp & 0xff00) >> 8;
	}
	else if(id == 3)
	{
		//500 mA
		unsigned short int16_tmp = (unsigned short)500;
		tx_obj.info.data[0] = int16_tmp & 0x00ff;
		tx_obj.info.data[1] = (int16_tmp & 0xff00) >> 8;
	}
	else if(id == 4)
	{
		//500 mA
		unsigned short int16_tmp = (unsigned short)4500;
		tx_obj.info.data[0] = int16_tmp & 0x00ff;
		tx_obj.info.data[1] = (int16_tmp & 0xff00) >> 8;
	}

	memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
	txframe.can_dlc = 6;
	SERCAN_write(rtsercan_fd, txframe);
	rt_print_CANFrame(txframe);
	usleep(1000);
	do{
		res = SERCAN_read(rtsercan_fd, &rxframe);
	}while(res == SERCAN_ERR_FREE);
	rt_print_CANFrame(rxframe);

	controlword(id, SUB_OBJ_ENABLE_HOMMING);
	StatusInfo info;

	while(1)
	{
		usleep(1000000);
		statusword(id, info.value);
		if(info.bit.bit10 == 1 && info.bit.bit12 == 1 && info.bit.bit15 == 1)
			break;
	}
	return;
}

int NMT_state(int id, unsigned char state)
{
	int res;
	memset(txframe.data, 0, sizeof(txframe.data));
	txframe.can_id = COB_NMT;
	txframe.data[0] = state;
	txframe.data[1] = id;
	txframe.can_dlc = 2;
	res = SERCAN_write(rtsercan_fd, txframe);
	usleep(2000);
	return res;
}

int modeofoperation(int id, unsigned char mode)
{
	int res;
	memset(txframe.data, 0, sizeof(txframe.data));
	memset(rxframe.data, 0, sizeof(rxframe.data));
	SDO_PACKET tx_obj;
	DATA_OBJECT tx_data;

	txframe.can_id = COB_SDO + id;
	tx_data.uint16Value[0] = OBJ_MODES_OPERATION;
	tx_obj.info.type = WRITE_REQUEST_1BYTE;
	tx_obj.info.index_low = tx_data.uint8Value[0];
	tx_obj.info.index_high = tx_data.uint8Value[1];
	tx_obj.info.subindex = OBJ_SUBINDEX_NULL;
	tx_obj.info.data[0] = mode;

	memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
	txframe.can_dlc = 5;
	SERCAN_write(rtsercan_fd, txframe);
	usleep(1000);
	do{
		res = SERCAN_read(rtsercan_fd, &rxframe);
	}while(res == SERCAN_ERR_FREE);
	rt_print_CANFrame(rxframe);

	if(mode == POSITION_MODE)
	{
		txframe.can_id = COB_SDO + id;
		tx_data.uint16Value[0] = OBJ_PROFILE_VELOCITY;
		tx_obj.info.type = WRITE_REQUEST_4BYTE;
		tx_obj.info.index_low = tx_data.uint8Value[0];
		tx_obj.info.index_high = tx_data.uint8Value[1];
		tx_obj.info.subindex = OBJ_SUBINDEX_NULL;

		unsigned int int32_tmp = (unsigned int)4000;
		tx_obj.info.data[0] = int32_tmp & 0x000000ff;
		tx_obj.info.data[1] = (int32_tmp & 0x0000ff00) >> 8;
		tx_obj.info.data[2] = (int32_tmp & 0x00ff0000) >> 16;
		tx_obj.info.data[3] = (int32_tmp & 0xff000000) >> 24;

		memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
		txframe.can_dlc = 8;
		SERCAN_write(rtsercan_fd, txframe);
		usleep(1000);
		do{
			res = SERCAN_read(rtsercan_fd, &rxframe);
		}while(res == SERCAN_ERR_FREE);
		rt_print_CANFrame(rxframe);
	}

	return res;
}

int controlword(int id, unsigned char data)
{
	int res;
	memset(txframe.data, 0, sizeof(txframe.data));
	memset(rxframe.data, 0, sizeof(rxframe.data));
	SDO_PACKET tx_obj;
	DATA_OBJECT tx_data;

	txframe.can_id = COB_SDO + id;
	tx_data.uint16Value[0] = OBJ_CONTROLWORD;
	tx_obj.info.type = WRITE_REQUEST_2BYTE;
	tx_obj.info.index_low = tx_data.uint8Value[0];
	tx_obj.info.index_high = tx_data.uint8Value[1];
	tx_obj.info.subindex = OBJ_SUBINDEX_NULL;
	tx_obj.info.data[0] = data;

	memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
	txframe.can_dlc = 6;
	SERCAN_write(rtsercan_fd, txframe);
	usleep(1000);
	do{
		res = SERCAN_read(rtsercan_fd, &rxframe);
	}while(res == SERCAN_ERR_FREE);
	rt_print_CANFrame(rxframe);
	usleep(100000);
	return res;
}

int statusword(int id, unsigned char *data)
{
	int res;
	memset(txframe.data, 0, sizeof(txframe.data));
	memset(rxframe.data, 0, sizeof(rxframe.data));
	SDO_PACKET tx_obj;
	DATA_OBJECT tx_data;

	txframe.can_id = COB_SDO + id;
	tx_data.uint16Value[0] = OBJ_STATUSWORD;
	tx_obj.info.type = READ_REQUEST;
	tx_obj.info.index_low = tx_data.uint8Value[0];
	tx_obj.info.index_high = tx_data.uint8Value[1];
	tx_obj.info.subindex = OBJ_SUBINDEX_NULL;

	memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
	txframe.can_dlc = 4;
	SERCAN_write(rtsercan_fd, txframe);
	usleep(1000);
	do{
		res = SERCAN_read(rtsercan_fd, &rxframe);
	}while(res == SERCAN_ERR_FREE);

	data[0] = rxframe.data[4];
	data[1] = rxframe.data[5];
	//rt_printf("data[0]:0x%x, data[1]:0x%x\n", data[0], data[1]);
	rt_print_CANFrame(rxframe);
	return res;
}

int ratetorque(int id, float *data)
{
	int res;
	memset(txframe.data, 0, sizeof(txframe.data));
	memset(rxframe.data, 0, sizeof(rxframe.data));
	SDO_PACKET tx_obj;
	DATA_OBJECT tx_data;

	txframe.can_id = COB_SDO + id;
	tx_data.uint16Value[0] = OBJ_RATETORQUE;
	tx_obj.info.type = READ_REQUEST;
	tx_obj.info.index_low = tx_data.uint8Value[0];
	tx_obj.info.index_high = tx_data.uint8Value[1];
	tx_obj.info.subindex = OBJ_SUBINDEX_NULL;

	memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
	txframe.can_dlc = 4;
	SERCAN_write(rtsercan_fd, txframe);
	usleep(1000);
	do{
		res = SERCAN_read(rtsercan_fd, &rxframe);
	}while(res == SERCAN_ERR_FREE);
	rt_print_CANFrame(rxframe);
	int tmp = (int)(rxframe.data[4] + (rxframe.data[5]<<8)
			+ (rxframe.data[6]<<16) + (rxframe.data[7]<<24));
	data[id-1] = (float)tmp/1000.0;
	return res;
}

int RPDO2_send(int id, short RPDO_VAL)
{
	memset(txframe.data, 0, sizeof(txframe.data));

	SDO_PACKET tx_obj;

	txframe.can_id = COB_RPDO2 + id;
	unsigned short tmp = (unsigned short)RPDO_VAL;
	tx_obj.value[0] = 0x0f;
	tx_obj.value[1] = 0x00;
	tx_obj.value[2] = tmp & 0x00ff;
	tx_obj.value[3] = (tmp & 0xff00) >> 8;

	memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
	txframe.can_dlc = 4;

	return SERCAN_write(rtsercan_fd, txframe);
}

int RPDO3_send(int id, int controlword, int despos)
{
	memset(txframe.data, 0, sizeof(txframe.data));

	SDO_PACKET tx_obj;

	txframe.can_id = COB_RPDO3 + id;
	unsigned short int16_tmp = (unsigned short)controlword;
	tx_obj.value[0] = int16_tmp & 0x00ff;
	tx_obj.value[1] = (int16_tmp & 0xff00) >> 8;

	unsigned int int32_tmp = (unsigned int)despos;
	tx_obj.value[2] = int32_tmp & 0x000000ff;
	tx_obj.value[3] = (int32_tmp & 0x0000ff00) >> 8;
	tx_obj.value[4] = (int32_tmp & 0x00ff0000) >> 16;
	tx_obj.value[5] = (int32_tmp & 0xff000000) >> 24;

	memcpy(txframe.data, tx_obj.value, sizeof(tx_obj.value));
	txframe.can_dlc = 6;

	return SERCAN_write(rtsercan_fd, txframe);
}


int TPDO2_read(int *d1, int *d2)
{
	int res;
	memset(rxframe.data, 0, sizeof(rxframe.data));
	res = SERCAN_read(rtsercan_fd, &rxframe);
	if(res==SERCAN_ERR_FREE)
	{
		int id = rxframe.can_id & 0x00F;

		d1[id-1] = (int)(rxframe.data[0] + (rxframe.data[1]<<8)
				+ (rxframe.data[2]<<16) + (rxframe.data[3]<<24));
		d2[id-1] = (int)(rxframe.data[4] + (rxframe.data[5]<<8)
				+ (rxframe.data[6]<<16) + (rxframe.data[7]<<24));
		return res;
	}
	else
		return res;
}

int TPDO2_read(int *d1, int *d2, int *d3)
{
	int res;
	memset(rxframe.data, 0, sizeof(rxframe.data));
	res = SERCAN_read(rtsercan_fd, &rxframe);
	if(res==SERCAN_ERR_FREE)
	{
		int id = rxframe.can_id - COB_TPDO2;

		d1[id-1] = 0; d2[id-1] = 0; d3[id-1] = 0;
		d1[id-1] = (int)(rxframe.data[0] + (rxframe.data[1]<<8));
		d2[id-1] = (int)(short)(rxframe.data[2] + (rxframe.data[3]<<8));
		d3[id-1] = (int)(rxframe.data[4] + (rxframe.data[5]<<8) + (rxframe.data[6]<<16) + (rxframe.data[7]<<24));

		return res;
	}
	else
		return res;
}

int epos_sync(void)
{
	int res;
	memset(txframe.data, 0, sizeof(txframe.data));
	txframe.can_id = COB_SYNC;
	txframe.can_dlc = 0;
	res = SERCAN_write(rtsercan_fd, txframe);
	return res;

}

void InputSaturation(StateInfo *joint)
{
	for(int i=0; i<NUM_CLIENT; i++)
	{
		if(joint->int16_val[i] >= 1000)
			joint->int16_val[i] = 1000;
		else if(joint->int16_val[i] <= -1000)
			joint->int16_val[i] = -1000;
	}
}

void demo1(StateInfo *joint){

	joint->int32_val[0] = 0;
	joint->int32_val[1] = 0;
	joint->int32_val[2] = 0;

	if(demo_step == 0)
	{
		joint->int32_val[demo_joint] = JOINT_SET_POINT;
		joint->int32_val[2] = (int)round(85*860.16);
		demo_delay++;
		if(demo_delay >= 50){
			demo_delay = 0;

			//status->uint16_value = joint->status[demo_joint];
			//if(status->bit.bit10 == 1){
			//	demo_step=1;
			//}
			if(joint->StatusWord[demo_joint].bit.bit10 == 1){
				demo_step=1;
			}
		}
	}
	else if(demo_step == 1)
	{
		joint->int32_val[demo_joint] = -JOINT_SET_POINT;
		joint->int32_val[2] = -(int)round(85*860.16);
		demo_delay++;
		if(demo_delay >= 50){
			demo_delay = 0;
			//status->uint16_value = joint->status[demo_joint];
			//if(status->bit.bit10 == 1){
			//	demo_step=2;
			//}
			if(joint->StatusWord[demo_joint].bit.bit10 == 1){
				demo_step=2;
			}
		}
	}
	else if(demo_step == 2)
	{
		demo_delay++;
		if(demo_delay >= 50){
			demo_delay = 0;
			//status->uint16_value = joint->status[demo_joint];
			//if(status->bit.bit10 == 1){
			if(joint->StatusWord[demo_joint].bit.bit10 == 1){
				demo_step=0;
				demo_joint++;
				if(demo_joint >= 2)
					demo_joint = 0;
			}
		}
	}
}

void demo2(StateInfo *joint){
	Kinematics demo2;
	Joint res;


	//float xd = R*cosf(2*M_PI*0.1*(float)joint->systime);
	//float yd = R*sinf(2*M_PI*0.1*(float)joint->systime);

	float R = 70;
	float ratio = 0.1;
	float xd = R*cosf(2*M_PI*ratio);
	float yd = R*sinf(2*M_PI*ratio);

	res = demo2.InverseKinematics(xd, yd);
	joint->int32_val[0] = (int)roundf(res.q[0]);
	joint->int32_val[1] = (int)roundf(res.q[1]);
	joint->int32_val[3] = (int)roundf(100000.0*sinf(2*M_PI*0.1*(float)joint->systime));

	if(joint->int32_val[0] >= 500000)
		joint->int32_val[0] = 500000;
	else if(joint->int32_val[0] <= -500000)
		joint->int32_val[0] = -500000;
	if(joint->int32_val[1] >= 500000)
		joint->int32_val[1] = 500000;
	else if(joint->int32_val[1] <= -500000)
		joint->int32_val[1] = -500000;

	//rt_printf("Joint1 = %d, Joint2 = %d\n", joint->int32_val[0], joint->int32_val[1]);
}

