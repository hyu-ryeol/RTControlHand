/*
 * TCPClientHandler.h
 *
 *  Created on: 2020. 10. 26.
 *      Author: parkjunho
 */

#ifndef NETWORK_TCPCLIENTHANDLER_H_
#define NETWORK_TCPCLIENTHANDLER_H_

#include "Poco/Net/Net.h"
#include "Poco/Net/StreamSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/DateTime.h"
#include "Poco/Timespan.h"
#include <iostream>

#define Index_HandCommand_request 0x3333

struct packet_data{
    float index;
    float subindex;
    float _x;
    float _y;
    float _z;
    float _u;
    float _v;
    float _w;
};

union TCP_Packet{
    packet_data info;
    char data[sizeof(info)];
};

class TCPClientHandler {
public:
	TCPClientHandler();
	virtual ~TCPClientHandler();
	void InitializeClient();
	int Update();
	void CloseClient();

private:

	const Poco::UInt16 PORT = 32452;

	Poco::DateTime now;
	Poco::Net::StreamSocket ss;

	TCP_Packet TxFrame, RxFrame;
};

#endif /* NETWORK_TCPCLIENTHANDLER_H_ */
