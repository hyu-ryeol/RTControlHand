/*
 * TCPClientHandler.cpp
 *
 *  Created on: 2020. 10. 26.
 *      Author: parkjunho
 */

#include "TCPClientHandler.h"

TCPClientHandler::TCPClientHandler() {
	// TODO Auto-generated constructor stub

}

TCPClientHandler::~TCPClientHandler() {
	// TODO Auto-generated destructor stub
}

void TCPClientHandler::InitializeClient()
{
	ss.connect(Poco::Net::SocketAddress("192.168.0.117", PORT));
}

int TCPClientHandler::Update()
{
	TxFrame.info.index = Index_HandCommand_request;
	TxFrame.info.subindex = 0x00;

	ss.sendBytes(TxFrame.data, sizeof(TxFrame.data));

	auto len = ss.receiveBytes(RxFrame.data, sizeof(RxFrame.data));
	if(len <= 0)
	{
		std::cout << "Disconnected!!"<< std::endl;
	}
	else
	{
		std::cout << "Received data : " << RxFrame.data <<std::endl;
	}

	return (int)RxFrame.info.subindex;
}

void TCPClientHandler::CloseClient()
{
	ss.close();
}
