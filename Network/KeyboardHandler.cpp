/*
 * KeyboardHandler.cpp
 *
 *  Created on: 2020. 10. 26.
 *      Author: parkjunho
 */

#include "../Network/KeyboardHandler.h"

KeyboardHandler::KeyboardHandler() {
	// TODO Auto-generated constructor stub

}

KeyboardHandler::~KeyboardHandler() {
	// TODO Auto-generated destructor stub
}

void KeyboardHandler::InitializeKeyboard()
{
	tcgetattr(0, &initial_settings);
	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
	new_settings.c_lflag &= ~ECHO;
	new_settings.c_cc[VMIN] = 0;
	new_settings.c_cc[VTIME] = 0;
	tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
}

void KeyboardHandler::CloseKeyboard()
{
	tcsetattr(0, TCSANOW, &initial_settings);
}

int KeyboardHandler::_kbhit()
{
	unsigned char ch;
	int nread;
	if(peek_character != -1)
		return 1;
	new_settings.c_cc[VMIN] = 0;
	tcsetattr(0, TCSANOW, &new_settings);
	nread = read(0, &ch, 1);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0, TCSANOW, &new_settings);
	if(nread == 1)
	{
		peek_character = ch;
		return -1;
	}
	return 0;
}

int KeyboardHandler::_getch()
{
	char ch;
	if(peek_character != -1)
	{
		ch = peek_character;
		peek_character = -1;
		return ch;
	}
	read(0, &ch, 1);
	return ch;
}

int KeyboardHandler::_putch(int c)
{
	int res;
	putchar(c);
	fflush(stdout);
	return c;
}

