/*
 * KeyboardHandler.h
 *
 *  Created on: 2020. 10. 26.
 *      Author: parkjunho
 */

#ifndef NETWORK_KEYBOARDHANDLER_H_
#define NETWORK_KEYBOARDHANDLER_H_

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

class KeyboardHandler {
public:
	KeyboardHandler();
	virtual ~KeyboardHandler();
	void InitializeKeyboard(void);
	void CloseKeyboard(void);

	int _kbhit();
	int _getch();
	int _putch(int c);

private:
	struct termios initial_settings, new_settings;
	int peek_character = -1;
};

#endif /* NETWORK_KEYBOARDHANDLER_H_ */
