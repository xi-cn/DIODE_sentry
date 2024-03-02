#pragma once

#include "PCANBasic.h"
#include <iostream>
#define UINT32 uint32_t
#include <limits.h>
#define MAX_PATH PATH_MAX
#define sprintf_s snprintf
#include <cstring>
#define strcpy_s(destination, destination_size, source) strcpy(destination, source)

#include <termios.h>
#include <unistd.h>

#include <thread>

static int _getch() {
	struct termios config;
	tcgetattr(STDIN_FILENO, &config);
	struct termios saved_config = config;
	config.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &config);
	int res = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &saved_config);
	return res;
}

class pcanThread
{
public:
	bool m_TimerOn; // Shows if thread run
	float quat[4], gyro[3], acc[3], bullet_speed;
	int mode;
private:
	const TPCANHandle PcanHandle = PCAN_USBBUS1;
	const bool IsFD = false;
	const TPCANBaudrate Bitrate = PCAN_BAUD_1M;
	const int TimerInterval = 25;
	// const int PcanID = 0x600;
	// const int MsgLen = 8;
	// const BYTE MsgType = PCAN_MESSAGE_STANDARD;

	bool m_DLLFound;
	TPCANMsg msgCanMessage;
public:
	// std::thread m_hTimer; // Used for reading
	// TimerRead constructor
	pcanThread();
	// TimerRead destructor
	~pcanThread();
	void ReadMessages();
	void WriteMessages();
	void TransformData(const VisionData &data);
private:
	TPCANStatus ReadMessage();
	bool CheckForLibrary();
	void ShowConfigurationHelp();
	void ShowCurrentConfiguration();
	void ShowStatus(TPCANStatus status);
	void FormatChannelName(TPCANHandle handle, LPSTR buffer);
	void GetTPCANHandleName(TPCANHandle handle, LPSTR buffer);
	void GetFormattedError(TPCANStatus error, LPSTR buffer);
	TPCANStatus WriteMessage();
	void ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer);

	int bytesToInt(BYTE bytes[]);
	float bytesToFloat(BYTE bytes[]);
	void floatTobytes(float data, BYTE bytes[]);
	void Can_receive(unsigned int ID, BYTE data[]);
	
};