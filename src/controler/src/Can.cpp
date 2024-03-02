#include "controler/Can.h"

int pcanThread::bytesToInt(BYTE bytes[])
{
    // 位操作时 使用一个unsigned int变量来作为位容器。
    int addr = bytes[0] & 0xFF;
    addr |= ((bytes[1] << 8) & 0xFF00);
    addr |= ((bytes[2] << 16) & 0xFF0000);
    addr |= ((bytes[3] << 24) & 0xFF000000);
    return addr;
}

float pcanThread::bytesToFloat(BYTE bytes[])
{
    // 位操作时 使用一个unsigned int变量来作为位容器。
    return *((float*)bytes);
}

void pcanThread::floatTobytes(float data, BYTE bytes[])
{
    // 位操作时 使用一个unsigned int变量来作为位容器。
    size_t length = sizeof(float);

    BYTE *pdata = (BYTE*)&data; //把float类型的指针强制转换为unsigned char型
    for (int i = 0; i < length; i++){
        bytes[i] = *pdata++;//把相应地址中的数据保存到unsigned char数组中
    }
    return;
}

void pcanThread::Can_receive(unsigned int ID, BYTE data[])
{
	switch(ID){
		case 0x402:
			quat[0] = bytesToFloat(data);
			quat[1] = bytesToFloat(data + 4);
		break;
		case 0x403:
			quat[2] = bytesToFloat(data);
			quat[3] = bytesToFloat(data + 4);
		break;
		case 0x404:
			gyro[0] = bytesToFloat(data);
			gyro[1] = bytesToFloat(data + 4);
		break;
		case 0x405:
			gyro[2] = bytesToFloat(data);
			acc[0] = bytesToFloat(data + 4);
		break;
		case 0x406:
			acc[1] = bytesToFloat(data);
			acc[2] = bytesToFloat(data + 4);
		break;
		case 0x407:
			bullet_speed = bytesToFloat(data);
			mode = bytesToInt(data + 4);
		break;
	}

	return ;
}

/**
 *@brief   转换数据
 *@param   data   类型  VisionData(union)  包含pitch,yaw,distance
 *@param   flag   类型  char   用于判断是否瞄准目标，0代表没有，1代表已经瞄准
 */
void pcanThread::TransformData(const VisionData &data)
{
	msgCanMessage.DATA[BYTE(0)] = data.pitch_angle.c[0];
	msgCanMessage.DATA[BYTE(1)] = data.pitch_angle.c[1];
	msgCanMessage.DATA[BYTE(2)] = data.pitch_angle.c[2];
	msgCanMessage.DATA[BYTE(3)] = data.pitch_angle.c[3];

	msgCanMessage.DATA[BYTE(4)] = data.yaw_angle.c[0];
	msgCanMessage.DATA[BYTE(5)] = data.yaw_angle.c[1];
	msgCanMessage.DATA[BYTE(6)] = data.yaw_angle.c[2];
	msgCanMessage.DATA[BYTE(7)] = data.yaw_angle.c[3];
}

pcanThread::pcanThread()
{
	ShowCurrentConfiguration(); // Shows the current parameters configuration

	// Checks if PCANBasic.dll is available, if not, the program terminates
	m_DLLFound = CheckForLibrary();
	if (!m_DLLFound)
		return;

	TPCANStatus stsResult;
	// Initialization of the selected channel
	stsResult = CAN_Initialize(PcanHandle, Bitrate);

	if (stsResult != PCAN_ERROR_OK)
	{
		std::cout << "Can not initialize. Please check the defines in the code.\n";
		ShowStatus(stsResult);
		return;
	}

	// Reading messages...
	std::cout << "Pcan successfully initialized.\n";
	msgCanMessage.ID = 0x600;
	msgCanMessage.LEN = 8;
	msgCanMessage.MSGTYPE = PCAN_MESSAGE_STANDARD;
	m_TimerOn = true;
}

pcanThread::~pcanThread()
{
	m_TimerOn = false;
	if (m_DLLFound)
		CAN_Uninitialize(PCAN_NONEBUS);
}

void pcanThread::ReadMessages()
{
	TPCANStatus stsResult;

	// We read at least one time the queue looking for messages. If a message is found, we look again trying to
	// find more. If the queue is empty or an error occurr, we get out from the dowhile statement.
	do
	{
		stsResult = ReadMessage();
		if (stsResult != PCAN_ERROR_OK && stsResult != PCAN_ERROR_QRCVEMPTY)
		{
			ShowStatus(stsResult);
			return;
		}
		usleep(100);
	} while (!(stsResult & PCAN_ERROR_QRCVEMPTY));
}

TPCANStatus pcanThread::ReadMessage()
{
	TPCANMsg CANMsg;
	TPCANTimestamp CANTimeStamp;

	// We execute the "Read" function of the PCANBasic
	TPCANStatus stsResult = CAN_Read(PcanHandle, &CANMsg, &CANTimeStamp);
	Can_receive(CANMsg.ID, CANMsg.DATA);
	if (stsResult != PCAN_ERROR_QRCVEMPTY)
		// We process the received message
		;
		// std::cout << "We process the received message\n";
	

	return stsResult;
}

void pcanThread::WriteMessages()
{
	TPCANStatus stsResult;

	stsResult = WriteMessage();

	// Checks if the message was sent
	if (stsResult != PCAN_ERROR_OK)
		ShowStatus(stsResult);
	else
		std::cout << "\nMessage was successfully SENT";
}

TPCANStatus pcanThread::WriteMessage()
{
	// Sends a CAN message with extended ID, and 8 data bytes

	return CAN_Write(PcanHandle, &msgCanMessage);
}

bool pcanThread::CheckForLibrary()
{
	// Check for dll file
	try
	{
		CAN_Uninitialize(PCAN_NONEBUS);
		return true;
	}
	catch (const std::exception&)
	{
		std::cout << ("Unable to find the library: PCANBasic::dll !\n");
		std::cout << ("Closing...\n");
		std::cout << "Press any key to continue...\n";
		_getch();
	}

	return false;
}

void pcanThread::ShowCurrentConfiguration()
{
	std::cout << "Parameter values used\n";
	std::cout << "----------------------\n";
	char buffer[MAX_PATH];
	FormatChannelName(PcanHandle, buffer);
	std::cout << "* PCANHandle: " << buffer << "\n";
	std::cout << "* IsFD: False\n";
	ConvertBitrateToString(Bitrate, buffer);
	std::cout << "* Bitrate: " << buffer << "\n";
	std::cout << "* TimerInterval: " << TimerInterval << "\n";
	std::cout << "\n";
}

void pcanThread::ShowStatus(TPCANStatus status)
{
	std::cout << "=========================================================================================\n";
	char buffer[MAX_PATH];
	GetFormattedError(status, buffer);
	std::cout << buffer << "\n";
	std::cout << "=========================================================================================\n";
}

void pcanThread::FormatChannelName(TPCANHandle handle, LPSTR buffer)
{
	TPCANDevice devDevice;
	BYTE byChannel;

	// Gets the owner device and channel for a PCAN-Basic handle
	if (handle < 0x100)
	{
		devDevice = (TPCANDevice)(handle >> 4);
		byChannel = (BYTE)(handle & 0xF);
	}
	else
	{
		devDevice = (TPCANDevice)(handle >> 8);
		byChannel = (BYTE)(handle & 0xFF);
	}

	// Constructs the PCAN-Basic Channel name and return it
	char handleBuffer[MAX_PATH];
	GetTPCANHandleName(handle, handleBuffer);
	sprintf_s(buffer, MAX_PATH, "%s %d (%Xh)", handleBuffer, byChannel, handle);
}

void pcanThread::GetTPCANHandleName(TPCANHandle handle, LPSTR buffer)
{
	strcpy_s(buffer, MAX_PATH, "PCAN_NONE");
	switch (handle)
	{
	case PCAN_PCIBUS1:
	case PCAN_PCIBUS2:
	case PCAN_PCIBUS3:
	case PCAN_PCIBUS4:
	case PCAN_PCIBUS5:
	case PCAN_PCIBUS6:
	case PCAN_PCIBUS7:
	case PCAN_PCIBUS8:
	case PCAN_PCIBUS9:
	case PCAN_PCIBUS10:
	case PCAN_PCIBUS11:
	case PCAN_PCIBUS12:
	case PCAN_PCIBUS13:
	case PCAN_PCIBUS14:
	case PCAN_PCIBUS15:
	case PCAN_PCIBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_PCI");
		break;

	case PCAN_USBBUS1:
	case PCAN_USBBUS2:
	case PCAN_USBBUS3:
	case PCAN_USBBUS4:
	case PCAN_USBBUS5:
	case PCAN_USBBUS6:
	case PCAN_USBBUS7:
	case PCAN_USBBUS8:
	case PCAN_USBBUS9:
	case PCAN_USBBUS10:
	case PCAN_USBBUS11:
	case PCAN_USBBUS12:
	case PCAN_USBBUS13:
	case PCAN_USBBUS14:
	case PCAN_USBBUS15:
	case PCAN_USBBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_USB");
		break;

	case PCAN_LANBUS1:
	case PCAN_LANBUS2:
	case PCAN_LANBUS3:
	case PCAN_LANBUS4:
	case PCAN_LANBUS5:
	case PCAN_LANBUS6:
	case PCAN_LANBUS7:
	case PCAN_LANBUS8:
	case PCAN_LANBUS9:
	case PCAN_LANBUS10:
	case PCAN_LANBUS11:
	case PCAN_LANBUS12:
	case PCAN_LANBUS13:
	case PCAN_LANBUS14:
	case PCAN_LANBUS15:
	case PCAN_LANBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_LAN");
		break;

	default:
		strcpy_s(buffer, MAX_PATH, "UNKNOWN");
		break;
	}
}

void pcanThread::GetFormattedError(TPCANStatus error, LPSTR buffer)
{
	// Gets the text using the GetErrorText API function. If the function success, the translated error is returned.
	// If it fails, a text describing the current error is returned.
	if (CAN_GetErrorText(error, 0x09, buffer) != PCAN_ERROR_OK)
		sprintf_s(buffer, MAX_PATH, "An error occurred. Error-code's text (%Xh) couldn't be retrieved", error);
}

void pcanThread::ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer)
{
	switch (bitrate)
	{
	case PCAN_BAUD_1M:
		strcpy_s(buffer, MAX_PATH, "1 MBit/sec");
		break;
	case PCAN_BAUD_800K:
		strcpy_s(buffer, MAX_PATH, "800 kBit/sec");
		break;
	case PCAN_BAUD_500K:
		strcpy_s(buffer, MAX_PATH, "500 kBit/sec");
		break;
	case PCAN_BAUD_250K:
		strcpy_s(buffer, MAX_PATH, "250 kBit/sec");
		break;
	case PCAN_BAUD_125K:
		strcpy_s(buffer, MAX_PATH, "125 kBit/sec");
		break;
	case PCAN_BAUD_100K:
		strcpy_s(buffer, MAX_PATH, "100 kBit/sec");
		break;
	case PCAN_BAUD_95K:
		strcpy_s(buffer, MAX_PATH, "95,238 kBit/sec");
		break;
	case PCAN_BAUD_83K:
		strcpy_s(buffer, MAX_PATH, "83,333 kBit/sec");
		break;
	case PCAN_BAUD_50K:
		strcpy_s(buffer, MAX_PATH, "50 kBit/sec");
		break;
	case PCAN_BAUD_47K:
		strcpy_s(buffer, MAX_PATH, "47,619 kBit/sec");
		break;
	case PCAN_BAUD_33K:
		strcpy_s(buffer, MAX_PATH, "33,333 kBit/sec");
		break;
	case PCAN_BAUD_20K:
		strcpy_s(buffer, MAX_PATH, "20 kBit/sec");
		break;
	case PCAN_BAUD_10K:
		strcpy_s(buffer, MAX_PATH, "10 kBit/sec");
		break;
	case PCAN_BAUD_5K:
		strcpy_s(buffer, MAX_PATH, "5 kBit/sec");
		break;
	default:
		strcpy_s(buffer, MAX_PATH, "Unknown Bitrate");
		break;
	}
}
