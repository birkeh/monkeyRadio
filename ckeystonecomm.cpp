#include "ckeystonecomm.h"

#ifdef QT_DEBUG
//#define OUTPUT_DEBUG
#endif

#ifdef Q_OS_WIN
#include <windows.h>
#endif
#include <QDebug>
#include <QThread>

#define HEAD	0xFE
#define END		0xFD

// Set GPIO function
#define DATA_IN					0
#define DATA_OUT				1
#define MUTE					2
#define I2S_DATA_IN				3
#define I2S_LRCLK_IN			4
#define I2S_SSCLK_IN			5
#define I2S_BCLK_IN				6
#define I2S_DATA_OUT			7
#define I2S_LRCLK_OUT			8
#define I2S_SSCLK_OUT			9
#define I2S_BCLK_OUT			10
#define SPDIF_OUT				11
#define SPI_DO					12
#define SPI_CLK					13
#define SPI_CS					14
#define SPI_DI					15

// Driving strength(0~3)
#define DRIVE_2MA				0
#define DRIVE_4MA				1
#define DRIVE_6MA				2
#define DRIVE_8MA				3

#define ANTENNA_RELAY_PIN		11

QString	programTypeText[]	=
{
	"",
	"News",
	"Current Affairs",
	"Information",
	"Sport",
	"Education",
	"Drama",
	"Arts",
	"Science",
	"Talk",
	"Pop Music",
	"Rock Music",
	"Easy Listening",
	"Light Classical",
	"Classical Music",
	"Other Music",
	"Weather",
	"Finance",
	"Children",
	"Factual",
	"Religion",
	"Phone In",
	"Travel",
	"Leisure",
	"Jazz and Blues",
	"Country Music",
	"National Music",
	"Oldies Music",
	"Folk Music",
	"Documentary",
	"Undefined",
	"Undefined"
};

cKeyStoneCOMM::cKeyStoneCOMM(const QString& commPort, QObject *parent) :
	QObject{parent},
	m_commPort{commPort},
	m_serialPort{nullptr},
	m_serialReadError{false},
	m_hardMute{false},
	m_initialized{false},
	m_totalProgram{0},
	m_currentProgram{0},
	m_mutePrevVolume{0},
	m_mainTransportID{0},
	m_MSCDataIndex{0},
	m_imageDataIndex{0}
{
}

cKeyStoneCOMM::~cKeyStoneCOMM()
{
	close();
}

bool cKeyStoneCOMM::open(bool _hardMute)
{
	close();

	m_hardMute		= _hardMute;
	m_serialPort	= new QSerialPort(m_commPort, this);
	if(!m_serialPort)
		return(false);

	m_serialPort->setBaudRate(QSerialPort::Baud115200);
	m_serialPort->setDataBits(QSerialPort::Data8);
	m_serialPort->setStopBits(QSerialPort::OneStop);
	m_serialPort->setParity(QSerialPort::NoParity);
	m_serialPort->setFlowControl(QSerialPort::NoFlowControl);

	if(!m_serialPort->open(QIODevice::ReadWrite))
	{
		qDebug() << "open failed: " << m_serialPort->errorString();
		close();
		return(false);
	}
#ifdef OUTPUT_DEBUG
	qDebug() << "open succeeded.";
#endif

	m_serialPort->setBaudRate(QSerialPort::Baud115200);
	m_serialPort->setDataBits(QSerialPort::Data8);
	m_serialPort->setStopBits(QSerialPort::OneStop);
	m_serialPort->setParity(QSerialPort::NoParity);
	m_serialPort->setFlowControl(QSerialPort::NoFlowControl);

#ifdef OUTPUT_DEBUG
	qDebug() << "*** Baud Rate:    " << m_serialPort->baudRate();
	qDebug() << "*** Data Bits:    " << m_serialPort->dataBits();
	qDebug() << "*** Stop Bits:    " << m_serialPort->stopBits();
	qDebug() << "*** Parity Bits:  " << m_serialPort->parity();
	qDebug() << "*** Flow Control: " << m_serialPort->flowControl();
#endif

	connect(m_serialPort, &QSerialPort::readyRead,     this, &cKeyStoneCOMM::serialReadReady);
	connect(m_serialPort, &QSerialPort::errorOccurred, this, &cKeyStoneCOMM::serialReadError);

	if(m_hardMute)
		hardMute();
	else
		hardUnmute();

	QThread::msleep(800);

	if(!isSysReady())
	{
		for(int i = 0;i < 2;i++)
		{
			if(hardResetRadio())
			{
				for(int x = 0;x < 2;x++)
				{
					if(isSysReady())
					{
						// Enable Pin 11 for antenna switching
						GPIO_SetFunction(ANTENNA_RELAY_PIN, DATA_OUT, DRIVE_8MA);
						m_totalProgram	= getTotalProgram();
						m_initialized	= true;
						qDebug() << "Initialized. Current programs: " << m_totalProgram;
						return(true);
					}
				}
			}
		}
		return(false);
	}
	else
	{
		// Enable Pin 11 for antenna switching
		GPIO_SetFunction(ANTENNA_RELAY_PIN, DATA_OUT, DRIVE_8MA);
		m_totalProgram = getTotalProgram();
	}

	m_initialized	= true;

#ifdef OUTPUT_DEBUG
	qDebug() << "Initialized. Current programs: " << m_totalProgram;
#endif

	return(true);
}

void cKeyStoneCOMM::close()
{
	if(m_serialPort)
	{
		if(m_serialPort->isOpen())
		{
			stopStream();
			m_serialPort->close();
		}
		delete m_serialPort;
		m_serialPort	= nullptr;
	}
	m_initialized	= false;
}

int16_t cKeyStoneCOMM::channelCount()
{
	return(m_totalProgram);
}

bool cKeyStoneCOMM::setVolume(int8_t volume)
{
	uint16_t		dwBytes;
	unsigned char	output[]	= {HEAD,0x01,0x22,0x01,0x00,0x01,0x00,END};
	unsigned char	input[8]	= {0};

	if((volume < 0) || (volume > 16))
		return(false);		// Volume out of range

	output[6]	= volume;

	if(volume == 0)
		if(m_hardMute)
			hardMute();

	if(!writeSerialBytes(output, 8, &dwBytes))
		return(false);

	if(dwBytes != 8)
		return(false);

	// need to check the return
	if(!readSerialBytes(input, &dwBytes))
		return(false);

	m_currentVolume	= volume;

	return(true);
}

bool cKeyStoneCOMM::startStream(STREAM_MODE mode, uint16_t channel)
{
	uint16_t		dwBytes;
	unsigned char	input[8]	= {0};
	unsigned char	output[]	= {HEAD,0x01,0x00,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x00,END};
	unsigned char	strChannel[4];

	if(mode == 0)
		if(m_hardMute)
			hardMute();

	if((mode < 0) || (mode > 1))
		return(false);

	output[6]	= mode;

	if(mode == 0)
	{
		//DAB GPIO 11 set to LOW for antenna switching
		GPIO_SetLevel(ANTENNA_RELAY_PIN, 0);
		output[10]	= (unsigned char)channel; // WARNING - This is not very safe, TODO
	}
	else
	{
		//FM GPIO 11 set to HIGH for antenna switching
		GPIO_SetLevel(ANTENNA_RELAY_PIN, 1);
		if(channel < 20000)
		{
			memcpy(&strChannel[0], &channel, sizeof(long));
			output[7]	= strChannel[3];
			output[8]	= strChannel[2];
			output[9]	= strChannel[1];
			output[10]	= strChannel[0];
		}
		else
			return(false);
	}

	if(!writeSerialBytes(output, 12, &dwBytes))
		return(false);

	if(dwBytes != 12)
		return(false);

	// need to check the return
	if(!readSerialBytes(input, &dwBytes))
		return(false);

	m_currentProgram	= channel;
	m_currentMode		= mode;

	return(true);
}

int16_t cKeyStoneCOMM::channel()
{
	return(m_currentProgram);
}

bool cKeyStoneCOMM::stopStream()
{
	uint16_t		dwBytes;
	unsigned char	output[]	= {HEAD,0x01,0x02,0x01,0x00,0x00,END};
	unsigned char	input[8]	= {0};

	if(m_hardMute)
		hardMute();

	if(!writeSerialBytes(output, 7, &dwBytes))
		return(false);

	if(dwBytes != 7)
		return(false);

	if(!readSerialBytes(input, &dwBytes))
		return(false);

	if(goodHeader(input, dwBytes))
	{
		if(input[2] == 0x01)
			return(true);
		else
			return(false);
	}
	else
		return(false);
}

bool cKeyStoneCOMM::nextStream()
{
	long	channel		= 0;
	char	currentMode	= playMode();

	if(currentMode == STREAM_MODE_DAB)
	{
		channel	= playIndex();
		if(channel < m_totalProgram-1)
			channel++;
		else
			channel	= 0;

		startStream(STREAM_MODE_DAB, channel);
		return(true);
	}

	if(currentMode == STREAM_MODE_FM)
	{
//HERWIG		FMSearch(1);
		return(true);

	}
	return(false);
}

bool cKeyStoneCOMM::prevStream()
{
	long	channel		= 0;
	char	currentMode	= playMode();

	if(currentMode == STREAM_MODE_DAB)
	{
		channel	= playIndex();
		if(channel > 0)
			channel--;
		else
			channel	= m_totalProgram-1;

		startStream(STREAM_MODE_DAB, channel);
		return(true);
	}

	if(currentMode == STREAM_MODE_FM)
	{
//HERWIG		FMSearch(0);
		return(true);

	}
	return(false);
}

cKeyStoneCOMM::STREAM_MODE cKeyStoneCOMM::playMode()
{
	uint16_t		dwBytes;
	unsigned char	input[8]	= {0};
	unsigned char	output[]	= {HEAD,0x01,0x11,0x01,0x00,0x00,END};

	if(!writeSerialBytes(output, 7, &dwBytes))
		return(STREAM_MODE_NONE);

	if(dwBytes != 7)
		return(STREAM_MODE_NONE);

	if(!readSerialBytes(input, &dwBytes))
		return(STREAM_MODE_NONE);

	if(!goodHeader(input, dwBytes))
		return(STREAM_MODE_NONE);
	else
	{
		if((input[1] == 0x01) && (input[2] == 0x11))
			return((STREAM_MODE)input[6]);
		else
			return(STREAM_MODE_NONE);
	}
}

int16_t cKeyStoneCOMM::playIndex()
{
	uint16_t		dwBytes;
	unsigned char	output[]	= {HEAD,0x01,0x12,0x01,0x00,0x00,END};
	unsigned char	input[11]	= {0};

	if(!writeSerialBytes(output, 7, &dwBytes))
		return(-1);

	if(!readSerialBytes(input, &dwBytes))
		return(-1);
	else
	{
		if(goodHeader(input, dwBytes))
		{
			long	channel = (0xFF & input[6]) << 24 | (0xFF & input[7]) << 16 | (0xFF & input[8]) << 8 | (0xFF & input[9]);
			return(channel);

		}
		else
			return(-1);
	}
}

int8_t cKeyStoneCOMM::volumePlus()
{
	char	currentVolume	= volume();
	if(currentVolume < 16)
		currentVolume++;

	if(setVolume(currentVolume))
		return(currentVolume);
	else
		return(-1);
}

int8_t cKeyStoneCOMM::volumeMinus()
{
	char	currentVolume	= volume();
	if(currentVolume > 0)
		currentVolume--;

	if(setVolume(currentVolume))
		return(currentVolume);
	else
		return(-1);
}

void cKeyStoneCOMM::volumeMute()
{
	char	currentVolume	= volume();
	if(currentVolume == 0)
		setVolume(m_mutePrevVolume);
	else
	{
		m_mutePrevVolume	= currentVolume;
		setVolume(0);
	}
}

int8_t cKeyStoneCOMM::volume()
{
	uint16_t		dwBytes;
	unsigned char	output[]	= {HEAD,0x01,0x23,0x01,0x00,0x00,END};
	unsigned char	input[8]	= {0};

	if(!writeSerialBytes(output, 7, &dwBytes))
		return(-1);

	if(dwBytes != 7)
		return(-1);

	if(!readSerialBytes(input, &dwBytes))
		return(-1);
	else
	{
		if(goodHeader(input, dwBytes))
		{
			m_currentVolume	= input[6];
			return(input[6]);
		}
		else
			return(-1);
	}
}

int8_t cKeyStoneCOMM::signalStrength()
{
	int		biterror;
	int8_t	signalStrength	= getSignalStrength(&biterror);
	if(signalStrength < 0)
		signalStrength	= 0;
	return(signalStrength);
}

QString cKeyStoneCOMM::programName()
{
	return(getProgramName(m_currentMode, m_currentProgram, 1));
}

QString cKeyStoneCOMM::programType()
{
	int8_t	programType	= getProgramType(m_currentMode, m_currentProgram);

	if(programType < 0 || programType > 31)
		return("");

	return(programTypeText[programType]);
}

QString cKeyStoneCOMM::programText()
{
	return(getProgramText());
}

bool cKeyStoneCOMM::hardMute()
{
	m_hardMute	= true;

#ifdef Q_OS_WIN
	if(!::EscapeCommFunction(m_serialPort->handle(), CLRRTS))
		return(false);
	if(!::EscapeCommFunction(m_serialPort->handle(), CLRDTR))
		return(false);
#elif defined Q_OS_LINUX
	int	status;
	ioctl(m_serialPort->handle, TIOCMGET, &status);
	status &= ~TIOCM_RTS;
	ioctl(m_serialPort->handle, TIOCMSET, &status);
#endif

	QThread::msleep(350);

	return(true);
}

bool cKeyStoneCOMM::hardUnmute()
{
	QThread::msleep(850);

#ifdef Q_OS_WIN
	if(!::EscapeCommFunction(m_serialPort->handle(), SETRTS))
		return(false);
	if(!::EscapeCommFunction(m_serialPort->handle(), CLRDTR))
		return(false);
#elif defined Q_OS_LINUX
	int	status;
	ioctl(m_serialPort->handle(), TIOCMGET, &status);
	status |= TIOCM_RTS;
	ioctl(m_serialPort->handle(), TIOCMSET, &status);
#endif

	m_hardMute	= false;

	return(true);
}

bool cKeyStoneCOMM::hardResetRadio()
{
#ifdef Q_OS_WIN
	if(EscapeCommFunction(m_serialPort->handle(), SETRTS))
	{
		EscapeCommFunction(m_serialPort->handle(), SETDTR);
		Sleep(100);
		if (EscapeCommFunction(m_serialPort->handle(), SETRTS))
		{
			EscapeCommFunction(m_serialPort->handle(), CLRDTR);
			QThread::msleep(1000);

			return(true);
		}
		else
			return(false);
	}
	else
		return(false);
#elif defined Q_OS_LINUX
	int status;
	ioctl(m_serialPort->handle(), TIOCMGET, &status);
	status |= TIOCM_DTR;                // set DTR pin low to reset module
	ioctl(m_serialPort->handle(), TIOCMSET, &status);
	usleep(100000);
	status &= ~TIOCM_DTR;               // set DTR pin HIGH to disable RESET
	ioctl(m_serialPort->handle(), TIOCMSET, &status);
	usleep(1000000);                  // delay 1 sec to wait for module to ready
#endif
}

bool cKeyStoneCOMM::isSysReady()
{
	uint16_t		dwBytes;

	unsigned char	input[8]	= {0};
	unsigned char	output[]	= {HEAD,0x00,0x00,0x01,0x00,0x00,END};

	if(m_serialPort && m_serialPort->isOpen())
	{
		if(!writeSerialBytes(output, 7, &dwBytes))
			return(false);

		if(!readSerialBytes(input, &dwBytes))
			return(false);

		if(dwBytes < 7)
			return(false);

		if(goodHeader(input, dwBytes))
		{
			if(input[2] == 0x01)
				return(true);
			else
				return(false);
		}
		else
			return(false);
	}
	else
		return(false);
}

bool cKeyStoneCOMM::readSerialBytes(unsigned char* buffer, uint16_t* bytesreadreturn)
{
	m_readData.clear();
	m_cntReadData			= 0;
	m_serialReadError		= false;

	while(m_serialPort->waitForReadyRead(100));

	if(m_serialReadError)
	{
		*bytesreadreturn	= 0;
		qDebug() << "error";
		return(false);
	}
	*bytesreadreturn		= m_readData.count();
	std::memcpy(buffer, m_readData.data(), *bytesreadreturn);

	return(true);
}

bool cKeyStoneCOMM::writeSerialBytes(unsigned char* buffer, uint16_t nNumberOfBytesToWrite, uint16_t* lpNumberOfBytesWritten)
{
	int		i;

	i	= m_serialPort->write(reinterpret_cast<char*>(buffer), nNumberOfBytesToWrite);
	if(i != nNumberOfBytesToWrite)
		return(false);

	*lpNumberOfBytesWritten	= i;
	return(true);
}

bool cKeyStoneCOMM::goodHeader(unsigned char* input, uint16_t dwBytes)
{
	if((input[0] == HEAD) && (input[dwBytes-1] == END))
		return(true);
	else
		return(false);
}

bool cKeyStoneCOMM::GPIO_SetLevel(char pin, char level)
{
	uint16_t		dwBytes;
	unsigned char	input[7]	= { 0 };
	unsigned char	output[]	= { HEAD,0x08,0x01,0x00,0x00,0x02,0x00,0x00,END };

	output[6]	= pin;
	output[7]	= level;

	if (!writeSerialBytes(output, 9, &dwBytes))
		return(false);

	if(dwBytes != 9)
		return(false);

	if(!readSerialBytes(input, &dwBytes))
		return(false);

	if(!goodHeader(input, dwBytes))
		return(false);
	else
	{
		if((input[1] == 0x00) && (input[2] == 0x01))
			return(true);
		else
			return(false);
	}
}

bool cKeyStoneCOMM::GPIO_GetLevel(char pin, char* level)
{
	uint16_t		dwBytes;
	unsigned char	input[8]	= { 0 };
	unsigned char	output[]	= { HEAD,0x08,0x02,0x00,0x00,0x01,0x00,END };

	output[6]	= pin;

	if(!writeSerialBytes(output, 9, &dwBytes))
		return(false);

	if(dwBytes != 9)
		return(false);

	if(!readSerialBytes(input, &dwBytes))
		return(false);

	if(!goodHeader(input, dwBytes))
		return(false);
	else
	{
		if((input[1] == 0x08) && (input[2] == 0x02))
		{
			*level = input[6];
			return(true);
		}
		else
			return(false);
	}
}

bool cKeyStoneCOMM::GPIO_SetFunction(char pin, char pinfunction, char drive)
{
	uint16_t		dwBytes;
	unsigned char	input[7]	= { 0 };
	unsigned char	output[]	= { HEAD,0x08,0x00,0x00,0x00,0x03,0x00,0x00,0x00,END };

	output[6]	= pin;
	output[7]	= pinfunction;
	output[8]	= drive;

	if(!writeSerialBytes(output, 10, &dwBytes))
		return(false);

	if(dwBytes != 10)
		return(false);

	if(!readSerialBytes(input, &dwBytes))
		return(false);

	if(!goodHeader(input, dwBytes))
		return(false);
	else
	{
		if((input[1] == 0x00) && (input[2] == 0x01))
		{
			return(true);
		}
		else
			return(false);
	}
}

int16_t cKeyStoneCOMM::getTotalProgram()
{
	uint16_t		dwBytes;
	unsigned char	output[]	= {HEAD,0x01,0x13,0x01,0x00,0x00,END};
	unsigned char	input[11]	= {0};

	if(!writeSerialBytes(output, 7, &dwBytes))
		return(0);

	if(!readSerialBytes(input, &dwBytes))
		return(0);
	else
	{
		if(goodHeader(input, dwBytes))
		{
			if(input[1] == 0x01)
			{
				uint16_t	total	= (0xFF & input[6]) << 24 | (0xFF & input[7]) << 16 | (0xFF & input[8]) << 8 |(0xFF & input[9]);
				m_totalProgram		= total;
				return(total);
			}
			else
				return(0);
		}
		else
			return(0);
	}
}

int8_t cKeyStoneCOMM::getSignalStrength(int* biterror)
{
	uint16_t		dwBytes;
	char			dabStrength	= 0;
	unsigned char	input[10]	= {0};
	unsigned char	output[]	= {HEAD,0x01,0x15,0x01,0x00,0x00,END};
	double			temp;

	if(!writeSerialBytes(output, 7, &dwBytes))
		return(-1);

	if(dwBytes != 7)
		return(-1);

	if(!readSerialBytes(input, &dwBytes))
		return(-1);

	if(dwBytes < 8)
		return(-1);

	if(goodHeader(input, dwBytes))
	{
		if(input[5] == 0x01)
		{
			*biterror	= 0;
			return(input[6]); //FM Strength
		}

		if(input[5] == 0x03)
		{
			if(input[6] > 0)
			{
				// calculate percentage and return bit error
				*biterror	= (0xFF & input[7]) << 8 |(0xFF & input[8]);
				temp		= 100.00/18.00;
				temp		= temp * input[6];
				dabStrength	= (char)temp;

				return(dabStrength);
			}
			else
				return(0);
		}

		return(-1);
	}
	else
		return(-1);
}

QString cKeyStoneCOMM::getProgramText()
{
	uint16_t		dwBytes;
	unsigned char	input[TEXT_BUFFER_LEN]	= {0};
	unsigned char	output[]				= {HEAD,0x01,0x2E,0x01,0x00,0x00,END};
	QString			programText;

	if(!writeSerialBytes(output, 7, &dwBytes))
		return(programText);

	if(dwBytes != 7)
		return(programText);

	if(!readSerialBytes(input, &dwBytes))
		return(programText);

	if(dwBytes < 8)
		return(programText);

	if(goodHeader(input, dwBytes))
	{
		if((input[1] == 0x01) && (input[2] == 0x2E))
		{
			int		textLen		= (0xFF & input[4]) << 8 |(0xFF & input[5]);

			if(textLen > TEXT_BUFFER_LEN-1)
				return(programText);	// buffer overrun

			int		unicodesize	= textLen/2;
			char	front;

			for(int i = 0;i < textLen;i += 2)  // WATCHOUT
			{
				front			= input[6+i];
				input[6+i]		= input[6+i+1];
				input[6+i+1]	= front;
			}
#ifdef WIN32
			programText	= QString::fromWCharArray((wchar_t *)&input[6], unicodesize-1).trimmed();

#ifdef LABVIEW
			int asciilen=WideCharToMultiByte(CP_ACP, 0, programText, unicodesize-1, NULL,0,NULL,NULL);
			char *asciitext = new char[asciilen+1];
			WideCharToMultiByte(CP_ACP, 0, programText, unicodesize-1, asciitext, asciilen, NULL, NULL);
			asciitext[asciilen]=0;
			memset((char*)programText,0, asciilen+1);
			memcpy((char*)programText, asciitext, asciilen);
			delete asciitext;
#endif

#endif

#ifdef __linux__
			unsigned char tempBuffer[TEXT_BUFFER_LEN*4]={0};

			for(int i=0; i< unicodesize;i++)
			{
				tempBuffer[i*4]=input[6+(i*2)];
				tempBuffer[(i*4)+1]=input[6+(i*2)+1];
				tempBuffer[(i*4)+2]=0;
				tempBuffer[(i*4)+3]=0;
			}

			programText	= QString::fromWCharArray((wchar_t *)&tempBuffer[0], unicodesize-1).trimmed();
#endif
			return(programText);

		}
		else if((input[1] == 0x00) && (input[2] == 0x02))
		{
			if(input[6] == 0x01)
				return(programText);
			else
				return(programText);
		}
		else
			return(programText);
	}
	else
		return(programText);
}

int8_t cKeyStoneCOMM::getProgramType(char mode, long dabIndex)
{
	uint16_t		dwBytes;
	unsigned char	input[9]		= {0};
	unsigned char	output[]		= {HEAD,0x01,0x2C,0x01,0x00,0x04,0xFF,0xFF,0xFF,0xFF,END};
	unsigned char	strChannel[4]	= {0};

	if((mode != STREAM_MODE_DAB) && (mode != STREAM_MODE_FM))
		return(-1);

	if(mode == STREAM_MODE_DAB)
	{
		memcpy(&strChannel[0], &dabIndex, sizeof(long));
		output[6]	= strChannel[3];
		output[7]	= strChannel[2];
		output[8]	= strChannel[1];
		output[9]	= strChannel[0];
	}

	if(!writeSerialBytes(output, 11, &dwBytes))
		return(-1);

	if(dwBytes != 11)
		return(-1);

	if(!readSerialBytes(input, &dwBytes))
		return(-1);

	if(dwBytes != 8)
		return(-1);

	if(goodHeader(input, dwBytes))
	{
		if((input[1] == 0x01) && (input[2] == 0x2C))
			return(input[6]);
		else
			return(-1);
	}
	else
		return(-1);
}

QString cKeyStoneCOMM::getProgramName(char mode, long dabIndex, char namemode)
{
	uint16_t		dwBytes;
	unsigned char	input[50]		= {0};
	unsigned char	output[]		= {HEAD,0x01,0x2D,0x01,0x00,0x05,0xFF,0xFF,0xFF,0xFF,0x01,END};
	unsigned char	strChannel[4]	= {0};
	QString			programName;

	if((mode != STREAM_MODE_DAB) && (mode != STREAM_MODE_FM))
		return(programName);

	if(mode == STREAM_MODE_DAB)
	{
		memcpy(&strChannel[0], &dabIndex, sizeof(long));
		output[6]	= strChannel[3];
		output[7]	= strChannel[2];
		output[8]	= strChannel[1];
		output[9]	= strChannel[0];
	}

	output[10]		= namemode;

	if(!writeSerialBytes(output, 12, &dwBytes))
		return(programName);

	if(dwBytes != 12)
		return(programName);

	if(!readSerialBytes(input, &dwBytes))
		return(programName);

	if(dwBytes < 8)
		return(programName);

	if(goodHeader(input, dwBytes))
	{
		if((input[1] == 0x01) && (input[2] == 0x2D))
		{
			int		textLen		= (int)input[5];

			if(textLen > 50)
				return(programName);		// prevent buffer overrun

			int		unicodesize	= textLen/2;
			char	front;

			for(int i = 0;i < textLen;i += 2)
			{
				front			= input[6+i];
				input[6+i]		= input[6+i+1];
				input[6+i+1]	= front;
			}

#ifdef WIN32
			programName	= QString::fromWCharArray((wchar_t*)&input[6], unicodesize-1).trimmed();

#ifdef LABVIEW
			int asciilen=WideCharToMultiByte(CP_ACP, 0, programName, unicodesize-1, NULL,0,NULL,NULL);
			char *asciitext = new char[asciilen+1];
			WideCharToMultiByte(CP_ACP, 0, programName, unicodesize-1, asciitext, asciilen, NULL, NULL);
			asciitext[asciilen]=0;
			memset((char*)programName,0, asciilen+1);
			memcpy((char*)programName, asciitext, asciilen);
			delete asciitext;
#endif

#endif

#ifdef __linux__
			unsigned char tempBuffer[TEXT_BUFFER_LEN*4]={0};

			for(int i=0; i< unicodesize;i++)
			{
				tempBuffer[i*4]=input[6+(i*2)];
				tempBuffer[(i*4)+1]=input[6+(i*2)+1];
				tempBuffer[(i*4)+2]=0;
				tempBuffer[(i*4)+3]=0;
			}

			programName	= QString::fromWCharArray((wchar_t *)&tempBuffer[0], unicodesize-1).trimmed();
#endif
			return(programName);

		}
		else
			return(programName);
	}
	else
		return(programName);
}

void cKeyStoneCOMM::serialReadReady()
{
	m_readData.append(m_serialPort->readAll());
}

//void cKeyStoneCOMM::handleTimeout()
//{
//}

void cKeyStoneCOMM::serialReadError(QSerialPort::SerialPortError error)
{
	if(error == QSerialPort::ReadError)
	{
		m_serialReadError	= true;
		m_serialPortError	= error;
#ifdef OUTPUT_DEBUG
		qDebug() << error;
#endif
	}
}


bool cKeyStoneCOMM::clearDatabase()
{
	uint16_t		dwBytes;
	unsigned char	input[8]	= {0};
	unsigned char	output[]	= {HEAD,0x00,0x01,0x01,0x00,0x01,0x01,END};

	if(!writeSerialBytes(output, 8, &dwBytes))
		return(false);

	// need to check the return
	if(!readSerialBytes(input, &dwBytes))
		return(false);
	else
	{
		if(goodHeader(input, dwBytes))
		{
			if(input[2] == 0x01)
			{
				QThread::msleep(1000);

//				while(!isSysReady())
//				{
//					QThread::msleep(1000);
//				}

				return(true);
			}
			else
				return(false);
		}
		else
			return(false);
	}

	return(true);
}

bool cKeyStoneCOMM::clearFM()
{
	return(true);
}

bool cKeyStoneCOMM::clearDAB()
{
	clearDatabase();

	return(true);
}
