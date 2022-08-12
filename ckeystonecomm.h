#ifndef CKEYSTONECOMM_H
#define CKEYSTONECOMM_H


#include <QObject>

#include <QSerialPort>
#include <QSerialPortInfo>


class cKeyStoneCOMM : public QObject
{
	Q_OBJECT

public:
	enum STREAM_MODE
	{
		STREAM_MODE_NONE	= -1,
		STREAM_MODE_DAB		= 0,
		STREAM_MODE_FM		= 1,
	};

	explicit cKeyStoneCOMM(const QString& commPort, QObject *parent = nullptr);
	~cKeyStoneCOMM();

	bool							open(bool _hardMute = false);
	void							close();

	int16_t							channelCount();
	STREAM_MODE						playMode();
	int16_t							playIndex();
	bool							setVolume(int8_t volume);
	bool							startStream(STREAM_MODE mode, uint16_t channel);
	bool							stopStream();
	bool							nextStream();
	bool							prevStream();
	int8_t							volumePlus();
	int8_t							volumeMinus();
	void							volumeMute();
	int8_t							volume();
	int8_t							signalStrength();
signals:

private:
	QString							m_commPort;
	QSerialPort*					m_serialPort;
	bool							m_serialReadError;
	QSerialPort::SerialPortError	m_serialPortError;
	bool							m_hardMute;

	bool							m_initialized;

	QByteArray						m_readData;
	int								m_cntReadData;

	uint16_t						m_totalProgram;
	int8_t							m_currentVolume;
	int8_t							m_mutePrevVolume;

	bool							hardMute();
	bool							hardUnmute();
	bool							hardResetRadio();

	bool							readSerialBytes (unsigned char* buffer, uint16_t* bytesreadreturn);
	bool							writeSerialBytes(unsigned char* buffer, uint16_t nNumberOfBytesToWrite, uint16_t* lpNumberOfBytesWritten);
	bool							goodHeader(unsigned char* input, uint16_t dwBytes);

	bool							isSysReady();
	bool							GPIO_SetLevel(char pin, char level);
	bool							GPIO_GetLevel(char pin, char* level);
	bool							GPIO_SetFunction(char pin, char pinfunction, char drive);

	int16_t							getTotalProgram();
	int8_t							getSignalStrength(int* biterror);
private slots:
	void							serialReadReady();
	void							serialReadError(QSerialPort::SerialPortError error);
};

#endif // CKEYSTONECOMM_H
