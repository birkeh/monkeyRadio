#ifndef CKEYSTONECOMM_H
#define CKEYSTONECOMM_H


#include <QObject>

#include <QSerialPort>
#include <QSerialPortInfo>


#define TEXT_BUFFER_LEN 300

#define MSC_DATA_LENGTH			(100 * 1024)
#define MAX_SEGMENT_SIZE		8191
#define MAX_IMAGE_SIZE			100 * 1024	// bytes, ETSI TS 101 499
#define MAX_NAME_LENGTH			100
#define EXPECTED_IMAGE_LENGTH	1000


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
	int16_t							channel();
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
	QString							programName();
	QString							programType();
	QString							programText();

	bool							clearFM();
	bool							clearDAB();
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
	uint16_t						m_currentProgram;
	STREAM_MODE						m_currentMode;
	int8_t							m_currentVolume;
	int8_t							m_mutePrevVolume;

	uint16_t						m_mainTransportID;
	bool							m_startCollectData;
	uint8_t							m_MSCData[100 * 1024];
	uint32_t						m_MSCDataIndex;

	unsigned char					m_imageData[MAX_IMAGE_SIZE];
	uint32_t						m_imageDataIndex;
	char							m_imageFileName[MAX_NAME_LENGTH];

	bool							hardMute();
	bool							hardUnmute();
	bool							hardResetRadio();

	int16_t							getTotalProgram();
	int8_t							getSignalStrength(int* biterror);

	QString							getProgramText();
	int8_t							getProgramType(char mode, long dabIndex);
	QString							getProgramName(char mode, long dabIndex, char namemode);

	bool							clearDatabase();

	bool							readSerialBytes (unsigned char* buffer, uint16_t* bytesreadreturn);
	bool							writeSerialBytes(unsigned char* buffer, uint16_t nNumberOfBytesToWrite, uint16_t* lpNumberOfBytesWritten);
	bool							goodHeader(unsigned char* input, uint16_t dwBytes);

	bool							isSysReady();
	bool							GPIO_SetLevel(char pin, char level);
	bool							GPIO_GetLevel(char pin, char* level);
	bool							GPIO_SetFunction(char pin, char pinfunction, char drive);
private slots:
	void							serialReadReady();
	void							serialReadError(QSerialPort::SerialPortError error);
};

#endif // CKEYSTONECOMM_H

/*
	long			commVersion(void);
	bool			hardResetRadio();
	bool			isSysReady();
	char			getPlayStatus();
	long			getPreset(char mode, char presetindex);
	bool			setPreset(char mode, char presetindex, unsigned long channel);
	bool			FMSearch(char direction);
	bool			DABAutoSearch(unsigned char startindex, unsigned char endindex);
	bool			DABAutoSearchNoClear(unsigned char startindex, unsigned char endindex);
	bool			getEnsembleName(long dabIndex, char namemode, wchar_t * programName);
	int				getDataRate();
	bool			setStereoMode(char mode);
	char			getFrequency();
	char			getStereoMode();
	char			getStereo();
	bool			clearDatabase();
	bool			setBBEEQ(char BBEOn, char EQMode, char BBELo, char BBEHi, char BBECFreq, char BBEMachFreq, char BBEMachGain, char BBEMachQ, char BBESurr, char BBEMp, char BBEHpF, char BBEHiMode );
	bool			getBBEEQ(char *BBEOn, char *EQMode, char *BBELo, char *BBEHi, char *BBECFreq, char *BBEMachFreq, char *BBEMachGain, char *BBEMachQ, char *BBESurr, char *BBEMp, char *BBEHpF, char *BBEHiMode);
	bool			setHeadroom(char headroom);
	char			getHeadroom();
	int16			getApplicationType(long dabIndex);
	bool			getApplicationData();
	int16			setApplicationType(uint16 appType1);
	bool			getProgramInfo(long dabIndex, unsigned char * ServiceComponentID, uint32 * ServiceID, uint16 * EnsembleID);
	//BOOL MotQuery(void);
	void			getImage(wchar_t *ImageFileName);
	//void MotReset(MotMode enMode);
	char			getDABSignalQuality();
	char			getServCompType(long dabIndex);

	// RTC Clock Date
	bool			syncRTC(bool sync);
	bool			getRTC(unsigned char *sec, unsigned char *min, unsigned char *hour, unsigned char *day, unsigned char *month, unsigned char* year);
	bool			enableSyncClock();
	bool			disableSyncClock();
	bool			getClockStatus();

	int				getSamplingRate();

	bool			enableI2S(bool enable);
	bool			GPIO_SetFunction(char pin, char pinfunction, char drive);
	bool			GPIO_SetLevel(char pin, char level);
	bool			GPIO_GetLevel(char pin, char* level);

	// Local functions
	void			hardMute();
	void			hardUnMute();
	bool			readSerialBytes (HANDLE serialhandle, unsigned char *buffer, uint16 noofbytestoread, uint16 *bytesreadreturn, uint16 exitFD);
	bool			writeSerialBytes(HANDLE serialhandle, LPCVOID lpBuffer, uint16 nNumberOfBytesToWrite, uint16* lpNumberOfBytesWritten, LPOVERLAPPED lpOverlapped);
	bool			goodHeader(unsigned char* input, uint16 dwBytes);

	bool			parseMSCData();
*/
