#include "cmainwindow.h"
#include "ui_cmainwindow.h"


cMainWindow::cMainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::cMainWindow),
	m_timerSignalStrength(nullptr),
	m_keystoneCOMM(nullptr)
{
	ui->setupUi(this);

	m_keystoneCOMM	= new cKeyStoneCOMM("COM7", this);

	m_timerSignalStrength	= new QTimer(this);
	connect(m_timerSignalStrength, &QTimer::timeout, this, &cMainWindow::updateSignalStrength);
	m_timerSignalStrength->start(1000);

	m_keystoneCOMM->open();
	m_keystoneCOMM->startStream(cKeyStoneCOMM::STREAM_MODE_DAB, 2);
}

cMainWindow::~cMainWindow()
{
	if(m_timerSignalStrength)
		delete m_timerSignalStrength;

	if(m_keystoneCOMM)
		delete m_keystoneCOMM;

	delete ui;
}

void cMainWindow::updateSignalStrength()
{
	int8_t	signalStrength	= m_keystoneCOMM->signalStrength();
	ui->m_signalStrength->setText(QString("%1").arg(signalStrength));
}
