#include "cmainwindow.h"
#include "ui_cmainwindow.h"


#define REFRESH_RATE	1000


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
	m_timerSignalStrength->start(REFRESH_RATE);

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
	ui->m_sender->setText(m_keystoneCOMM->programName());
	ui->m_type->setText(m_keystoneCOMM->programType());
	ui->m_text->setText(m_keystoneCOMM->programText());
	int16_t	channel			= m_keystoneCOMM->channel();
	ui->m_channel->setText(QString("%1").arg(channel));
	int8_t	volume			= m_keystoneCOMM->volume();
	ui->m_volume->setText(QString("%1").arg(volume));
}

void cMainWindow::on_m_channelDown_clicked()
{
	m_keystoneCOMM->prevStream();
}


void cMainWindow::on_m_channelUp_clicked()
{
	m_keystoneCOMM->nextStream();
}

void cMainWindow::on_m_volumeDown_clicked()
{
	m_keystoneCOMM->volumeMinus();
}


void cMainWindow::on_m_volumeUp_clicked()
{
	m_keystoneCOMM->volumePlus();
}

void cMainWindow::on_m_clearDAB_clicked()
{
	m_keystoneCOMM->clearDAB();
}
