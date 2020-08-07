#include "cmainwindow.h"
#include "ui_cmainwindow.h"

#include <QDebug>


cMainWindow::cMainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::cMainWindow),
	m_lpMonkeyBoard(nullptr),
	m_lpMainScreen(nullptr),
	m_lpSettingsScreen(nullptr)
{
	ui->setupUi(this);

#ifdef __linux__
	m_lpMonkeyBoard	= new cMonkeyBoard("/dev/ttyACM0", this);
#else
	m_lpMonkeyBoard	= new cMonkeyBoard("\\\\.\\COM4", this);
#endif

	if(!m_lpMonkeyBoard->isValid())
		return;

	m_lpMainScreen		= new cMainScreen(m_lpMonkeyBoard, this);
	ui->m_lpMainStack->addWidget(m_lpMainScreen);

	m_lpSettingsScreen	= new cSettingsScreen(this);
	ui->m_lpMainStack->addWidget(m_lpSettingsScreen);

	connect(m_lpSettingsScreen,	&cSettingsScreen::backClicked,	this,	&cMainWindow::onSettingsBack);

	connect(m_lpMainScreen,		&cMainScreen::settingsClicked,	this,	&cMainWindow::onSettings);
	connect(m_lpMainScreen,		&cMainScreen::powerOffClicked,	this,	&cMainWindow::onPowerOff);

	move(0, 0);
}

cMainWindow::~cMainWindow()
{
	if(m_lpMonkeyBoard)
		delete m_lpMonkeyBoard;

	if(m_lpSettingsScreen)
		delete m_lpSettingsScreen;

	if(m_lpMainScreen)
		delete m_lpMainScreen;
	delete ui;
}

void cMainWindow::onSettings()
{
	ui->m_lpMainStack->setCurrentWidget(m_lpSettingsScreen);
}

void cMainWindow::onSettingsBack()
{
	ui->m_lpMainStack->setCurrentWidget(m_lpMainScreen);
}

void cMainWindow::onPowerOff()
{
	QWidget::close();
}
