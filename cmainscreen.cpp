#include "cmainscreen.h"
#include "ui_cmainscreen.h"


cMainScreen::cMainScreen(cMonkeyBoard* lpMonkeyBoard, QWidget *parent) :
	QWidget(parent),
	ui(new Ui::cMainScreen),
	m_lpChannelListModel(nullptr),
	m_lpMonkeyBoard(lpMonkeyBoard)
{
	ui->setupUi(this);

	m_lpChannelListModel	= new QStandardItemModel(0, 1);
	ui->m_lpChannelList->setModel(m_lpChannelListModel);

	connect(ui->m_lpSettings,	&QPushButton::clicked,	this,	&cMainScreen::settingsClicked);
	connect(ui->m_lpPowerOff,	&QPushButton::clicked,	this,	&cMainScreen::powerOffClicked);

	QStringList	dabChannelList	= m_lpMonkeyBoard->dabChannelList();

	for(int x = 0;x < dabChannelList.count();x++)
		m_lpChannelListModel->appendRow(new QStandardItem(dabChannelList[x]));
}

cMainScreen::~cMainScreen()
{
	delete ui;
}
