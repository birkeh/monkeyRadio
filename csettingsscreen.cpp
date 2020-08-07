#include "csettingsscreen.h"
#include "ui_csettingsscreen.h"


cSettingsScreen::cSettingsScreen(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::cSettingsScreen)
{
	ui->setupUi(this);

	connect(ui->m_lpBack,	&QPushButton::clicked,	this,	&cSettingsScreen::backClicked);
}

cSettingsScreen::~cSettingsScreen()
{
	delete ui;
}
