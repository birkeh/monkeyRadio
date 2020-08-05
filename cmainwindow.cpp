#include "cmainwindow.h"
#include "ui_cmainwindow.h"


cMainWindow::cMainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::cMainWindow)
{
	ui->setupUi(this);

	connect(ui->m_lpExit,	&QPushButton::clicked,	this,	&cMainWindow::onExit);

	move(0, 0);
}

cMainWindow::~cMainWindow()
{
	delete ui;
}

void cMainWindow::onExit()
{
	QWidget::close();
}
