#ifndef CMAINWINDOW_H
#define CMAINWINDOW_H


#include "cmainscreen.h"
#include "csettingsscreen.h"

#include "cmonkeyboard.h"

#include <QMainWindow>


namespace Ui {
class cMainWindow;
}

class cMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit cMainWindow(QWidget *parent = 0);
	~cMainWindow();

private:
	Ui::cMainWindow*				ui;

	cMonkeyBoard*					m_lpMonkeyBoard;

	cMainScreen*					m_lpMainScreen;
	cSettingsScreen*				m_lpSettingsScreen;

private slots:
	void							onSettings();
	void							onSettingsBack();
	void							onPowerOff();
};

#endif // CMAINWINDOW_H
