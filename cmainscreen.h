#ifndef CMAINSCREEN_H
#define CMAINSCREEN_H


#include "cmonkeyboard.h"

#include <QWidget>
#include <QStandardItemModel>


namespace Ui {
class cMainScreen;
}

class cMainScreen : public QWidget
{
	Q_OBJECT

public:
	explicit cMainScreen(cMonkeyBoard* lpMonkeyBoard, QWidget *parent = nullptr);
	~cMainScreen();

private:
	Ui::cMainScreen*	ui;
	QStandardItemModel*	m_lpChannelListModel;

	cMonkeyBoard*		m_lpMonkeyBoard;

signals:
	void				settingsClicked();
	void				powerOffClicked();
};

#endif // CMAINSCREEN_H
