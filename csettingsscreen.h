#ifndef FORMCSETTINGSSCREEN_H
#define FORMCSETTINGSSCREEN_H

#include <QWidget>

namespace Ui {
class cSettingsScreen;
}

class cSettingsScreen : public QWidget
{
	Q_OBJECT

public:
	explicit cSettingsScreen(QWidget *parent = nullptr);
	~cSettingsScreen();

private:
	Ui::cSettingsScreen*	ui;

signals:
	void					backClicked();
};

#endif // FORMCSETTINGSSCREEN_H
