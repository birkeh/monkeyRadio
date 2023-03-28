#ifndef CMAINWINDOW_H
#define CMAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include "ckeystonecomm.h"



QT_BEGIN_NAMESPACE
namespace Ui { class cMainWindow; }
QT_END_NAMESPACE

class cMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	cMainWindow(QWidget *parent = nullptr);
	~cMainWindow();

private:
	Ui::cMainWindow*	ui;
	QTimer*				m_timerSignalStrength;
	cKeyStoneCOMM*		m_keystoneCOMM;

public slots:
	void				updateSignalStrength();
private slots:
	void on_m_channelDown_clicked();
	void on_m_channelUp_clicked();
	void on_m_volumeDown_clicked();
	void on_m_volumeUp_clicked();
	void on_m_clearDAB_clicked();
};

#endif // CMAINWINDOW_H
