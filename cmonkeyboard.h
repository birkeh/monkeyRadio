#ifndef CMONKEYBOARD_H
#define CMONKEYBOARD_H


#include <QObject>
#include <QString>


class cMonkeyBoard : public QObject
{
	Q_OBJECT

public:
	cMonkeyBoard(const QString &port, QObject* object = nullptr);
	~cMonkeyBoard();

	bool		isValid();
	QStringList	dabChannelList();

private:
	bool		m_isValid;
	QString		m_port;
	QStringList	m_DABChannelList;

	bool		openRadioPort(const QString& port);
	void		getDABChannelList();
};

#endif // CMONKEYBOARD_H
