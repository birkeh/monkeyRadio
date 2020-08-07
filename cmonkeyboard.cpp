#include "cmonkeyboard.h"
#include "KeyStoneCOMM.h"

#include <QDebug>


cMonkeyBoard::cMonkeyBoard(const QString &port, QObject* object) :
	QObject(object),
	m_isValid(false),
	m_port(port)
{
	if(!openRadioPort(port))
	{
		m_port	= "";
		return;
	}

	getDABChannelList();
}

cMonkeyBoard::~cMonkeyBoard()
{
	if(m_isValid)
		CloseRadioPort();
}

bool cMonkeyBoard::isValid()
{
	return(m_isValid);
}

QStringList cMonkeyBoard::dabChannelList()
{
	return(m_DABChannelList);
}

bool cMonkeyBoard::openRadioPort(const QString& port)
{
	if((m_isValid = OpenRadioPort(port.toLocal8Bit().data(), true)) == true)
		return(true);

	return(false);
}

void cMonkeyBoard::getDABChannelList()
{
	m_DABChannelList.clear();

	long	totalDABProgram = GetTotalProgram();
	wchar_t	programNameTmp[300];

	for(long i = 0;i < totalDABProgram;i++)
	{
		if(GetProgramName(0, i, 1, programNameTmp))
		{
			QString	tmp	= QString::fromWCharArray(programNameTmp, wcslen(programNameTmp));

			m_DABChannelList.append(tmp);
		}
	}
	m_DABChannelList.sort(Qt::CaseInsensitive);
}
