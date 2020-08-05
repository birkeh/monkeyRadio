#include "cmainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	cMainWindow w;

	w.setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
	w.show();

	return a.exec();
}
