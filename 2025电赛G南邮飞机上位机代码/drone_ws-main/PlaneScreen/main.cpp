#include <QCoreApplication>
#include<QApplication>
#include <QDebug>
#include"home.h"
#include <QApplication>
#include "home.h" // 假设你有 home 类

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    home homeWindow;
    homeWindow.setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
    homeWindow.show();
    return a.exec();
}