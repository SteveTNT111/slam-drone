#ifndef HOME_H
#define HOME_H

#include <QDialog>
#include <QTcpSocket>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include<QPushButton>
#include"blink.h"
#include<QTimer>
#include "plane_targets.h"
#include"targetInfo.h"
class home : public QDialog
{
    Q_OBJECT

public:
    explicit home(QWidget *parent = nullptr);
    ~home();

    private slots:
        void ReadData();
        void HandleTaskChosen();
        void HandleExitButton();
        void HandleFlash();
private:
    void parseJson(const QByteArray &jsonData);
    void createUI();
    void sendData();

    QJsonObject dataSend;

    QTcpSocket *socket;

    QPushButton *exitButton;
    QTimer *reconnectTimer;
    QTimer *taskIDTimer;
    QPushButton *taskButton1;
    QPushButton *taskButton2;
    QPushButton *launchButton;
    // 手动创建的控件
    QLabel *armLabel;
    QLabel *armValueLabel;
    QLabel *positionLabel;
    QLabel *positionLabelValue;
    QLabel *flyStateLabel;
    QLabel *flyStateValueLabel;
    QLabel *targetPointLabel;
    QLabel *targetPointValueLabel;
    QLabel *taskLabel;
    QLabel *taskValueLabel;
    // 布局
    QVBoxLayout *mainLayout;

    // 数据成员
    double currentX, currentY, currentZ,targetX, targetY, targetZ,currentYaw;
    Target receivedTarget;
    bool isArmedFlag;
    QString flyMode;
    int32_t taskID;
    QString flyState;
    int32_t flashID;

    enum COLOR{
        RED = 1,
        BLUE = 2,
        GREEN = 3
    };
};

#endif // HOME_H