#include "home.h"
QString labelStyle = "QLabel { font-size: 66px; font-weight: bold; }";
QString triggeredButtonStyle = "QPushButton {background-color: rgb(0, 255, 0);font-size:40px;}";
QString commonButtonStyle = "QPushButton {background-color: rgb(255, 255, 255);font-size:40px;}";
home::home(QWidget *parent)
    : QDialog(parent), currentX(0.0), currentY(0.0), currentZ(0.0),currentYaw(0.0), isArmedFlag(false), flyMode("UNKNOWN"),flashID(0)
{
    createUI();
    setWindowTitle("无人机状态监控");
    dataSend["task"] = 1;
    dataSend["launch"] = false;
    dataSend["sx"] = 0.0;
    dataSend["sy"] = 0.0;
    dataSend["sn"] = "nothing";

    socket = new QTcpSocket(this);
    reconnectTimer = new QTimer(this);
    reconnectTimer->setInterval(500); // 每0.5秒重试一次
    taskIDTimer = new QTimer(this);
    taskIDTimer->setInterval(500);
    connect(socket, &QTcpSocket::connected, this, [this]{
        qDebug() << "Connected to server";
        reconnectTimer->stop(); // 连接上了就停止重连
    });
    connect(socket, &QTcpSocket::disconnected, this, [this]{
        // 断开后再次尝试重连
        if (!reconnectTimer->isActive())
            reconnectTimer->start();
    });
    connect(socket, &QTcpSocket::readyRead, this, &home::ReadData);

    connect(reconnectTimer, &QTimer::timeout, this, [this]{
        if (socket->state() == QAbstractSocket::UnconnectedState) {
            socket->abort(); // 清理旧连接
            socket->connectToHost("127.0.0.1", 8000);
        }
    });
    connect(taskIDTimer, &QTimer::timeout, this, [this]
    {  //qDebug() << "id timer";
        SharedData& data = SharedData::getInstance();
        if (taskID == 1)
        {
            taskButton1->setStyleSheet(triggeredButtonStyle);
            taskButton2->setStyleSheet(commonButtonStyle);
        }
        if (taskID == 2)
        {
            taskButton2->setStyleSheet(triggeredButtonStyle);
            taskButton1->setStyleSheet(commonButtonStyle);
        }
            Target& target = data.getChosenTarget();
            dataSend["sx"] = target.x;
            dataSend["sy"] = target.y;
            dataSend["sn"] = target.name;
            sendData();
    });
    socket->connectToHost("127.0.0.1", 8000);
    reconnectTimer->start();
    taskIDTimer->start();
    // 退出按钮
    connect(exitButton, &QPushButton::clicked, this, &home::HandleExitButton);
    connect(taskButton1,&QPushButton::clicked,this,&home::HandleTaskChosen);
    connect(taskButton2,&QPushButton::clicked,this,&home::HandleTaskChosen);
    connect(launchButton,&QPushButton::clicked,this,[this]
    {
       dataSend["launch"] = true;
    });
}

void home::createUI()
{
    // 创建主布局
    mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(10, 10, 10, 10);
    mainLayout->setSpacing(15);  // 增大间距
    mainLayout->setAlignment(Qt::AlignTop);

    // ================= 添加退出按钮（右上角） =================
    QHBoxLayout *topLayout = new QHBoxLayout();
    launchButton = new QPushButton("启动",this);
    launchButton->setFixedSize(320, 120);  // 固定按钮大小
    launchButton->setStyleSheet("font-size: 40px;");  // 设置按钮字体
    topLayout->addWidget(launchButton);
    topLayout->addStretch();

    exitButton = new QPushButton("全屏", this);
    exitButton->setFixedSize(320, 120);  // 固定按钮大小
    exitButton->setStyleSheet("font-size: 40px;");  // 设置按钮字体
    topLayout->addWidget(exitButton);
    mainLayout->addLayout(topLayout);  // 将按钮布局添加到主布局

    // 创建武装状态行
    QHBoxLayout *armLayout = new QHBoxLayout();
    armLabel = new QLabel("是否已解锁:", this);
    armValueLabel = new QLabel("否", this);
    armLabel->setStyleSheet(labelStyle);
    armValueLabel->setStyleSheet(labelStyle);
    armLayout->addWidget(armLabel);
    armLayout->addWidget(armValueLabel);
    mainLayout->addLayout(armLayout);

    // 创建位置状态行
    QHBoxLayout *positionLayout = new QHBoxLayout();
    positionLabel = new QLabel("当前位置:", this);
    positionLabelValue = new QLabel("(0.0, 0.0, 0.0, 0.0)", this);
    positionLabel->setStyleSheet(labelStyle);
    positionLabelValue->setStyleSheet(labelStyle);
    positionLayout->addWidget(positionLabel);
    positionLayout->addWidget(positionLabelValue);
    mainLayout->addLayout(positionLayout);

    // 创建飞行模式行
    QHBoxLayout *flyStateLayout = new QHBoxLayout();
    flyStateLabel = new QLabel("飞行模式:", this);
    flyStateValueLabel = new QLabel("UNKNOWN", this);
    flyStateLabel->setStyleSheet(labelStyle);
    flyStateValueLabel->setStyleSheet(labelStyle);
    flyStateLayout->addWidget(flyStateLabel);
    flyStateLayout->addWidget(flyStateValueLabel);
    mainLayout->addLayout(flyStateLayout);

    // 创建目标点行
    QHBoxLayout *targetPointLayout = new QHBoxLayout();
    targetPointLabel = new QLabel("目标点:", this);
    targetPointValueLabel = new QLabel("未收到", this);
    targetPointLabel->setStyleSheet(labelStyle);
    targetPointValueLabel->setStyleSheet(labelStyle);
    targetPointLayout->addWidget(targetPointLabel);
    targetPointLayout->addWidget(targetPointValueLabel);
    mainLayout->addLayout(targetPointLayout);
    // 创建任务状态行
    QHBoxLayout *taskLayout = new QHBoxLayout();
    taskLabel = new QLabel("任务:", this);
    taskValueLabel = new QLabel("未收到", this);
    taskLabel->setStyleSheet(labelStyle);
    taskValueLabel->setStyleSheet(labelStyle);
    taskLayout->addWidget(taskLabel);
    taskLayout->addWidget(taskValueLabel);
    mainLayout->addLayout(taskLayout);
    mainLayout->addStretch();

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    taskButton1 = new QPushButton("任务1", this);
    taskButton1->setFixedSize(320, 120);  // 固定按钮大小
    taskButton1->setStyleSheet(commonButtonStyle);  // 设置按钮字体
    taskButton2 = new QPushButton("任务2", this);
    taskButton2->setFixedSize(320, 120);  // 固定按钮大小
    taskButton2->setStyleSheet(commonButtonStyle);  // 设置按钮字体
    buttonLayout->addWidget(taskButton1);
    buttonLayout->addWidget(taskButton2);
    mainLayout->addLayout(buttonLayout);


    // 设置布局
    setLayout(mainLayout);
}

home::~home()
{

}

void home::parseJson(const QByteArray &jsonData)
{
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(jsonData, &parseError);
    SharedData& currentTargets = SharedData::getInstance();
    auto& soughtTargt = SharedData::getInstance();
    if (parseError.error != QJsonParseError::NoError) {
        qWarning() << "JSON parse error:" << parseError.errorString();
        return;
    }

    QJsonObject obj = doc.object();
    currentX = obj["cx"].toDouble();
    currentY = obj["cy"].toDouble();
    currentZ = obj["cz"].toDouble();
    currentYaw = obj["cyaw"].toDouble();
    targetX = obj["tx"].toDouble();
    targetY = obj["ty"].toDouble();
    targetZ = obj["tz"].toDouble();
    isArmedFlag = obj["armed"].toBool();
    flyMode = obj["mode"].toString();
    taskID = obj["task_id"].toInt();
    flyState = obj["state"].toString();
    flashID = obj["flash_id"].toInt();
    receivedTarget.x = obj["sx"].toDouble();
    receivedTarget.y = obj["sy"].toDouble();
    receivedTarget.name = obj["sn"].toString();
    currentTargets.addTargetIfNew(receivedTarget);
    currentTargets.saveTargetToFile(receivedTarget);

    // 更新UI显示
    armValueLabel->setText(isArmedFlag ? "是" : "否");
    positionLabelValue->setText(QString("(%1, %2, %3, %4)").arg(currentX,0,'f',2)
                                                            .arg(currentY,0,'f',2)
                                                            .arg(currentZ,0,'f',2)
                                                            .arg(currentYaw,0,'f',2));
    targetPointValueLabel->setText(QString("(%1, %2, %3)").arg(targetX,0,'f',2)
                                                                .arg(targetY,0,'f',2)
                                                                .arg(targetZ,0,'f',2));
    flyStateValueLabel->setText(flyMode);
    taskValueLabel->setText(flyState);
    HandleFlash();
    
}

void home::ReadData()
{
    while (socket->canReadLine()) {
        QByteArray data = socket->readLine().trimmed();
        parseJson(data);
    }
}
void home::HandleTaskChosen()
{
    QObject* send_obj = sender();
    if (send_obj == taskButton1)
    {
        taskValueLabel->setText("钻环");
        dataSend["task"] = 1;
        sendData();
    }
    if (send_obj == taskButton2)
    {
        taskValueLabel->setText("绕杆");
        dataSend["task"] = 2;
        sendData();
    }
}
void home::sendData()
{
    QJsonDocument doc(dataSend);
    QByteArray jsonData = doc.toJson(QJsonDocument::Compact);
    jsonData.append("\n");
    if (socket->state() == QAbstractSocket::ConnectedState)   socket->write(jsonData);
}

void home::HandleExitButton() {
    // 判断当前窗口是否处于全屏状态
    if (this->windowState() & Qt::WindowFullScreen) {
        // 若已全屏，则关闭界面
        this->close();
        exitButton->setText("全屏");
    } else {
        // 若未全屏，则切换为全屏模式
        exitButton->setText("退出");
        this->showFullScreen();
    }
}
void home::HandleFlash()
{
     if(flashID)
     {
         // QColor color;
         // switch (flashID)
         // {
         // case RED:
         //     color = QColor(Qt::red);
         //     break;
         // case BLUE:
         //     color = QColor(Qt::blue);
         //     break;
         // case GREEN:
         //     color = QColor(Qt::green);
         // default:
         //     break;
         // }
         Blink* b = Blink::getInstance(flashID*1000);
         b->show();
         flashID = 0; // 重置闪烁ID
     }
}