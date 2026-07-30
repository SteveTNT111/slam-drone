#include "targetInfo.h"
#include <QMessageBox>
#include <QFont>

TargetInfo::TargetInfo(QWidget *parent)
    : QDialog(parent)
    , mainLayout(nullptr)
    , scrollArea(nullptr)
    , scrollContent(nullptr)
    , scrollLayout(nullptr)
{
    setupUI();
    loadTargets(); // 只加载一次数据
}

TargetInfo::~TargetInfo()
{
}

void TargetInfo::setupUI()
{
    setWindowTitle("目标信息");
    setModal(true); // 设置为模态对话框
    resize(600, 500); // 增大窗口尺寸

    // 主布局
    mainLayout = new QVBoxLayout(this);

    // 创建滚动区域
    scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    // 滚动内容容器
    scrollContent = new QWidget;
    scrollLayout = new QVBoxLayout(scrollContent);
    scrollLayout->setAlignment(Qt::AlignTop);

    scrollArea->setWidget(scrollContent);
    mainLayout->addWidget(scrollArea);

    // 添加关闭按钮
    QPushButton* closeButton = new QPushButton("关闭");
    closeButton->setFont(QFont("Arial", 12, QFont::Bold));
    closeButton->setMinimumHeight(40);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
    mainLayout->addWidget(closeButton);
}

void TargetInfo::loadTargets()
{
    SharedData& sharedData = SharedData::getInstance();
    sharedData.LoadTargets();
    // 读取一次目标数据
    std::vector<Target> targets;
    {
        std::lock_guard<std::mutex> lock(sharedData.getMutex());
        targets = sharedData.getTargets();
    }

    // 创建所有目标项
    for (size_t i = 0; i < targets.size(); ++i) {
        if (targets[i].name!="nothing") createTargetItem(targets[i], static_cast<int>(i));
    }
}

void TargetInfo::createTargetItem(const Target& target, int index)
{
    // 创建水平布局的目标项
    QWidget* itemWidget = new QWidget;
    QHBoxLayout* itemLayout = new QHBoxLayout(itemWidget);

    // 设置样式
    itemWidget->setStyleSheet("QWidget { border: 2px solid gray; margin: 3px; padding: 10px; background-color: #f0f0f0; }");

    // 设置大字体
    QFont labelFont("Arial", 14, QFont::Bold);
    QFont buttonFont("Arial", 12, QFont::Bold);

    // 名称标签
    QLabel* nameLabel = new QLabel(target.name);
    nameLabel->setMinimumWidth(120);
    nameLabel->setFont(labelFont);
    nameLabel->setStyleSheet(posLabelStyle);

    // X坐标标签
    QLabel* xLabel = new QLabel(QString("X: %1").arg(target.x, 0, 'f', 2));
    xLabel->setMinimumWidth(100);
    xLabel->setFont(labelFont);
    xLabel->setStyleSheet(posLabelStyle);

    // Y坐标标签
    QLabel* yLabel = new QLabel(QString("Y: %1").arg(target.y, 0, 'f', 2));
    yLabel->setMinimumWidth(100);
    yLabel->setFont(labelFont);
    yLabel->setStyleSheet(posLabelStyle);

    // 救援按钮
    QPushButton* rescueButton = new QPushButton("救援");
    rescueButton->setFixedSize(320, 120);
    rescueButton->setFont(buttonFont);
    rescueButton->setStyleSheet(buttonStyle);
    
    // 连接救援按钮信号
    connect(rescueButton, &QPushButton::clicked, [this, target,rescueButton]() {
        onRescueButtonClicked(target);
        rescueButton->setStyleSheet("QPushButton {background-color: rgb(0, 255, 0);font-size:40px;}");
    });
    
    // 添加到布局
    itemLayout->addWidget(nameLabel);
    itemLayout->addWidget(xLabel);
    itemLayout->addWidget(yLabel);
    itemLayout->addWidget(rescueButton);
    itemLayout->addStretch(); // 添加弹性空间
    
    // 添加到滚动布局
    scrollLayout->addWidget(itemWidget);
}

void TargetInfo::onRescueButtonClicked(const Target& target)
{
    SharedData& sharedData = SharedData::getInstance();
        std::lock_guard<std::mutex> lock(sharedData.getMutex());
        Target& chosenTarget = sharedData.getChosenTarget();

        // 设置选中的目标
        chosenTarget.x = target.x;
        chosenTarget.y = target.y;
        chosenTarget.name = target.name;
}