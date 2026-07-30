#pragma once

#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include "plane_targets.h"

class TargetInfo : public QDialog
{
    Q_OBJECT

public:
    explicit TargetInfo(QWidget *parent = nullptr);
    ~TargetInfo();

private:
    void onRescueButtonClicked(const Target& target);
    void setupUI();
    void loadTargets();
    void createTargetItem(const Target& target, int index);

    QVBoxLayout* mainLayout;
    QScrollArea* scrollArea;
    QWidget* scrollContent;
    QVBoxLayout* scrollLayout;

    QString posLabelStyle = "QLabel { font-size: 66px; font-weight: bold; }";
    QString buttonStyle = "QPushButton {background-color: rgb(255, 255, 255);font-size:40px;}";
};