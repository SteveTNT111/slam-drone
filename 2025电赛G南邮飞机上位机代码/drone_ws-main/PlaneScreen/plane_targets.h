#pragma once

#include <vector>
#include <mutex>
#include<QString>
#include <fstream>      // 用于 std::ifstream
#include <sstream>      // 用于 std::istringstream
#include <string>       // 用于 std::string 和 std::getline
#include <algorithm>
#include<QDir>
struct Target {
    double x;
    double y;
    QString name;
    bool operator==(const Target& other) const
    {
        return name == other.name;
    }
};

class SharedData {
public:
    static SharedData& getInstance() {
        static SharedData instance;
        return instance;
    }
    
    std::vector<Target>& getTargets() {
        return targets_;
    }
    Target& getChosenTarget()
    {
        return target_chosen_;
    }
    std::mutex& getMutex() {
        return mutex_;
    }
    void addTargetIfNew(const Target& newTarget) {
        std::lock_guard<std::mutex> lock(mutex_);

        // 使用迭代器遍历现有目标
        auto it = std::find(targets_.begin(), targets_.end(), newTarget);

        // 如果没有找到相似的目标，则添加到容器尾部
        if (it == targets_.end()) {
            targets_.push_back(newTarget);

        }
    }
    void saveTargetToFile(const Target& target) {
        std::lock_guard<std::mutex> lock(mutex_);

        std::ofstream file("../targets.txt", std::ios::app);  // 以追加模式打开文件
        if (!file.is_open()) {
            return;
        }
        if (target.name != "nothing")
        {
            file << target.x << " " << target.y << " " << target.name.toStdString() << std::endl;
        }
        file.close();
    }
    void LoadTargets()
    {
        qDebug() << "[DEBUG] LoadTargets() 开始执行";

        std::ifstream file("../targets.txt");
        if (!file.is_open()) {
            qDebug() << "[DEBUG] 无法打开文件: ../targets.txt";
            qDebug() << "[DEBUG] 当前工作目录:" << QDir::currentPath();
            return;
        }

        qDebug() << "[DEBUG] 成功打开文件: targets.txt";

        std::string line;
        int lineNumber = 0;
        int validTargets = 0;

        while (std::getline(file, line)) {
            lineNumber++;
            qDebug() << "[DEBUG] 读取第" << lineNumber << "行:" << QString::fromStdString(line);

            // 跳过空行
            if (line.empty()) {
                qDebug() << "[DEBUG] 跳过空行";
                continue;
            }

            std::istringstream iss(line);
            double x, y;
            std::string name;

            // 读取 x 坐标、y 坐标和名称
            if (iss >> x >> y >> name) {
                qDebug() << "[DEBUG] 解析成功 - x:" << x << ", y:" << y << ", name:" << QString::fromStdString(name);

                Target target;
                target.x = x;
                target.y = y;
                target.name = QString::fromStdString(name);

                // 添加到 targets_ 容器中
                size_t originalSize = targets_.size();
                addTargetIfNew(target);
                size_t newSize = targets_.size();

                if (newSize > originalSize) {
                    validTargets++;
                    qDebug() << "[DEBUG] 新目标已添加，当前目标总数:" << newSize;
                } else {
                    qDebug() << "[DEBUG] 目标已存在，未添加重复目标";
                }
            } else {
                qDebug() << "[DEBUG] 解析失败，行格式错误:" << QString::fromStdString(line);
            }
        }

        file.close();
        qDebug() << "[DEBUG] LoadTargets() 执行完成";
        qDebug() << "[DEBUG] 总共读取" << lineNumber << "行，有效目标" << validTargets << "个";
        qDebug() << "[DEBUG] 最终目标列表大小:" << targets_.size();
    }
private:
    SharedData()
    {
        target_chosen_.x = -1;
        target_chosen_.y = -1;
        target_chosen_.name = "nothing";
    };


    std::vector<Target> targets_;
    Target target_chosen_;
    std::mutex mutex_;
};

