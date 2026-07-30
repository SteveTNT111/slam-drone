#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <string>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <random>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <condition_variable>

class ChessboardPathPlanner {
public:
    ChessboardPathPlanner(int width, int height);
    ~ChessboardPathPlanner();

    void setObstacles(const std::vector<std::string>& obstacles);
    void setStart(const std::string& start);
    std::vector<std::string> getPath();
    void printPath();

    // 新增接口
    void startPlanning();  // 开始持续规划
    void stopPlanning();   // 停止规划

    bool isPlanning() const { return is_planning_; }

    // 获取规划统计信息
    struct PlanningStats {
        int iteration = 0;
        int path_length = 0;
        double best_cost = std::numeric_limits<double>::infinity();
        int repeat_count = 0;
        int turn_count = 0;
    };
    PlanningStats getStats() const { return stats_; }

private:
    struct Point {
        int x, y;
        Point(int x = 0, int y = 0) : x(x), y(y) {}
        bool operator<(const Point& other) const {
            if (y != other.y) return y < other.y;
            return x < other.x;
        }
        bool operator==(const Point& other) const {
            return x == other.x && y == other.y;
        }
        bool operator!=(const Point& other) const {
            return !(*this == other);
        }
    };



    // 在类里加上
    std::condition_variable cv_;
    std::mutex cv_mutex_;
    bool iteration_done_ = false;


    int width_, height_;
    std::vector<std::vector<int>> grid_;  // 0:空地, 1:障碍物
    Point start_;
    bool has_start_ = false;
    std::vector<Point> path_;
    std::vector<Point> best_path_;

    // 蚁群算法参数
    static constexpr int ANT_COUNT = 20;
    static constexpr double ALPHA = 1.0;    // 信息素重要程度
    static constexpr double BETA = 2.0;     // 启发函数重要程度
    static constexpr double RHO = 0.1;      // 信息素挥发系数
    static constexpr double Q = 100.0;      // 信息素强度

    // 四维信息素矩阵: pheromone[from_x][from_y][to_x][to_y]
    std::vector<std::vector<std::vector<std::vector<double>>>> pheromone_;

    // 线程控制
    std::atomic<bool> is_planning_{false};
    std::thread planning_thread_;
    mutable PlanningStats stats_;
    mutable std::mutex stats_mutex_;

    // 随机数生成器
    std::mt19937 rng_;

    // 辅助方法
    Point toInternal(const std::string& coord) const;
    std::string toExternal(const Point& p) const;
    bool isValid(const Point& p) const;
    bool isWalkable(const Point& p) const;

    // 蚁群算法相关方法
    std::vector<Point> getNeighbors(const Point& pos) const;
    int calculateDirectionChange(const Point* prev_pos, const Point& curr_pos, const Point& next_pos) const;
    std::pair<int, int> countPathStats(const std::vector<Point>& path) const;
    std::vector<Point> antSearch();
    std::vector<Point> bfsPath(const Point& start, const Point& end) const;
    void updatePheromone(const std::vector<std::vector<Point>>& paths, const std::vector<double>& costs);
    double calculateCost(const std::vector<Point>& path) const;
    void planningLoop();
    void initializePheromone();
    bool checkReachability() const;

    // 备降点相关
    std::vector<Point> getLandingPoints() const;
    std::vector<Point> findReturnPath(const Point& current) const;
};