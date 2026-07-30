//
// Created by yangj on 2025/8/15.
//

//#include "ChessboardPathPlanner.hpp"
#include "offboard/ChessboardPathPlanner.hpp"

#include <mutex>
#include <unordered_set>

ChessboardPathPlanner::ChessboardPathPlanner(int width, int height)
        : width_(width), height_(height), rng_(std::chrono::steady_clock::now().time_since_epoch().count()) {
    grid_.resize(height_, std::vector<int>(width_, 0));
    initializePheromone();
}

ChessboardPathPlanner::~ChessboardPathPlanner() {
    stopPlanning();
}

void ChessboardPathPlanner::initializePheromone() {
    pheromone_.resize(height_);
    for (int i = 0; i < height_; ++i) {
        pheromone_[i].resize(width_);
        for (int j = 0; j < width_; ++j) {
            pheromone_[i][j].resize(height_);
            for (int k = 0; k < height_; ++k) {
                pheromone_[i][j][k].resize(width_, 0.1);
            }
        }
    }
}

void ChessboardPathPlanner::setObstacles(const std::vector<std::string>& obstacles) {
    // 清空现有障碍物
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < width_; ++j) {
            grid_[i][j] = 0;
        }
    }

    // 设置新障碍物
    for (const auto& s : obstacles) {
        try {
            Point p = toInternal(s);
            if (isValid(p)) {
                grid_[p.y][p.x] = 1;
            }
        } catch (...) {
            std::cerr << "Invalid obstacle coordinate: " << s << std::endl;
        }
    }

    // 重新初始化信息素
    initializePheromone();
}

void ChessboardPathPlanner::setStart(const std::string& start) {
    try {
        start_ = toInternal(start);
        if (grid_[start_.y][start_.x] == 1) {
            throw std::runtime_error("Start is on obstacle.");
        }
        has_start_ = true;
    } catch (...) {
        std::cerr << "Invalid start coordinate: " << start << std::endl;
        has_start_ = false;
    }
}

std::vector<std::string> ChessboardPathPlanner::getPath() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    std::vector<std::string> result;
    for (const auto& p : best_path_) {
        result.push_back(toExternal(p));
    }
    return result;
}

void ChessboardPathPlanner::printPath() {
    auto path_str = getPath();
    if (path_str.empty()) {
        std::cout << "No path found.\n";
        return;
    }
    std::cout << "Path (" << path_str.size() - 1 << " steps): ";
    for (size_t i = 0; i < path_str.size(); ++i) {
        std::cout << path_str[i];
        if (i + 1 < path_str.size()) std::cout << " -> ";
    }
    std::cout << std::endl;
}

void ChessboardPathPlanner::startPlanning() {
    if (is_planning_ || !has_start_) {
        return;
    }

    if (!checkReachability()) {
        std::cerr << "存在无法到达的区域，请重新设置障碍物" << std::endl;
        return;
    }

    is_planning_ = true;
    planning_thread_ = std::thread(&ChessboardPathPlanner::planningLoop, this);
}

void ChessboardPathPlanner::stopPlanning() {
    is_planning_ = false;
    if (planning_thread_.joinable()) {
        planning_thread_.join();
    }
        // ✅ 等待迭代完成
    {
        std::unique_lock<std::mutex> lock(cv_mutex_);
        cv_.wait(lock, [this]() { return iteration_done_; });
    }

    if (planning_thread_.joinable()) {
        planning_thread_.join();
    }
}


bool ChessboardPathPlanner::checkReachability() const {
    std::unordered_set<long long> visited;
    std::queue<Point> queue;

    queue.push(start_);
    visited.insert(static_cast<long long>(start_.y) * width_ + start_.x);

    while (!queue.empty()) {
        Point current = queue.front();
        queue.pop();

        for (const auto& neighbor : getNeighbors(current)) {
            long long key = static_cast<long long>(neighbor.y) * width_ + neighbor.x;
            if (visited.find(key) == visited.end()) {
                visited.insert(key);
                queue.push(neighbor);
            }
        }
    }

    // 检查是否所有非障碍物格子都被访问
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < width_; ++j) {
            if (grid_[i][j] == 0) {
                long long key = static_cast<long long>(i) * width_ + j;
                if (visited.find(key) == visited.end()) {
                    return false;
                }
            }
        }
    }
    return true;
}

void ChessboardPathPlanner::planningLoop() {
    int iteration = 0;
    double best_cost = std::numeric_limits<double>::infinity();

    while (is_planning_) {
        std::vector<std::vector<Point>> paths;
        std::vector<double> costs;

        // 每只蚂蚁搜索
        for (int ant = 0; ant < ANT_COUNT && is_planning_; ++ant) {
            auto path = antSearch();
            double cost = calculateCost(path);
            paths.push_back(path);
            costs.push_back(cost);

            // 更新最佳路径
            if (cost < best_cost) {
                best_cost = cost;
                auto [repeats, turns] = countPathStats(path);

                {
                    std::lock_guard<std::mutex> lock(stats_mutex_);
                    best_path_ = path;
                    stats_.best_cost = cost;
                    stats_.path_length = static_cast<int>(path.size());
                    stats_.repeat_count = repeats;
                    stats_.turn_count = turns;
                }
            }
        }

        // 更新信息素
        updatePheromone(paths, costs);

        ++iteration;
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.iteration = iteration;
        }

        // ✅ 通知迭代完成
        {
            std::lock_guard<std::mutex> lock(cv_mutex_);
            iteration_done_ = true;
        }
        cv_.notify_one();

        // 控制搜索速度
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::vector<ChessboardPathPlanner::Point> ChessboardPathPlanner::getNeighbors(const Point& pos) const {
    std::vector<Point> neighbors;
    int dx[] = {0, 1, 0, -1};  // 右、下、左、上
    int dy[] = {1, 0, -1, 0};

    for (int i = 0; i < 4; ++i) {
        Point next(pos.x + dx[i], pos.y + dy[i]);
        if (isWalkable(next)) {
            neighbors.push_back(next);
        }
    }
    return neighbors;
}

int ChessboardPathPlanner::calculateDirectionChange(const Point* prev_pos, const Point& curr_pos, const Point& next_pos) const {
    if (!prev_pos) return 0;

    Point prev_dir(curr_pos.x - prev_pos->x, curr_pos.y - prev_pos->y);
    Point next_dir(next_pos.x - curr_pos.x, next_pos.y - curr_pos.y);

    if (prev_dir.x == next_dir.x && prev_dir.y == next_dir.y) {
        return 0;  // 方向不变
    } else if (prev_dir.x * next_dir.x + prev_dir.y * next_dir.y == -1) {
        return 1;  // 180度转向（回头）
    } else {
        return 1;  // 90度转向
    }
}

std::pair<int, int> ChessboardPathPlanner::countPathStats(const std::vector<Point>& path) const {
    if (path.empty()) return {0, 0};

    // 统计重复格子
    std::map<std::pair<int,int>, int> visit_count;
    for (const auto& pos : path) {
        visit_count[{pos.x, pos.y}]++;
    }

    int repeat_count = 0;
    for (const auto& pair : visit_count) {
        if (pair.second > 1) {
            repeat_count += pair.second - 1;
        }
    }

    // 统计转弯数
    int turn_count = 0;
    for (size_t i = 2; i < path.size(); ++i) {
        turn_count += calculateDirectionChange(&path[i-2], path[i-1], path[i]);
    }

    return {repeat_count, turn_count};
}

std::vector<ChessboardPathPlanner::Point> ChessboardPathPlanner::antSearch() {
    std::unordered_set<long long> unvisited;

    // 收集所有未访问的点
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < width_; ++j) {
            if (grid_[i][j] == 0 && !(i == start_.y && j == start_.x)) {
                unvisited.insert(static_cast<long long>(i) * width_ + j);
            }
        }
    }

    std::vector<Point> path = {start_};
    Point current = start_;
    Point prev_pos(-1, -1);  // 无效的前一位置

    while (!unvisited.empty()) {
        auto neighbors = getNeighbors(current);
        std::vector<Point> valid_neighbors;

        for (const auto& n : neighbors) {
            long long key = static_cast<long long>(n.y) * width_ + n.x;
            if (unvisited.find(key) != unvisited.end() || n == start_) {
                valid_neighbors.push_back(n);
            }
        }

        if (valid_neighbors.empty()) {
            // 需要回溯找到最近的未访问点
            double min_dist = std::numeric_limits<double>::infinity();
            Point target(-1, -1);

            for (long long key : unvisited) {
                int y = static_cast<int>(key / width_);
                int x = static_cast<int>(key % width_);
                Point unv(x, y);
                double dist = std::abs(unv.x - current.x) + std::abs(unv.y - current.y);
                if (dist < min_dist) {
                    min_dist = dist;
                    target = unv;
                }
            }

            if (target.x >= 0) {
                auto sub_path = bfsPath(current, target);
                if (!sub_path.empty()) {
                    path.insert(path.end(), sub_path.begin() + 1, sub_path.end());
                    current = target;
                    unvisited.erase(static_cast<long long>(target.y) * width_ + target.x);
                    prev_pos = sub_path.size() > 1 ? sub_path[sub_path.size()-2] : current;
                }
            } else {
                break;
            }
        } else {
            // 根据信息素和启发信息选择下一个位置
            std::vector<double> probabilities;

            for (const auto& next_pos : valid_neighbors) {
                // 信息素浓度
                double pheromone_level = pheromone_[current.y][current.x][next_pos.y][next_pos.x];

                // 启发信息
                double heuristic = 10.0;
                long long next_key = static_cast<long long>(next_pos.y) * width_ + next_pos.x;
                if (unvisited.find(next_key) == unvisited.end()) {
                    heuristic = 0.1;
                }

                // 方向改变惩罚
                Point* prev_ptr = (prev_pos.x >= 0) ? &prev_pos : nullptr;
                int dir_change = calculateDirectionChange(prev_ptr, current, next_pos);
                heuristic /= (1.0 + dir_change);

                double prob = std::pow(pheromone_level, ALPHA) * std::pow(heuristic, BETA);
                probabilities.push_back(prob);
            }

            // 轮盘赌选择
            double total_prob = 0.0;
            for (double p : probabilities) total_prob += p;

            Point next_pos;
            if (total_prob > 0) {
                std::uniform_real_distribution<double> dist(0.0, total_prob);
                double rand_val = dist(rng_);
                double cumulative = 0.0;
                size_t selected = 0;

                for (size_t i = 0; i < probabilities.size(); ++i) {
                    cumulative += probabilities[i];
                    if (rand_val <= cumulative) {
                        selected = i;
                        break;
                    }
                }
                next_pos = valid_neighbors[selected];
            } else {
                std::uniform_int_distribution<int> dist(0, static_cast<int>(valid_neighbors.size()) - 1);
                next_pos = valid_neighbors[dist(rng_)];
            }

            path.push_back(next_pos);
            prev_pos = current;
            current = next_pos;

            long long next_key = static_cast<long long>(next_pos.y) * width_ + next_pos.x;
            unvisited.erase(next_key);
        }
    }

    // 返回备降点
    auto return_path = findReturnPath(current);
    if (!return_path.empty()) {
        path.insert(path.end(), return_path.begin() + 1, return_path.end());
    }

    return path;
}

std::vector<ChessboardPathPlanner::Point> ChessboardPathPlanner::findReturnPath(const Point& current) const {
    auto landing_points = getLandingPoints();
    std::vector<std::vector<Point>> return_paths;

    for (const auto& landing : landing_points) {
        auto path = bfsPath(current, landing);
        if (!path.empty()) {
            return_paths.push_back(path);
        }
    }

    if (return_paths.empty()) return {};

    // 选择最短路径
    auto min_path = *std::min_element(return_paths.begin(), return_paths.end(),
                                      [](const std::vector<Point>& a, const std::vector<Point>& b) {
                                          return a.size() < b.size();
                                      });

    return min_path;
}

std::vector<ChessboardPathPlanner::Point> ChessboardPathPlanner::getLandingPoints() const {
    std::vector<Point> landing_points;

    // A7B1 (对应内部坐标 (6, 0)) 和 A8B1 (对应内部坐标 (7, 0))
    if (isWalkable(Point(6, 0))) landing_points.push_back(Point(6, 0));
    // if (isWalkable(Point(7, 0))) landing_points.push_back(Point(7, 0));

    // A9B2 (对应内部坐标 (8, 1)) 和 A9B3 (对应内部坐标 (8, 2))
    // if (isWalkable(Point(8, 1))) landing_points.push_back(Point(8, 1));
    if (isWalkable(Point(8, 2))) landing_points.push_back(Point(8, 2));

    return landing_points;
}

std::vector<ChessboardPathPlanner::Point> ChessboardPathPlanner::bfsPath(const Point& start, const Point& end) const {
    std::queue<std::pair<Point, std::vector<Point>>> queue;
    std::unordered_set<long long> visited;

    queue.push({start, {start}});
    visited.insert(static_cast<long long>(start.y) * width_ + start.x);

    while (!queue.empty()) {
        auto [current, path] = queue.front();
        queue.pop();

        if (current == end) {
            return path;
        }

        for (const auto& neighbor : getNeighbors(current)) {
            long long key = static_cast<long long>(neighbor.y) * width_ + neighbor.x;
            if (visited.find(key) == visited.end()) {
                visited.insert(key);
                std::vector<Point> new_path = path;
                new_path.push_back(neighbor);
                queue.push({neighbor, new_path});
            }
        }
    }

    return {};  // 无路径
}

void ChessboardPathPlanner::updatePheromone(const std::vector<std::vector<Point>>& paths, const std::vector<double>& costs) {
    // 挥发
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < width_; ++j) {
            for (int k = 0; k < height_; ++k) {
                for (int l = 0; l < width_; ++l) {
                    pheromone_[i][j][k][l] *= (1.0 - RHO);
                }
            }
        }
    }

    // 添加新信息素
    for (size_t i = 0; i < paths.size(); ++i) {
        if (costs[i] <= 0) continue;

        double delta = Q / costs[i];
        const auto& path = paths[i];

        for (size_t j = 0; j + 1 < path.size(); ++j) {
            const Point& curr = path[j];
            const Point& next = path[j + 1];
            pheromone_[curr.y][curr.x][next.y][next.x] += delta;
            pheromone_[next.y][next.x][curr.y][curr.x] += delta;
        }
    }
}

double ChessboardPathPlanner::calculateCost(const std::vector<Point>& path) const {
    if (path.empty()) return std::numeric_limits<double>::infinity();

    auto [repeat_count, turn_count] = countPathStats(path);

    // 代价 = 重复格子数*10 + 转弯数*2
    return static_cast<double>(repeat_count * 10 + turn_count * 2);
}

ChessboardPathPlanner::Point ChessboardPathPlanner::toInternal(const std::string& coord) const {
    size_t b_pos = coord.find('B');
    if (coord.empty() || coord[0] != 'A' || b_pos == std::string::npos)
        throw std::invalid_argument("Invalid format");

    std::string a_part = coord.substr(1, b_pos - 1);
    std::string b_part = coord.substr(b_pos + 1);

    int x = std::stoi(a_part) - 1;
    int y = std::stoi(b_part) - 1;

    if (x < 0 || x >= width_ || y < 0 || y >= height_)
        throw std::invalid_argument("Out of bounds");
    return Point(x, y);
}

std::string ChessboardPathPlanner::toExternal(const Point& p) const {
    return "A" + std::to_string(p.x + 1) + "B" + std::to_string(p.y + 1);
}

bool ChessboardPathPlanner::isValid(const Point& p) const {
    return p.x >= 0 && p.x < width_ && p.y >= 0 && p.y < height_;
}

bool ChessboardPathPlanner::isWalkable(const Point& p) const {
    return isValid(p) && grid_[p.y][p.x] == 0;
}