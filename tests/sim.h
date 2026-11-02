#pragma once
// テスト用の小さなシミュレータ: 線分でできた部屋の中で LiDAR のスキャンを作り、オドメトリにずれを足す
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "mapper_gmapping/mapper.h"

namespace testsim {

struct Segment {
    double x0, y0, x1, y1;
};

// 10 m x 8 m の部屋 (-5..5, -4..4) に、箱 2 つと仕切り 1 つ
inline std::vector<Segment> room() {
    std::vector<Segment> w;
    auto box = [&w](double x0, double y0, double x1, double y1) {
        w.push_back({x0, y0, x1, y0});
        w.push_back({x1, y0, x1, y1});
        w.push_back({x1, y1, x0, y1});
        w.push_back({x0, y1, x0, y0});
    };
    box(-5.0, -4.0, 5.0, 4.0);
    box(-1.0, -0.8, 1.0, 0.8);    // 真ん中の箱 (ロボットはこの周りを回る)
    box(3.0, 2.5, 3.8, 3.3);      // 右上の小さな箱
    w.push_back({-5.0, 2.0, -3.5, 2.0});  // 左の仕切り
    return w;
}

inline double cast(const std::vector<Segment>& walls, double x, double y, double a, double max_range) {
    const double dx = std::cos(a), dy = std::sin(a);
    double best = std::numeric_limits<double>::infinity();
    for (const auto& s : walls) {
        const double ex = s.x1 - s.x0, ey = s.y1 - s.y0;
        const double den = dx * ey - dy * ex;
        if (std::fabs(den) < 1e-12) continue;
        const double t = ((s.x0 - x) * ey - (s.y0 - y) * ex) / den;
        const double u = ((s.x0 - x) * dy - (s.y0 - y) * dx) / den;
        if (t > 1e-9 && u >= 0.0 && u <= 1.0 && t < best) best = t;
    }
    return best <= max_range ? best : std::numeric_limits<double>::infinity();
}

struct LidarConfig {
    double angle_min = -2.35, angle_max = 2.35;
    int beams = 271;
    double range_min = 0.05, range_max = 8.0;
    double noise = 0.01;
    // 当たらなかったビームの値 (mobile_sim は range_max + 1)
    double no_hit = 9.0;
};

inline mapper_gmapping::Scan scan(const std::vector<Segment>& walls, const mapper_gmapping::Pose2D& pose, double stamp,
                                  const LidarConfig& cfg, std::mt19937& rng) {
    std::normal_distribution<double> noise(0.0, cfg.noise);
    mapper_gmapping::Scan s;
    s.stamp = stamp;
    s.angle_min = cfg.angle_min;
    s.angle_increment = (cfg.angle_max - cfg.angle_min) / (cfg.beams - 1);
    s.range_min = cfg.range_min;
    s.range_max = cfg.range_max;
    for (int i = 0; i < cfg.beams; ++i) {
        const double d = cast(walls, pose.x, pose.y, pose.theta + s.angle_min + s.angle_increment * i, cfg.range_max);
        s.ranges.push_back(std::isfinite(d) ? static_cast<float>(d + noise(rng)) : static_cast<float>(cfg.no_hit));
    }
    return s;
}

// 対向 2 輪のロボットを v, w で dt だけ動かす
inline mapper_gmapping::Pose2D move(const mapper_gmapping::Pose2D& p, double v, double w, double dt) {
    const double th = p.theta + 0.5 * w * dt;
    return {p.x + v * dt * std::cos(th), p.y + v * dt * std::sin(th), std::atan2(std::sin(p.theta + w * dt), std::cos(p.theta + w * dt))};
}

}  // namespace testsim
