// mapper_gmapping_core のテスト (issue #4)。
// tests/sim.h の部屋をロボットが 1 周半し、ずれのあるオドメトリと LiDAR のスキャンから地図と姿勢を推定する
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iterator>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mapper_gmapping/mapper.h"
#include "mapper_gmapping/version.h"
#include "sim.h"

using mapper_gmapping::Mapper;
using mapper_gmapping::OccupancyGrid;
using mapper_gmapping::Params;
using mapper_gmapping::Pose2D;

static int failures = 0;

#define CHECK(cond)                                                                    \
    do {                                                                               \
        if (!(cond)) {                                                                 \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                                \
        }                                                                              \
    } while (0)

static double dist(const Pose2D& a, const Pose2D& b) { return std::hypot(a.x - b.x, a.y - b.y); }
static double angle_diff(double a, double b) { return std::fabs(std::atan2(std::sin(a - b), std::cos(a - b))); }

// 部屋の中を回る経路を、決まった手順で走らせる。各スキャンの (真の姿勢, オドメトリ, スキャン) を f に渡す
struct Run {
    std::vector<Pose2D> truth;
    Pose2D odom_end;
};

static Run drive(const std::function<void(const Pose2D&, const Pose2D&, const mapper_gmapping::Scan&)>& f,
                 double loops) {
    const auto walls = testsim::room();
    const testsim::LidarConfig lidar;
    std::mt19937 rng(42);
    std::normal_distribution<double> v_noise(0.0, 0.02), w_noise(0.0, 0.02);
    // 角を回る四角の経路。x 方向 5.5 m、y 方向 4.3 m
    const std::vector<std::pair<double, double>> corners = {{-3.0, -2.5}, {2.5, -2.5}, {2.5, 1.8}, {-3.0, 1.8}};
    const double dt = 0.02, scan_period = 0.1, v = 0.5, w = 0.5;
    Pose2D truth{-3.0, -2.5, 0.0};
    Pose2D odom = truth;
    double t = 0.0, next_scan = 0.0;
    Run run;

    auto step = [&](double cv, double cw) {
        truth = testsim::move(truth, cv, cw, dt);
        // オドメトリ: 並進 2 % 長く、回転 3 % 大きく測るずれと、ばらつき
        odom = testsim::move(odom, cv * 1.02 + (cv != 0.0 ? v_noise(rng) : 0.0) * 0.1,
                             cw * 1.03 + (cw != 0.0 ? w_noise(rng) : 0.0) * 0.1, dt);
        t += dt;
        if (t >= next_scan) {
            next_scan += scan_period;
            run.truth.push_back(truth);
            f(truth, odom, testsim::scan(walls, truth, t, lidar, rng));
        }
    };

    const int legs = static_cast<int>(std::round(loops * 4));
    for (int leg = 0; leg < legs; ++leg) {
        const auto& to = corners[static_cast<size_t>((leg + 1) % 4)];
        // 角へ向かって進む (向きのずれは比例制御で直す)。近づかなくなったら止める
        double remaining = std::hypot(to.first - truth.x, to.second - truth.y);
        while (remaining > v * dt) {
            const double bearing = std::atan2(to.second - truth.y, to.first - truth.x);
            step(v, 2.0 * std::atan2(std::sin(bearing - truth.theta), std::cos(bearing - truth.theta)));
            const double r = std::hypot(to.first - truth.x, to.second - truth.y);
            if (r >= remaining) break;
            remaining = r;
        }
        // 次の角の方向へ、その場で左に回る
        const auto& next = corners[static_cast<size_t>((leg + 2) % 4)];
        const double target = std::atan2(next.second - to.second, next.first - to.first);
        for (int i = 0; i < 1000 && angle_diff(truth.theta, target) > w * dt; ++i) step(0.0, w);
    }
    run.odom_end = odom;
    return run;
}

static Params test_params() {
    Params p;
    p.particles = 30;
    p.linearUpdate = 0.5;
    p.angularUpdate = 0.25;
    p.seed = 1;
    return p;
}

static void test_slam() {
    Mapper mapper(test_params());
    Pose2D last_truth;
    int scans = 0;
    const Run run = drive(
        [&](const Pose2D& truth, const Pose2D& odom, const mapper_gmapping::Scan& scan) {
            mapper.add_scan(scan, odom);
            last_truth = truth;
            ++scans;
            if (std::getenv("CORE_TEST_PROGRESS") && scans % 50 == 0) {
                std::printf("scan %d updates %d traj %zu\n", scans, mapper.updates(), mapper.trajectory().size());
                std::fflush(stdout);
            }
        },
        1.5);

    const Pose2D est = mapper.pose();
    const double slam_err = dist(est, last_truth);
    const double odom_err = dist(run.odom_end, last_truth);
    std::printf("scans=%d updates=%d slam_err=%.3f m (%.3f rad) odom_err=%.3f m (%.3f rad)\n", scans,
                mapper.updates(), slam_err, angle_diff(est.theta, last_truth.theta), odom_err,
                angle_diff(run.odom_end.theta, last_truth.theta));
    CHECK(mapper.initialized());
    CHECK(mapper.updates() > 20);
    CHECK(slam_err < 0.2);
    CHECK(angle_diff(est.theta, last_truth.theta) < 0.05);
    // GMapping の向きは周回で ±π を越えて積もるので、返す前に (-π, π] に収める (1.5 周するとここで分かる)
    CHECK(std::fabs(est.theta) <= M_PI + 1e-9);
    for (const auto& p : mapper.particles()) CHECK(std::fabs(p.theta) <= M_PI + 1e-9);
    CHECK(slam_err < odom_err);
    CHECK(mapper.particles().size() == 30);
    CHECK(mapper.trajectory().size() == static_cast<size_t>(mapper.updates()));

    // 地図と真の部屋を比べる (地図の座標系は最初のオドメトリ = 真の初期姿勢)
    const OccupancyGrid map = mapper.map();
    CHECK(map.width > 0 && map.height > 0);
    CHECK(std::fabs(map.resolution - 0.05) < 1e-12);
    auto occupied_near = [&map](double x, double y) {
        for (double dx = -0.1; dx <= 0.1001; dx += 0.05)
            for (double dy = -0.1; dy <= 0.1001; dy += 0.05)
                if (map.at_world(x + dx, y + dy) == 100) return true;
        return false;
    };
    int wall_points = 0, wall_hits = 0;
    for (const auto& s : testsim::room()) {
        const double len = std::hypot(s.x1 - s.x0, s.y1 - s.y0);
        for (double u = 0.0; u <= len; u += 0.05) {
            ++wall_points;
            if (occupied_near(s.x0 + (s.x1 - s.x0) * u / len, s.y0 + (s.y1 - s.y0) * u / len)) ++wall_hits;
        }
    }
    int path_points = 0, path_free = 0;
    for (size_t i = 0; i < run.truth.size(); i += 5) {
        ++path_points;
        if (map.at_world(run.truth[i].x, run.truth[i].y) == 0) ++path_free;
    }
    // 壁でないのに占有になっているセル (真の壁から 0.15 m より遠い占有セル)
    int occupied = 0, stray = 0;
    const auto walls = testsim::room();
    for (int y = 0; y < map.height; ++y) {
        for (int x = 0; x < map.width; ++x) {
            if (map.at(x, y) != 100) continue;
            ++occupied;
            const double wx = map.origin.x + (x + 0.5) * map.resolution;
            const double wy = map.origin.y + (y + 0.5) * map.resolution;
            double nearest = 1e9;
            for (const auto& s : walls) {
                const double ex = s.x1 - s.x0, ey = s.y1 - s.y0;
                double u = ((wx - s.x0) * ex + (wy - s.y0) * ey) / (ex * ex + ey * ey);
                u = std::min(1.0, std::max(0.0, u));
                nearest = std::min(nearest, std::hypot(wx - (s.x0 + u * ex), wy - (s.y0 + u * ey)));
            }
            if (nearest > 0.15) ++stray;
        }
    }
    std::printf("map %dx%d: walls %d/%d occupied, path %d/%d free, stray occupied %d/%d\n", map.width,
                map.height, wall_hits, wall_points, path_free, path_points, stray, occupied);
    CHECK(wall_hits >= 0.9 * wall_points);
    CHECK(path_free >= 0.95 * path_points);
    CHECK(stray <= 0.05 * occupied);

    // map_server の形式で保存する
    const std::string base = "core_test_map";
    mapper_gmapping::save_map(map, base);
    std::ifstream yaml(base + ".yaml");
    std::string text((std::istreambuf_iterator<char>(yaml)), std::istreambuf_iterator<char>());
    CHECK(text.find("image: core_test_map.png\n") != std::string::npos);
    CHECK(text.find("resolution: 0.05\n") != std::string::npos);
    CHECK(text.find("origin: [") != std::string::npos);
    std::ifstream png(base + ".png", std::ios::binary);
    std::string bytes((std::istreambuf_iterator<char>(png)), std::istreambuf_iterator<char>());
    CHECK(bytes.size() > static_cast<size_t>(map.width) * map.height);
    CHECK(bytes.compare(1, 3, "PNG") == 0);
}

static void test_png() {
    // 2x2 の画像のバイト列が、Python の zlib で作ったものと一致する (zlib の stored ブロック、CRC、Adler-32)
    const auto png = mapper_gmapping::encode_png_gray8(2, 2, {0, 254, 205, 0});
    const std::vector<uint8_t> expected = {
        0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52,
        0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x00, 0x57, 0xDD, 0x52,
        0xF8, 0x00, 0x00, 0x00, 0x11, 0x49, 0x44, 0x41, 0x54, 0x78, 0x01, 0x01, 0x06, 0x00, 0xF9, 0xFF,
        0x00, 0x00, 0xFE, 0x00, 0xCD, 0x00, 0x05, 0x98, 0x01, 0xCC, 0xD7, 0xDE, 0xB4, 0x6C, 0x00, 0x00,
        0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82};
    CHECK(png == expected);

    // 地図の画素: 下の行が画像の最後の行になる
    OccupancyGrid g;
    g.width = 2;
    g.height = 2;
    g.resolution = 1.0;
    g.data = {100, 0, -1, 0};  // (0,0)=占有 (1,0)=空き (0,1)=不明 (1,1)=空き
    const auto px = mapper_gmapping::map_to_pixels(g);
    CHECK((px == std::vector<uint8_t>{205, 254, 0, 254}));
    CHECK(g.at_world(0.5, 0.5) == 100);
    CHECK(g.at_world(1.5, 1.5) == 0);
    CHECK(g.at_world(-0.5, 0.5) == -1);
}

static void test_lifecycle() {
    const auto walls = testsim::room();
    std::mt19937 rng(1);
    const auto scan = testsim::scan(walls, Pose2D{0.0, -2.5, 0.0}, 0.0, testsim::LidarConfig(), rng);

    { Mapper m; CHECK(!m.initialized()); CHECK(m.map().width == 0); }
    {
        // 最初のスキャンの直後に壊す (GMapping そのままだと二重解放になる)
        Mapper m(test_params());
        CHECK(m.add_scan(scan, Pose2D{0.0, -2.5, 0.0}));
        CHECK(m.updates() == 1);
        CHECK(dist(m.pose(), Pose2D{0.0, -2.5, 0.0}) < 1e-9);
    }
    {
        Mapper m(test_params());
        m.add_scan(scan, Pose2D{});
        m.reset();
        CHECK(!m.initialized());
        CHECK(m.updates() == 0);
        CHECK(m.add_scan(scan, Pose2D{}));
        // ビームの数が変わったら例外
        auto other = scan;
        other.ranges.pop_back();
        bool threw = false;
        try {
            m.add_scan(other, Pose2D{});
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
    }
    {
        // 空のスキャンは例外
        Mapper m;
        bool threw = false;
        try {
            m.add_scan(mapper_gmapping::Scan{}, Pose2D{});
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
        CHECK(!m.initialized());
    }
}

// 引数 --quick なら、時間のかかる SLAM のテストを飛ばす (サニタイザ付きのビルドで使う)
int main(int argc, char** argv) {
    const bool quick = argc > 1 && std::string(argv[1]) == "--quick";
    CHECK(!mapper_gmapping::version().empty());
    test_png();
    test_lifecycle();
    if (!quick) test_slam();
    if (failures) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::puts("ok");
    return 0;
}
