#pragma once

// GMapping (OpenJUIZ/openslam_gmapping) を包む、juiz に依存しない SLAM の本体 (issue #4)。
//
//   mapper_gmapping::Mapper mapper(params);
//   mapper.add_scan(scan, odom);          // スキャンと、その時刻のオドメトリの姿勢
//   mapper.pose();                        // 推定した姿勢 (最良の粒子)
//   mapper.map();                         // 占有格子
//   mapper_gmapping::save_map(mapper.map(), "maps/room");   // room.yaml と room.png
//
// パラメータの名前と既定値は ROS の slam_gmapping に合わせている (調整の知見をそのまま使えるように)。

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mapper_gmapping {

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// 平面の LiDAR の 1 回分の走査 (juiz_std.sensor.LaserScan と同じ意味)
struct Scan {
    double stamp = 0.0;  // [s]
    double angle_min = 0.0;
    double angle_increment = 0.0;
    double range_min = 0.0;
    double range_max = 0.0;
    std::vector<float> ranges;  // range_min〜range_max の外の値・NaN・inf は「当たらなかった」とみなす
};

struct Params {
    // 粒子フィルタ
    int particles = 30;
    double resampleThreshold = 0.5;
    // 地図の初期の範囲と解像度 [m]。範囲はロボットが外に出ると広がる
    double xmin = -10.0, ymin = -10.0, xmax = 10.0, ymax = 10.0;
    double delta = 0.05;
    // 占有とみなす割合 (地図を作るとき)
    double occ_thresh = 0.25;
    // スキャンマッチング。maxUrange・maxRange が 0 以下なら、最初のスキャンの range_max から決める
    double maxUrange = 0.0;
    double maxRange = 0.0;
    double sigma = 0.05;
    int kernelSize = 1;
    double lstep = 0.05;
    double astep = 0.05;
    int iterations = 5;
    double lsigma = 0.075;
    double ogain = 3.0;
    int lskip = 0;
    double minimumScore = 0.0;
    // 運動モデル (オドメトリの誤差)
    double srr = 0.1, srt = 0.2, str = 0.1, stt = 0.2;
    // 更新する移動量 [m]・[rad] と時間 [s] (負なら時間では更新しない)
    double linearUpdate = 1.0;
    double angularUpdate = 0.5;
    double temporalUpdate = -1.0;
    // 尤度のサンプリング
    double llsamplerange = 0.01, llsamplestep = 0.01;
    double lasamplerange = 0.005, lasamplestep = 0.005;
    // ロボットに対する LiDAR の取り付け位置
    Pose2D laser_pose;
    // 乱数の種 (0 なら設定しない)
    unsigned long seed = 0;
    // GMapping の出力を標準エラーに出す
    bool verbose = false;
};

// 占有格子。data は行優先で、(0, 0) が左下 (origin の位置)、x が右、y が上。
// 値は -1 = 不明、0 = 空き、100 = 占有 (ROS の nav_msgs/OccupancyGrid と同じ)
struct OccupancyGrid {
    int width = 0;
    int height = 0;
    double resolution = 0.0;
    Pose2D origin;  // 左下の画素の左下の角の世界座標
    std::vector<int8_t> data;

    int8_t at(int x, int y) const { return data[static_cast<size_t>(y) * width + x]; }
    // 世界座標 (wx, wy) の値。範囲外は -1
    int8_t at_world(double wx, double wy) const;
};

class Mapper {
public:
    explicit Mapper(const Params& params = Params());
    ~Mapper();
    Mapper(const Mapper&) = delete;
    Mapper& operator=(const Mapper&) = delete;

    // スキャンを足す。odom はそのスキャンの時刻のオドメトリの姿勢。
    // 最初のスキャンで、ビームの数と角度から GMapping を初期化する。
    // 粒子を更新したら true (移動量が linearUpdate・angularUpdate に満たなければ false)
    bool add_scan(const Scan& scan, const Pose2D& odom);

    // 最良の粒子の姿勢 (地図の座標系)。初期化前はオドメトリをそのまま返す
    Pose2D pose() const;
    // 地図の座標系からオドメトリの座標系への変換 (pose = map_to_odom ∘ odom)
    Pose2D map_to_odom() const;
    // 全粒子の姿勢
    std::vector<Pose2D> particles() const;
    // 最良の粒子の軌跡 (古い順)
    std::vector<Pose2D> trajectory() const;
    // 最良の粒子の軌跡から地図を作る。重いので、呼ぶ側で間隔を空ける
    OccupancyGrid map() const;

    bool initialized() const;
    // 粒子を更新した回数
    int updates() const;
    const Params& params() const;
    // 地図と粒子を捨てて、最初のスキャンを待つ状態に戻す
    void reset();
    // ロボットに対する LiDAR の取り付け位置を変える。GMapping は初期化のときにしか読まないので、
    // 値が変わったら reset() してやり直す。変わったら true
    bool set_laser_pose(const Pose2D& laser_pose);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// 地図を ROS map_server の形式で書く (<path>.yaml と <path>.png)。
// png は 8 bit のグレースケールで、占有 = 0、空き = 254、不明 = 205。
// yaml の image は png のファイル名 (yaml からの相対)
void save_map(const OccupancyGrid& map, const std::string& path);

// 8 bit のグレースケールの PNG を作る (圧縮しない)。pixels は上の行から
std::vector<uint8_t> encode_png_gray8(int width, int height, const std::vector<uint8_t>& pixels);

// 地図を画像の画素にする (上の行から。占有 = 0、空き = 254、不明 = 205)
std::vector<uint8_t> map_to_pixels(const OccupancyGrid& map);

}  // namespace mapper_gmapping
