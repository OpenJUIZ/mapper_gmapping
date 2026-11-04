// mapper_gmapping のコンポーネント (issue #5)。
//
// コンテナ gmapping が mapper_gmapping::Mapper を持ち、トピック odom・scan を購読して地図を作る。
// 型は juiz の標準の型 (src/component/generated/、scripts/gen_types.sh で作る)。
//
//   gmapping_set_odometry(odom)  odom (juiz_std.geometry.Pose2DStamped) を購読。最新のオドメトリを覚えるだけ
//   gmapping_add_scan(scan)      scan (juiz_std.sensor.LaserScan) を購読。覚えているオドメトリと組にして SLAM を進める
//                                処理したら、推定した姿勢を ContainerPose として基準 map_frame (既定 map) に書く (Transformations)
//   gmapping_set_laser_pose(mount) lidar_mount (juiz_std.geometry.PoseStamped、ロボットから見た LiDAR) を購読。
//                                取り付け位置が変わったら地図をやり直す
//   gmapping_get_pose()          推定した姿勢 (juiz_std.geometry.Pose2DStamped、frame_id は map)
//   gmapping_get_map()           地図の画像 (8 bit のグレースケール。占有 0、空き 254、不明 205)
//   gmapping_get_map_info()      地図の解像度・原点・大きさ (mapconf と同じ項目)
//   gmapping_save_map(path)      <path>.yaml と <path>.png に保存する
//   gmapping_reset()             地図と粒子を捨てて、やり直す
//   gmapping_get_state()         画面用: 姿勢、粒子、軌跡、更新回数など
//
// コンテナ gmapping_odom_frame は、SLAM が推定したオドメトリの原点 (map から見た odom、ROS の map -> odom) を
// 自分の ContainerPose として書く。juiz の ContainerProcess は自分の位置しか書けないので、mapper とは別のコンテナにした。
// conf でこのコンテナから mobile_sim の odom0 へ System をまたぐ Transform を張ると、mobile_sim の中で
// map -> (このコンテナ) -> odom0 -> robot0 -> lidar0 が 1 本の木になる (juiz 0.2.1、#376)。
//
//   gmapping_odom_frame_update(odom) odom を購読。引数 mapper のコンテナの map -> odom を、基準 map_frame から見た
//                                自分の位置として書き、Pose2DStamped (frame_id は map_frame) で返す。
//                                mapper が地図を作り始める前は書かずに、単位の姿勢を返す
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "juiz/juiz.h"
#include "juiz_idl.h"
#include "mapper_gmapping/mapper.h"
#include "mapper_gmapping/version.h"

namespace {

using juiz::Value;
using juiz_std::geometry::Pose2DStamped;
using juiz_std::geometry::PoseStamped;
using juiz_std::sensor::LaserScan;
using mapper_gmapping::Pose2D;

const Value* field(const Value& obj, const std::string& name) {
    if (!obj.isObjectValue()) return nullptr;
    const auto& m = obj.objectValue();
    auto it = m.find(name);
    return it == m.end() ? nullptr : &it->second;
}

double number_field(const Value& obj, const std::string& name, double fallback) {
    const Value* v = field(obj, name);
    if (!v) return fallback;
    if (v->isDoubleValue()) return v->doubleValue();
    if (v->isIntValue()) return static_cast<double>(v->intValue());
    throw std::invalid_argument(name + " must be a number");
}

double to_seconds(const juiz_std::common::Time& t) { return static_cast<double>(t.sec) + t.nanosec * 1e-9; }

juiz_std::common::Time to_time(double t) {
    juiz_std::common::Time out;
    const double sec = std::floor(t);
    out.sec = static_cast<int64_t>(sec);
    out.nanosec = static_cast<int32_t>(std::min(999999999LL, std::llround((t - sec) * 1e9)));
    return out;
}

Value pose_value(const Pose2D& p) {
    return Value{{"x", Value(p.x)}, {"y", Value(p.y)}, {"theta", Value(p.theta)}};
}

Value pose_list(const std::vector<Pose2D>& poses) {
    std::vector<Value> list;
    list.reserve(poses.size());
    for (const auto& p : poses) list.push_back(Value(std::vector<double>{p.x, p.y, p.theta}));
    return Value(list);
}

// 四元数の z 軸まわりの回転角 (yaw)
double yaw_of(const juiz_std::geometry::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// コンテナの引数のうち、Transformations まわりの設定
struct FrameSettings {
    std::string map_frame = "map";   // 推定した姿勢を書く基準 (空なら書かない)
    bool wait_laser_pose = false;    // lidar_mount を受け取るまでスキャンを捨てる
};

FrameSettings parse_frames(const juiz::CapsuleMap& args) {
    FrameSettings f;
    if (args.has("map_frame")) f.map_frame = args.get_string("map_frame");
    if (args.has("wait_laser_pose")) f.wait_laser_pose = args.get_bool("wait_laser_pose");
    return f;
}

// コンテナの引数 (conf の containers の項目) からパラメータを読む。書いていない項目は既定値
mapper_gmapping::Params parse_params(const juiz::CapsuleMap& args, double& map_update_interval) {
    mapper_gmapping::Params p;
    auto num = [&args](const char* name, double fallback) {
        if (!args.has(name)) return fallback;
        try {
            return args.get_float(name);
        } catch (...) {
            return static_cast<double>(args.get_int(name));
        }
    };
    auto integer = [&args, &num](const char* name, int fallback) {
        return static_cast<int>(std::lround(num(name, fallback)));
    };
    p.particles = integer("particles", p.particles);
    p.resampleThreshold = num("resampleThreshold", p.resampleThreshold);
    p.xmin = num("xmin", p.xmin);
    p.ymin = num("ymin", p.ymin);
    p.xmax = num("xmax", p.xmax);
    p.ymax = num("ymax", p.ymax);
    p.delta = num("delta", p.delta);
    p.occ_thresh = num("occ_thresh", p.occ_thresh);
    p.maxUrange = num("maxUrange", p.maxUrange);
    p.maxRange = num("maxRange", p.maxRange);
    p.sigma = num("sigma", p.sigma);
    p.kernelSize = integer("kernelSize", p.kernelSize);
    p.lstep = num("lstep", p.lstep);
    p.astep = num("astep", p.astep);
    p.iterations = integer("iterations", p.iterations);
    p.lsigma = num("lsigma", p.lsigma);
    p.ogain = num("ogain", p.ogain);
    p.lskip = integer("lskip", p.lskip);
    p.minimumScore = num("minimumScore", p.minimumScore);
    p.srr = num("srr", p.srr);
    p.srt = num("srt", p.srt);
    p.str = num("str", p.str);
    p.stt = num("stt", p.stt);
    p.linearUpdate = num("linearUpdate", p.linearUpdate);
    p.angularUpdate = num("angularUpdate", p.angularUpdate);
    p.temporalUpdate = num("temporalUpdate", p.temporalUpdate);
    p.llsamplerange = num("llsamplerange", p.llsamplerange);
    p.llsamplestep = num("llsamplestep", p.llsamplestep);
    p.lasamplerange = num("lasamplerange", p.lasamplerange);
    p.lasamplestep = num("lasamplestep", p.lasamplestep);
    p.seed = static_cast<unsigned long>(num("seed", 0.0));
    if (args.has("verbose")) p.verbose = args.get_bool("verbose");
    if (args.has("laser_pose")) {
        const Value lp(args.get_object("laser_pose"));
        p.laser_pose = Pose2D{number_field(lp, "x", 0.0), number_field(lp, "y", 0.0), number_field(lp, "theta", 0.0)};
    }
    map_update_interval = num("map_update_interval", 2.0);
    return p;
}

}  // namespace

class GMappingContainer;

// gmapping のコンテナを名前で引く表 (gmapping_odom_frame が mapper を見つけるため)。
// ロックは「表 → コンテナ」の順にだけ取る
namespace registry {
std::mutex& mutex() {
    static std::mutex m;
    return m;
}
std::map<std::string, GMappingContainer*>& table() {
    static std::map<std::string, GMappingContainer*> t;
    return t;
}
}  // namespace registry

class GMappingContainer {
public:
    GMappingContainer(const mapper_gmapping::Params& params, double map_update_interval, const FrameSettings& frames,
                      const std::string& name)
        : mapper_(params), map_update_interval_(map_update_interval), frames_(frames), name_(name) {
        if (name_.empty()) return;
        std::lock_guard<std::mutex> lock(registry::mutex());
        registry::table()[name_] = this;
    }

    ~GMappingContainer() {
        if (name_.empty()) return;
        std::lock_guard<std::mutex> lock(registry::mutex());
        auto it = registry::table().find(name_);
        if (it != registry::table().end() && it->second == this) registry::table().erase(it);
    }

    // map から見た odom (オドメトリの原点)。初期化前は nullopt
    std::optional<Pose2D> map_to_odom() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!mapper_.initialized()) return std::nullopt;
        return mapper_.map_to_odom();
    }

    double odom_time() {
        std::lock_guard<std::mutex> lock(mutex_);
        return odom_time_;
    }

    const std::string& map_frame() const { return frames_.map_frame; }

    bool initialized() {
        std::lock_guard<std::mutex> lock(mutex_);
        return mapper_.initialized();
    }

    // ロボットから見た LiDAR の位置を受け取る。変わったら地図と粒子を捨ててやり直す
    bool set_laser_pose(const PoseStamped& mount) {
        std::lock_guard<std::mutex> lock(mutex_);
        const Pose2D p{mount.pose.position.x, mount.pose.position.y, yaw_of(mount.pose.orientation)};
        have_laser_pose_ = true;
        if (!mapper_.set_laser_pose(p)) return false;
        map_ = mapper_gmapping::OccupancyGrid();
        map_updates_ = -1;
        return true;
    }

    // 推定した姿勢を ContainerPose に書けなかった理由を覚える (gmapping_get_state の pose_error で見える)
    void record_pose_error(const std::string& what) {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_error_ = what;
    }

    bool set_odometry(const Pose2DStamped& odom) {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_ = Pose2D{odom.pose.position.x, odom.pose.position.y, odom.pose.theta};
        odom_time_ = to_seconds(odom.header.stamp);
        have_odom_ = true;
        return true;
    }

    bool add_scan(const LaserScan& v) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++scans_;
        if (!have_odom_ || (frames_.wait_laser_pose && !have_laser_pose_)) {
            ++dropped_;
            return false;
        }
        try {
            mapper_gmapping::Scan scan;
            scan.stamp = to_seconds(v.header.stamp);
            scan.angle_min = v.angle_min;
            scan.angle_increment = v.angle_increment;
            scan.range_min = v.range_min;
            scan.range_max = v.range_max;
            scan.ranges = v.ranges;
            return mapper_.add_scan(scan, odom_);
        } catch (const std::exception& e) {
            ++dropped_;
            last_error_ = e.what();
            return false;
        }
    }

    Pose2DStamped pose() {
        std::lock_guard<std::mutex> lock(mutex_);
        const Pose2D p = mapper_.pose();
        Pose2DStamped out;
        out.header.stamp = to_time(odom_time_);
        out.header.frame_id = "map";
        out.pose.position.x = p.x;
        out.pose.position.y = p.y;
        out.pose.theta = p.theta;
        return out;
    }

    std::optional<juiz::Image> map_image() {
        std::lock_guard<std::mutex> lock(mutex_);
        refresh_map(false);
        if (map_.width <= 0) return std::nullopt;
        const std::vector<uint8_t> pixels = mapper_gmapping::map_to_pixels(map_);
        return juiz::Image(static_cast<uint32_t>(map_.width), static_cast<uint32_t>(map_.height),
                           juiz::PixelFormat::L8, pixels.data());
    }

    Value map_info() {
        std::lock_guard<std::mutex> lock(mutex_);
        refresh_map(false);
        return Value{{"resolution", Value(map_.resolution)},
                     {"origin", Value(std::vector<double>{map_.origin.x, map_.origin.y, map_.origin.theta})},
                     {"width", Value(static_cast<int64_t>(map_.width))},
                     {"height", Value(static_cast<int64_t>(map_.height))},
                     {"updates", Value(static_cast<int64_t>(map_updates_))},
                     {"occupied_thresh", Value(0.65)},
                     {"free_thresh", Value(0.196)},
                     {"negate", Value(static_cast<int64_t>(0))}};
    }

    std::string save_map(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        refresh_map(true);
        mapper_gmapping::save_map(map_, path);
        return path + ".yaml";
    }

    bool reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        mapper_.reset();
        map_ = mapper_gmapping::OccupancyGrid();
        map_updates_ = -1;
        scans_ = dropped_ = 0;
        last_error_.clear();
        return true;
    }

    Value state() {
        std::lock_guard<std::mutex> lock(mutex_);
        return Value{{"initialized", Value(mapper_.initialized())},
                     {"updates", Value(static_cast<int64_t>(mapper_.updates()))},
                     {"scans", Value(scans_)},
                     {"dropped", Value(dropped_)},
                     {"time", Value(odom_time_)},
                     {"pose", pose_value(mapper_.pose())},
                     {"odom", pose_value(odom_)},
                     {"map_to_odom", pose_value(mapper_.map_to_odom())},
                     {"particles", pose_list(mapper_.particles())},
                     {"trajectory", pose_list(mapper_.trajectory())},
                     {"map_updates", Value(static_cast<int64_t>(map_updates_))},
                     {"laser_pose", pose_value(mapper_.params().laser_pose)},
                     {"waiting_laser_pose", Value(frames_.wait_laser_pose && !have_laser_pose_)},
                     {"map_frame", Value(frames_.map_frame)},
                     {"pose_error", Value(pose_error_)},
                     {"last_error", Value(last_error_)}};
    }

    // プロセスの関数から漏れた例外を覚える (gmapping_get_state の last_error で見える)
    void record_error(const std::string& what) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = what;
    }

private:
    // 地図は重いので、更新があって前回から map_update_interval 秒たったときだけ作り直す
    void refresh_map(bool force) {
        if (map_updates_ == mapper_.updates()) return;
        const auto now = std::chrono::steady_clock::now();
        const double since = std::chrono::duration<double>(now - map_built_at_).count();
        if (!force && map_.width > 0 && since < map_update_interval_) return;
        map_ = mapper_.map();
        map_updates_ = mapper_.updates();
        map_built_at_ = now;
    }

    std::mutex mutex_;
    mapper_gmapping::Mapper mapper_;
    double map_update_interval_;
    FrameSettings frames_;
    std::string name_;
    bool have_laser_pose_ = false;
    std::string pose_error_;
    bool have_odom_ = false;
    Pose2D odom_;
    double odom_time_ = 0.0;
    int64_t scans_ = 0;
    int64_t dropped_ = 0;
    std::string last_error_;
    mapper_gmapping::OccupancyGrid map_;
    int map_updates_ = -1;
    std::chrono::steady_clock::time_point map_built_at_{};
};

// juiz_sdk 0.1.0 のプロセスの入口は juiz の Value の例外しか捕まえないので、それ以外の例外が C ABI を越えると
// juiz ごと落ちる (書けないパスへの save_map など)。ここで捕まえて覚え、nullopt (juiz ではエラー) を返す
template <typename F>
auto guarded(GMappingContainer* c, F&& f) -> decltype(f()) {
    try {
        return f();
    } catch (const std::exception& e) {
        c->record_error(e.what());
    } catch (...) {
        c->record_error("unknown error");
    }
    return std::nullopt;
}

static auto gmapping_manif = ContainerManifest("gmapping");
GMappingContainer* gmapping(juiz::CapsuleMap args) {
    double map_update_interval = 2.0;
    const auto params = parse_params(args, map_update_interval);
    const std::string name = args.has("name") ? args.get_string("name") : std::string();
    return new GMappingContainer(params, map_update_interval, parse_frames(args), name);
}
COMPONENT_CONTAINER_FACTORY(gmapping_manif, gmapping);

// SLAM が推定したオドメトリの原点 (map -> odom) を自分の ContainerPose として持つコンテナ
class OdomFrameContainer {
public:
    OdomFrameContainer(std::string mapper, std::string map_frame)
        : mapper_(std::move(mapper)), map_frame_(std::move(map_frame)) {}

    // mapper の map -> odom を読む。mapper が無い・初期化前なら nullopt
    std::optional<std::pair<Pose2D, double>> read() const {
        std::lock_guard<std::mutex> lock(registry::mutex());
        auto it = registry::table().find(mapper_);
        if (it == registry::table().end()) return std::nullopt;
        const auto m2o = it->second->map_to_odom();
        if (!m2o) return std::nullopt;
        return std::make_pair(*m2o, it->second->odom_time());
    }

    const std::string& mapper() const { return mapper_; }
    const std::string& map_frame() const { return map_frame_; }

private:
    std::string mapper_;
    std::string map_frame_;
};

static auto gmapping_odom_frame_manif = ContainerManifest("gmapping_odom_frame");
OdomFrameContainer* gmapping_odom_frame(juiz::CapsuleMap args) {
    const std::string mapper = args.has("mapper") ? args.get_string("mapper") : std::string("mapper0");
    const std::string map_frame = args.has("map_frame") ? args.get_string("map_frame") : std::string("map");
    return new OdomFrameContainer(mapper, map_frame);
}
COMPONENT_CONTAINER_FACTORY(gmapping_odom_frame_manif, gmapping_odom_frame);

static auto gmapping_odom_frame_update_manif = ProcessManifest("gmapping_odom_frame_update")
                                                    .add_typed_arg<Pose2DStamped>("odom", "odometry (only a trigger)")
                                                    .returns<Pose2DStamped>();
std::optional<Pose2DStamped> gmapping_odom_frame_update(OdomFrameContainer* c, const Pose2DStamped&) {
    // juiz の Value 以外の例外を C ABI の外へ出さない (guarded と同じ理由)
    try {
        const auto r = c->read();
        Pose2DStamped out;
        out.header.frame_id = c->map_frame();
        if (!r) return out;  // mapper がまだ地図を作り始めていない (ContainerPose は作ったときの単位のまま)
        const Pose2D& p = r->first;
        juiz::set_pose(c, c->map_frame(),
                       juiz::make_pose(p.x, p.y, 0.0, 0.0, 0.0, std::sin(p.theta / 2.0), std::cos(p.theta / 2.0)));
        out.header.stamp = to_time(r->second);
        out.pose.position.x = p.x;
        out.pose.position.y = p.y;
        out.pose.theta = p.theta;
        return out;
    } catch (const std::exception& e) {
        // 書けない (map -> このコンテナ の Transform が無いなど)。juiz の側にエラーとして返る
        std::cerr << "gmapping_odom_frame_update: " << e.what() << std::endl;
        return std::nullopt;
    }
}
COMPONENT_CONTAINER_PROCESS_FACTORY(OdomFrameContainer, gmapping_odom_frame_update_manif, gmapping_odom_frame_update);

static auto gmapping_set_odometry_manif =
    ProcessManifest("gmapping_set_odometry").add_typed_arg<Pose2DStamped>("odom", "odometry pose");
std::optional<bool> gmapping_set_odometry(GMappingContainer* c, const Pose2DStamped& odom) {
    return guarded(c, [&]() -> std::optional<bool> { return c->set_odometry(odom); });
}
COMPONENT_CONTAINER_PROCESS_FACTORY(GMappingContainer, gmapping_set_odometry_manif, gmapping_set_odometry);

static auto gmapping_add_scan_manif = ProcessManifest("gmapping_add_scan").add_typed_arg<LaserScan>("scan", "laser scan");
std::optional<bool> gmapping_add_scan(GMappingContainer* c, const LaserScan& scan) {
    return guarded(c, [&]() -> std::optional<bool> {
        const bool updated = c->add_scan(scan);
        if (c->initialized() && !c->map_frame().empty()) {
            // 推定した姿勢 (最良の粒子 + その後のオドメトリ) を、基準 map_frame から見たこのコンテナの位置として
            // スキャンのたびに書く (conf に map_frame -> このコンテナ の Transform が要る)。書けなくても SLAM は続ける
            const Pose2DStamped p = c->pose();
            const double th = p.pose.theta;
            try {
                juiz::set_pose(c, c->map_frame(),
                               juiz::make_pose(p.pose.position.x, p.pose.position.y, 0.0, 0.0, 0.0, std::sin(th / 2.0),
                                               std::cos(th / 2.0)));
            } catch (const juiz::TransformationsError& e) {
                c->record_pose_error(e.what());
            }
        }
        return updated;
    });
}
COMPONENT_CONTAINER_PROCESS_FACTORY(GMappingContainer, gmapping_add_scan_manif, gmapping_add_scan);

static auto gmapping_set_laser_pose_manif = ProcessManifest("gmapping_set_laser_pose")
                                                 .add_typed_arg<PoseStamped>("mount", "LiDAR pose seen from the robot");
std::optional<bool> gmapping_set_laser_pose(GMappingContainer* c, const PoseStamped& mount) {
    return guarded(c, [&]() -> std::optional<bool> { return c->set_laser_pose(mount); });
}
COMPONENT_CONTAINER_PROCESS_FACTORY(GMappingContainer, gmapping_set_laser_pose_manif, gmapping_set_laser_pose);

static auto gmapping_get_pose_manif = ProcessManifest("gmapping_get_pose").returns<Pose2DStamped>();
std::optional<Pose2DStamped> gmapping_get_pose(GMappingContainer* c) {
    return guarded(c, [&]() -> std::optional<Pose2DStamped> { return c->pose(); });
}
COMPONENT_CONTAINER_PROCESS_FACTORY(GMappingContainer, gmapping_get_pose_manif, gmapping_get_pose);

static auto gmapping_get_map_manif = ProcessManifest("gmapping_get_map");
std::optional<juiz::Image> gmapping_get_map(GMappingContainer* c) {
    return guarded(c, [&]() -> std::optional<juiz::Image> { return c->map_image(); });
}
COMPONENT_CONTAINER_PROCESS_FACTORY(GMappingContainer, gmapping_get_map_manif, gmapping_get_map);

static auto gmapping_get_map_info_manif = ProcessManifest("gmapping_get_map_info");
std::optional<juiz::Value> gmapping_get_map_info(GMappingContainer* c) {
    return guarded(c, [&]() -> std::optional<juiz::Value> { return c->map_info(); });
}
COMPONENT_CONTAINER_PROCESS_FACTORY(GMappingContainer, gmapping_get_map_info_manif, gmapping_get_map_info);

static auto gmapping_save_map_manif =
    ProcessManifest("gmapping_save_map").add_string_arg("path", "writes <path>.yaml and <path>.png", "map");
std::optional<std::string> gmapping_save_map(GMappingContainer* c, const std::string& path) {
    return guarded(c, [&]() -> std::optional<std::string> { return c->save_map(path); });
}
COMPONENT_CONTAINER_PROCESS_FACTORY(GMappingContainer, gmapping_save_map_manif, gmapping_save_map);

static auto gmapping_reset_manif = ProcessManifest("gmapping_reset");
std::optional<bool> gmapping_reset(GMappingContainer* c) {
    return guarded(c, [&]() -> std::optional<bool> { return c->reset(); });
}
COMPONENT_CONTAINER_PROCESS_FACTORY(GMappingContainer, gmapping_reset_manif, gmapping_reset);

static auto gmapping_get_state_manif = ProcessManifest("gmapping_get_state");
std::optional<juiz::Value> gmapping_get_state(GMappingContainer* c) {
    return guarded(c, [&]() -> std::optional<juiz::Value> { return c->state(); });
}
COMPONENT_CONTAINER_PROCESS_FACTORY(GMappingContainer, gmapping_get_state_manif, gmapping_get_state);

static auto mapper_gmapping_version_manif = ProcessManifest("mapper_gmapping_version");
std::optional<std::string> mapper_gmapping_version() { return mapper_gmapping::version(); }
COMPONENT_PROCESS_FACTORY(mapper_gmapping_version_manif, mapper_gmapping_version);

auto component_manifest() {
    return ComponentManifest("mapper_gmapping")
        .add_process(mapper_gmapping_version_manifest())
        .add_container(gmapping_manifest()
                           .add_process(gmapping_set_odometry_manifest())
                           .add_process(gmapping_add_scan_manifest())
                           .add_process(gmapping_set_laser_pose_manifest())
                           .add_process(gmapping_get_pose_manifest())
                           .add_process(gmapping_get_map_manifest())
                           .add_process(gmapping_get_map_info_manifest())
                           .add_process(gmapping_save_map_manifest())
                           .add_process(gmapping_reset_manifest())
                           .add_process(gmapping_get_state_manifest()))
        .add_container(gmapping_odom_frame_manifest().add_process(gmapping_odom_frame_update_manifest()));
}
DEFINE_COMPONENT_MANIFEST_ENTRY_POINT(component_manifest)
