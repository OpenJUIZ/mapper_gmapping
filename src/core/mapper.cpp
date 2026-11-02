#include "mapper_gmapping/mapper.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>

#include <gmapping/gridfastslam/gridslamprocessor.h>
#include <gmapping/scanmatcher/scanmatcher.h>
#include <gmapping/sensor/sensor_range/rangereading.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>
#include <gmapping/utils/stat.h>

namespace mapper_gmapping {

namespace {

// GMapping は処理の途中で std::cerr に多くを書く (infoStream と関係なく)。verbose でなければその間だけ黙らせる
class CerrSilencer {
public:
    explicit CerrSilencer(bool silence) : old_(silence ? std::cerr.rdbuf(nullptr) : nullptr), active_(silence) {}
    ~CerrSilencer() {
        if (active_) {
            std::cerr.rdbuf(old_);
            std::cerr.clear();
        }
    }
    CerrSilencer(const CerrSilencer&) = delete;
    CerrSilencer& operator=(const CerrSilencer&) = delete;

private:
    std::streambuf* old_;
    bool active_;
};

// GridSlamProcessor の壊し方の直し:
// - デストラクタは粒子ごとに node を delete するが、init() の直後は全粒子が根の node を共有しているので
//   二重に解放してしまう。同じ node を指す粒子を外してから壊す
// - 各 node のスキャン (reading) は processScan が new するが、どこでも delete されない。
//   軌跡の木の node が全部消えた後で、ここで消す
class Processor : public GMapping::GridSlamProcessor {
public:
    explicit Processor(std::ostream& info) : GMapping::GridSlamProcessor(info) {}
    ~Processor() override {
        std::set<TNode*> seen;
        for (auto& particle : m_particles) {
            if (particle.node && !seen.insert(particle.node).second) {
                particle.node = nullptr;
            }
        }
        for (auto& particle : m_particles) {
            for (TNode* node = particle.node; node; node = node->parent) {
                if (node->reading) readings_.insert(node->reading);
            }
        }
        // readings_ は、基底のデストラクタが node を消した後で、メンバとして消える
    }

private:
    struct ReadingSet {
        std::set<const GMapping::RangeReading*> items;
        void insert(const GMapping::RangeReading* r) { items.insert(r); }
        ~ReadingSet() {
            for (const auto* r : items) delete r;
        }
    };
    ReadingSet readings_;
};

GMapping::OrientedPoint to_gmapping(const Pose2D& p) { return GMapping::OrientedPoint(p.x, p.y, p.theta); }
Pose2D from_gmapping(const GMapping::OrientedPoint& p) { return Pose2D{p.x, p.y, std::atan2(std::sin(p.theta), std::cos(p.theta))}; }

double normalize_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// a ∘ b (b を a の座標系で表したものを、a の親の座標系にする)
Pose2D compose(const Pose2D& a, const Pose2D& b) {
    const double c = std::cos(a.theta), s = std::sin(a.theta);
    return Pose2D{a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalize_angle(a.theta + b.theta)};
}

Pose2D inverse(const Pose2D& a) {
    const double c = std::cos(a.theta), s = std::sin(a.theta);
    return Pose2D{-c * a.x - s * a.y, s * a.x - c * a.y, normalize_angle(-a.theta)};
}

}  // namespace

int8_t OccupancyGrid::at_world(double wx, double wy) const {
    if (resolution <= 0.0) return -1;
    const int x = static_cast<int>(std::floor((wx - origin.x) / resolution));
    const int y = static_cast<int>(std::floor((wy - origin.y) / resolution));
    if (x < 0 || y < 0 || x >= width || y >= height) return -1;
    return at(x, y);
}

struct Mapper::Impl {
    Params params;
    std::ostream null_stream{nullptr};
    std::unique_ptr<Processor> processor;
    std::unique_ptr<GMapping::RangeSensor> sensor;
    std::vector<double> angles;
    double max_range = 0.0;
    double max_urange = 0.0;
    Pose2D last_odom;
    int updates = 0;

    explicit Impl(const Params& p) : params(p) {}

    ~Impl() { destroy(); }

    void destroy() {
        CerrSilencer quiet(!params.verbose);
        processor.reset();
        sensor.reset();
        angles.clear();
        updates = 0;
    }

    void init(const Scan& scan, const Pose2D& odom) {
        const size_t n = scan.ranges.size();
        if (n == 0) throw std::invalid_argument("mapper_gmapping: the scan has no beams");
        if (n > LASER_MAXBEAMS) throw std::invalid_argument("mapper_gmapping: the scan has more than 2048 beams");
        if (!(scan.range_max > 0.0) || !std::isfinite(scan.range_max)) {
            throw std::invalid_argument("mapper_gmapping: range_max of the scan must be positive");
        }
        if (scan.angle_increment == 0.0 || !std::isfinite(scan.angle_increment)) {
            throw std::invalid_argument("mapper_gmapping: angle_increment of the scan must not be 0");
        }
        // 当たらなかったビームは max_range にする。max_range > max_urange なので、GMapping はそのビームを
        // max_urange で切り、端点の無い「空き」としてだけ使う
        max_urange = params.maxUrange > 0.0 ? params.maxUrange : scan.range_max;
        max_range = params.maxRange > 0.0 ? params.maxRange : scan.range_max + 0.5;
        if (max_range <= max_urange) max_range = max_urange + 0.5;

        sensor = std::make_unique<GMapping::RangeSensor>(
            "FLASER", static_cast<unsigned int>(n), std::fabs(scan.angle_increment),
            to_gmapping(params.laser_pose), 0.0, max_range);
        angles.resize(n);
        for (size_t i = 0; i < n; ++i) {
            angles[i] = scan.angle_min + scan.angle_increment * static_cast<double>(i);
            sensor->beams()[i].pose.theta = angles[i];
        }
        sensor->updateBeamsLookup();

        std::ostream& info = params.verbose ? std::cerr : null_stream;
        processor = std::make_unique<Processor>(info);
        GMapping::SensorMap sensors;
        sensors.insert(std::make_pair(sensor->getName(), sensor.get()));
        processor->setSensorMap(sensors);
        processor->setMatchingParameters(max_urange, max_range, params.sigma, params.kernelSize, params.lstep,
                                         params.astep, params.iterations, params.lsigma, params.ogain,
                                         static_cast<unsigned int>(params.lskip));
        processor->setMotionModelParameters(params.srr, params.srt, params.str, params.stt);
        processor->setUpdateDistances(params.linearUpdate, params.angularUpdate, params.resampleThreshold);
        processor->setUpdatePeriod(params.temporalUpdate);
        processor->setgenerateMap(false);
        processor->setminimumScore(params.minimumScore);
        processor->setllsamplerange(params.llsamplerange);
        processor->setllsamplestep(params.llsamplestep);
        processor->setlasamplerange(params.lasamplerange);
        processor->setlasamplestep(params.lasamplestep);
        if (params.seed != 0) GMapping::sampleGaussian(1.0, params.seed);
        processor->init(static_cast<unsigned int>(std::max(1, params.particles)), params.xmin, params.ymin,
                        params.xmax, params.ymax, params.delta, to_gmapping(odom));
    }

    std::vector<double> readings(const Scan& scan) const {
        std::vector<double> r(scan.ranges.size());
        for (size_t i = 0; i < r.size(); ++i) {
            const double d = scan.ranges[i];
            const bool hit = std::isfinite(d) && d >= scan.range_min && d <= scan.range_max;
            r[i] = hit ? d : max_range;
        }
        return r;
    }

    const GMapping::GridSlamProcessor::Particle* best() const {
        if (!processor) return nullptr;
        const auto& particles = processor->getParticles();
        if (particles.empty()) return nullptr;
        const int i = processor->getBestParticleIndex();
        return &particles[static_cast<size_t>(std::max(0, i))];
    }
};

Mapper::Mapper(const Params& params) : impl_(std::make_unique<Impl>(params)) {}
Mapper::~Mapper() = default;

bool Mapper::add_scan(const Scan& scan, const Pose2D& odom) {
    CerrSilencer quiet(!impl_->params.verbose);
    if (!impl_->processor) {
        impl_->init(scan, odom);
    } else if (scan.ranges.size() != impl_->angles.size()) {
        throw std::invalid_argument("mapper_gmapping: the number of beams changed from " +
                                    std::to_string(impl_->angles.size()) + " to " +
                                    std::to_string(scan.ranges.size()));
    }
    impl_->last_odom = odom;
    const std::vector<double> r = impl_->readings(scan);
    GMapping::RangeReading reading(static_cast<unsigned int>(r.size()), r.data(), impl_->sensor.get(), scan.stamp);
    reading.setPose(to_gmapping(odom));
    const bool processed = impl_->processor->processScan(reading);
    if (processed) ++impl_->updates;
    return processed;
}

Pose2D Mapper::pose() const {
    const auto* b = impl_->best();
    return b ? from_gmapping(b->pose) : impl_->last_odom;
}

Pose2D Mapper::map_to_odom() const { return compose(pose(), inverse(impl_->last_odom)); }

std::vector<Pose2D> Mapper::particles() const {
    std::vector<Pose2D> out;
    if (!impl_->processor) return out;
    for (const auto& p : impl_->processor->getParticles()) out.push_back(from_gmapping(p.pose));
    return out;
}

std::vector<Pose2D> Mapper::trajectory() const {
    std::vector<Pose2D> out;
    const auto* b = impl_->best();
    if (!b) return out;
    for (const auto* node = b->node; node; node = node->parent) {
        if (node->reading) out.push_back(from_gmapping(node->pose));
    }
    std::reverse(out.begin(), out.end());
    return out;
}

OccupancyGrid Mapper::map() const {
    OccupancyGrid grid;
    const Params& p = impl_->params;
    grid.resolution = p.delta;
    const auto* b = impl_->best();
    if (!b) return grid;

    CerrSilencer quiet(!p.verbose);
    GMapping::ScanMatcher matcher;
    matcher.setLaserParameters(static_cast<unsigned int>(impl_->angles.size()), impl_->angles.data(),
                               to_gmapping(p.laser_pose));
    matcher.setlaserMaxRange(impl_->max_range);
    matcher.setusableRange(impl_->max_urange);
    matcher.setgenerateMap(true);

    GMapping::ScanMatcherMap smap(GMapping::Point((p.xmin + p.xmax) * 0.5, (p.ymin + p.ymax) * 0.5),
                                  p.xmin, p.ymin, p.xmax, p.ymax, p.delta);
    // 最良の粒子の軌跡の各 node に、そのときのスキャンが付いている。古いほうから地図に足す
    std::vector<const GMapping::GridSlamProcessor::TNode*> nodes;
    for (const auto* node = b->node; node; node = node->parent) {
        if (node->reading) nodes.push_back(node);
    }
    for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
        const GMapping::RangeReading& reading = *(*it)->reading;
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, (*it)->pose, &reading[0]);
        matcher.registerScan(smap, (*it)->pose, &reading[0]);
    }

    grid.width = smap.getMapSizeX();
    grid.height = smap.getMapSizeY();
    // map2world は画素の中心を返す
    const GMapping::Point c0 = smap.map2world(0, 0);
    grid.origin = Pose2D{c0.x - p.delta * 0.5, c0.y - p.delta * 0.5, 0.0};
    grid.data.assign(static_cast<size_t>(grid.width) * grid.height, -1);
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            const double occ = smap.cell(x, y);
            int8_t v = -1;
            if (occ >= 0.0) v = occ > p.occ_thresh ? 100 : 0;
            grid.data[static_cast<size_t>(y) * grid.width + x] = v;
        }
    }
    return grid;
}

bool Mapper::initialized() const { return impl_->processor != nullptr; }
int Mapper::updates() const { return impl_->updates; }
const Params& Mapper::params() const { return impl_->params; }
void Mapper::reset() { impl_->destroy(); }

}  // namespace mapper_gmapping
