// 地図を ROS map_server の形式 (YAML + PNG) で書く
#include <algorithm>
#include <array>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "mapper_gmapping/mapper.h"

namespace mapper_gmapping {

namespace {

constexpr uint8_t kOccupied = 0;
constexpr uint8_t kFree = 254;
constexpr uint8_t kUnknown = 205;

uint32_t crc32(const uint8_t* data, size_t n, uint32_t crc = 0) {
    static const std::array<uint32_t, 256> table = [] {
        std::array<uint32_t, 256> t{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            t[i] = c;
        }
        return t;
    }();
    crc = ~crc;
    for (size_t i = 0; i < n; ++i) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

void put_u32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>(v >> 24));
    out.push_back(static_cast<uint8_t>(v >> 16));
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v));
}

void put_chunk(std::vector<uint8_t>& out, const char* type, const std::vector<uint8_t>& data) {
    put_u32(out, static_cast<uint32_t>(data.size()));
    const size_t start = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), data.begin(), data.end());
    put_u32(out, crc32(out.data() + start, out.size() - start));
}

// 圧縮しない deflate (stored ブロック) の zlib ストリーム
std::vector<uint8_t> zlib_store(const std::vector<uint8_t>& raw) {
    std::vector<uint8_t> z{0x78, 0x01};
    size_t pos = 0;
    do {
        const size_t len = std::min<size_t>(65535, raw.size() - pos);
        const bool last = pos + len == raw.size();
        z.push_back(last ? 1 : 0);
        z.push_back(static_cast<uint8_t>(len));
        z.push_back(static_cast<uint8_t>(len >> 8));
        z.push_back(static_cast<uint8_t>(~len));
        z.push_back(static_cast<uint8_t>(~len >> 8));
        z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos),
                 raw.begin() + static_cast<std::ptrdiff_t>(pos + len));
        pos += len;
    } while (pos < raw.size());
    uint32_t a = 1, b = 0;
    for (uint8_t v : raw) {
        a = (a + v) % 65521;
        b = (b + a) % 65521;
    }
    put_u32(z, (b << 16) | a);
    return z;
}

std::string file_name(const std::string& path) {
    const size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

}  // namespace

std::vector<uint8_t> encode_png_gray8(int width, int height, const std::vector<uint8_t>& pixels) {
    if (width <= 0 || height <= 0 || pixels.size() != static_cast<size_t>(width) * height) {
        throw std::invalid_argument("encode_png_gray8: the size does not match the pixels");
    }
    std::vector<uint8_t> out{0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    std::vector<uint8_t> ihdr;
    put_u32(ihdr, static_cast<uint32_t>(width));
    put_u32(ihdr, static_cast<uint32_t>(height));
    ihdr.insert(ihdr.end(), {8, 0, 0, 0, 0});  // 8 bit、グレースケール、deflate、フィルタ 0、インターレース無し
    put_chunk(out, "IHDR", ihdr);
    std::vector<uint8_t> raw;
    raw.reserve(static_cast<size_t>(width + 1) * height);
    for (int y = 0; y < height; ++y) {
        raw.push_back(0);  // フィルタ無し
        const auto row = pixels.begin() + static_cast<std::ptrdiff_t>(y) * width;
        raw.insert(raw.end(), row, row + width);
    }
    put_chunk(out, "IDAT", zlib_store(raw));
    put_chunk(out, "IEND", {});
    return out;
}

std::vector<uint8_t> map_to_pixels(const OccupancyGrid& map) {
    std::vector<uint8_t> pixels(static_cast<size_t>(map.width) * map.height);
    for (int y = 0; y < map.height; ++y) {
        const int row = map.height - 1 - y;  // 画像は上の行から、地図は下の行から
        for (int x = 0; x < map.width; ++x) {
            const int8_t v = map.at(x, y);
            pixels[static_cast<size_t>(row) * map.width + x] = v < 0 ? kUnknown : (v >= 50 ? kOccupied : kFree);
        }
    }
    return pixels;
}

void save_map(const OccupancyGrid& map, const std::string& path) {
    if (map.width <= 0 || map.height <= 0) throw std::runtime_error("save_map: the map is empty");
    const std::string png_path = path + ".png";
    const std::vector<uint8_t> png = encode_png_gray8(map.width, map.height, map_to_pixels(map));
    {
        std::ofstream f(png_path, std::ios::binary);
        if (!f) throw std::runtime_error("save_map: cannot write " + png_path);
        f.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
    }
    std::ostringstream yaml;
    yaml << std::setprecision(9);
    yaml << "image: " << file_name(png_path) << "\n"
         << "resolution: " << map.resolution << "\n"
         << "origin: [" << map.origin.x << ", " << map.origin.y << ", " << map.origin.theta << "]\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n"
         << "negate: 0\n";
    const std::string yaml_path = path + ".yaml";
    std::ofstream f(yaml_path);
    if (!f) throw std::runtime_error("save_map: cannot write " + yaml_path);
    f << yaml.str();
}

}  // namespace mapper_gmapping
