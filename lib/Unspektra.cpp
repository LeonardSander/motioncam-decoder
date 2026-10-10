#include <motioncam/Decoder.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <zlib.h>

#if defined(_WIN32)
#define UNSPEKTRA_SEEK _fseeki64
#define UNSPEKTRA_TELL _ftelli64
#else
#define UNSPEKTRA_SEEK fseeko
#define UNSPEKTRA_TELL ftello
#endif

namespace motioncam {
namespace {
uint32_t big32(const uint8_t* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
           (uint32_t(p[2]) << 8) | p[3];
}
uint64_t big64(const uint8_t* p) {
    return (uint64_t(big32(p)) << 32) | big32(p + 4);
}
uint32_t little32(const uint8_t* p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
           (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
uint16_t little16(const uint8_t* p) {
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}
float littleFloat(const uint8_t* p) {
    const uint32_t bits = little32(p);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}
std::vector<uint8_t> decodeBase64(const std::string& input) {
    static const std::string alphabet =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::vector<uint8_t> output;
    uint32_t bits = 0;
    int count = 0;
    for (char c : input) {
        if (c == '=') break;
        const auto digit = alphabet.find(c);
        if (digit == std::string::npos) throw IOException("Invalid Unspektra base64");
        bits = (bits << 6) | static_cast<uint32_t>(digit);
        count += 6;
        if (count >= 8) {
            count -= 8;
            output.push_back(static_cast<uint8_t>(bits >> count));
        }
    }
    return output;
}
std::string inflateRecipe(const std::vector<uint8_t>& payload) {
    z_stream stream{};
    if (inflateInit(&stream) != Z_OK) throw IOException("Unspektra zlib initialization failed");
    std::string output;
    output.resize(4 * 1024 * 1024);
    stream.next_in = const_cast<Bytef*>(payload.data());
    stream.avail_in = static_cast<uInt>(payload.size());
    stream.next_out = reinterpret_cast<Bytef*>(output.data());
    stream.avail_out = static_cast<uInt>(output.size());
    const int result = inflate(&stream, Z_FINISH);
    const auto used = stream.total_out;
    inflateEnd(&stream);
    if (result != Z_STREAM_END || stream.avail_in != 0)
        throw IOException("Invalid Unspektra recipe zlib stream");
    output.resize(used);
    return output;
}
}

std::vector<uint8_t> Decoder::unspektraRecord(uint64_t offset, size_t maxSize) const {
    if (offset > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) ||
        UNSPEKTRA_SEEK(mFile, static_cast<int64_t>(offset), SEEK_SET) != 0)
        throw IOException("Invalid Unspektra record offset");
    uint8_t header[12];
    if (std::fread(header, 1, sizeof(header), mFile) != sizeof(header))
        throw IOException("Truncated Unspektra record header");
    const auto size = big32(header);
    if (big32(header + 4) != 0 || size > maxSize)
        throw IOException("Invalid Unspektra record size or type");
    std::vector<uint8_t> payload(size);
    if (std::fread(payload.data(), 1, size, mFile) != size)
        throw IOException("Truncated Unspektra record");
    if (crc32(0, payload.data(), size) != big32(header + 8))
        throw IOException("Unspektra record CRC32 mismatch");
    return payload;
}

void Decoder::initUnspektra() {
    uint8_t rest[12];
    if (std::fread(rest, 1, sizeof(rest), mFile) != sizeof(rest) || big32(rest) != 2)
        throw IOException("Unsupported Unspektra version");
    const auto index = nlohmann::json::parse(unspektraRecord(big64(rest + 4), 16 * 1024 * 1024));
    if (!index.contains("frames") || !index["frames"].is_array() || index["frames"].empty())
        throw IOException("Invalid Unspektra frame index");
    for (const auto& item : index["frames"]) {
        const auto timestamp = item.at("timestamp").get<Timestamp>();
        UnspektraEntry entry;
        entry.recipeOffset = item.at("recipe").get<uint64_t>();
        entry.rawOffset = item.at("raw").get<uint64_t>();
        if (!mUnspektraEntries.emplace(timestamp, entry).second)
            throw IOException("Duplicate Unspektra timestamp");
        mFrameList.push_back(timestamp);
    }
    mMetadata = unspektraFrameMetadata(mFrameList.front()).at("_container");
}

nlohmann::json Decoder::unspektraFrameMetadata(Timestamp timestamp) const {
    const auto entry = mUnspektraEntries.find(timestamp);
    if (entry == mUnspektraEntries.end()) throw IOException("Unspektra frame not found");
    const auto recipe = nlohmann::json::parse(inflateRecipe(
        unspektraRecord(entry->second.recipeOffset, 16 * 1024 * 1024)));
    if (recipe.value("version", 0) != 6) throw IOException("Unsupported Unspektra recipe");
    const auto raw = decodeBase64(recipe.at("raw").get<std::string>());
    const auto color = decodeBase64(recipe.at("color").get<std::string>());
    const auto shading = recipe.contains("shading")
        ? decodeBase64(recipe.at("shading").get<std::string>())
        : std::vector<uint8_t>{};
    if (raw.size() < 72 || color.size() < 16)
        throw IOException("Incomplete Unspektra camera metadata");
    const uint32_t width = little32(raw.data());
    const uint32_t height = little32(raw.data() + 4);
    const uint32_t cfaSize = 2 * little32(raw.data() + 0x40);
    const uint32_t cfaPhase = little32(raw.data() + 0x18);
    if (width < 2 || height < 2 || width > 16384 || height > 16384)
        throw IOException("Invalid Unspektra raster dimensions");
    if (cfaSize != 2 && cfaSize != 4 && cfaSize != 6 && cfaSize != 8)
        throw IOException("Unsupported Unspektra CFA size");
    if (cfaPhase > 3 || cfaPhase != little32(raw.data() + 0x44))
        throw IOException("Invalid Unspektra CFA phase");
    static constexpr const char* phaseNames[] = {"rggb", "grbg", "gbrg", "bggr"};
    const auto& capture = recipe.at("metadata");
    const double red = littleFloat(color.data());
    const double blue = littleFloat(color.data() + 8);
    const double white = littleFloat(raw.data() + 0x30);
    if (!(red > 0 && blue > 0 && white > 0 && white <= 65535))
        throw IOException("Invalid Unspektra levels or white balance");
    std::array<float, 4> black{};
    for (int i = 0; i < 4; ++i) black[i] = littleFloat(raw.data() + 0x20 + 4 * i);
    nlohmann::json frame{
        {"width", width}, {"height", height},
        {"originalWidth", width}, {"originalHeight", height},
        {"rowStride", width * 2}, {"pixelFormat", "RAW16"},
        {"compressionType", 0}, {"cfaSize", cfaSize},
        {"dynamicWhiteLevel", white}, {"dynamicBlackLevel", black},
        {"asShotNeutral", {1.0 / red, 1.0, 1.0 / blue}},
        {"iso", capture.value("iso", 100)},
        {"exposureTime", capture.value("exposure", 0.0)},
        {"aperture", capture.value("aperture", 0.0)},
        {"focalLength", capture.value("focalLength", 0.0)},
        {"equivalentFocalLength", capture.value("equivalentFocalLength", 0.0)},
        {"lensModel", capture.value("lens", std::string{})},
        {"timestamp", std::to_string(timestamp)},
        {"orientation", 0},
        {"recvdTimestampMs", std::to_string(recipe.value("capturedAt", int64_t{0}))}
    };
    if (!shading.empty()) {
        if (color.size() < 144)
            throw IOException("Incomplete Unspektra shading dimensions");
        const float mapWidth = littleFloat(color.data() + 136);
        const float mapHeight = littleFloat(color.data() + 140);
        if (!std::isfinite(mapWidth) || !std::isfinite(mapHeight) ||
            mapWidth < 2 || mapHeight < 2 || mapWidth > 512 || mapHeight > 512 ||
            mapWidth != std::floor(mapWidth) || mapHeight != std::floor(mapHeight) ||
            shading.size() != static_cast<size_t>(mapWidth * mapHeight * 4 * sizeof(float)))
            throw IOException("Invalid Unspektra shading map");
        const auto samples = static_cast<size_t>(mapWidth * mapHeight);
        std::array<std::vector<float>, 4> maps;
        for (auto& map : maps) map.reserve(samples);
        for (size_t i = 0; i < samples; ++i) {
            for (size_t channel = 0; channel < 4; ++channel) {
                const float gain = littleFloat(shading.data() + 4 * (4 * i + channel));
                if (!std::isfinite(gain) || gain <= 0.0f)
                    throw IOException("Invalid Unspektra shading gain");
                maps[channel].push_back(gain);
            }
        }
        frame["lensShadingMapWidth"] = static_cast<int>(mapWidth);
        frame["lensShadingMapHeight"] = static_cast<int>(mapHeight);
        frame["lensShadingMap"] = std::move(maps);
    }
    const auto& identity = capture.value("identity", nlohmann::json::object());
    // The color blob contains a camera-to-working matrix, but its relationship
    // to DNG color profile matrices is not established. No CFA phase field has
    // The CFA phase is stored twice in the raw descriptor. Its two bits give
    // the red pixel's x/y offset within the 2x2 color group.
    frame["_container"] = {
        {"sensorArrangement", phaseNames[cfaPhase]}, {"blackLevel", black},
        {"whiteLevel", white},
        {"uniqueCameraModel", identity.value("manufacturer", std::string{}) + " " +
                              identity.value("model", std::string{})},
        {"extraData", {{"audioChannels", 0}, {"audioSampleRate", 0},
            {"postProcessSettings", {{"metadata", {
                {"buildManufacturer", identity.value("manufacturer", std::string{})},
                {"buildModel", identity.value("model", std::string{})}
            }}}}}}
    };
    return frame;
}

void Decoder::loadUnspektraFrame(Timestamp timestamp, std::vector<uint8_t>& outData,
                                 nlohmann::json& outMetadata) {
    outMetadata = unspektraFrameMetadata(timestamp);
    const auto& entry = mUnspektraEntries.at(timestamp);
    const auto payload = unspektraRecord(entry.rawOffset, 256 * 1024 * 1024);
    if (payload.size() < 16 || std::memcmp(payload.data(), "UFR1", 4) != 0)
        throw IOException("Invalid UFR1 frame");
    const auto width = little32(payload.data() + 4);
    const auto height = little32(payload.data() + 8);
    if (width != outMetadata.at("width").get<uint32_t>() ||
        height != outMetadata.at("height").get<uint32_t>() ||
        little32(payload.data() + 12) != outMetadata.at("cfaSize").get<uint32_t>() ||
        width % 2 || height % 2)
        throw IOException("Unsupported UFR1 raster");
    outData.assign(static_cast<size_t>(width) * height * 2, 0);
    size_t position = 16;
    for (uint32_t tileY = 0; tileY < height; tileY += 16) {
        for (uint32_t tileX = 0; tileX < width; tileX += 16) {
            for (uint32_t group = 0; group < 4; ++group) {
                if (position + 3 > payload.size()) throw IOException("Truncated UFR1 block");
                const unsigned bits = payload[position];
                const unsigned reference = little16(payload.data() + position + 1);
                if (bits > 16 || position + 3 + 8 * bits > payload.size())
                    throw IOException("Invalid UFR1 block width");
                const uint8_t* packed = payload.data() + position + 3;
                for (unsigned i = 0; i < 64; ++i) {
                    const unsigned bit = i * bits;
                    uint32_t word = 0;
                    for (unsigned byte = 0; byte < 3 && bit / 8 + byte < 8 * bits; ++byte)
                        word |= uint32_t(packed[bit / 8 + byte]) << (8 * byte);
                    const unsigned delta = bits ? ((word >> (bit % 8)) & ((1u << bits) - 1)) : 0;
                    const unsigned value = reference + delta;
                    if (value > 65535) throw IOException("UFR1 sample overflow");
                    const unsigned x = tileX + 2 * (i % 8) + (group & 1);
                    const unsigned y = tileY + 2 * (i / 8) + (group >> 1);
                    if (x >= width || y >= height) continue;
                    const size_t dest = 2 * (static_cast<size_t>(y) * width + x);
                    outData[dest] = static_cast<uint8_t>(value);
                    outData[dest + 1] = static_cast<uint8_t>(value >> 8);
                }
                position += 3 + 8 * bits;
            }
        }
    }
    if (position != payload.size()) throw IOException("Trailing UFR1 data");
    outMetadata.erase("_container");
}
} // namespace motioncam
