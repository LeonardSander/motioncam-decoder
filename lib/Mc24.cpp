#include <motioncam/Decoder.hpp>
#include <motioncam/Mc24.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <set>
#include <utility>

#if defined(_WIN32)
#define MC24_SEEK _fseeki64
#define MC24_TELL _ftelli64
#else
#define MC24_SEEK fseeko
#define MC24_TELL ftello
#endif

namespace motioncam {
namespace {
uint16_t u16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }
uint32_t u32(const uint8_t *p) { return uint32_t(u16(p)) | uint32_t(u16(p + 2)) << 16; }
uint64_t u64(const uint8_t *p) { return uint64_t(u32(p)) | uint64_t(u32(p + 4)) << 32; }
float f32(const uint8_t *p) {
    auto bits = u32(p);
    float v;
    std::memcpy(&v, &bits, 4);
    return v;
}
void require(bool ok, const char *message) {
    if (!ok)
        throw IOException(message);
}

using Tags = std::map<uint16_t, std::vector<uint8_t>>;
Tags tags(const std::vector<uint8_t> &record) {
    Tags result;
    size_t pos = u16(record.data() + 30);
    require(pos >= 40 && pos <= record.size(), "Invalid mc24 metadata header");
    while (pos + 4 <= record.size()) {
        const auto type = u16(record.data() + pos), size = u16(record.data() + pos + 2);
        pos += 4;
        require(size <= record.size() - pos, "Truncated mc24 metadata tag");
        result[type] = {record.begin() + pos, record.begin() + pos + size};
        pos += (size + 3u) & ~size_t(3);
    }
    require(pos == record.size(), "Invalid mc24 metadata alignment");
    return result;
}
const uint8_t *tag(const Tags &t, uint16_t id, size_t size) {
    auto i = t.find(id);
    return i != t.end() && i->second.size() >= size ? i->second.data() : nullptr;
}
std::string stringTag(const Tags &t, uint16_t id) {
    auto i = t.find(id);
    if (i == t.end())
        return {};
    auto end = std::find(i->second.begin(), i->second.end(), uint8_t{0});
    return {i->second.begin(), end};
}
nlohmann::json matrixTag(const uint8_t *p) {
    auto result = nlohmann::json::array();
    for (unsigned i = 0; i < 9; ++i) {
        const auto n = static_cast<int32_t>(u32(p + 8 * i));
        const auto d = static_cast<int32_t>(u32(p + 8 * i + 4));
        require(d != 0, "Invalid mc24 matrix denominator");
        result.push_back(double(n) / d);
    }
    return result;
}
class Mc24AudioLoader final : public AudioChunkLoader {
  public:
    using Reader = std::function<std::vector<uint8_t>(uint64_t)>;
    Mc24AudioLoader(std::vector<BufferOffset> offsets, Reader reader)
        : offsets(std::move(offsets)), reader(std::move(reader)) {}
    bool next(AudioChunk &output) override {
        if (position == offsets.size())
            return false;
        const auto record = reader(offsets[position++].offset);
        require(record.size() >= 64, "Truncated mc24 audio header");
        const unsigned frames = u32(record.data() + 48), channels = u16(record.data() + 56);
        const unsigned format = record[58];
        require(channels && channels <= 32, "Invalid mc24 audio channels");
        const size_t samples = size_t(frames) * channels;
        const size_t bytes = format == 1 ? 2 : format == 4 ? 4 : 0;
        require(bytes && samples <= ((record.size() - 64) / bytes),
                "Unsupported mc24 audio format");
        output = AudioChunk{};
        output.timestamp = static_cast<Timestamp>(u64(record.data() + 32));
        output.format = format == 4 ? AudioSampleFormat::Float32 : AudioSampleFormat::Int16;
        if (format == 4) {
            output.float32Data.resize(samples);
            for (size_t i = 0; i < samples; ++i)
                output.float32Data[i] = f32(record.data() + 64 + 4 * i);
        } else {
            output.int16Data.resize(samples);
            for (size_t i = 0; i < samples; ++i)
                output.int16Data[i] = static_cast<int16_t>(u16(record.data() + 64 + 2 * i));
        }
        return true;
    }

  private:
    std::vector<BufferOffset> offsets;
    Reader reader;
    size_t position = 0;
};
} // namespace
std::vector<uint8_t> Decoder::mc24Record(uint64_t offset, bool full) const {
    require(offset <= mMc24FileSize && mMc24FileSize - offset >= 40, "Invalid mc24 record offset");
    require(MC24_SEEK(mFile, static_cast<int64_t>(offset), SEEK_SET) == 0,
            "Cannot seek mc24 record");
    std::vector<uint8_t> data(40);
    require(std::fread(data.data(), 1, 40, mFile) == 40, "Truncated mc24 record");
    const unsigned size = u32(data.data() + 4), head = u32(data.data() + 8);
    require(size >= 40 && size <= 256 * 1024 * 1024 && head >= 40 && head <= size &&
                size <= mMc24FileSize - offset,
            "Invalid mc24 record size");
    data.resize(full ? size : head);
    require(std::fread(data.data() + 40, 1, data.size() - 40, mFile) == data.size() - 40,
            "Truncated mc24 record payload");
    if (mMc24ValidateChecksums) {
        // CRC covers the used header, omitting its own four-byte field.
        std::vector<uint8_t> checked;
        checked.reserve(head - 4);
        checked.insert(checked.end(), data.begin(), data.begin() + 12);
        checked.insert(checked.end(), data.begin() + 16, data.begin() + head);
        require(mc24::crc32c(checked.data(), checked.size()) == u32(data.data() + 12),
                "mc24 record CRC32C mismatch");
    }
    return data;
}
void Decoder::initMc24() {
    const char *validate = std::getenv("MOTIONCAM_MC24_VALIDATE_CHECKSUMS");
    mMc24ValidateChecksums = validate && std::strcmp(validate, "1") == 0;
    require(MC24_SEEK(mFile, 0, SEEK_END) == 0, "Cannot measure mc24 container");
    const auto length = MC24_TELL(mFile);
    require(length >= 200, "Truncated mc24 container");
    mMc24FileSize = static_cast<uint64_t>(length);
    const auto header = mc24Record(0);
    require(header.size() >= 200 && std::memcmp(header.data() + 40, "\x89MCPRO\r\n", 8) == 0 &&
                u16(header.data() + 48) == 1,
            "Unsupported mc24 container version");
    const auto ht = tags(header);
    const auto recordingText = stringTag(ht, 0x100a);
    const auto recording =
        recordingText.empty() ? nlohmann::json::object() : nlohmann::json::parse(recordingText);

    static constexpr const char *phases[] = {"rggb", "grbg", "gbrg", "bggr"};
    require(header[107] < 4, "Unsupported mc24 CFA");
    const auto rotation = u16(header.data() + 186);
    mMc24Orientation = rotation == 90 ? 0 : rotation == 270 ? 1 : rotation == 180 ? 3 : 2;
    std::array<float, 4> black{};
    for (unsigned i = 0; i < 4; ++i)
        black[i] = float(u32(header.data() + 144 + 4 * i));
    const auto manufacturer = stringTag(ht, 0x1001), model = stringTag(ht, 0x1002);
    mMetadata = {
        {"sensorArrangement", phases[header[107]]},
        {"uniqueCameraModel", manufacturer + " " + model},
        {"whiteLevel", u32(header.data() + 140)},
        {"blackLevel", black},
        {"extraData",
         {{"audioChannels", 0},
          {"audioSampleRate", 0},
          {"postProcessSettings",
           {{"metadata", {{"buildManufacturer", manufacturer}, {"buildModel", model}}}}}}}};
    for (const auto &field :
         std::array<std::pair<uint16_t, const char *>, 6>{{{0x1010, "colorMatrix1"},
                                                           {0x1011, "colorMatrix2"},
                                                           {0x1014, "calibrationMatrix1"},
                                                           {0x1015, "calibrationMatrix2"},
                                                           {0x1012, "forwardMatrix1"},
                                                           {0x1013, "forwardMatrix2"}}})
        if (const auto p = tag(ht, field.first, 72))
            mMetadata[field.second] = matrixTag(p);
    if (const auto p = tag(ht, 0x1016, 4)) {
        const auto illuminant = [](unsigned value) -> const char * {
            switch (value) {
            case 17:
                return "standarda";
            case 18:
                return "standardb";
            case 19:
                return "standardc";
            case 21:
                return "d65";
            case 22:
                return "d75";
            case 23:
                return "d50";
            case 20:
                return "d55";
            default:
                return "";
            }
        };
        mMetadata["colorIlluminant1"] = illuminant(u16(p));
        mMetadata["colorIlluminant2"] = illuminant(u16(p + 2));
    }
    mMetadata["lensModel"] = stringTag(ht, 0x100b);
    const unsigned requestedBits = recording.value("bits", 0u);
    mMetadata["mc24InputBitDepth"] = requestedBits ? requestedBits : unsigned(header[137]);
    mMetadata["mc24InputLog"] = recording.value("log", false);
    mMetadata["mc24Compression"] = recording.value("level", std::string("Lossless"));

    std::map<uint32_t, Timestamp> frameIds;
    std::map<uint32_t, uint64_t> metadata;
    for (uint64_t offset = 0; offset < mMc24FileSize;) {
        const auto record = mc24Record(offset);
        const auto p = record.data();
        const auto fixed = u16(p + 30);
        if (std::memcmp(p, "MCRF", 4) == 0) {
            require(record.size() >= 96 && fixed >= 96, "Truncated mc24 frame header");
            const auto timestamp = static_cast<Timestamp>(u64(p + 32));
            require(timestamp >= 0 && !mMc24Entries.count(timestamp),
                    "Duplicate mc24 frame timestamp");
            require(frameIds.emplace(u32(p + 40), timestamp).second, "Duplicate mc24 frame number");
            mMc24Entries[timestamp].frameOffset = offset;
        } else if (std::memcmp(p, "MCPM", 4) == 0) {
            require(record.size() >= 48, "Truncated mc24 frame metadata");
            metadata[u32(p + 44)] = offset;
        } else if (std::memcmp(p, "MCRL", 4) == 0) {
            require(record.size() >= 68 && fixed == 68, "Truncated mc24 linearization table");
            const auto count = u16(p + 42);
            require(count && record.size() == 68 + 2u * count, "Invalid mc24 table size");
            auto &table = mMc24Tables[u32(p + 24)];
            table.resize(count);
            for (unsigned i = 0; i < count; ++i)
                table[i] = u16(p + 68 + 2 * i);
            require(std::is_sorted(table.begin(), table.end()), "Invalid mc24 linearization table");
        } else if (std::memcmp(p, "MCPB", 4) == 0 && record.size() >= 56 && u32(p + 40) == 1) {
            const unsigned width = u16(p + 48), height = u16(p + 50);
            require(width >= 2 && height >= 2 && width <= 512 && height <= 512 &&
                        record.size() == 56 + size_t(width) * height * 16,
                    "Invalid mc24 shading map");
            std::array<std::vector<float>, 4> maps;
            for (size_t i = 0; i < size_t(width) * height; ++i)
                for (size_t c = 0; c < 4; ++c) {
                    const float gain = f32(p + 56 + 16 * i + 4 * c);
                    require(std::isfinite(gain) && gain > 0, "Invalid mc24 shading gain");
                    maps[c].push_back(gain);
                }
            mMc24Shading[u32(p + 24)] = {{"lensShadingMapWidth", width},
                                         {"lensShadingMapHeight", height},
                                         {"lensShadingMap", maps}};
        } else if (std::memcmp(p, "MCPA", 4) == 0) {
            require(record.size() >= 64, "Truncated mc24 audio record");
            mAudioOffsets.push_back(
                {static_cast<int64_t>(offset), static_cast<Timestamp>(u64(p + 32))});
            mMetadata["extraData"]["audioChannels"] = u16(p + 56);
            mMetadata["extraData"]["audioSampleRate"] = u32(p + 52);
        } else if (std::memcmp(p, "MCPG", 4) == 0) {
            mGyroOffsets.push_back(
                {static_cast<int64_t>(offset), static_cast<Timestamp>(u64(p + 32))});
        }
        offset += u32(p + 4);
    }
    require(!mMc24Entries.empty(), "mc24 container contains no frames");
    for (const auto &frame : frameIds) {
        const auto md = metadata.find(frame.first);
        require(md != metadata.end(), "Missing mc24 frame metadata");
        std::vector<Tags> deltas;
        std::set<uint32_t> visited;
        uint32_t id = frame.first;
        for (;;) {
            require(visited.insert(id).second && visited.size() <= 1024,
                    "Invalid mc24 metadata dependency chain");
            const auto source = metadata.find(id);
            require(source != metadata.end(), "Missing mc24 metadata base");
            const auto record = mc24Record(source->second);
            deltas.push_back(tags(record));
            if (u16(record.data() + 28) & 2)
                break;
            id = u32(record.data() + 40);
        }
        auto &resolved = mMc24Entries.at(frame.second).metadataTags;
        for (auto delta = deltas.rbegin(); delta != deltas.rend(); ++delta)
            for (auto &item : *delta)
                resolved[item.first] = std::move(item.second);
    }
    for (const auto &entry : mMc24Entries)
        mFrameList.push_back(entry.first);
    mAudioLoader = std::make_unique<Mc24AudioLoader>(
        mAudioOffsets, [this](uint64_t offset) { return mc24Record(offset); });
}

nlohmann::json Decoder::mc24FrameMetadata(Timestamp timestamp) const {
    const auto entry = mMc24Entries.find(timestamp);
    require(entry != mMc24Entries.end(), "mc24 frame not found");
    const auto frame = mc24Record(entry->second.frameOffset);
    const auto p = frame.data();
    const auto &t = entry->second.metadataTags;
    std::vector<uint16_t> linearization;
    if (p[54]) {
        const auto table = mMc24Tables.find(u32(p + 68));
        require(p[54] == 1 && table != mMc24Tables.end(), "Missing mc24 linearization table");
        linearization = table->second;
    } else if (p[53]) {
        require(p[53] < 16, "Invalid mc24 sample shift");
        linearization.resize(65536);
        for (unsigned i = 0; i < linearization.size(); ++i)
            linearization[i] = static_cast<uint16_t>(std::min(65535u, i << p[53]));
    }

    const unsigned width = u16(p + 44), height = u16(p + 46);
    require(width >= 2 && height >= 2 && width <= 16384 && height <= 16384,
            "Invalid mc24 raster dimensions");
    nlohmann::json result = {{"width", width},
                             {"height", height},
                             {"originalWidth", width},
                             {"originalHeight", height},
                             {"rowStride", width * 2},
                             {"pixelFormat", "RAW16"},
                             {"compressionType", 0},
                             {"cfaSize", 2},
                             {"timestamp", std::to_string(timestamp)},
                             {"orientation", mMc24Orientation},
                             {"dynamicWhiteLevel", mMetadata.at("whiteLevel")},
                             {"dynamicBlackLevel", mMetadata.at("blackLevel")},
                             {"asShotNeutral", {1.0, 1.0, 1.0}},
                             {"lensModel", mMetadata.value("lensModel", std::string{})}};
    if (!linearization.empty())
        result["linearizationTable"] = std::move(linearization);
    if (const auto v = tag(t, 1, 8))
        result["exposureTime"] = u64(v);
    if (const auto v = tag(t, 3, 4))
        result["iso"] = u32(v);
    if (const auto v = tag(t, 5, 16)) {
        std::array<float, 4> black{};
        for (unsigned i = 0; i < 4; ++i)
            black[i] = f32(v + 4 * i);
        result["dynamicBlackLevel"] = black;
    }
    if (const auto v = tag(t, 6, 4))
        result["dynamicWhiteLevel"] = u32(v);
    if (const auto v = tag(t, 8, 16)) {
        const float r = f32(v), g1 = f32(v + 4), g2 = f32(v + 8), b = f32(v + 12);
        require(std::isfinite(r) && std::isfinite(b) && std::isfinite(g1) && std::isfinite(g2) &&
                    r > 0 && b > 0 && g1 > 0 && g2 > 0,
                "Invalid mc24 white balance");
        result["asShotNeutral"] = {(g1 + g2) / (2 * r), 1.0, (g1 + g2) / (2 * b)};
    }
    if (const auto v = tag(t, 0xa, 64)) {
        std::array<double, 8> profile{};
        for (unsigned i = 0; i < 8; ++i) {
            const uint64_t bits = u64(v + 8 * i);
            std::memcpy(&profile[i], &bits, 8);
            require(std::isfinite(profile[i]), "Invalid mc24 noise profile");
        }
        result["noiseProfile"] = {
            profile[0], profile[1], (profile[2] + profile[4]) / 2, (profile[3] + profile[5]) / 2,
            profile[6], profile[7]};
    }
    if (const auto v = tag(t, 0xe, 4))
        result["focalLength"] = f32(v);
    if (const auto v = tag(t, 0xf, 4))
        result["aperture"] = f32(v);
    if (const auto v = tag(t, 0x1c, 4)) {
        const auto map = mMc24Shading.find(u32(v));
        if (map != mMc24Shading.end())
            result.update(map->second);
    }
    return result;
}

void Decoder::loadMc24Frame(Timestamp timestamp, std::vector<uint8_t> &outData,
                            nlohmann::json &outMetadata) {
    outMetadata = mc24FrameMetadata(timestamp);
    const auto record = mc24Record(mMc24Entries.at(timestamp).frameOffset, true);
    const auto p = record.data();
    const unsigned width = u16(p + 44), height = u16(p + 46), strips = u16(p + 60),
                   rows = u16(p + 62);
    const size_t fixed = u16(p + 30), head = u32(p + 8);
    require(rows && strips == (height + rows - 1) / rows && fixed + size_t(strips) * 8 == head &&
                p[92] == 1,
            "Invalid mc24 strip table");
    const auto tableSize =
        outMetadata.contains("linearizationTable") ? outMetadata["linearizationTable"].size() : 0;
    const size_t sampleCount = size_t(width) * height;
    outData.resize(sampleCount * sizeof(uint16_t));
    auto *samples = reinterpret_cast<uint16_t *>(outData.data());
    size_t position = head;
    for (unsigned strip = 0; strip < strips; ++strip) {
        const auto size = u32(p + fixed + 4 * strip);
        require(size <= record.size() - position, "Truncated mc24 strip");
        if (mMc24ValidateChecksums)
            require(mc24::crc32c(p + position, size) == u32(p + fixed + 4 * strips + 4 * strip),
                    "mc24 strip CRC32C mismatch");
        unsigned packing = p[57];
        if (packing == 3) {
            const unsigned split = u16(p + 94);
            require(split > 0 && split < strips, "Invalid mc24 packing split");
            packing = (p[93] >> (strip < split ? 0 : 4)) & 15;
        }
        mc24::decodeStrip(p + position, size, samples + size_t(strip) * rows * width, width,
                          std::min(rows, height - strip * rows), p[55], packing, p[56],
                          u16(p + 58));
        position += size;
    }
    require(record.size() - position <= 7, "Trailing mc24 frame data");
    if (tableSize) {
        uint16_t maximum = 0;
        for (size_t i = 0; i < sampleCount; ++i)
            maximum = std::max(maximum, samples[i]);
        require(maximum < tableSize, "mc24 quantized sample out of range");
    }
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    for (size_t i = 0; i < sampleCount; ++i) {
        const auto value = samples[i];
        outData[2 * i] = static_cast<uint8_t>(value);
        outData[2 * i + 1] = static_cast<uint8_t>(value >> 8);
    }
#endif
}
void Decoder::loadMc24Audio(std::vector<AudioChunk> &output) {
    Mc24AudioLoader loader(mAudioOffsets, [this](uint64_t offset) { return mc24Record(offset); });
    AudioChunk chunk;
    while (loader.next(chunk))
        output.push_back(chunk);
}
void Decoder::loadMc24Motion(std::vector<MotionSample> &output, bool acceleration) {
    for (const auto &offset : mGyroOffsets) {
        const auto data = mc24Record(offset.offset);
        require(data.size() >= 48 && data[44] == 1 && data[45] == 32,
                "Unsupported mc24 motion format");
        const auto count = u32(data.data() + 40);
        require(data.size() == 48 + size_t(count) * 32, "Invalid mc24 motion record size");
        for (unsigned i = 0; i < count; ++i) {
            const auto p = data.data() + 48 + 32 * i;
            MotionSample sample;
            sample.timestampNs = static_cast<Timestamp>(u64(p));
            const unsigned base = acceleration ? 20 : 8;
            sample.x = f32(p + base);
            sample.y = f32(p + base + 4);
            sample.z = f32(p + base + 8);
            require(std::isfinite(sample.x) && std::isfinite(sample.y) && std::isfinite(sample.z),
                    "Invalid mc24 motion sample");
            output.push_back(sample);
        }
    }
}
} // namespace motioncam
