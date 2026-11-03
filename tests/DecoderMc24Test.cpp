#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <motioncam/Decoder.hpp>
#include <motioncam/Mc24.hpp>
#include <stdexcept>

#include "DecoderMc24Vectors.hpp"

namespace {
using Bytes = std::vector<uint8_t>;
void check(bool ok) {
    if (!ok)
        throw std::runtime_error("mc24 test failed");
}
void writeTestInput(const std::filesystem::path &path, const Bytes &data) {
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char *>(data.data()), data.size());
    check(bool(f));
}
} // namespace
void checkCrcPaths() {
    const auto *known = reinterpret_cast<const uint8_t *>("123456789");
    check(motioncam::mc24::crc32c(known, 9) == 0xe3069283u);
    check(motioncam::mc24::crc32cPortable(known, 9) == 0xe3069283u);
    Bytes input(264);
    for (size_t i = 0; i < input.size(); ++i)
        input[i] = uint8_t(i * 73 + 19);
    for (size_t offset = 0; offset < 8; ++offset)
        for (size_t size = 0; size <= 256; ++size) {
            uint32_t reference = ~uint32_t{0};
            for (size_t i = 0; i < size; ++i) {
                reference ^= input[offset + i];
                for (unsigned bit = 0; bit < 8; ++bit)
                    reference = (reference >> 1) ^ ((reference & 1) ? 0x82f63b78u : 0);
            }
            reference = ~reference;
            check(motioncam::mc24::crc32c(input.data() + offset, size) == reference);
            check(motioncam::mc24::crc32cPortable(input.data() + offset, size) == reference);
        }
}
void checkBlockWidths() {
    for (const auto &block : mc24_test::blocks) {
        std::array<uint16_t, 32> decoded{};
        motioncam::mc24::decodeStrip(block.bytes.data(), block.bytes.size(), decoded.data(), 32, 1,
                                     0, block.packing, 0, 0);
        check(decoded == block.expected);
    }
}

void setChecksumValidation(bool enabled) {
#ifdef _WIN32
    _putenv_s("MOTIONCAM_MC24_VALIDATE_CHECKSUMS", enabled ? "1" : "");
#else
    if (enabled)
        setenv("MOTIONCAM_MC24_VALIDATE_CHECKSUMS", "1", 1);
    else
        unsetenv("MOTIONCAM_MC24_VALIDATE_CHECKSUMS");
#endif
}
int main() {
    const auto path =
        std::filesystem::temp_directory_path() /
        ("motioncam-mc24-decoder-test-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".mcpro");
    try {
        setChecksumValidation(true);
        checkCrcPaths();
        checkBlockWidths();
        // Golden lane-packed block: two absolute seeds of 64, then zero deltas.
        const Bytes packed{7, 64, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        std::array<uint16_t, 32> pixels{};
        motioncam::mc24::decodeStrip(packed.data(), packed.size(), pixels.data(), 32, 1, 0, 1, 0,
                                     0);
        for (auto value : pixels)
            check(value == 64);
        bool truncated = false;
        try {
            motioncam::mc24::decodeStrip(packed.data(), packed.size() - 1, pixels.data(), 32, 1, 0,
                                         1, 0, 0);
        } catch (const motioncam::IOException &) {
            truncated = true;
        }
        check(truncated);
        auto bytes = mc24_test::container;
        writeTestInput(path, bytes);
        {
            motioncam::Decoder decoder(path.string());
            check(decoder.getFrames().size() == 3);
            check(decoder.getFrames()[1] - decoder.getFrames()[0] == 16666667);
            check(decoder.getFrames()[2] - decoder.getFrames()[1] == 24333333);
            // Random access must resolve the second frame's inherited exposure.
            nlohmann::json md;
            std::vector<uint8_t> raw;
            decoder.loadFrame(decoder.getFrames()[1], raw, md);
            check(md.at("iso") == 101 && md.at("exposureTime") == 8333333 &&
                  md.at("orientation") == 0);
            check(raw.size() == 32);
            check(md.at("linearizationTable").size() == 256);
            check(md.at("linearizationTable")[100] == 200);
            for (unsigned y = 0; y < 4; ++y) {
                const std::array<std::array<unsigned, 4>, 4> expectedRows{{{{128, 140, 132, 146}},
                                                                           {{130, 142, 134, 148}},
                                                                           {{132, 144, 136, 150}},
                                                                           {{134, 146, 138, 152}}}};
                const auto &expected = expectedRows[y];
                for (unsigned x = 0; x < 4; ++x)
                    check((raw[8 * y + 2 * x] | unsigned(raw[8 * y + 2 * x + 1]) << 8) ==
                          expected[x] / 2);
            }
            decoder.loadFrame(decoder.getFrames()[0], raw, md, 2);
            check(raw.size() == 32 && md.at("width") == 4 && md.at("iso") == 100);
            std::vector<motioncam::AudioChunk> audio;
            decoder.loadAudio(audio);
            check(audio.size() == 1 && audio[0].float32Data == std::vector<float>({0.5f, -0.5f}));
            check(decoder.audioSampleRateHz() == 48000 && decoder.numAudioChannels() == 2);
            check(decoder.loadAudio().next(audio[0]) && !decoder.loadAudio().next(audio[0]));
            std::vector<motioncam::MotionSample> gyro, accel;
            decoder.loadGyroData(gyro);
            decoder.loadAccelerometerData(accel);
            check(decoder.hasGyroData() && decoder.hasAccelerometerData());
            check(gyro.size() == 1 && accel.size() == 1 && gyro[0].x == 1 && accel[0].z == 1);
        }
        // Corrupt a packed sample while keeping the record header intact.
        bytes[200 + 584 + 106] ^= 1;
        writeTestInput(path, bytes);
        bool rejected = false;
        try {
            motioncam::Decoder d(path.string());
            nlohmann::json md;
            Bytes raw;
            d.loadFrame(d.getFrames()[0], raw, md);
        } catch (const motioncam::IOException &) {
            rejected = true;
        }
        check(rejected);
        // The same valid payload with a stale checksum is accepted by default.
        setChecksumValidation(false);
        {
            motioncam::Decoder decoder(path.string());
            nlohmann::json metadata;
            Bytes raw;
            decoder.loadFrame(decoder.getFrames()[0], raw, metadata);
            check(raw[0] == 65);
        }
        setChecksumValidation(true);
        bytes = mc24_test::container;
        bytes[12] ^= 1;
        writeTestInput(path, bytes);
        rejected = false;
        try {
            motioncam::Decoder decoder(path.string());
        } catch (const motioncam::IOException &) {
            rejected = true;
        }
        check(rejected);
        setChecksumValidation(false);
        {
            motioncam::Decoder decoder(path.string());
            check(decoder.getFrames().size() == 3);
        }
        setChecksumValidation(true);
        bytes = mc24_test::container;
        bytes.resize(bytes.size() - 1);
        writeTestInput(path, bytes);
        rejected = false;
        try {
            motioncam::Decoder d(path.string());
        } catch (const motioncam::IOException &) {
            rejected = true;
        }
        check(rejected);
        std::filesystem::remove(path);
        std::cout << "mc24 container, metadata inheritance, linearization, audio, motion and "
                     "corruption checks passed\n";
    } catch (const std::exception &error) {
        std::filesystem::remove(path);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
