#include <motioncam/Decoder.hpp>
#include <motioncam/RawData.hpp>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <limits>
#include <set>

#if defined(_WIN32)
    #define FSEEK _fseeki64
    #define FTELL _ftelli64
#elif defined(__unix__) || defined(__linux__) || defined(__APPLE__)
    #define _FILE_OFFSET_BITS 64
    
    #define FSEEK fseeko
    #define FTELL ftello
#else
    #error Unknown platform
#endif

namespace motioncam {
    constexpr int MOTIONCAM_COMPRESSION_TYPE_LEGACY = 6;
    constexpr int MOTIONCAM_COMPRESSION_TYPE = 7;

    namespace {
        class AudioChunkLoaderImpl : public AudioChunkLoader {
            public:
                AudioChunkLoaderImpl(FILE* f, const std::vector<BufferOffset>& offsets);
                bool next(AudioChunk& output);
                
            private:
                FILE* mFile;
                const std::vector<BufferOffset>& mOffsets;
                
                size_t mIdx;
        };
    
        void read(FILE* f, void* data, size_t size, size_t items=1) {
            if(std::fread(data, size, items, f) != items) {
                throw IOException("Failed to read data");
            }
        }
    
        bool loadAudioChunk(FILE* f, const BufferOffset& o, AudioChunk& outChunk) {
            if(FSEEK(f, o.offset, SEEK_SET) != 0)
                return false;
            
            // Get audio data header
            Item audioDataItem{};
            read(f, &audioDataItem, sizeof(Item));
            
            // Determine format from item type
            if(audioDataItem.type == Type::AUDIO_DATA_F32) {
                outChunk.format = AudioSampleFormat::Float32;
                outChunk.float32Data.resize(audioDataItem.size / sizeof(float));
                read(f, (void*)outChunk.float32Data.data(), audioDataItem.size);
            } 
            else if(audioDataItem.type == Type::AUDIO_DATA) {
                outChunk.format = AudioSampleFormat::Int16;
                outChunk.int16Data.resize(audioDataItem.size / sizeof(int16_t));
                read(f, (void*)outChunk.int16Data.data(), audioDataItem.size);
            } 
            else {
                throw IOException("Invalid audio data");
            }

            // Metadata should follow (this was added later so some files may not have it)
            Item audioMetadataItem{};
            read(f, &audioMetadataItem, sizeof(Item));
            
            Timestamp audioTimestamp = -1;
            
            if(audioMetadataItem.type == Type::AUDIO_DATA_METADATA) {
                AudioMetadata metadata;
                
                read(f, &metadata, sizeof(AudioMetadata));
                audioTimestamp = metadata.timestampNs;
            }
        
            outChunk.timestamp = audioTimestamp;
            
            return true;
        }
    }
    //
    
    AudioChunkLoaderImpl::AudioChunkLoaderImpl(FILE* f, const std::vector<BufferOffset>& offsets) :
        mFile(f), mOffsets(offsets), mIdx(0) {
    }
    
    bool AudioChunkLoaderImpl::next(AudioChunk& output) {
        if(mIdx >= mOffsets.size())
            return false;
        
        if(!loadAudioChunk(mFile, mOffsets[mIdx], output)) {
            return false;
        }
        
        ++mIdx;
        return true;
    }
    
    //

    Decoder::Decoder(FILE* file) : mFile(file), mBufferStartOffset(0) {
        if(!mFile)
            throw IOException("Invalid file");

        try {
            init();
        }
        catch(...) {
            std::fclose(mFile);
            mFile = nullptr;
            throw;
        }
    }

    Decoder::Decoder(const std::string& path) : mFile(std::fopen(path.c_str(), "rb")), mBufferStartOffset(0) {
        if(!mFile)
            throw IOException("Failed to open " + path);

        try {
            init();
        }
        catch(...) {
            std::fclose(mFile);
            mFile = nullptr;
            throw;
        }
    }
    
    Decoder::~Decoder() {
        if(mFile)
            std::fclose(mFile);
    }

    void Decoder::init() {
        uint8_t signature[8]{};
        read(signature, sizeof(signature));
        if(std::memcmp(signature, "UNSPKTRA", sizeof(signature)) == 0) {
            mUnspektra = true;
            initUnspektra();
            mAudioLoader = std::make_unique<AudioChunkLoaderImpl>(mFile, mAudioOffsets);
            return;
        }
        if(FSEEK(mFile, 0, SEEK_SET) != 0)
            throw IOException("Failed to rewind container");
        Header header{};
        
        // Check validity of file
        read(&header, sizeof(Header));

        // Support current version and also version 3
        if((header.version != CONTAINER_VERSION))
            throw IOException("Invalid container version");

        if(std::memcmp(header.ident, CONTAINER_ID, sizeof(CONTAINER_ID)) != 0)
            throw IOException("Invalid header id");

        // Read camera metadata
        Item metadataItem{};
        read(&metadataItem, sizeof(Item));
        
        if(metadataItem.type != Type::METADATA)
            throw IOException("Invalid camera metadata");
        
        std::vector<uint8_t> metadataJson(metadataItem.size);
        read(metadataJson.data(), metadataItem.size);
        
        // Keep the camera metadata
        auto cameraMetadataString = std::string(metadataJson.begin(), metadataJson.end());
        mMetadata = nlohmann::json::parse(cameraMetadataString);

        mBufferStartOffset = FTELL(mFile);
        if(mBufferStartOffset < 0)
            throw IOException("Failed to get buffer start offset");
  
        readIndex();

        reindexOffsets();

        readExtra();
        
        // Create audio loader
        mAudioLoader = std::make_unique<AudioChunkLoaderImpl>(mFile, mAudioOffsets);
    }
    
    const std::vector<Timestamp>& Decoder::getFrames() const {
        return mFrameList;
    }
    
    const nlohmann::json& Decoder::getContainerMetadata() const {
        return mMetadata;
    }
    
    int Decoder::audioSampleRateHz() const {
        return mMetadata["extraData"]["audioSampleRate"];
    }
    
    int Decoder::numAudioChannels() const {
        return mMetadata["extraData"]["audioChannels"];
    }
    
    void Decoder::loadAudio(std::vector<AudioChunk>& outAudioChunks) {
        for(const auto& o : mAudioOffsets) {
            AudioChunk chunk;
            
            if(!loadAudioChunk(mFile, o, chunk))
                continue;

            outAudioChunks.emplace_back(chunk);
        }
    }
    
    AudioChunkLoader& Decoder::loadAudio() const {
        return *mAudioLoader;
    }

    bool Decoder::hasGyroData() const {
        return !mGyroOffsets.empty();
    }

    bool Decoder::hasAccelerometerData() const {
        return !mAccelerometerOffsets.empty();
    }

    void Decoder::loadGyroData(std::vector<MotionSample>& outGyroSamples) {
        for(const auto& offset : mGyroOffsets) {
            if(FSEEK(mFile, offset.offset, SEEK_SET) != 0)
                throw IOException("Invalid gyro data offset");

            Item item{};
            read(&item, sizeof(Item));

            if(item.type != Type::GYRO_DATA || item.size < sizeof(GyroDataHeader) || !payloadFitsInFile(item.size))
                throw IOException("Invalid gyro data");

            GyroDataHeader header{};
            read(&header, sizeof(GyroDataHeader));

            const uint64_t expectedSize = sizeof(GyroDataHeader)
                + static_cast<uint64_t>(header.numSamples) * sizeof(MotionSample);
            if(header.version != GYRO_DATA_VERSION || header.numSamples == 0 || expectedSize != item.size)
                throw IOException("Invalid gyro data");

            if(header.numSamples > outGyroSamples.max_size() - outGyroSamples.size())
                throw IOException("Too many gyro samples");

            const size_t oldSize = outGyroSamples.size();
            outGyroSamples.resize(oldSize + header.numSamples);
            read(outGyroSamples.data() + oldSize, sizeof(MotionSample), header.numSamples);
        }
    }

    void Decoder::loadAccelerometerData(std::vector<MotionSample>& outSamples) {
        for(const auto& offset : mAccelerometerOffsets) {
            if(FSEEK(mFile, offset.offset, SEEK_SET) != 0)
                throw IOException("Invalid accelerometer data offset");

            Item item{};
            read(&item, sizeof(Item));

            if(item.type != Type::ACCELEROMETER_DATA || item.size < sizeof(AccelerometerDataHeader)
                || !payloadFitsInFile(item.size))
                throw IOException("Invalid accelerometer data");

            AccelerometerDataHeader header{};
            read(&header, sizeof(AccelerometerDataHeader));

            const uint64_t expectedSize = sizeof(AccelerometerDataHeader)
                + static_cast<uint64_t>(header.numSamples) * sizeof(MotionSample);
            if(header.version != ACCELEROMETER_DATA_VERSION || header.numSamples == 0 || expectedSize != item.size)
                throw IOException("Invalid accelerometer data");

            if(header.numSamples > outSamples.max_size() - outSamples.size())
                throw IOException("Too many accelerometer samples");

            const size_t oldSize = outSamples.size();
            outSamples.resize(oldSize + header.numSamples);
            read(outSamples.data() + oldSize, sizeof(MotionSample), header.numSamples);
        }
    }
    
    void Decoder::loadFrame(const Timestamp timestamp, std::vector<uint8_t>& outData,
                            nlohmann::json& outMetadata, int widthOverride) {
        if (mUnspektra) {
            // UFR1 declares its own raster width; a MotionCam stride repair
            // suffix does not apply to this container.
            loadUnspektraFrame(timestamp, outData, outMetadata);
            return;
        }
        if(mFrameOffsetMap.find(timestamp) == mFrameOffsetMap.end())
            throw IOException("Frame not found (timestamp: " + std::to_string(timestamp) + ")");

        int64_t offset = mFrameOffsetMap.at(timestamp).offset;

        if(FSEEK(mFile, offset, SEEK_SET) != 0)
            throw IOException("Invalid offset");

        Item bufferItem{};
        read(&bufferItem, sizeof(Item));

        if(bufferItem.type != Type::BUFFER)
            throw IOException("Invalid buffer type");

        mTmpBuffer.resize(bufferItem.size);

        read(mTmpBuffer.data(), bufferItem.size);

        // Get metadata
        Item metadataItem{};
        read(&metadataItem, sizeof(Item));

        if(metadataItem.type != Type::METADATA)
            throw IOException("Invalid metadata");

        std::vector<uint8_t> metadataJson(metadataItem.size);
        read(metadataJson.data(), metadataItem.size);

        std::string metadataString = std::string(metadataJson.begin(), metadataJson.end());
        outMetadata = nlohmann::json::parse(metadataString);

        const int metadataWidth = outMetadata["width"];
        // Never discard samples when the override is narrower than metadata.
        // The caller needs the complete decoded stream in order to reinterpret
        // incorrect row boundaries with the requested stride.
        const int width = widthOverride > metadataWidth ? widthOverride : metadataWidth;
        const int height = outMetadata["height"];
        const int compressionType = outMetadata["compressionType"];

        // Decompress the buffer
        const size_t outputSizeBytes = sizeof(uint16_t) * width*height;
        outData.resize(outputSizeBytes);

        if(compressionType == MOTIONCAM_COMPRESSION_TYPE) {
            if(raw::Decode(reinterpret_cast<uint16_t*>(outData.data()), width, height, mTmpBuffer.data(), mTmpBuffer.size()) <= 0)
                throw IOException("Failed to uncompress frame");
        }
        else if(compressionType == MOTIONCAM_COMPRESSION_TYPE_LEGACY) {
            if(raw::DecodeLegacy(reinterpret_cast<uint16_t*>(outData.data()), width, height, mTmpBuffer.data(), mTmpBuffer.size()) <= 0)
                throw IOException("Failed to uncompress legacy frame");
        }
        else {
            throw IOException("Invalid compression type");
        }

        // Consumers using a stride override must see the decoded row width.
        // They can subsequently crop it back to the intended visible width.
        if (widthOverride > 0)
            outMetadata["width"] = widthOverride;
    }

    void Decoder::loadFrameMetadata(const Timestamp timestamp, nlohmann::json& outMetadata) {
        if (mUnspektra) {
            outMetadata = unspektraFrameMetadata(timestamp);
            return;
        }
        if(mFrameOffsetMap.find(timestamp) == mFrameOffsetMap.end())
            throw IOException("Frame not found (timestamp: " + std::to_string(timestamp) + ")");
        
        int64_t offset = mFrameOffsetMap.at(timestamp).offset;

        if(FSEEK(mFile, offset, SEEK_SET) != 0)
            throw IOException("Invalid offset");

        Item bufferItem{};
        read(&bufferItem, sizeof(Item));

        if(bufferItem.type != Type::BUFFER)
            throw IOException("Invalid buffer type");

        if(FSEEK(mFile, bufferItem.size, SEEK_CUR) != 0)
            throw IOException("Failed to skip buffer data");

        // Get metadata
        Item metadataItem{};
        read(&metadataItem, sizeof(Item));
        
        if(metadataItem.type != Type::METADATA)
            throw IOException("Invalid metadata");
        
        std::vector<uint8_t> metadataJson(metadataItem.size);
        read(metadataJson.data(), metadataItem.size);
        
        std::string metadataString = std::string(metadataJson.begin(), metadataJson.end());
        outMetadata = nlohmann::json::parse(metadataString);       
    }

    void Decoder::readIndex() {
        // Seek to index item
        if(FSEEK(mFile, -static_cast<long>(sizeof(BufferIndex) + sizeof(Item)), SEEK_END) != 0)
            throw IOException("Failed to get end chunk");

        Item bufferIndexItem{};
        read(&bufferIndexItem, sizeof(Item));
        
        if(bufferIndexItem.type != Type::BUFFER_INDEX)
            throw IOException("Invalid file");
        
        BufferIndex index{};
        read(&index, sizeof(BufferIndex));
        
        // Check validity of index
        if(index.magicNumber != INDEX_MAGIC_NUMBER)
            throw IOException("Corrupted file");
        
        mOffsets.resize(index.numOffsets);
        
        // Read the index
        if(FSEEK(mFile, index.indexDataOffset, SEEK_SET) != 0) {
            throw IOException("Invalid index");
            return;
        }
        
        read(mOffsets.data(), sizeof(BufferOffset), mOffsets.size());
    }
    
    void Decoder::reindexOffsets() {
        // File offsets preserve capture order even when a damaged timestamp
        // would sort a frame to the beginning or end of the clip.
        std::sort(mOffsets.begin(), mOffsets.end(), [](const auto& a, const auto&b) {
            return a.offset < b.offset;
        });

        // Repair isolated timestamp outliers before they reach VFR/CFR
        // mapping. A bad clock value commonly appears as -1 or as a single
        // enormous jump while the timestamps on either side retain the
        // camera's normal cadence.
        constexpr long double outlierRatio = 100.0L;
        auto distance = [](Timestamp a, Timestamp b) {
            return std::fabs(static_cast<long double>(a) - static_cast<long double>(b));
        };
        auto clampedTimestamp = [](long double value) {
            const auto minimum = static_cast<long double>(
                std::numeric_limits<Timestamp>::min());
            const auto maximum = static_cast<long double>(
                std::numeric_limits<Timestamp>::max());
            return static_cast<Timestamp>(std::max(minimum, std::min(maximum, value)));
        };
        if (mOffsets.size() >= 3) {
            const auto secondStep = distance(mOffsets[2].timestamp, mOffsets[1].timestamp);
            if (secondStep > 0.0L &&
                distance(mOffsets[1].timestamp, mOffsets[0].timestamp) > outlierRatio * secondStep)
                mOffsets[0].timestamp = clampedTimestamp(
                    2.0L * static_cast<long double>(mOffsets[1].timestamp) -
                    static_cast<long double>(mOffsets[2].timestamp));

            for (size_t index = 1; index + 1 < mOffsets.size(); ++index) {
                const Timestamp previous = mOffsets[index - 1].timestamp;
                const Timestamp current = mOffsets[index].timestamp;
                const Timestamp next = mOffsets[index + 1].timestamp;
                if (next <= previous) continue;
                const long double localStep = distance(next, previous) / 2.0L;
                if (localStep > 0.0L &&
                    distance(current, previous) > outlierRatio * localStep &&
                    distance(next, current) > outlierRatio * localStep)
                    mOffsets[index].timestamp = clampedTimestamp(
                        (static_cast<long double>(previous) +
                         static_cast<long double>(next)) / 2.0L);
            }

            const size_t last = mOffsets.size() - 1;
            const auto previousStep = distance(
                mOffsets[last - 1].timestamp, mOffsets[last - 2].timestamp);
            if (previousStep > 0.0L &&
                distance(mOffsets[last].timestamp, mOffsets[last - 1].timestamp) >
                    outlierRatio * previousStep)
                mOffsets[last].timestamp = clampedTimestamp(
                    2.0L * static_cast<long double>(mOffsets[last - 1].timestamp) -
                    static_cast<long double>(mOffsets[last - 2].timestamp));
        }
        
        mFrameList.clear();
        mFrameOffsetMap.clear();

        // Timestamp is the public frame key, so equal repaired or source
        // timestamps must not collapse multiple physical frames into one map
        // entry. A one-nanosecond nudge is immaterial to timing but preserves
        // a distinct lookup key for every frame.
        std::set<Timestamp> usedTimestamps;
        for(auto& i : mOffsets) {
            if(usedTimestamps.count(i.timestamp)) {
                if(i.timestamp == std::numeric_limits<Timestamp>::max()) {
                    do { --i.timestamp; } while(usedTimestamps.count(i.timestamp));
                } else {
                    do { ++i.timestamp; } while(usedTimestamps.count(i.timestamp));
                }
            }
            usedTimestamps.insert(i.timestamp);
            mFrameList.push_back(i.timestamp);
            mFrameOffsetMap.insert({ i.timestamp, i });
        }
    }
    
    void Decoder::readExtra() {
        const auto finalFrame = std::max_element(mOffsets.begin(), mOffsets.end(), [](const auto& a, const auto& b) {
            return a.offset < b.offset;
        });
        const int64_t curOffset = finalFrame == mOffsets.end() ? mBufferStartOffset : finalFrame->offset;

        if(FSEEK(mFile, curOffset, SEEK_SET) != 0)
            return;

        while(true) {
            Item item{};

            if(std::fread(&item, sizeof(Item), 1, mFile) != 1)
                break;
            
            // Skip things we don't need
            if(item.type == Type::BUFFER || item.type == Type::METADATA || item.type == Type::AUDIO_DATA
                || item.type == Type::AUDIO_DATA_METADATA || item.type == Type::AUDIO_DATA_F32
                || item.type == Type::GYRO_DATA) {
                if(FSEEK(mFile, item.size, SEEK_CUR) != 0)
                    break;
            }
            else if(item.type == Type::AUDIO_INDEX) {
                AudioIndex index{};
                
                read(&index, sizeof(AudioIndex));

                // Read all audio offsets
                mAudioOffsets.resize(index.numOffsets);
                
                read(mAudioOffsets.data(), sizeof(BufferOffset), mAudioOffsets.size());
            }
            else if(item.type == Type::GYRO_INDEX) {
                readGyroIndex(item.size);
            }
            else if(item.type == Type::ACCELEROMETER_INDEX) {
                readAccelerometerIndex(item.size);
            }
            else if(item.type == Type::ACCELEROMETER_DATA || item.type == Type::OIS_DATA
                || item.type == Type::OIS_INDEX) {
                // OIS items can precede the accelerometer index in the tail.
                // Skip their payloads without interpreting or allocating them.
                if(!payloadFitsInFile(item.size) || FSEEK(mFile, item.size, SEEK_CUR) != 0)
                    throw IOException("Invalid motion data payload");
            }
            else {
                break;
            }
        }
    }

    void Decoder::readGyroIndex(const uint32_t itemSize) {
        if(itemSize < sizeof(GyroIndex) || !payloadFitsInFile(itemSize))
            throw IOException("Invalid gyro index");

        GyroIndex index{};
        read(&index, sizeof(GyroIndex));

        const uint64_t expectedSize = sizeof(GyroIndex)
            + static_cast<uint64_t>(index.numOffsets) * sizeof(BufferOffset);
        const uint64_t maxOffsets = static_cast<uint64_t>(mOffsets.size()) + 1;
        if(index.version != GYRO_INDEX_VERSION || expectedSize != itemSize || index.numOffsets > maxOffsets)
            throw IOException("Invalid gyro index");

        mGyroOffsets.resize(index.numOffsets);
        read(mGyroOffsets.data(), sizeof(BufferOffset), mGyroOffsets.size());
    }

    void Decoder::readAccelerometerIndex(const uint32_t itemSize) {
        if(itemSize < sizeof(AccelerometerIndex) || !payloadFitsInFile(itemSize))
            throw IOException("Invalid accelerometer index");

        AccelerometerIndex index{};
        read(&index, sizeof(AccelerometerIndex));

        const uint64_t expectedSize = sizeof(AccelerometerIndex)
            + static_cast<uint64_t>(index.numOffsets) * sizeof(BufferOffset);
        const uint64_t maxOffsets = static_cast<uint64_t>(mOffsets.size()) + 1;
        if(index.version != ACCELEROMETER_INDEX_VERSION || expectedSize != itemSize || index.numOffsets > maxOffsets)
            throw IOException("Invalid accelerometer index");

        mAccelerometerOffsets.resize(index.numOffsets);
        read(mAccelerometerOffsets.data(), sizeof(BufferOffset), mAccelerometerOffsets.size());
    }

    bool Decoder::payloadFitsInFile(const uint32_t itemSize) const {
        const int64_t payloadOffset = FTELL(mFile);
        if(payloadOffset < 0 || FSEEK(mFile, 0, SEEK_END) != 0)
            return false;

        const int64_t fileSize = FTELL(mFile);
        const bool restored = FSEEK(mFile, payloadOffset, SEEK_SET) == 0;
        return restored && fileSize >= payloadOffset
            && static_cast<uint64_t>(itemSize) <= static_cast<uint64_t>(fileSize - payloadOffset);
    }
    
    void Decoder::read(void* data, size_t size, size_t items) const {
        ::motioncam::read(mFile, data, size, items);
    }

} // namespace motioncam
