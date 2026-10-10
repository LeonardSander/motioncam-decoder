/*
 * Copyright 2023 MotionCam
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef Decoder_hpp
#define Decoder_hpp

#include <motioncam/Container.hpp>
#include <motioncam/MotionSample.hpp>
#include <nlohmann/json.hpp>

#include <string>
#include <vector>
#include <map>

namespace motioncam {
    typedef int64_t Timestamp;
    
    // Audio sample format enum
    enum class AudioSampleFormat {
        Int16,
        Float32
    };
    
    // Audio chunk with variant data for int16 or float32 samples
    struct AudioChunk {
        Timestamp timestamp;
        AudioSampleFormat format;
        std::vector<int16_t> int16Data;
        std::vector<float> float32Data;
        
        AudioChunk() : timestamp(-1), format(AudioSampleFormat::Int16) {}
        
        size_t sampleCount() const {
            return format == AudioSampleFormat::Float32 ? float32Data.size() : int16Data.size();
        }
    };

    class MotionCamException : public std::runtime_error {
    public:
        MotionCamException(const std::string& error) : runtime_error(error) {}
    };
    
    class IOException : public MotionCamException {
    public:
        IOException(const std::string& error) : MotionCamException(error) {}
    };

    class AudioChunkLoader {
        public:
            virtual ~AudioChunkLoader() = default;
            virtual bool next(AudioChunk& output) = 0;
    };
    
    class Decoder {
    public:
        Decoder(const std::string& path);
        Decoder(FILE* file);

        ~Decoder();

        // Get container metadata
        const nlohmann::json& getContainerMetadata() const;
        
        // Get all frame timestamps in container
        const std::vector<Timestamp>& getFrames() const;
        
        // Load a single frame and its metadata.
        void loadFrame(const Timestamp timestamp, std::vector<uint8_t>& outData,
                       nlohmann::json& outMetadata, int widthOverride = 0);

        // Load metadata of a single frame.
        void loadFrameMetadata(const Timestamp timestamp, nlohmann::json& outMetadata);
        
        // Audio sample rate
        int audioSampleRateHz() const;
        
        // Number of channels in audio
        int numAudioChannels() const;
        
        // Load all audio chunks.
        void loadAudio(std::vector<AudioChunk>& outAudioChunks);
        
        // Load audio in chunks
        AudioChunkLoader& loadAudio() const;

        // Whether the container has embedded raw gyro samples.
        bool hasGyroData() const;

        // Load all raw gyro samples. Timestamps use the same nanosecond timeline
        // as frame timestamps; axis values are radians per second.
        void loadGyroData(std::vector<MotionSample>& outGyroSamples);

        // Whether the container has embedded accelerometer samples.
        bool hasAccelerometerData() const;

        // Append all accelerometer samples in chunk order. Timestamps use the
        // frame nanosecond timeline; values are m/s^2 including gravity, retaining
        // the source platform's sensor axes/signs. No resampling or conversion.
        // Containers without accelerometer data leave outSamples unchanged.
        void loadAccelerometerData(std::vector<MotionSample>& outSamples);
        
    private:
        struct UnspektraEntry {
            uint64_t recipeOffset = 0;
            uint64_t rawOffset = 0;
        };
        void initUnspektra();
        nlohmann::json unspektraFrameMetadata(Timestamp timestamp) const;
        std::vector<uint8_t> unspektraRecord(uint64_t offset, size_t maxSize) const;
        void loadUnspektraFrame(Timestamp timestamp, std::vector<uint8_t>& outData,
                                nlohmann::json& outMetadata);
        void init();
        void read(void* data, size_t size, size_t items=1) const;
        void readIndex();
        void readGyroIndex(uint32_t itemSize);
        void readAccelerometerIndex(uint32_t itemSize);
        bool payloadFitsInFile(uint32_t itemSize) const;
        void reindexOffsets();
        void readExtra();
        void uncompress(const std::vector<uint8_t>& src, std::vector<uint8_t>& dst);
        
    private:
        FILE* mFile;
        int64_t mBufferStartOffset;
        std::vector<BufferOffset> mOffsets;
        std::vector<BufferOffset> mAudioOffsets;
        std::vector<BufferOffset> mGyroOffsets;
        std::vector<BufferOffset> mAccelerometerOffsets;
        std::map<Timestamp, BufferOffset> mFrameOffsetMap;
        std::vector<Timestamp> mFrameList;
        nlohmann::json mMetadata;
        std::vector<uint8_t> mTmpBuffer;
        std::unique_ptr<AudioChunkLoader> mAudioLoader;
        bool mUnspektra = false;
        std::map<Timestamp, UnspektraEntry> mUnspektraEntries;
    };
} // namespace motioncam

#endif /* Decoder_hpp */
