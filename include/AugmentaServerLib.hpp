#pragma once

#include <vector>
#include <memory>
#include <string>
#include <array>
#include <optional>

namespace AugmentaServerProtocol
{
    enum class ClusterState : int
    {
        // TODO: Add doc !
    };

    namespace ControlChannel
    {
        class Handshake
        {
        };

        class HandskaheResponse
        {
        };

        class Update
        {
        };
    }

    class DataBlob
    {
    public:
        class SceneInfoPacket
        {
        public:
            int getAddressLength() const { return addressLength; }

            template <typename CharT>
            void getAddress(CharT *outStr) const
            {
                static_assert(sizeof(CharT) == 4);
                std::memcpy(outStr, addressPtr, sizeof(CharT));
            }

        private:
            friend struct DataBlob;

            int addressLength;
            const std::byte *addressPtr = nullptr;
        };

        class ClusterProperty
        {
        public:
            ClusterState getState() const { return state; }

            template <typename Vector3f>
            void getCentroid(Vector3f *outCentroid) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(outCentroid, centroid.data(), sizeof(Vector3f));
            }

            template <typename Vector3f>
            void getVelocity(Vector3f *outVelocity) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(outVelocity, velocity.data(), sizeof(Vector3f));
            }

            template <typename Vector3f>
            void getBoundingBoxCenter(Vector3f *outBoundingBoxCenter) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(outBoundingBoxCenter, boundingBoxCenter.data(), sizeof(Vector3f));
            }

            template <typename Vector3f>
            void getBoundingBoxSize(Vector3f *out) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(out, boundingBoxSize.data(), sizeof(Vector3f));
            }

            template <typename Vector3f>
            void getWeight(Vector3f *out) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(out, weight.data(), sizeof(Vector3f));
            }

            template <typename Vector3f>
            void getRotationEuler(Vector3f *out) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(out, rotation.data(), sizeof(Vector3f))
            }

            template <typename Vector4f>
            void getRotationQuat(Vector4f *out) const
            {
                static_assert(sizeof(Vector4f) == 16);
                std::memcpy(out, rotation.data(), sizeof(Vector4f))
            }

            template <typename Vector3f>
            void getLookAt(Vector3f *out) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(out, lookAt.data(), sizeof(Vector3f));
            }

        private:
            friend struct DataBlob;

            ClusterState state;
            std::array<float, 3> centroid;
            std::array<float, 3> velocity;
            std::array<float, 3> boundingBoxCenter;
            std::array<float, 3> boundingBoxSize;
            std::array<float, 3> weight;

            std::array<float, 4> rotation;

            std::array<float, 3> lookAt;
        };

        class PointCloudProperty
        {
        public:
            int getPointCount() const { return pointsCount; }

            template <typename Vector3f>
            void getPointsData(Vector3f *outData) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(outData, pointsPtr, pointsCount * sizeof(Vector3f));
            }

        private:
            friend struct DataBlob;

            int pointsCount;
            const std::byte *pointsPtr;
        };

        class ObjectPacket
        {
        public:
            bool hasCluster() const { return cluster.has_value(); }
            const ClusterProperty &getCluster() const { return cluster.value(); }

            bool hasPointCloud() const { return pointCloud.has_value(); }
            const PointCloudProperty &getPointCloud() const { return pointCloud.value(); }

            int getID() const { return id; }

        private:
            friend struct DataBlob;

            int id;
            std::optional<ClusterProperty> cluster;
            std::optional<PointCloudProperty> pointCloud;
        };

        // TODO
        class ZonePacket
        {
        public:
        private:
            friend struct DataBlob;
        };

        size_t getObjectCount() const { return objects.size(); }
        const std::vector<ObjectPacket> &getObjects() const { return objects; }

        size_t getZoneCount() const { return zones.size(); }
        const std::vector<ZonePacket> &getZones() const { return zones; }

        const SceneInfoPacket &getSceneInfo() const { return sceneInfo; }

        static DataBlob parse(const std::byte *buffer, size_t bufferSize);

    private:
        DataBlob(const std::byte *blob, size_t blobSize);

        SceneInfoPacket sceneInfo;
        std::vector<ObjectPacket> objects;
        std::vector<ZonePacket> zones;

        size_t processPacket(const std::byte *packetBegin, size_t packetSize);

        size_t processObjectPacket(const std::byte *packetBegin, ObjectPacket &outObject);
        size_t processZonePacket(const std::byte *dataBegin, ZonePacket &outZone);
        size_t processScenePacket(const std::byte *packetBegin, SceneInfoPacket &outZone);

        size_t processPointCloudProperty(const std::byte *pointCloudBegin, ObjectPacket &object);
        size_t processClusterProperty(const std::byte *clusterBegin, ObjectPacket &object);
    };

    class ControlMessageParser
    {
    public:
        const std::string *message;

    private:
        ControlMessageParser(const std::string &inMessage);
    };
}