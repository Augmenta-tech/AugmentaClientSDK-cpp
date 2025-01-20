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

    enum class BoundingBoxRotationMode
    {
        Degrees,
        Radians,
        Quaternions,
    };

    class DataBlob
    {
        friend class DataBlobParser;

    public:
        class SceneInfoPacket
        {
            friend class DataBlobParser;

        public:
            int getAddressLength() const { return addressLength; }

            template <typename CharT>
            void getAddress(CharT *outStr) const
            {
                static_assert(sizeof(CharT) == 4);
                std::memcpy(outStr, addressPtr, sizeof(CharT));
            }

        private:
            int addressLength;
            const std::byte *addressPtr = nullptr;
        };

        class ClusterProperty
        {
            friend class DataBlobParser;

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
            void getBoundingBoxRotationEuler(Vector3f *out) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(out, rotation.data(), sizeof(Vector3f))
            }

            template <typename Vector4f>
            void getBoundingBoxRotationQuaternions(Vector4f *out) const
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
            ClusterState state;
            std::array<float, 3> centroid;
            std::array<float, 3> velocity;
            std::array<float, 3> boundingBoxCenter;
            std::array<float, 3> boundingBoxSize;
            std::array<float, 3> weight;

            std::array<float, 4> boundingBoxRotation;

            std::array<float, 3> lookAt;
        };

        class PointCloudProperty
        {
            friend class DataBlobParser;

        public:
            int getPointCount() const { return pointsCount; }

            template <typename Vector3f>
            void getPointsData(Vector3f *outData) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(outData, pointsPtr, pointsCount * sizeof(Vector3f));
            }

            float getPoint(size_t pointIdx, float *outPoint) const
            {
                std::memcpy(outPoint, pointsPtr + (pointIdx * sizeof(float) * 3), 3);
            }

        private:
            int pointsCount;
            const std::byte *pointsPtr;
        };

        class ObjectPacket
        {
            friend class DataBlobParser;

        public:
            bool hasCluster() const { return cluster.has_value(); }
            const ClusterProperty &getCluster() const { return cluster.value(); }

            bool hasPointCloud() const { return pointCloud.has_value(); }
            const PointCloudProperty &getPointCloud() const { return pointCloud.value(); }

            int getID() const { return id; }

        private:
            int id;
            std::optional<ClusterProperty> cluster;
            std::optional<PointCloudProperty> pointCloud;
        };

        // TODO
        class ZonePacket
        {
            friend class DataBlobParser;

        public:
        private:
        };

        size_t getObjectCount() const { return objects.size(); }
        const std::vector<ObjectPacket> &getObjects() const { return objects; }

        size_t getZoneCount() const { return zones.size(); }
        const std::vector<ZonePacket> &getZones() const { return zones; }

        const SceneInfoPacket &getSceneInfo() const { return sceneInfo; }

    private:
        SceneInfoPacket sceneInfo;
        std::vector<ObjectPacket> objects;
        std::vector<ZonePacket> zones;
    };

    class ControlMessage
    {
        friend class ControlMessageParser;

    public:
        enum class Type
        {
            None,
            Update,
            Setup,
        };

        class Container
        {
            friend class ControlMessageParser;

        public:
            enum class Type
            {
                Unknown,
                Container,
                Zone,
                Scene
            };

            bool isZone() const { return type == Container::Type::Zone; }
            bool isScene() const { return type == Container::Type::Scene; }

            bool hasChilren() { return !children.empty(); };

            const std::string &getName() const { return name; }
            const std::string &getAddress() const { return address; }
            const std::array<float, 3> &getPosition() const { return position; };
            const std::array<float, 3> &getRotation() const { return rotation; };
            const std::array<float, 4> &getColor() const { return color; };

            bool hasSizeProperty() const { return size.has_value(); }
            const std::array<float, 3> &getSizeProperty() const { return size.value(); }

        private:
            Container::Type type = Type::Unknown;

            std::vector<Container> children;

            std::string name;
            std::string address;
            std::array<float, 3> position;
            std::array<float, 3> rotation;
            std::array<float, 4> color;

            // Scene only
            std::optional<std::array<float, 3>> size;

            // TODO:
            // ShapeObject/Zone only
            // shape  "None", "Box", "Cylinder", "Sphere", "Path", "Grid", "Polygon", "Segment"
            // TODO: Per-shape parameters
        };

        bool isUpdate() { return type == Type::Update; };
        bool isSetup() { return type == Type::Setup; };

        const Container &getContainer() const { return container; }

    private:
        Type type = Type::None;
        Container container;
    };

    // TODO
    class Client
    {
    public:
        const char *getHandShakeMessage();

        // TODO: Might need to not be static
        // in order to retain settings and do stuff accordingly (for example, reading 3 or 4 values rotations)
        static DataBlob parseDataBlob(const std::byte *blob, size_t blobSize);
        static ControlMessage parseControlMessage(const std::string &rawMessage);

    private:
        int protocolVersion = 2;
        std::vector<std::string> tags;
        bool streamClouds;
        bool streamClusters;
        bool streamClusterPoints;
        int downSample;
        BoundingBoxRotationMode boundingBoxRotationMode;
        // TODO: Axis transform options
    };
}