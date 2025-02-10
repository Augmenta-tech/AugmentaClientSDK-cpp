#pragma once

#include <vector>
#include <memory>
#include <string>
#include <array>
#include <optional>
#include <variant>
#include <cassert>

namespace AugmentaServerProtocol
{
    struct ProtocolOptions
    {
        enum class RotationMode
        {
            Radians,
            Degrees,
            Quaternions,
        };

        enum class AxisTransformMode
        {
            // TODO
        };

        int version = 2;
        int downSample = 1;
        bool streamClouds = true;
        bool streamClusters = true;
        bool streamClusterPoints = true;
        bool streamZonePoints = false;
        RotationMode boxRotationMode = RotationMode::Quaternions;
        AxisTransformMode axisTransformMode; // TODO: Default ?
        bool useCompression = true;
        bool usePolling = false;
    };

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
            Vector3f getCentroid() const
            {
                static_assert(sizeof(Vector3f) == 12);

                Vector3f outValue;
                std::memcpy(&outValue, centroid.data(), sizeof(Vector3f));
                return outValue;
            }

            template <typename Vector3f>
            Vector3f getVelocity() const
            {
                static_assert(sizeof(Vector3f) == 12);

                Vector3f outValue;
                std::memcpy(&outValue, velocity.data(), sizeof(Vector3f));
                return outValue;
            }

            template <typename Vector3f>
            Vector3f getBoundingBoxCenter() const
            {
                static_assert(sizeof(Vector3f) == 12);

                Vector3f outValue;
                std::memcpy(&outValue, boundingBoxCenter.data(), sizeof(Vector3f));
                return outValue;
            }

            template <typename Vector3f>
            Vector3f getBoundingBoxSize() const
            {
                static_assert(sizeof(Vector3f) == 12);

                Vector3f outValue;
                std::memcpy(&outValue, boundingBoxSize.data(), sizeof(Vector3f));
                return outValue;
            }

            float getWeight() const { return weight; }

            template <typename Vector3f>
            Vector3f getBoundingBoxRotationEuler() const
            {
                static_assert(sizeof(Vector3f) == 12);

                Vector3f outValue;
                std::memcpy(&outValue, boundingBoxRotation.data(), sizeof(Vector3f));
                return outValue;
            }

            template <typename Vector4f>
            Vector4f getBoundingBoxRotationQuaternions() const
            {
                static_assert(sizeof(Vector4f) == 16);

                Vector4f outValue;
                std::memcpy(&outValue, boundingBoxRotation.data(), sizeof(Vector4f));
                return outValue;
            }

            template <typename Vector3f>
            Vector3f getLookAt() const
            {
                static_assert(sizeof(Vector3f) == 12);
                Vector3f outValue;
                std::memcpy(&outValue, lookAt.data(), sizeof(Vector3f));
                return outValue;
            }

        private:
            ClusterState state;
            std::array<float, 3> centroid;
            std::array<float, 3> velocity;
            std::array<float, 3> boundingBoxCenter;
            std::array<float, 3> boundingBoxSize;
            float weight;

            std::array<float, 4> boundingBoxRotation;

            std::array<float, 3> lookAt;
        };

        class PointCloudProperty
        {
            friend class DataBlobParser;

        public:
            int getPointCount() const { return pointsCount; }

            // Copy the all cloud points at once. outData should be a contiguous container of Vector3f type (3*float = 12 bytes)
            template <typename Vector3f>
            void getPointsData(Vector3f *outData) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(outData, pointsPtr, pointsCount * sizeof(Vector3f));
            }

            // Return the point (Vec3f) at a given idx. Prefer copying all point data at once if you can.
            template <typename Vector3f>
            Vector3f getPoint(size_t pointIdx) const
            {
                static_assert(sizeof(Vector3f) == 12);

                Vector3f outPoint;
                std::memcpy(&outPoint, pointsPtr + (pointIdx * sizeof(Vector3f)), 3);
                return outPoint;
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

        class ZonePacket
        {
            friend class DataBlobParser;

        public:
            class SliderProperty
            {
                friend class DataBlobParser;

            public:
                float getValue() const { return value; }

            private:
                float value;
            };

            class XYPadProperty
            {
                friend class DataBlobParser;

            public:
                float getX() const { return x; }
                float getY() const { return y; }

            private:
                float x;
                float y;
            };

            class Property
            {
                friend class DataBlobParser;

            public:
                enum class Type : int
                {
                    Unknown = -1,
                    Slider = 0,
                    XYPad = 1,
                    PointCloud = 2,

                };

            public:
                Type getType() const { return type; }

                template <typename T>
                const T *getExtraData() const { return std::get_if<T>(&data); }

            private:
                Type type = Type::Unknown;
                std::variant<SliderProperty, XYPadProperty, PointCloudProperty> data;
            };

        public:
            const std::string &getControlID() const { return controlID; }
            uint8_t getEnters() const { return enters; }
            uint8_t getLeaves() const { return leaves; }
            int getPresence() const { return presence; }
            float getDensity() const { return density; }
            int getPropertiesCount() const { return properties.size(); }
            const std::vector<Property>& getProperties() const { return properties; }

        private:
            std::string controlID;
            uint8_t enters;
            uint8_t leaves;
            int presence;
            float density;
            std::vector<Property> properties;
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
        enum class Status
        {
            Unknown,
            Ok,
            Error,
        };

        enum class Type
        {
            Unknown,
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

            struct BoxShapeParameters
            {
                std::array<float, 3> size;
            };

            struct CylinderShapeParameters
            {
                float radius;
                float height;
            };

            struct SphereShapeParameters
            {
                float radius;
            };

            struct PathShapeParameters
            {
            };

            struct GridShapeParameters
            {
            };

            struct PolygonShapeParameters
            {
            };

            struct SegmentShapeParameters
            {
            };

            enum class ShapeType
            {
                Unknown,
                Box,
                Cylinder,
                Sphere,
                Path,
                Grid,
                Polygon,
                Segment,
            };

            struct ZoneParameters
            {
                ShapeType shapeType = ShapeType::Unknown;
                std::variant<BoxShapeParameters,
                             CylinderShapeParameters,
                             SphereShapeParameters,
                             PathShapeParameters,
                             GridShapeParameters,
                             PolygonShapeParameters,
                             SegmentShapeParameters>
                    shapeParameters;

                ShapeType getShapeType() const { return shapeType; }

                const BoxShapeParameters *getBoxShapeParameters() const { return getShapeParameters<BoxShapeParameters>(); }
                const CylinderShapeParameters *getCylinderShapeParameters() const { return getShapeParameters<CylinderShapeParameters>(); }
                const SphereShapeParameters *getSphereShapeParameters() const { return getShapeParameters<SphereShapeParameters>(); }
                const PathShapeParameters *getPathShapeParameters() const { return getShapeParameters<PathShapeParameters>(); }
                const GridShapeParameters *getGridShapeParameters() const { return getShapeParameters<GridShapeParameters>(); }
                const PolygonShapeParameters *getPolygonShapeParameters() const { return getShapeParameters<PolygonShapeParameters>(); }
                const SegmentShapeParameters *getSegmentShapeParameters() const { return getShapeParameters<SegmentShapeParameters>(); }

            private:
                template <typename T>
                const T *getShapeParameters() const { return std::get_if<T>(&shapeParameters); }
            };

            struct SceneParameters
            {
                std::array<float, 3> size;
            };

            struct ContainerParameters
            {
            };

            bool isZone() const { return type == Container::Type::Zone; }
            bool isScene() const { return type == Container::Type::Scene; }
            bool isContainer() const { return type == Container::Type::Container; }
            bool hasChilren() { return !children.empty(); };

            const std::string &getName() const { return name; }
            const std::string &getAddress() const { return address; }
            const std::array<float, 3> &getPosition() const { return position; };
            const std::array<float, 3> &getRotation() const { return rotation; };
            const std::array<float, 4> &getColor() const { return color; };

            const SceneParameters *getSceneParameters() const { return getParameters<SceneParameters>(); }
            const ZoneParameters *getZoneParameters() const { return getParameters<ZoneParameters>(); }
            const ContainerParameters *getContainerParameters() const { return getParameters<ContainerParameters>(); }

        private:
            Container::Type type = Type::Unknown;

            // Common object properties
            std::vector<Container> children;
            std::string name;
            std::string address;
            std::array<float, 3> position;
            std::array<float, 3> rotation;
            std::array<float, 4> color;

            // Per-type parameters
            std::variant<SceneParameters, ZoneParameters, ContainerParameters> parameters;

            template <typename T>
            const T *getParameters() const { return std::get_if<T>(&parameters); }
        };

        bool isUpdate() const { return type == Type::Update; };
        bool isSetup() const { return type == Type::Setup; };

        Status getStatus() const { return status; }
        const std::string &getErrorMessage() const { return errorMessage; }
        int getServerProtocolVersion() const { return serverProtocolVersion; }
        const Container &getRootObject() const { return rootObject; }

    private:
        Type type = Type::Unknown;
        Container rootObject;

        Status status = Status::Unknown;
        std::string errorMessage;
        int serverProtocolVersion = 2;
    };

    class Client
    {
    public:
        void initialize(const ProtocolOptions &options);

        std::string getHandShakeMessage() const;
        std::string getPollMessage() const;

        DataBlob parseDataBlob(const std::byte *blob, size_t blobSize);
        ControlMessage parseControlMessage(const char *rawMessage);

    private:
        bool initialized = false;

        std::vector<std::string> tags;
        ProtocolOptions options;

        std::vector<std::byte> uncompressedBuffer;
    };
}