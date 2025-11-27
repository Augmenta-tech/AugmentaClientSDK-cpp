#pragma once

#include <vector>
#include <memory>
#include <string>
#include <array>
#include <optional>
#include <variant>
#include <cassert>
#include <cstring>

namespace Augmenta
{
    struct ProtocolOptions
    {
        enum class RotationMode
        {
            Radians,
            Degrees,
            Quaternions,
        };

        struct AxisTransform
        {
            enum class AxisMode
            {
                ZUpRightHanded,
                ZUpLeftHanded,
                YUpRightHanded,
                YUpLeftHanded,
            };

            enum class OriginMode
            {
                BottomLeft,
                BottomRight,
                TopLeft,
                TopRight,
            };

            enum class CoordinateSpace
            {
                Absolute,
                Relative,
                Normalized,
            };

            AxisMode axis = AxisMode::ZUpRightHanded;
            OriginMode origin = OriginMode::BottomLeft;
            bool flipX = false;
            bool flipY = false;
            bool flipZ = false;
            CoordinateSpace coordinateSpace = CoordinateSpace::Absolute;
            // public originOffset; // TODO
            // public customMatrix; // TODO

            bool operator==(const AxisTransform& other) const;
            bool operator!=(const AxisTransform& other) const;
        };

        int version = 2;
        std::vector<std::string> tags;
        int downSample = 1;
        bool streamClouds = true;
        bool streamClusters = true;
        bool streamClusterPoints = true;
		bool streamSkeletonData = true;
		bool streamMetadata = true;
        bool streamZonePoints = false;
        RotationMode boxRotationMode = RotationMode::Quaternions;
        AxisTransform axisTransform; // TODO: Default ?
        bool useCompression = true;
        bool usePolling = false;
        bool displayPointIntensity = false;

        bool operator==(const ProtocolOptions& other) const;
        bool operator!=(const ProtocolOptions& other) const;
    };

    enum class ClusterState : int
    {
        Entered = 0,
        Updated = 1,
        WillLeave = 2,
        Ghost = 3,
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
            /// @brief The state of the cluster can be used to react to important events such as entrance/leave.
            ClusterState getState() const { return state; }

            /// @brief The centroid of the pointcloud from which this cluster was created. 
            /// Note that it can be different from the boundingBoxCenter !
            template <typename Vector3f>
            Vector3f getCentroid() const
            {
                static_assert(sizeof(Vector3f) == 12);

                Vector3f outValue;
                std::memcpy(&outValue, centroid.data(), sizeof(Vector3f));
                return outValue;
            }

            /// @brief Cluster velocity between this frame and the previous.
            template <typename Vector3f>
            Vector3f getVelocity() const
            {
                static_assert(sizeof(Vector3f) == 12);

                Vector3f outValue;
                std::memcpy(&outValue, velocity.data(), sizeof(Vector3f));
                return outValue;
            }

            /// @brief The center of the cluster bounding box. In most cases, this is what you'll want to use to place a boint right below a person.
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

            /// @brief This is a factor of confidence in the cluster that can vary between [0, 1] over time.
            /// This is mostly useful for advanced uses such as merging clusters coming from multiple Augmenta servers.
            float getWeight() const { return weight; }

            template <typename Vector3f>
            Vector3f getBoundingBoxRotationEuler() const
            {
                static_assert(sizeof(Vector3f) == 12);

                Vector3f outValue;
                std::memcpy(&outValue, boundingBoxRotation.data(), sizeof(Vector3f));
                return outValue;
            }

            /// @brief Returns the bounding box rotation in quaternion. The rotation mode must be set to quaternions.
            /// @warning The memory layout of the output type is expected to be x,y,z,w !
            /// If your type uses another layout, you can use the getBoundingBoRotationQuaterionX/Y/Z/W methods to get individual components.
            /// @tparam Vector4f: Quaternion float type (x,y,z,w)
            template <typename Vector4f>
            Vector4f getBoundingBoxRotationQuaternions() const
            {
                static_assert(sizeof(Vector4f) == 16);

                Vector4f outValue;
                std::memcpy(&outValue, boundingBoxRotation.data(), sizeof(Vector4f));
                return outValue;
            }

            /// @brief Returns the X component of the bounding box rotation quaternion. Rotation mode must be set to quaternions.
            float getBoundingBoxRotationQuaternionX() const
            {
                float outValue;
                std::memcpy(&outValue, boundingBoxRotation.data(), sizeof(float));
                return outValue;
            }

            /// @brief Returns the Y component of the bounding box rotation quaternion. Rotation mode must be set to quaternions.
            float getBoundingBoxRotationQuaternionY() const
            {
                float outValue;
                std::memcpy(&outValue, boundingBoxRotation.data() + 1, sizeof(float));
                return outValue;
            }

            /// @brief Returns the Z component of the bounding box rotation quaternion. Rotation mode must be set to quaternions.
            float getBoundingBoxRotationQuaternionZ() const
            {
                float outValue;
                std::memcpy(&outValue, boundingBoxRotation.data() + 2, sizeof(float));
                return outValue;
            }

            /// @brief Returns the W component of the bounding box rotation quaternion. Rotation mode must be set to quaternions.
            float getBoundingBoxRotationQuaternionW() const
            {
                float outValue;
                std::memcpy(&outValue, boundingBoxRotation.data() + 3, sizeof(float));
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

            /// @brief Copy all the cloud points at once. outData should be a contiguous container of Vector3f type (3*float = 12 bytes)
            template <typename Vector3f>
            void getPointsData(Vector3f *outData) const
            {
                static_assert(sizeof(Vector3f) == 12);
                std::memcpy(outData, pointsPtr, pointsCount * sizeof(Vector3f));
            }

            /// @brief Return the point (Vec3f) at a given idx. Prefer copying all point data at once if you can.
            template <typename Vector3f>
            Vector3f getPoint(size_t pointIdx) const
            {
                static_assert(sizeof(Vector3f) == 12);

                Vector3f outPoint;
                std::memcpy(&outPoint, pointsPtr + (pointIdx * sizeof(Vector3f)), sizeof(Vector3f));
                return outPoint;
            }

        private:
            int pointsCount;
            const std::byte *pointsPtr = nullptr;
        };

        /// @brief Can represent clusters and point clouds. You can consider 3 cases:
        /// - object has a point cloud but no cluster: it is a raw point cloud, most likely covering the whole scene
        /// - object has a cluster but no point cloud: it is a simple cluster
        /// - object has a cluster and a point cloud: it is a cluster with an associated point cloud 
        class ObjectPacket
        {
            friend class DataBlobParser;

        public:
            bool hasCluster() const { return cluster.has_value(); }
            bool hasPointCloud() const { return pointCloud.has_value(); }

            const ClusterProperty &getCluster() const
            {
                assert(hasCluster());
                return cluster.value();
            }

            const PointCloudProperty &getPointCloud() const
            {
                assert(hasPointCloud());
                return pointCloud.value();
            }

            int getID() const { return id; }

        private:
            int id;
            std::optional<ClusterProperty> cluster;
            std::optional<PointCloudProperty> pointCloud;
        };

        /// @brief Zone Events will be emitted by a Zone to notify users of change. They can be linked to the zone that emitted them.
        class ZoneEventPacket
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
                enum class Type : uint8_t
                {
                    Unknown = UINT8_MAX,
                    Slider = 0,
                    XYPad = 1,
                    PointCloud = 2,
                };

            public:
                Type getType() const { return type; }
                bool isSlider() const { return type == Type::Slider; }
                bool isXYPad() const { return type == Type::XYPad; }
                bool isPointCloud() const { return type == Type::PointCloud; }

                const SliderProperty *getSliderParameters() const
                {
                    assert(isSlider());
                    return getExtraData<SliderProperty>();
                }

                const XYPadProperty *getXYPadParameters() const
                {
                    assert(isXYPad());
                    return getExtraData<XYPadProperty>();
                }

                const PointCloudProperty *getPointCloudParameters() const
                {
                    assert(isPointCloud());
                    return getExtraData<PointCloudProperty>();
                }

            private:
                template <typename T>
                const T *getExtraData() const { return std::get_if<T>(&data); }

                Type type = Type::Unknown;
                std::variant<SliderProperty, XYPadProperty, PointCloudProperty> data;
            };

        public:
            [[deprecated("controlID is deprecated. Use getEmitterZoneAddress() instead")]]
            const std::string &getControlID() const { return getEmitterZoneAddress(); }
            
            /// @brief Return the address of the zone that emitted this event. You can use
            /// that to match the event with the zone container received at setup time.
            const std::string &getEmitterZoneAddress() const { return emitterZoneAddress; }

            /// @brief Number of clusters that entered the zone this frame
            uint8_t getEnters() const { return enters; }

            /// @brief Number of clusters that left the zone this frame
            uint8_t getLeaves() const { return leaves; }

            /// @brief Number of cluster present in the zone this frame
            int getPresence() const { return presence; }

            float getDensity() const { return density; }
            
            int getPropertiesCount() const { return properties.size(); }
            const std::vector<Property> &getProperties() const { return properties; }

        private:
            std::string emitterZoneAddress;
            uint8_t enters;
            uint8_t leaves;
            int presence;
            float density;
            std::vector<Property> properties;
        };

        [[deprecated("ZonePacket is deprecated. Please use ZoneEventPacket instead.")]]
        typedef ZoneEventPacket ZonePacket;

        size_t getObjectCount() const { return objects.size(); }
        const std::vector<ObjectPacket> &getObjects() const { return objects; }

        [[deprecated("DataBlob::getZoneCount() has been renamed, please use DataBlob::getZoneEventCount() instead.")]]
        size_t getZoneCount() const { return getZoneEventCount(); }
        size_t getZoneEventCount() const { return zoneEvents.size(); }

        [[deprecated("DataBlob::getZones() has been renamed, please use DataBlob::getZoneEvents() instead.")]]
        const std::vector<ZoneEventPacket> &getZones() const { return getZoneEvents(); }
        const std::vector<ZoneEventPacket> &getZoneEvents() const { return zoneEvents; }
        
        const SceneInfoPacket &getSceneInfo() const { return sceneInfo; }

    private:
        SceneInfoPacket sceneInfo;
        std::vector<ObjectPacket> objects;
        std::vector<ZoneEventPacket> zoneEvents;
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

            class ZoneParameters
            {
                friend class ControlMessageParser;

            public:
                ShapeType getShapeType() const { return shapeType; }
                bool isBox() const { return shapeType == ShapeType::Box; }
                bool isCylinder() const { return shapeType == ShapeType::Cylinder; }
                bool isSphere() const { return shapeType == ShapeType::Sphere; }
                bool isPath() const { return shapeType == ShapeType::Path; }
                bool isGrid() const { return shapeType == ShapeType::Grid; }
                bool isPolygon() const { return shapeType == ShapeType::Polygon; }
                bool isSegment() const { return shapeType == ShapeType::Segment; }

                const BoxShapeParameters *getBoxShapeParameters() const
                {
                    assert(isBox());
                    return getShapeParameters<BoxShapeParameters>();
                }

                const CylinderShapeParameters *getCylinderShapeParameters() const
                {
                    assert(isCylinder());
                    return getShapeParameters<CylinderShapeParameters>();
                }

                const SphereShapeParameters *getSphereShapeParameters() const
                {
                    assert(isSphere());
                    return getShapeParameters<SphereShapeParameters>();
                }

                const PathShapeParameters *getPathShapeParameters() const
                {
                    assert(isPath());
                    return getShapeParameters<PathShapeParameters>();
                }

                const GridShapeParameters *getGridShapeParameters() const
                {
                    assert(isGrid());
                    return getShapeParameters<GridShapeParameters>();
                }

                const PolygonShapeParameters *getPolygonShapeParameters() const
                {
                    assert(isPolygon());
                    return getShapeParameters<PolygonShapeParameters>();
                }

                const SegmentShapeParameters *getSegmentShapeParameters() const
                {
                    assert(isSegment());
                    return getShapeParameters<SegmentShapeParameters>();
                }

            private:
                template <typename T>
                const T *getShapeParameters() const { return std::get_if<T>(&shapeParameters); }

                ShapeType shapeType = ShapeType::Unknown;
                std::variant<BoxShapeParameters,
                             CylinderShapeParameters,
                             SphereShapeParameters,
                             PathShapeParameters,
                             GridShapeParameters,
                             PolygonShapeParameters,
                             SegmentShapeParameters>
                    shapeParameters;
            };

            struct SceneParameters
            {
                std::array<float, 3> size;
            };

            struct ContainerParameters
            {
            };

            Type getType() const { return type; }
            bool isZone() const { return type == Container::Type::Zone; }
            bool isScene() const { return type == Container::Type::Scene; }
            bool isContainer() const { return type == Container::Type::Container; }
            bool hasChilren() const { return !children.empty(); };
            const std::vector<Container> &getChildren() const { return children; }
            const std::string &getName() const { return name; }
            const std::string &getAddress() const { return address; }
            const std::array<float, 3> &getPosition() const { return position; };
            const std::array<float, 3> &getRotation() const { return rotation; };
            const std::array<float, 4> &getColor() const { return color; };

            const SceneParameters *getSceneParameters() const
            {
                assert(isScene());
                return getParameters<SceneParameters>();
            }

            const ZoneParameters *getZoneParameters() const
            {
                assert(isZone());
                return getParameters<ZoneParameters>();
            }

            const ContainerParameters *getContainerParameters() const
            {
                assert(isContainer());
                return getParameters<ContainerParameters>();
            }

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
        void initialize(const std::string &clientName, const ProtocolOptions &options);
        void shutdown();

        /// @brief Clear requested tags list
        void clearTags();

        /// @brief Add a server-side tag to request.  
        void addTag(const std::string& tag);

        std::string getRegisterMessage() const;
        std::string getPollMessage() const;

        /// @brief Parse a data blob and return a DataBlob object that can be used to query relevant information from it.
        /// @warning The DataBlob keeps references to the parsed buffer. Users are responsible for keeping the buffer available as long as they are using the DataBlob.  
        DataBlob parseDataBlob(const std::byte *blob, size_t blobSize);
        ControlMessage parseControlMessage(const char *rawMessage);

    private:
        std::string name;
        ProtocolOptions options;
        std::vector<std::string> tags;

        bool initialized = false;

        std::vector<std::byte> uncompressedBuffer;
    };
}