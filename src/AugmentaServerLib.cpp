#pragma once

#include "AugmentaServerLib.hpp"

#include "json.hpp"

namespace
{
    // Read an int from a byte buffer. Returns the number of bytes read.
    template <typename T>
    static size_t ReadBinary(const std::byte *buffer, T *out)
    {
        std::memcpy(out, buffer, sizeof(T));
        return sizeof(T);
    }

    // Read a contiguous vector from a byte buffer. Returns the number of bytes read.
    // The output vector's memory should have been allocated beforehand.
    template <typename T>
    static size_t ReadVector(const std::byte *buffer, T *out, size_t elementCount)
    {
        std::memcpy(out, buffer, elementCount * sizeof(T));
        return elementCount * sizeof(T);
    }

    template <typename T>
    T ReadJSON(const nlohmann::json &json, const std::string &key, const T &defaultValue)
    {
        auto it = json.find(key);
        if (it == json.end())
        {
            return defaultValue;
        }
        return *it;
    }

    template <size_t ArraySize>
    std::array<float, ArraySize> ReadJSON(const nlohmann::json &json, const std::string &key, const std::array<float, 3> &defaultValue)
    {
        auto it = json.find(key);
        if (it == json.end())
        {
            return defaultValue;
        }

        std::array<float, ArraySize> value;
        for (size_t i = 0; i < ArraySize; ++i)
        {
            value[i] = json[i];
        }
        return value;
    }

    bool isBigEndian()
    {
        union
        {
            uint32_t i;
            char c[4];
        } bint = {0x01020304};

        return bint.c[0] == 1;
    }
}

namespace AugmentaServerProtocol
{
    struct DataBlobParser
    {
        enum class PacketType : uint8_t
        {
            Object = 0,
            Zone = 1,
            Scene = 2,
            Bundle = 255,
        };

        enum class PropertyType : int
        {
            Points = 0,
            Cluster = 1,
        };

        static size_t processPacket(const std::byte *packetBuffer, DataBlob &outDataBlob, const ProtocolOptions &options)
        {
            size_t offset = 0;

            int packetSize;
            offset += ReadBinary(packetBuffer, &packetSize);

            PacketType type;
            offset += ReadBinary(packetBuffer + offset, &type);

            switch (type)
            {
            case PacketType::Bundle:
            {
                int packetCount;
                offset += ReadBinary(packetBuffer + offset, &packetCount);

                for (int packetIdx = 0; packetIdx < packetCount; ++packetIdx)
                {
                    auto subPacketBegin = packetBuffer + offset;
                    offset += processPacket(subPacketBegin, outDataBlob, options);
                }
                break;
            }

            case PacketType::Object:
            {
                auto &object = outDataBlob.objects.emplace_back();
                offset += processObjectPacket(packetBuffer, object, options);
                break;
            }

            case PacketType::Zone:
            {
                auto &zone = outDataBlob.zones.emplace_back();
                offset += processZonePacket(packetBuffer, zone, options);
                break;
            }

            case PacketType::Scene:
            {
                offset += processScenePacket(packetBuffer + offset, outDataBlob.sceneInfo, options);
                break;
            }

            default:
            {
                assert(false); // Unexpected packet type !
            }
            }

            return offset;
        }

        static size_t processObjectPacket(const std::byte *packetBuffer, DataBlob::ObjectPacket &outObject, const ProtocolOptions &options)
        {
            // At this point packetSize and packetType have already been read
            size_t offset = 0;

            int objectID;
            offset += ReadBinary(packetBuffer + offset, &outObject.id);

            int propertiesCount;
            offset += ReadBinary(packetBuffer + offset, &propertiesCount);

            for (int propertyIdx = 0; propertyIdx < propertiesCount; ++propertyIdx)
            {
                int propertySize;
                offset += ReadBinary(packetBuffer + offset, &propertySize);

                PropertyType propertyID;
                offset += ReadBinary(packetBuffer + offset, &propertyID);

                switch (propertyID)
                {
                case PropertyType::Points:
                {
                    offset += processPointCloudProperty(packetBuffer + offset, outObject, options);
                    break;
                }

                case PropertyType::Cluster:
                {
                    offset += processClusterProperty(packetBuffer + offset, outObject, options);
                    break;
                }
                }
            }

            return offset;
        }

        static size_t processPointCloudProperty(const std::byte *buffer, DataBlob::ObjectPacket &object, const ProtocolOptions &options)
        {
            assert(!object.hasPointCloud());

            auto &pointCloud = object.pointCloud.emplace();

            // Size and ID have already been read
            size_t offset = 0;

            offset += ReadBinary(buffer + offset, &pointCloud.pointsCount);
            pointCloud.pointsPtr = buffer + offset;

            return offset + (pointCloud.pointsCount * sizeof(float) * 3);
        }

        static size_t processClusterProperty(const std::byte *buffer, DataBlob::ObjectPacket &object, const ProtocolOptions &options)
        {
            assert(!object.hasCluster());

            auto &cluster = object.cluster.emplace();

            // Size and ID have already been read
            size_t offset = 0;

            offset += ReadBinary(buffer + offset, &(cluster.state));
            offset += ReadVector<float>(buffer + offset, cluster.centroid.data(), 3);
            offset += ReadVector<float>(buffer + offset, cluster.velocity.data(), 3);
            offset += ReadVector<float>(buffer + offset, cluster.boundingBoxCenter.data(), 3);
            offset += ReadVector<float>(buffer + offset, cluster.boundingBoxSize.data(), 3);
            offset += ReadBinary(buffer + offset, &cluster.weight);

            if (options.boxRotationMode == ProtocolOptions::RotationMode::Quaternions)
            {
                offset += ReadVector<float>(buffer + offset, cluster.boundingBoxRotation.data(), 4);
            }
            else
            {
                offset += ReadVector<float>(buffer + offset, cluster.boundingBoxRotation.data(), 3);
            }

            offset += ReadVector<float>(buffer + offset, cluster.lookAt.data(), 3);

            return offset;
        }

        static size_t processZonePacket(const std::byte *buffer, DataBlob::ZonePacket &outZone, const ProtocolOptions &options)
        {
            // TODO
            return 0;
        }

        static size_t processScenePacket(const std::byte *buffer, DataBlob::SceneInfoPacket &outScene, const ProtocolOptions &options)
        {
            // Packet size and ID have already been read
            size_t offset = 0;

            offset += ReadBinary(buffer + offset, &outScene.addressLength);
            outScene.addressPtr = buffer + offset;

            return offset + (outScene.addressLength * sizeof(char));
        }
    };

    struct ControlMessageParser
    {
        static void parseContainer(const nlohmann::json &containerJson, ControlMessage::Container &outContainer)
        {
            outContainer.address = containerJson.value("address", "");
            outContainer.name = containerJson.value("name", "");
            outContainer.position = ReadJSON<std::array<float, 3>>(containerJson, "position", {0, 0, 0});
            outContainer.rotation = ReadJSON<std::array<float, 3>>(containerJson, "rotation", {0, 0, 0});
            outContainer.color = ReadJSON<std::array<float, 4>>(containerJson, "color", {0, 0, 0, 0});

            auto typeJson = containerJson.find("type");
            if (typeJson != containerJson.end())
            {
                const std::string &typeStr = *typeJson;
                if (typeStr == "Shape")
                {
                    outContainer.type = ControlMessage::Container::Type::Zone;
                    auto &parameters = outContainer.parameters.emplace<ControlMessage::Container::ZoneParameters>();

                    auto shapeJson = containerJson["shape"];
                    auto shapeType = shapeJson["shape"];

                    if (shapeType == "Box")
                    {
                        auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::BoxShapeParameters>();
                        shapeParameters.size = ReadJSON<std::array<float, 3>>(shapeJson, "size", {0, 0, 0});
                    }
                    else if (shapeType == "Cylinder")
                    {
                        auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::CylinderShapeParameters>();
                        shapeParameters.height = shapeJson["height"];
                        shapeParameters.radius = shapeJson["radius"];
                    }
                    else if (shapeType == "Sphere")
                    {
                        auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::SphereShapeParameters>();
                        shapeParameters.radius = shapeJson["radius"];
                    }
                    else if (shapeType == "Path")
                    {
                        auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::PathShapeParameters>();
                    }
                    else if (shapeType == "Grid")
                    {
                        auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::GridShapeParameters>();
                    }
                    else if (shapeType == "Polygon")
                    {
                        auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::PolygonShapeParameters>();
                    }
                    else if (shapeType == "Segment")
                    {
                        auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::SegmentShapeParameters>();
                    }
                }
                else if (typeStr == "Scene")
                {
                    outContainer.type = ControlMessage::Container::Type::Scene;

                    auto &parameters = outContainer.parameters.emplace<ControlMessage::Container::SceneParameters>();
                    parameters.size = ReadJSON<std::array<float, 3>>(containerJson, "size", {0, 0, 0});
                }
                else // Container
                {
                    outContainer.type = ControlMessage::Container::Type::Container;
                    auto &parameters = outContainer.parameters.emplace<ControlMessage::Container::ContainerParameters>();
                }
            }

            auto childrenJson = containerJson.find("children");
            if (childrenJson != containerJson.end())
            {
                for (const auto &childJson : *childrenJson)
                {
                    auto &child = outContainer.children.emplace_back();
                    parseContainer(childJson, child);
                }
            }
        }

        static void parseControlMessage(const nlohmann::json &messageJson, ControlMessage &outMessage)
        {
            auto status = messageJson.value("status", "");
            if (status == "ok")
            {
                outMessage.status = ControlMessage::Status::Ok;
            }
            else if (status == "error")
            {
                outMessage.status = ControlMessage::Status::Error;
                outMessage.errorMessage = messageJson.value("error", "");
            }

            outMessage.serverProtocolVersion = messageJson.value("version", 1);

            auto setupIt = messageJson.find("setup");
            if (setupIt != messageJson.end())
            {
                outMessage.type = ControlMessage::Type::Setup;
                ControlMessageParser::parseContainer(*setupIt, outMessage.rootObject);
            }

            auto updateIt = messageJson.find("update");
            if (updateIt != messageJson.end())
            {
                outMessage.type = ControlMessage::Type::Update;
                ControlMessageParser::parseContainer(*updateIt, outMessage.rootObject);
            }
        }
    };

    void Client::initialize(const ProtocolOptions &desiredOptions)
    {
        options = desiredOptions;
        initialized = true;
    }

    DataBlob Client::parseDataBlob(const std::byte *buffer, size_t bufferSize)
    {
        assert(initialized);

        if (isBigEndian())
        {
            throw(std::runtime_error("The system appears to be big endian, while only little endian is supported."));
        }

        DataBlob blob;
        DataBlobParser::processPacket(buffer, blob, options);

        return blob;
    }

    ControlMessage Client::parseControlMessage(const char *rawMessage)
    {
        assert(initialized);

        auto messageJson = nlohmann::json::parse(rawMessage);

        ControlMessage outMessage;
        ControlMessageParser::parseControlMessage(messageJson, outMessage);

        return outMessage;
    }
}
