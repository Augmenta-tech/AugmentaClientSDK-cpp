#pragma once

#include "AugmentaServerLib.hpp"

#include "json.hpp"

namespace
{
    // Read an int from a byte buffer. Returns the number of bytes read.
    template<typename T>
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

        static size_t processPacket(const std::byte *packetBuffer, size_t packetSize, DataBlob &outDataBlob)
        {
            size_t offset = 0;

            PacketType type = static_cast<PacketType>(*packetBuffer);
            offset += 1;

            switch (type)
            {
            case PacketType::Bundle:
            {
                while (offset != packetSize)
                {
                    int subPacketSize;
                    ReadBinary(packetBuffer + offset + sizeof(PacketType), &subPacketSize);

                    auto subPacketBegin = packetBuffer + offset;
                    offset += processPacket(subPacketBegin, subPacketSize, outDataBlob);
                }

                break;
            }

            case PacketType::Object:
            {
                auto &object = outDataBlob.objects.emplace_back();
                offset += processObjectPacket(packetBuffer + offset, object);
                break;
            }

            case PacketType::Zone:
            {
                auto &zone = outDataBlob.zones.emplace_back();
                offset += processZonePacket(packetBuffer + offset, zone);
                break;
            }

            case PacketType::Scene:
            {
                offset += processScenePacket(packetBuffer + offset, outDataBlob.sceneInfo);
                break;
            }

            default:
            {
                assert(false); // Unexpected packet type !
            }
            }

            return offset;
        }

        static size_t processObjectPacket(const std::byte *packetBuffer, DataBlob::ObjectPacket &outObject)
        {
            // At this point packetType has already been read
            size_t offset = 0;

            int packetSize;
            offset += ReadBinary(packetBuffer + offset, &packetSize);

            int objectID;
            offset += ReadBinary(packetBuffer + offset, &outObject.id);

            while (offset != packetSize)
            {
                int propertyID;
                offset += ReadBinary(packetBuffer + offset, &propertyID);

                int propertySize;
                offset += ReadBinary(packetBuffer + offset, &propertySize);

                switch (static_cast<PropertyType>(propertyID))
                {
                case PropertyType::Points:
                {
                    offset += processPointCloudProperty(packetBuffer + offset, outObject);
                    break;
                }

                case PropertyType::Cluster:
                {
                    offset += processClusterProperty(packetBuffer + offset, outObject);
                    break;
                }
                }
            }

            return offset;
        }

        static size_t processPointCloudProperty(const std::byte *buffer, DataBlob::ObjectPacket &object)
        {
            assert(!object.hasPointCloud());

            auto &pointCloud = object.pointCloud.emplace();

            size_t offset = 0;

            offset += ReadBinary(buffer + offset, &pointCloud.pointsCount);
            pointCloud.pointsPtr = buffer + offset;

            return offset + (pointCloud.pointsCount * sizeof(float) * 4);
        }

        static size_t processClusterProperty(const std::byte *buffer, DataBlob::ObjectPacket &object)
        {
            assert(!object.hasCluster());

            auto &cluster = object.cluster.emplace();

            size_t offset = 0;

            int stateInt;
            offset += ReadBinary(buffer + offset, &stateInt);
            cluster.state = static_cast<ClusterState>(stateInt);

            offset += ReadVector<float>(buffer + offset, cluster.centroid.data(), 3);
            offset += ReadVector<float>(buffer + offset, cluster.velocity.data(), 3);
            offset += ReadVector<float>(buffer + offset, cluster.boundingBoxCenter.data(), 3);
            offset += ReadVector<float>(buffer + offset, cluster.boundingBoxSize.data(), 3);
            offset += ReadBinary(buffer + offset, &cluster.weight);

            // TODO: This only works with quaternion mode
            offset += ReadVector<float>(buffer + offset, cluster.boundingBoxRotation.data(), 4);

            offset += ReadVector<float>(buffer + offset, cluster.lookAt.data(), 3);

            return offset;
        }

        static size_t processZonePacket(const std::byte *buffer, DataBlob::ZonePacket &outZone)
        {
            // TODO
            return 0;
        }

        static size_t processScenePacket(const std::byte *buffer, DataBlob::SceneInfoPacket &outScene)
        {
            // PacketType has already been read
            size_t offset = 0;

            int packetSize;
            offset += ReadBinary(buffer + offset, &packetSize);
            offset += ReadBinary(buffer + offset, &outScene.addressLength);
            outScene.addressPtr = buffer + offset;

            return offset + (outScene.addressLength * sizeof(char));
        }
    };

    struct ControlMessageParser
    {
        static void parseContainer(const nlohmann::json &containerJson, ControlMessage::Container &outContainer)
        {
            // TODO: replace with optionnals ?
            // TODO: What *is* optionnal, and what isn't ?
            outContainer.address = containerJson.value("address", "");
            outContainer.name = containerJson.value("name", "");
            outContainer.position = ReadJSON<std::array<float, 3>>(containerJson, "position", {0, 0, 0});
            outContainer.rotation = ReadJSON<std::array<float, 3>>(containerJson, "rotation", {0, 0, 0});
            outContainer.color = ReadJSON<std::array<float, 4>>(containerJson, "color", {0, 0, 0, 0});

            auto typeJson = containerJson.find("type");
            if (typeJson != containerJson.end())
            {
                const std::string &typeStr = *typeJson;
                if (typeStr == "Zone")
                {
                    outContainer.type = ControlMessage::Container::Type::Zone;
                    // TODO
                }
                else if (typeStr == "Scene")
                {
                    outContainer.type = ControlMessage::Container::Type::Scene;
                    outContainer.size = ReadJSON<std::array<float, 3>>(containerJson, "size", {0, 0, 0});
                }
                else // Container
                {
                    outContainer.type = ControlMessage::Container::Type::Container;
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
            if (messageJson.contains("status"))
            {
                if (messageJson["status"] == "ok")
                {
                    if (messageJson.contains("setup"))
                    {
                        outMessage.type = ControlMessage::Type::Setup;
                        const auto &setupJson = messageJson["setup"];
                        ControlMessageParser::parseContainer(setupJson, outMessage.container);
                    }
                }
            }
            else if (messageJson.contains("update"))
            {
                outMessage.type = ControlMessage::Type::Update;
                const auto &updateJson = messageJson["update"];
                ControlMessageParser::parseContainer(updateJson, outMessage.container);
            }
        }
    };

    DataBlob Client::parseDataBlob(const std::byte *buffer, size_t bufferSize)
    {
        if (isBigEndian())
        {
            throw(std::runtime_error("The system appears to be big endian, while only little endian is supported."));
        }

        DataBlob blob;
        DataBlobParser::processPacket(buffer, bufferSize, blob);

        return blob;
    }

    ControlMessage Client::parseControlMessage(const std::string &rawMessage)
    {
        auto messageJson = nlohmann::json::parse(rawMessage);
        ControlMessage outMessage;
        ControlMessageParser::parseControlMessage(messageJson, outMessage);

        return outMessage;
    }
}
