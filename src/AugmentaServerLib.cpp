#pragma once

#include "AugmentaServerLib.hpp"

#include "json.hpp"

namespace
{
    // Read an int from a byte buffer. Returns the number of bytes read.
    static size_t ReadInt(const std::byte* buffer, int *out)
    {
        std::memcpy(out, buffer, sizeof(int));
        return sizeof(int);
    }

    // Read a contiguous vector from a byte buffer. Returns the number of bytes read.
    // The output vector's memory should have been allocated beforehand.
    template <typename T>
    static size_t ReadVector(const std::byte* buffer, T *out, size_t elementCount)
    {
        std::memcpy(out, buffer, elementCount * sizeof(T));
        return elementCount * sizeof(T);
    }

    static size_t ReadFloat(const std::vector<std::byte>::iterator &it, float *out)
    {
        std::memcpy(out, &(*it), sizeof(float));
        return sizeof(float);
    }

    template <typename Vector3f>
    static Vector3f ReadVector3f(const nlohmann::json &json)
    {
        Vector3f out;
        out[0] = json[0];
        out[1] = json[1];
        out[2] = json[2];
        return out;
    }

    template <typename Vector4f>
    static Vector4f ReadVector4f(const nlohmann::json &json)
    {
        Vector4f out;
        out[0] = json[0];
        out[1] = json[1];
        out[2] = json[2];
        out[3] = json[3];
        return out;
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

    DataBlob::DataBlob(const std::byte* blob, size_t blobSize)
    {
        processPacket(blob, blobSize);
    }

    DataBlob DataBlob::parse(const std::byte* buffer, size_t bufferSize)
    {
        if (isBigEndian())
        {
            throw(std::runtime_error("The system appears to be big endian, while only little endian is supported."));
        }

        return DataBlob(buffer, bufferSize);
    }

    size_t DataBlob::processPacket(const std::byte* packetBuffer, size_t packetSize)
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
                ReadInt(packetBuffer + offset + sizeof(PacketType), &subPacketSize);

                auto subPacketBegin = packetBuffer + offset;
                offset += processPacket(subPacketBegin, subPacketSize);
            }

            break;
        }

        case PacketType::Object:
        {
            auto &object = objects.emplace_back();
            offset += processObjectPacket(packetBuffer + offset, object);
            break;
        }

        case PacketType::Zone:
        {
            auto &zone = zones.emplace_back();
            offset += processZonePacket(packetBuffer + offset, zone);
            break;
        }

        case PacketType::Scene:
        {
            offset += processScenePacket(packetBuffer + offset, sceneInfo);
            break;
        }

        default:
        {
            assert(false); // Unexpected packet type !
        }
        }

        return offset;
    }

    size_t DataBlob::processObjectPacket(const std::byte* packetBuffer, ObjectPacket &outObject)
    {
        // At this point packetType has already been read
        size_t offset = 0;

        int packetSize;
        offset += ReadInt(packetBuffer + offset, &packetSize);

        int objectID;
        offset += ReadInt(packetBuffer + offset, &outObject.id);

        while (offset != packetSize)
        {
            int propertyID;
            offset += ReadInt(packetBuffer + offset, &propertyID);

            int propertySize;
            offset += ReadInt(packetBuffer + offset, &propertySize);

            switch (static_cast<PropertyType>(propertyID))
            {
            case PropertyType::Points:
            {
                PointCloudProperty &pc = outObject.pointClouds.emplace_back();
                offset += processPointCloudProperty(packetBuffer + offset, pc);
                break;
            }

            case PropertyType::Cluster:
            {
                ClusterProperty &cluster = outObject.clusters.emplace_back();
                offset += processClusterProperty(packetBuffer + offset, cluster);
                break;
            }
            }
        }

        return offset;
    }

    size_t DataBlob::processPointCloudProperty(const std::byte* buffer, PointCloudProperty &outPointCloud)
    {
        size_t offset = 0;

        offset += ReadInt(buffer + offset, &outPointCloud.pointsCount);
        outPointCloud.pointsPtr = buffer + offset;

        return offset + (outPointCloud.pointsCount * sizeof(float) * 4); 
    }

    size_t DataBlob::processClusterProperty(const std::byte* buffer, ClusterProperty &outCluster)
    {
        size_t offset = 0;

        int stateInt;
        offset += ReadInt(buffer + offset, &stateInt);
        outCluster.state = static_cast<ClusterState>(stateInt);

        offset += ReadVector<float>(buffer + offset, outCluster.centroid.data(), 3);
        offset += ReadVector<float>(buffer + offset, outCluster.velocity.data(), 3);
        offset += ReadVector<float>(buffer + offset, outCluster.boundingBoxCenter.data(), 3);
        offset += ReadVector<float>(buffer + offset, outCluster.boundingBoxSize.data(), 3);
        offset += ReadVector<float>(buffer + offset, outCluster.weight.data(), 3);

        // TODO: This only works with quaternion mode
        offset += ReadVector<float>(buffer + offset, outCluster.rotation.data(), 4);

        offset += ReadVector<float>(buffer + offset, outCluster.lookAt.data(), 3);

        return offset;
    }

    size_t DataBlob::processZonePacket(const std::byte* buffer, ZonePacket &outZone)
    {
        // TODO
        return 0;
    }

    size_t DataBlob::processScenePacket(const std::byte* buffer, SceneInfoPacket &outScene)
    {
        // PacketType has already been read
        size_t offset = 0;

        int packetSize;
        offset += ReadInt(buffer + offset, &packetSize);
        offset += ReadInt(buffer + offset, &outScene.addressLength);
        outScene.addressPtr = buffer + offset;
        
        return offset + (outScene.addressLength * sizeof(char));
    }
}
