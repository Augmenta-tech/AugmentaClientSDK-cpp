#include "AugmentaServerLib.hpp"

#include "json.hpp"

#pragma once

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

    DataBlobParser::DataBlobParser(const std::vector<std::byte> &inBuffer)
        : buffer(&inBuffer)
    {
        if (isBigEndian())
        {
            throw(std::runtime_error("The system appears to be big endian, while only little endian is supported."));    
        }

        auto begin = buffer->begin();
        auto end = buffer->end();

        processPacket(begin, end);
    }

    size_t DataBlobParser::processPacket(const std::vector<std::byte>::const_iterator &packetBegin, const std::vector<std::byte>::const_iterator &packetEnd)
    {
        auto packetIt = packetBegin;

        PacketType type = static_cast<PacketType>(*packetIt);
        packetIt += 1;

        switch (type)
        {
        case PacketType::Bundle:
        {
            while (packetIt != packetEnd)
            {
                int subPacketSize;
                ReadInt(packetIt + sizeof(PacketType), &subPacketSize);

                auto subPacketBegin = packetIt;
                auto subPacketEnd = packetIt + subPacketSize;
                packetIt += processPacket(subPacketBegin, subPacketEnd);
            }

            return std::distance(packetBegin, packetIt);
        }

        case PacketType::Object:
            auto &object = objects.emplace_back();
            packetIt += processObjectPacket(packetIt, object);
            break;

        case PacketType::Zone:
            auto &zone = zones.emplace_back();
            packetIt += processZonePacket(packetIt, zone);
            break;

        case PacketType::Scene:
            packetIt += processScenePacket(packetIt, scene);
            break;

        default:
            assert(false); // Unexpected packet type !
        }

        return std::distance(packetBegin, packetIt);
    }

    size_t DataBlobParser::processObjectPacket(const std::vector<std::byte>::const_iterator &packetBegin, ObjectPacket &outObject)
    {
        // At this point packetType has already been read
        auto packetIt = packetBegin;

        int packetSize;
        packetIt += ReadInt(packetIt, &packetSize);
        auto packetEnd = packetBegin + packetSize;

        int objectID;
        packetIt += ReadInt(packetBegin, &outObject.id);

        while (packetIt != packetEnd)
        {
            int propertyID;
            packetIt += ReadInt(packetIt, &propertyID);

            int propertySize;
            packetIt += ReadInt(packetIt, &propertySize);

            switch (static_cast<PropertyType>(propertyID))
            {
            case PropertyType::Points:
                PointCloudProperty &pc = outObject.pointClouds.emplace_back();
                packetIt += processPointCloudProperty(packetIt, pc);
                break;

            case PropertyType::Cluster:
                ClusterProperty &cluster = outObject.clusters.emplace_back();
                packetIt += processClusterProperty(packetIt, cluster);
                break;
            }
        }

        return std::distance(packetBegin, packetIt);
    }

    size_t DataBlobParser::processPointCloudProperty(const std::vector<std::byte>::const_iterator &pointCloudBegin, PointCloudProperty &outPointCloud)
    {
        auto pointCloudIt = pointCloudBegin;

        pointCloudIt += ReadInt(pointCloudIt, &outPointCloud.pointsCount);
        outPointCloud.pointsPtr = &*pointCloudIt;

        return std::distance(pointCloudBegin, pointCloudIt);
    }

    size_t DataBlobParser::processClusterProperty(const std::vector<std::byte>::const_iterator &clusterBegin, ClusterProperty &outCluster)
    {
        auto clusterIt = clusterBegin;

        int stateInt;
        clusterIt += ReadInt(clusterIt, &stateInt);
        outCluster.state = static_cast<ClusterState>(stateInt);

        clusterIt += ReadVector<float>(clusterIt, outCluster.centroid.data(), 3);
        clusterIt += ReadVector<float>(clusterIt, outCluster.velocity.data(), 3);
        clusterIt += ReadVector<float>(clusterIt, outCluster.boundingBoxCenter.data(), 3);
        clusterIt += ReadVector<float>(clusterIt, outCluster.boundingBoxSize.data(), 3);
        clusterIt += ReadVector<float>(clusterIt, outCluster.weight.data(), 3);

        // TODO: Does not work without protocol 2.1 changes
        clusterIt += ReadVector<float>(clusterIt, outCluster.rotation.data(), 4);

        clusterIt += ReadVector<float>(clusterIt, outCluster.lookAt.data(), 4);

        return std::distance(clusterBegin, clusterIt);
    }

    size_t DataBlobParser::processZonePacket(const std::vector<std::byte>::const_iterator &dataBegin, ZonePacket &outZone)
    {
        // TODO
    }

    size_t DataBlobParser::processScenePacket(const std::vector<std::byte>::const_iterator &packetBegin, SceneInfoPacket &outScene)
    {
        // ID has already been read
        auto sceneIt = packetBegin;

        int packetSize;
        sceneIt += ReadInt(sceneIt, &packetSize);
        auto packetEnd = packetBegin + packetSize;

        sceneIt += ReadInt(sceneIt, &outScene.addressLength);

        outScene.addressPtr = &*sceneIt;
        sceneIt += outScene.addressLength;

        return std::distance(packetBegin, sceneIt);
    }

    template <typename T>
    size_t Read(const std::vector<std::byte> &buffer, T *out)
    {
        std::memcpy(out, buffer, sizeof(T));
        return sizeof(T);
    }

    // Read an int from a byte buffer. Returns the number of bytes read.
    static size_t ReadInt(const std::vector<std::byte>::const_iterator &dataIt, int *out)
    {
        std::memcpy(out, &(*dataIt), sizeof(int));
        return sizeof(int);
    }

    // Read a contiguous vector from a byte buffer. Returns the number of bytes read.
    // The output vector's memory should have been allocated beforehand.
    template <typename T>
    static size_t ReadVector(const std::vector<std::byte>::const_iterator &bufferIt, T *out, size_t elementCount)
    {
        std::memcpy(out, &(*bufferIt), elementCount * sizeof(T));
        return elementCount * sizeof(T);
    }

    static size_t ReadFloat(const std::vector<std::byte>::iterator &it, float *out)
    {
        std::memcpy(out, &(*it), sizeof(float));
        return sizeof(float);
    }

    // static std::string ReadString(const std::vector<std::byte>& data, int offset, int length)
    //{
    //	// TODO
    //	return Encoding.UTF8.GetString(data.Slice(offset, length));
    // }

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
}

bool isBigEndian()
{
    union {
        uint32_t i;
        char c[4];
    } bint = {0x01020304};

    return bint.c[0] == 1;
}