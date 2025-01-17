#include "AugmentServerLib.hpp"

#include <vector>
#include <array>

using CustomPointCloudDataStructure = std::vector<std::array<float, 3>>;

void main()
{
    CustomPointCloudDataStructure pointCloud;

    std::vector<std::byte> receivedBinaryBlob;

    AugmentaServerProtocol::DataBlobParser parser(receivedBinaryBlob);

    for (const auto &object : parser.getObjects())
    {
        for (auto &pcInfo : object.getPointClouds())
        {
            // Allocate memory as needed
            pointCloud.resize(pcInfo.getPointCount());

            // Copy from binary blob to our own pointcloud data structure
            pcInfo.getPointsData(pointCloud.data());
        }
    }
}