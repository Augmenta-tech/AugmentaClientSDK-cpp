#include "../include/AugmentaClientSDK.hpp"

#include <vector>
#include <array>
#include <iostream>

using Vector3f = std::array<float, 3>;
using String = std::string;

// Sample data types. Replace with your own !
using PointCloud = std::vector<Vector3f>;

struct Cluster 
{
    Vector3f boundingBoxPosition;
    Vector3f boundingBoxSize;
    Vector3f centroid;
    Vector3f velocity;
    float weight;
    PointCloud pointCloud;
};

struct ZoneEvent
{
    std::array<float, 2> xyPad;
    int presence;
    float sliderValue;
    int enters;
    int leaves;
    PointCloud pointCloud;
};

struct HierarchyObject
{
    String type;
    String name;
    String address;
    Vector3f position;
    Vector3f orientation;
    std::vector<HierarchyObject> children;
};

struct ExampleWebSocketClient
{
    Augmenta::Client augmentaClient;

    // 1 -----
    // Before anything else we need to initialize the client with the desired options
    void Initialize()
    {
        Augmenta::ProtocolOptions options;
        options.usePolling = false;
        // Specify the options you need

        augmentaClient.initialize("my name", options);
    }

    // 2 -----
    // When the client is notified that a connection was opened, we send a registry request
    void OnConnectionOpened()
    {
        // Use the lib to generate the message according to your options
        std::string requestMessage = augmentaClient.getRegisterMessage();
        SendMessageToServer(requestMessage);
    }

    // 3 ------
    //  The lib can be used to parse a data blob or messages to a more developer-friendly format
    void OnMessageReceived(const std::string &message)
    {
        const auto &parsedMessage = augmentaClient.parseControlMessage(message.c_str());

        if (parsedMessage.isSetup())
        {
            if (parsedMessage.getStatus() == Augmenta::ControlMessage::Status::Error)
            {
                std::cout << "An error occured: " << parsedMessage.getErrorMessage() << std::endl;
                return;
            }

            const auto &world = parsedMessage.getRootObject();
            TraverseObjectHierarchy(&world);
        }
        else if (parsedMessage.isUpdate())
        {
            // Same thing on update, save the data you need !
            TraverseObjectHierarchy(&parsedMessage.getRootObject());
        }
    }

    // 4 (cont.) ------
    void OnDataBlobReceived(const std::vector<std::byte> &dataBlob)
    {
        // You might want to keep that somewhere else
        std::vector<Cluster> frameClusters;
        std::vector<PointCloud> frameScenePointClouds;
        std::vector<ZoneEvent> frameZoneEvents;

        auto parsedData = augmentaClient.parseDataBlob(dataBlob.data(), dataBlob.size());

        // Scene info
        const auto &sceneInfo = parsedData.getSceneInfo();
        
        String scenePath = sceneInfo.getAddress();

        // Objects (Clusters and Point Clouds)
        for (auto &object : parsedData.getObjects())
        {
            const auto &objectID = object.getID();
            if (object.hasCluster())
            {
                auto &clusterInfo = object.getCluster();
            
                Cluster& cluster = frameClusters.emplace_back();
                cluster.boundingBoxPosition = clusterInfo.getBoundingBoxCenter<Vector3f>();
                cluster.boundingBoxSize = clusterInfo.getBoundingBoxSize<Vector3f>();
                cluster.centroid = clusterInfo.getCentroid<Vector3f>();
                cluster.velocity = clusterInfo.getVelocity<Vector3f>();
                cluster.weight = clusterInfo.getWeight();

                if (object.hasPointCloud())
                {
                    // If the object has both a cluster and point cloud property,
                    // it is the cluster's contained point cloud
                    auto& pcInfo = object.getPointCloud();

                    cluster.pointCloud.resize(pcInfo.getPointCount());
                    pcInfo.getPointsData(cluster.pointCloud.data());
                }
            }
            else if (object.hasPointCloud())
            {
                auto &pcInfo = object.getPointCloud();

                // Do something with the data
                // For example you can copy the point data to your own data structure.
                PointCloud &pc = frameScenePointClouds.emplace_back();
                pc.resize(pcInfo.getPointCount());
                pcInfo.getPointsData(pc.data());
            }
        }

        // Zone Events
        for (const auto &zoneEventInfo : parsedData.getZoneEvents())
        {
            const auto emitterZoneAddress = zoneEventInfo.getEmitterZoneAddress();
            // You could use that to map the event to your zone struct

            ZoneEvent& zoneEvent = frameZoneEvents.emplace_back();
            zoneEvent.enters = zoneEventInfo.getEnters();
            zoneEvent.leaves = zoneEventInfo.getLeaves();
            zoneEvent.presence = zoneEventInfo.getPresence();

            for (const auto &property : zoneEventInfo.getProperties())
            {
                switch (property.getType())
                {
                case Augmenta::DataBlob::ZoneEventPacket::Property::Type::Slider:
                {
                    const auto &sliderData = property.getSliderParameters();
                    zoneEvent.sliderValue = sliderData->getValue();
                    break;
                }
                case Augmenta::DataBlob::ZoneEventPacket::Property::Type::XYPad:
                {
                    const auto &xyData = property.getXYPadParameters();
                    zoneEvent.xyPad = {xyData->getX(), xyData->getY()};
                    break;
                }
                case Augmenta::DataBlob::ZoneEventPacket::Property::Type::PointCloud:
                {
                    const auto& pcParams = property.getPointCloudParameters();
                    zoneEvent.pointCloud.resize(pcParams->getPointCount());
                    pcParams->getPointsData(zoneEvent.pointCloud.data());
                }
                }
            }
        }
    }

    void SendMessageToServer(const std::string &message)
    {
        // ... do your thing
    }

    // Helper: Recursively traverse the object hierarchy (received on setup or update) and get the data you need
    void TraverseObjectHierarchy(const Augmenta::ControlMessage::Container *container)
    {
        if (container->isScene())
        {
            const auto &sceneParameters = container->getSceneParameters();
            const auto &sceneSize = sceneParameters->size;
            // Maybe save that somewhere ? 
        }

        if (container->isZone())
        {
            const auto &zoneParameters = container->getZoneParameters();
            if (zoneParameters->isBox())
            {
                const auto &boxParameters = zoneParameters->getBoxShapeParameters();
                boxParameters->size;
                // You might want to do something with this ?
            }
        }

        for (const auto &child : container->getChildren())
        {
            TraverseObjectHierarchy(&child);
        }
    }
};