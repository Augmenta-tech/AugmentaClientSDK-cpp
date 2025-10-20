#include "../include/AugmentaClientSDK.hpp"

#include <vector>
#include <array>
#include <iostream>

// Sample data types. Replace with your own types !
using Vector3f = std::array<float, 3>;
using PointCloud = std::vector<Vector3f>;
struct Cluster {
    Vector3f position;
};
using String = std::string;

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
        auto parsedData = augmentaClient.parseDataBlob(dataBlob.data(), dataBlob.size());

        // Scene info
        const auto &sceneInfo = parsedData.getSceneInfo();
        
        String scenePath;
        scenePath.resize(sceneInfo.getAddressLength());
        sceneInfo.getAddress(scenePath.data());

        // Objects
        for (auto &object : parsedData.getObjects())
        {
            const auto &objectID = object.getID();
            if (object.hasCluster())
            {
                auto &clusterInfo = object.getCluster();

                if (clusterInfo.getState() == Augmenta::ClusterState::Entered)
                {
                    // This is a new cluster, add it to our list
                }

                if (clusterInfo.getState() == Augmenta::ClusterState::WillLeave)
                {
                    // Clean up leaving clusters
                }
            }

            if (object.hasPointCloud())
            {
                auto &pcInfo = object.getPointCloud();

                // Do something with the data
                // For example you can copy the point data to your own data structure.
                PointCloud pc;
                pc.resize(pcInfo.getPointCount());
                pcInfo.getPointsData(pc.data());
            }
        }

        // Zones
        for (const auto &zone : parsedData.getZones())
        {
            for (const auto &property : zone.getProperties())
            {
                switch (property.getType())
                {
                case Augmenta::DataBlob::ZoneEventPacket::Property::Type::Slider:
                {
                    const auto &sliderData = property.getSliderParameters();
                    auto value = sliderData->getValue();
                }
                case Augmenta::DataBlob::ZoneEventPacket::Property::Type::XYPad:
                {
                    const auto &xyData = property.getXYPadParameters();
                    auto x = xyData->getX();
                    auto y = xyData->getY();
                }
                    // ...
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