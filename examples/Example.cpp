#include "../include/AugmentaServerLib.hpp"

#include <vector>
#include <array>

using CustomPointCloudDataStructure = std::vector<std::array<float, 3>>;

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
    // When the client is notified that a connection was opened, we can send a registry request
    void OnConnectionOpened()
    {
        // Use the lib to generate the message according to your options
        std::string requestMessage = augmentaClient.getRegisterMessage();
        SendMessageToServer(requestMessage);
    }

    // 3 ------
    //  The lib can be used to parse a data blob or messages to a more developer-friendly format
    void OnMessageReceived(const std::string& message)
    {
        const auto& parsedMessage = augmentaClient.parseControlMessage(message.c_str());
    
        if (parsedMessage.isSetup())
        {
            if (parsedMessage.getStatus() == Augmenta::ControlMessage::Status::Ok)
            {
                // Handshake successful !
            }
        }
        else if (parsedMessage.isUpdate())
        {
            // Handle updates as necessary
        }
    }

    // 4 (cont.) ------
    void OnDataBlobReceived(const std::vector<std::byte>& dataBlob)
    {
        auto parsedData = augmentaClient.parseDataBlob(dataBlob.data(), dataBlob.size());
    
        // Scene info
        auto& sceneInfo = parsedData.getSceneInfo();
        // Do something with the data
    
        // Objects
        for (auto& object : parsedData.getObjects())
        {
            if (object.hasCluster())
            {
                auto& clusterInfo = object.getCluster();
                // Do something with the data
            }
    
            if (object.hasPointCloud())
            {
                auto& pcInfo = object.getPointCloud();
                // Do something with the data
                
                // For example you can copy the point data to your own data structure.
                CustomPointCloudDataStructure pc;
                pc.resize(pcInfo.getPointCount());
                pcInfo.getPointsData(pc.data());
            }
        }
    
        // Zones
        for (const auto& zone : parsedData.getZones())
        {
            for (const auto& property : zone.getProperties())
            {
                switch(property.getType())
                {
                    case Augmenta::DataBlob::ZonePacket::Property::Type::Slider:
                    {
                        const auto& sliderData = property.getExtraData<Augmenta::DataBlob::ZonePacket::SliderProperty>();
                        auto value = sliderData->getValue();
                    }
                    case Augmenta::DataBlob::ZonePacket::Property::Type::XYPad:
                    {
                        const auto& xyData = property.getExtraData<Augmenta::DataBlob::ZonePacket::XYPadProperty>();
                        auto x = xyData->getX();
                        auto y = xyData->getY();
                    }
                    // ...
                }
            }
        }
    }

    void SendMessageToServer(const std::string& message)
    {
        // ... do your thing
    }
};