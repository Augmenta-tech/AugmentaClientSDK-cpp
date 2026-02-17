# Augmenta Client C++ SDK

The goal of this library is to make consuming the stream output of an Augmenta server as easy as possible. As of right now this only refers to the data emitted through websocket by the Websocket Output node, but other network protocols might be added later on.

## Features
The features currently implemented are:
- **Tracking data** : Augmenta sends a list of objects with their id, centroid position, bounding box and velocity (with optional point cloud data).
- **Volumetric data** : The point cloud of the whole scene. Richness of the point cloud can be controled through the Augmenta software or through the sdk. Point cloud from only specific zones can be requested.
- **Zones events** : You can define 3D zones in the Augmenta software and receive events like entering, leaving, presence, and other dynamic values. 3D zones are sent through the SDK to synchronize and display them, manipulating them from the SDK is planned but not yet available.

Features yet to be implemented are:
- **Bi-directional communication** : While you can use OSC to control Augmenta, the same is not yet possible through the websocket protocol.
- **Skeleton tracking** : We plan to add support for receiving skeleton data in the future.

## Building
We use CMake as our buildsystem.


To build a standalone version of the lib:

```
git clone https://github.com/Augmenta-tech/augmenta-server-lib-cpp
cd augmenta-server-lib-cpp
mkdir build
cmake -B build -S .
cmake --build build
```

## Using the SDK
The SDK revolves around creating an `Augmenta::Client` object and using it to parse data blobs and messages received.
See [examples/Example.cpp](examples/Example.cpp) for a full usage example.

Augmenta's server and software are designed to run perpetually. This include handling automatic restarts in case of crashes, reboot or any kind of downtime. 
When Implementing the SDK, you must take this into account and make sure to handle the connection lifecycle properly. For example, you must try to reconnect regularly in case of disconnection and make sure to shutdown the client properly when closing the application or changing urls.

### Start connection
When starting a connection, call `client.initialize(MyClientName: string, Options: ProtocolOptions)`.
When disconnecting, during closing or when changing urls, call `client.shutdown();`.

## Option
Here are all the available options for the client initialization: 

```cpp

    struct ProtocolOptions
    {

        /// @brief Specifies the mode for representing rotations for all non-streamed objects
        enum class RotationMode
        {
            Radians,
            Degrees,
            Quaternions,
        };

        /// @brief Defines a coordinate system transformation configuration, including axis orientation, origin placement, axis flipping, and coordinate space type. 
        /// 
        /// Augmenta internally use a left handed Y-up coordinate system with the origin at the bottom left of the scene, but 
        /// you can specify a different configuration here. Augmenta will take care of transforming all the data to match it.
        struct AxisTransform
        {
            /// @brief Defines the coordinate system axis mode.
            enum class AxisMode
            {
                ZUpRightHanded,
                ZUpLeftHanded,
                YUpRightHanded,
                YUpLeftHanded, // used in the front-end GUI of Augmenta
            };

            /// @brief Specifies the origin position for coordinate system or layout calculations.
            enum class OriginMode
            {
                BottomLeft,
                BottomRight,
                TopLeft,
                TopRight,
            };

            /// @brief Defines the coordinate space in which scenes, zones, cluster and point
            /// cloud data will be provided.
            /// 
            /// - Absolute referes to World space coordinates in relation
            ///   to the origin visible in Augmenta's front-end ui. 
            /// - Relative give coordinates to the parent. Clusters are
            ///   considered childs of the scene they are computed in and are then
            ///   placed based on that scene's origin.
            /// - Normalized gives coordinates relative between 0 and 1 based on
            ///   the scene dimensions.
            enum class CoordinateSpace
            {
                Absolute,
                Relative,
                Normalized,
            };

            AxisMode axis = AxisMode::ZUpRightHanded;
            OriginMode origin = OriginMode::BottomLeft;

            /// @brief finer controls to flip specific axis if needed. 
            bool flipX = false;
            bool flipY = false;
            bool flipZ = false;
            CoordinateSpace coordinateSpace = CoordinateSpace::Absolute;

            bool operator==(const AxisTransform& other) const;
            bool operator!=(const AxisTransform& other) const;
        };

        /// @brief Protocol version 1 or 2 (better use 2)
        int version = 2;
        
        /// @brief tags are used to filter what content to be streamed by the server. 
        /// You may add tags to scenes in the backend GUI and have only one scene's 
        /// clusters, zone events and point clouds streamed by adding the 
        /// corresponding tag to this list. If the list is empty, all content 
        /// will be streamed. 
        std::vector<std::string> tags;

        /// @brief Downsampling factor for point clouds. 
        /// 
        /// It is very usefull when you want to reduce the number of points
        /// received for performance reasons. You may even cap your global 
        /// cluster size by dynamically changing this property.
        /// 
        /// For example, a value of 2 means that only every 2nd point will be 
        /// streamed. Must be superior or equal to 1.
        int downSample = 1;

        /// @brief Whether the server should send the scene Point Cloud data.
        bool streamClouds = true;
        
        /// @brief Whether the server should send the scene Clusters data.
        bool streamClusters = true;

        /// @brief Whether the server should add the points that make up a cluster to each cluster packet.
        bool streamClusterPoints = true;

        /// @brief Whether the server should add the points included in a zone to each zone packet.
        bool streamZonePoints = false;

        RotationMode boxRotationMode = RotationMode::Quaternions;
        AxisTransform axisTransform; // TODO: Default ?

        /// @brief enable ZSTD compression on the binary data's stream
        bool useCompression = true;

        // @brief By default, Augmenta will send data as soon as possible. you can enable
        // this option to only receive data on demand (by sending a poll request)
        bool usePolling = false;

    };
```

> **Note:** To change any parameter after initialization, shutdown the client re-initialize it with the new options. 
>  
> Id and names will persist as long as the server is not restarted. Beware that changes may have occured during the downtime.

## Data types
The SDK will not provide fixed types for the received Object to allow for implementor's discretion regarding about math libraries or features to use.
Instead, here is an optional abstract representation of the data types used in the SDK:

```md
Pseudo-code
// Full data structure. Match Augmenta's protocol but might be overkill for some use cases.

Type AugmentaPointCloud : Vector3 array

Type AugmentaCluster :
	- stat : Enum { 0 = Enter, 1 = Update, 2 = Will Leave, 3 = Ghost }
	- Centroid: Vector3
	- Velocity: Vector3
	- BoundingBoxCenter: Vector3
	- BoundingBoxSize: Vector3
	- weight: Quaternion or Vector3 (Quaternion, degree or radian based on boxRotationMode option)
	- LookAt: Vector3

type AugmentaStreamObject 	
	- id: int
	- hasCluster: boolean
	- hasPointCloud: Boolean
	- pointCloud: Optional<PointCloud> (streamClusterPoints result in Objects with cluster also having pointClouds, streamClouds can result in Objects with pointClouds but no cluster)
	- cluster: Optional<Cluster> (setting streamClusters to false will result in no Object having clusters)

type AugmentaArborescenceNode :
	- type: string ("Scene", "Shape", "Container")
	- name: string
	- address: string (OSC Path exposed by hovering the element in the backend GUI)
	- position: Vector3
	- rotation: Quaternion or Vector3 (Quaternion, degree or radian based on RotationMode option)
	- children: array of AugmentaArborescenceNode

type AugmentaScene extend AugmentaArborescenceNode : 
	- type: "Scene"
	- size: Vector3
	- pointCloud: Optional<PointCloud> set using streamClouds

type zone : AugmentaArborescenceNode : 
	- type: "zone"
	- sliderValue: float
	- xpad: float
	- ypad:float
	- presence: int
	- onEnterCallback: function 
	- onLeaveCallback: function
	- pointCloud: Optional<PointCloud> only if streamZonePoints is On
	- shape:
		- type: string ("box", "cylinder", "sphere
		... shape parameters

```

> **Note:** Points Cloud attached to a Zone or cluster are positionned relative to the parent Scene when *CoordinateSpace::Relative* is used.

## Dependencies and ressources
 - [zstd](https://github.com/facebook/zstd)
 - [nlohmann::json](https://github.com/nlohmann/json)
 - [Augmenta Websocket protocol V2](https://augmentatech.notion.site/WebSocket-Protocol-Specification-v2-637551d8e04a4015a56526d80e1b10f0?pvs=74)
