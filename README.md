# Augmenta Client C++ SDK

The goal of this library is to make consuming the stream output of an Augmenta server as easy as possible. 
As of right now this only refers to the data emitted through websocket by the Websocket Output node, but other network protocols might be added later on.

## Features
The SDK can be used to parse different type of data received from Augmenta:
- **Tracking data** : A list of tracked objects/persons with their ID, position, bounding box and velocity
- **Volumetric data** : Filtered point cloud data, either per-cluster, per-zone, or from the whole scene
- **Zones events** : Be notified of events like enter, leave, presence... affecting a specific zone in the world
- **World hierarchy** : The structure of the world as set up on Augmenta's side: scenes, zones, etc.

Features yet to be implemented are:
- **Skeleton tracking**
- **Custom cluster data**

## Protocol
This SDK can help you parse data received from the server, but you'll still need to respect the [Augmenta Websocket Protocol Specification](https://augmentatech.notion.site/Augmenta-WebSocket-Protocol-Specification-v2-637551d8e04a4015a56526d80e1b10f0?pvs=74). Make sure to give it a read ! 

## Using the SDK
### Integrate in your project
The SDK is composed of a single header/implementation file pair. We provide support for CMake. You can integrate it the way you see fit:
- using CMake
- as a git submodule
- by dropping the files directly in your project

### Implement in your codebase
The SDK revolves around creating an `Augmenta::Client` object and using it to parse data blobs and messages received.
See [examples/Example.cpp](examples/Example.cpp) for a full usage example.

### Websocket implementation
This SDK does not come with a websocket implementation, as we expect most users' environments to contain one already. If that is not the case we can recommend the [websocketpp](https://github.com/zaphoyd/websocketpp) library.

### Lifetime
Augmenta's servers and software are designed to run perpetually. They will automatically restarts in case of crashes, reboot, power loss, etc.
Make sure to take this into account and handle the connection lifecycle properly. For example, you should try to reconnect to the server automatically in case the connection is lost (which could happen in cases of network outage for example).

### Options
Using the SDK, you can select options to control the format and kind of data that the Augmenta server will send (see the implementation for descriptions).

While implementing a client, keep in mind that some of those options should be controllable by your users (which type of data will be sent) while some other might be locked by you, the developper (things like your software's coordinate system).

> **Note:** Changed options will only be taken into account after re-initializing the client. 

## Contribute to the SDK
We use CMake as our buildsystem.
To build a standalone version of the lib:

```
git clone https://github.com/Augmenta-tech/augmenta-server-lib-cpp
cd augmenta-server-lib-cpp
mkdir build
cmake -B build -S .
cmake --build build
```

## Dependencies and ressources
 - [zstd](https://github.com/facebook/zstd)
 - [nlohmann::json](https://github.com/nlohmann/json)

> Please reach out to us if you have any questions - Augmenta Team