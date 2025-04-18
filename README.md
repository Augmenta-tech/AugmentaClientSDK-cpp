# Augmenta Client C++ SDK

The goal of this library is to make consuming the stream output of an Augmenta server as easy as possible. As of right now this only refers to the data emitted through websocket by the Websocket Output node, but other network protocols might be added later on.

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

## Dependencies
 - [zstd](https://github.com/facebook/zstd)
 - [nlohmann::json](https://github.com/nlohmann/json)