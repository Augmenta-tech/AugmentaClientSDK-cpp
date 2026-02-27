#include "AugmentaClientSDK.hpp"

#include <json.hpp>
#include <zstd.h>

namespace
{
	const std::unordered_map<Augmenta::ProtocolOptions::RotationMode, std::string> rotationModeNames = {
		{Augmenta::ProtocolOptions::RotationMode::Degrees, "degrees"},
		{Augmenta::ProtocolOptions::RotationMode::Radians, "radians"},
		{Augmenta::ProtocolOptions::RotationMode::Quaternions, "quaternions"},
	};

	const std::unordered_map<Augmenta::ProtocolOptions::AxisTransform::AxisMode, std::string> axisModeNames = {
		{Augmenta::ProtocolOptions::AxisTransform::AxisMode::ZUpRightHanded, "z_up_right"},
		{Augmenta::ProtocolOptions::AxisTransform::AxisMode::ZUpLeftHanded, "z_up_left"},
		{Augmenta::ProtocolOptions::AxisTransform::AxisMode::YUpRightHanded, "y_up_right"},
		{Augmenta::ProtocolOptions::AxisTransform::AxisMode::YUpLeftHanded, "y_up_left"},
	};

	const std::unordered_map<Augmenta::ProtocolOptions::AxisTransform::OriginMode, std::string> originModeNames = {
		{Augmenta::ProtocolOptions::AxisTransform::OriginMode::BottomLeft, "bottom_left"},
		{Augmenta::ProtocolOptions::AxisTransform::OriginMode::BottomRight, "bottom_right"},
		{Augmenta::ProtocolOptions::AxisTransform::OriginMode::TopLeft, "top_left"},
		{Augmenta::ProtocolOptions::AxisTransform::OriginMode::TopRight, "top_right"},
	};

	const std::unordered_map<Augmenta::ProtocolOptions::AxisTransform::CoordinateSpace, std::string> coordinateSpaceNames = {
		{Augmenta::ProtocolOptions::AxisTransform::CoordinateSpace::Absolute, "absolute"},
		{Augmenta::ProtocolOptions::AxisTransform::CoordinateSpace::Relative, "relative"},
		{Augmenta::ProtocolOptions::AxisTransform::CoordinateSpace::Normalized, "normalized"},
	};

	// Read an int from a byte buffer. Returns the number of bytes read.
	template <typename T>
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

namespace Augmenta
{
	bool ProtocolOptions::AxisTransform::operator==(const AxisTransform &other) const
	{
		return axis == other.axis &&
			   origin == other.origin &&
			   flipX == other.flipX &&
			   flipY == other.flipY &&
			   flipZ == other.flipZ &&
			   coordinateSpace == other.coordinateSpace;
	}

	bool ProtocolOptions::AxisTransform::operator!=(const AxisTransform &other) const
	{
		return !(*this == other);
	}

	bool ProtocolOptions::operator==(const ProtocolOptions &other) const
	{
		if (tags.size() != other.tags.size())
		{
			return false;
		}

		for (int idx = 0; idx < tags.size(); ++idx)
		{
			if (tags[idx] != other.tags[idx])
			{
				return false;
			}
		}

		return version == other.version &&
			   downSample == other.downSample &&
			   streamClouds == other.streamClouds &&
			   streamClusters == other.streamClusters &&
			   streamClusterPoints == other.streamClusterPoints &&
			   streamZonePoints == other.streamZonePoints &&
			   boxRotationMode == other.boxRotationMode &&
			   axisTransform == other.axisTransform &&
			   useCompression == other.useCompression &&
			   usePolling == other.usePolling;
	}

	bool ProtocolOptions::operator!=(const ProtocolOptions &other) const
	{
		return !(*this == other);
	}

	struct DataBlobParser
	{
		enum class PacketType : uint8_t
		{
			Object = 0,
			ZoneEvent = 1,
			Scene = 2,
			Bundle = 255,
		};

		enum class PropertyType : int
		{
			Points = 0,
			Cluster = 1,
		};

		static size_t processPacket(const std::byte *packetBuffer, DataBlob &outDataBlob, const ProtocolOptions &options)
		{
			size_t offset = 0;

			int packetSize;
			offset += ReadBinary(packetBuffer, &packetSize);

			PacketType type;
			offset += ReadBinary(packetBuffer + offset, &type);

			switch (type)
			{
			case PacketType::Bundle:
			{
				int packetCount;
				offset += ReadBinary(packetBuffer + offset, &packetCount);

				for (int packetIdx = 0; packetIdx < packetCount; ++packetIdx)
				{
					auto subPacketBegin = packetBuffer + offset;
					offset += processPacket(subPacketBegin, outDataBlob, options);
				}
				break;
			}

			case PacketType::Object:
			{
				auto &object = outDataBlob.objects.emplace_back();
				offset += processObjectPacket(packetBuffer + offset, object, options);
				break;
			}

			case PacketType::ZoneEvent:
			{
				auto &zoneEvent = outDataBlob.zoneEvents.emplace_back();
				offset += processZonePacket(packetBuffer + offset, zoneEvent, options);
				break;
			}

			case PacketType::Scene:
			{
				offset += processScenePacket(packetBuffer + offset, outDataBlob.sceneInfo, options);
				break;
			}

			default:
			{
				assert(false); // Unexpected packet type !
			}
			}

			return offset;
		}

		static size_t processObjectPacket(const std::byte *packetBuffer, DataBlob::ObjectPacket &outObject, const ProtocolOptions &options)
		{
			// At this point packetSize and packetType have already been read
			size_t offset = 0;

			int objectID;
			offset += ReadBinary(packetBuffer + offset, &outObject.id);

			int propertiesCount;
			offset += ReadBinary(packetBuffer + offset, &propertiesCount);

			for (int propertyIdx = 0; propertyIdx < propertiesCount; ++propertyIdx)
			{
				int propertySize;
				offset += ReadBinary(packetBuffer + offset, &propertySize);

				PropertyType propertyID;
				offset += ReadBinary(packetBuffer + offset, &propertyID);

				switch (propertyID)
				{
				case PropertyType::Points:
				{
					offset += processPointCloudProperty(packetBuffer + offset, outObject, options);
					break;
				}

				case PropertyType::Cluster:
				{
					offset += processClusterProperty(packetBuffer + offset, outObject, options);
					break;
				}
				}
			}

			return offset;
		}

		static size_t processPointCloudProperty(const std::byte *buffer, DataBlob::ObjectPacket &object, const ProtocolOptions &options)
		{
			assert(!object.hasPointCloud());

			auto &pointCloud = object.pointCloud.emplace();

			// Size and ID have already been read
			size_t offset = 0;

			offset += ReadBinary(buffer + offset, &pointCloud.pointsCount);
			pointCloud.pointsPtr = buffer + offset;

			return offset + (pointCloud.pointsCount * sizeof(float) * 3);
		}

		static size_t processClusterProperty(const std::byte *buffer, DataBlob::ObjectPacket &object, const ProtocolOptions &options)
		{
			assert(!object.hasCluster());

			auto &cluster = object.cluster.emplace();

			// Size and ID have already been read
			size_t offset = 0;

			offset += ReadBinary(buffer + offset, &cluster.state);
			offset += ReadVector<float>(buffer + offset, cluster.centroid.data(), 3);
			offset += ReadVector<float>(buffer + offset, cluster.velocity.data(), 3);
			offset += ReadVector<float>(buffer + offset, cluster.boundingBoxCenter.data(), 3);
			offset += ReadVector<float>(buffer + offset, cluster.boundingBoxSize.data(), 3);
			offset += ReadBinary(buffer + offset, &cluster.weight);

			if (options.boxRotationMode == ProtocolOptions::RotationMode::Quaternions)
			{
				offset += ReadVector<float>(buffer + offset, cluster.boundingBoxRotation.data(), 4);
			}
			else
			{
				offset += ReadVector<float>(buffer + offset, cluster.boundingBoxRotation.data(), 3);
			}

			offset += ReadVector<float>(buffer + offset, cluster.lookAt.data(), 3);

			return offset;
		}

		static size_t processZonePacket(const std::byte *buffer, DataBlob::ZoneEventPacket &outZoneEvent, const ProtocolOptions &options)
		{
			size_t offset = 0;

			// Size and type have already been read
			int emitterZoneAddressSize;
			offset += ReadBinary(buffer + offset, &emitterZoneAddressSize);

			outZoneEvent.emitterZoneAddress.resize(emitterZoneAddressSize);
			offset += ReadVector<char>(buffer + offset, outZoneEvent.emitterZoneAddress.data(), emitterZoneAddressSize);

			offset += ReadBinary(buffer + offset, &outZoneEvent.enters);
			offset += ReadBinary(buffer + offset, &outZoneEvent.leaves);
			offset += ReadBinary(buffer + offset, &outZoneEvent.presence);
			offset += ReadBinary(buffer + offset, &outZoneEvent.density);

			int propertiesCount;
			offset += ReadBinary(buffer + offset, &propertiesCount);

			for (int propertyIdx = 0; propertyIdx < propertiesCount; ++propertyIdx)
			{
				auto &property = outZoneEvent.properties.emplace_back();

				offset += sizeof(int); // Skip size
				offset += ReadBinary(buffer + offset, &property.type);

				switch (property.type)
				{
				case DataBlob::ZoneEventPacket::Property::Type::Slider:
				{
					auto &data = property.data.emplace<DataBlob::ZoneEventPacket::SliderProperty>();
					offset += ReadBinary(buffer + offset, &data.value);
					break;
				}
				case DataBlob::ZoneEventPacket::Property::Type::XYPad:
				{
					auto &data = property.data.emplace<DataBlob::ZoneEventPacket::XYPadProperty>();
					offset += ReadBinary(buffer + offset, &data.x);
					offset += ReadBinary(buffer + offset, &data.y);
					break;
				}
				case DataBlob::ZoneEventPacket::Property::Type::PointCloud:
				{
					auto &data = property.data.emplace<DataBlob::PointCloudProperty>();
					offset += ReadBinary(buffer + offset, &data.pointsCount);
					data.pointsPtr = buffer + offset;
					offset += data.pointsCount * sizeof(float) * 3;
					break;
				}
				default:
					throw(std::runtime_error("Unknown zone event property type encountered."));
				}
			}

			return offset;
		}

		static size_t processScenePacket(const std::byte *buffer, DataBlob::SceneInfoPacket &outScene, const ProtocolOptions &options)
		{
			// Packet size and ID have already been read
			size_t offset = 0;

			int addressSize = 0;
			offset += ReadBinary(buffer + offset, &addressSize);

			outScene.sceneAddress.resize(addressSize);
			offset += ReadVector<char>(buffer + offset, outScene.sceneAddress.data(), addressSize);

			return offset;
		}
	};

	struct ControlMessageParser
	{
		static void parseContainer(const nlohmann::json &containerJson, ControlMessage::Container &outContainer)
		{
			outContainer.name = containerJson.value("name", "");

			auto typeJson = containerJson.find("type");
			if (typeJson != containerJson.end())
			{
				outContainer.address = containerJson.value("address", "");
				outContainer.position = ReadJSON<std::array<float, 3>>(containerJson, "position", {0, 0, 0});
				outContainer.rotation = ReadJSON<std::array<float, 3>>(containerJson, "rotation", {0, 0, 0});
				outContainer.color = ReadJSON<std::array<float, 4>>(containerJson, "color", {0, 0, 0, 0});

				const std::string &typeStr = *typeJson;
				if (typeStr == "Zone")
				{
					outContainer.type = ControlMessage::Container::Type::Zone;
					auto &parameters = outContainer.parameters.emplace<ControlMessage::Container::ZoneParameters>();

					auto shapeJson = containerJson["shape"];
					auto shapeTypeStr = shapeJson["type"];
					assert(!shapeTypeStr.empty());

					if (shapeTypeStr == "Box")
					{
						parameters.shapeType = ControlMessage::Container::ShapeType::Box;
						
						auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::BoxShapeParameters>();
						shapeParameters.size = ReadJSON<std::array<float, 3>>(shapeJson, "boxSize", {0, 0, 0});
					}
					else if (shapeTypeStr == "Cylinder")
					{
						parameters.shapeType = ControlMessage::Container::ShapeType::Cylinder;
						
						auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::CylinderShapeParameters>();
						shapeParameters.height = shapeJson["height"];
						shapeParameters.radius = shapeJson["radius"];
					}
					else if (shapeTypeStr == "Sphere")
					{
						parameters.shapeType = ControlMessage::Container::ShapeType::Sphere;
						
						auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::SphereShapeParameters>();
						shapeParameters.radius = shapeJson["radius"];
					}
					else if (shapeTypeStr == "Path")
					{
						parameters.shapeType = ControlMessage::Container::ShapeType::Path;
						
						auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::PathShapeParameters>();
					}
					else if (shapeTypeStr == "Grid")
					{
						parameters.shapeType = ControlMessage::Container::ShapeType::Grid;
						
						auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::GridShapeParameters>();
					}
					else if (shapeTypeStr == "Polygon")
					{
						parameters.shapeType = ControlMessage::Container::ShapeType::Polygon;
						
						auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::PolygonShapeParameters>();
					}
					else if (shapeTypeStr == "Segment")
					{
						parameters.shapeType = ControlMessage::Container::ShapeType::Segment;
						
						auto &shapeParameters = parameters.shapeParameters.emplace<ControlMessage::Container::SegmentShapeParameters>();
					}
					else
					{
						assert(false); // Unexpected shape type
					}
				}
				else if (typeStr == "Scene")
				{
					outContainer.type = ControlMessage::Container::Type::Scene;

					auto &parameters = outContainer.parameters.emplace<ControlMessage::Container::SceneParameters>();
					parameters.size = ReadJSON<std::array<float, 3>>(containerJson, "size", {0, 0, 0});
				}
				else // Container
				{
					outContainer.type = ControlMessage::Container::Type::Container;
					auto &parameters = outContainer.parameters.emplace<ControlMessage::Container::ContainerParameters>();
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
			auto status = messageJson.value("status", "");
			if (status == "ok")
			{
				outMessage.status = ControlMessage::Status::Ok;
			}
			else if (status == "error")
			{
				outMessage.status = ControlMessage::Status::Error;
				outMessage.errorMessage = messageJson.value("error", "");
			}

			outMessage.serverProtocolVersion = messageJson.value("version", 2);

			auto setupIt = messageJson.find("setup");
			if (setupIt != messageJson.end())
			{
				outMessage.type = ControlMessage::Type::Setup;
				const auto worldIt = setupIt->find("world");
				ControlMessageParser::parseContainer(*worldIt, outMessage.rootObject);
			}

			auto updateIt = messageJson.find("update");
			if (updateIt != messageJson.end())
			{
				outMessage.type = ControlMessage::Type::Update;
				auto updatedObject = updateIt->front();
				ControlMessageParser::parseContainer(updatedObject, outMessage.rootObject);
			}
		}
	};

	void Client::initialize(const std::string &clientName, const ProtocolOptions &desiredOptions)
	{
		name = clientName;
		options = desiredOptions;

		initialized = true;
	}

	void Client::shutdown()
	{
		uncompressedBuffer.clear();

		initialized = false;

		options = ProtocolOptions();
		name = "";
	}

	void Client::clearTags()
	{
		tags.clear();
	}
	
	void Client::addTag(const std::string& tag)
	{
		tags.push_back(tag);
	}

	DataBlob Client::parseDataBlob(const std::byte *buffer, size_t bufferSize)
	{
		assert(initialized);

		if (isBigEndian())
		{
			throw(std::runtime_error("The system appears to be big endian, while only little endian is supported."));
		}

		const std::byte *dataBuffer = buffer;

		if (options.useCompression)
		{
			// Decompress
			size_t uncompressedSize = ZSTD_getFrameContentSize(buffer, bufferSize);

			if (ZSTD_isError(uncompressedSize))
			{
				throw std::runtime_error("Error during data blob decompression.");
			}

			uncompressedBuffer.resize(uncompressedSize);
			size_t actualSize = ZSTD_decompress(uncompressedBuffer.data(), uncompressedSize, buffer, bufferSize);
			uncompressedBuffer.resize(actualSize); // Shrink back if necessary

			dataBuffer = uncompressedBuffer.data();
		};

		DataBlob blob;
		DataBlobParser::processPacket(dataBuffer, blob, options);

		return blob;
	}

	ControlMessage Client::parseControlMessage(const char *rawMessage)
	{
		assert(initialized);

		auto messageJson = nlohmann::json::parse(rawMessage);

		ControlMessage outMessage;
		ControlMessageParser::parseControlMessage(messageJson, outMessage);

		return outMessage;
	}

	std::string Client::getRegisterMessage() const
	{
		assert(initialized);

		nlohmann::json optionsJson;
		optionsJson["version"] = options.version;
		optionsJson["tags"] = tags;
		optionsJson["streamClouds"] = options.streamClouds;
		optionsJson["streamClusters"] = options.streamClusters;
		optionsJson["streamClusterPoints"] = options.streamClusterPoints;
		optionsJson["streamZonePoints"] = options.streamZonePoints;
		optionsJson["downSample"] = options.downSample;
		optionsJson["boxRotationMode"] = rotationModeNames.at(options.boxRotationMode);
		optionsJson["useCompression"] = options.useCompression;
		optionsJson["usePolling"] = options.usePolling;

		nlohmann::json axisTransformJson;
		axisTransformJson["axis"] = axisModeNames.at(options.axisTransform.axis);
		axisTransformJson["origin"] = originModeNames.at(options.axisTransform.origin);
		axisTransformJson["flipX"] = options.axisTransform.flipX;
		axisTransformJson["flipY"] = options.axisTransform.flipY;
		axisTransformJson["flipZ"] = options.axisTransform.flipZ;
		axisTransformJson["coordinateSpace"] = coordinateSpaceNames.at(options.axisTransform.coordinateSpace);
		// TODO: OriginOffset
		// TODO: customMatrix

		optionsJson["axisTransform"] = axisTransformJson;

		nlohmann::json registerJson;
		registerJson["name"] = name;
		registerJson["application-name"] = applicationName;
		registerJson["application-version"] = applicationVersion;
		registerJson["options"] = optionsJson;

		nlohmann::json dataJson;
		dataJson["register"] = registerJson;
		return dataJson.dump();
	}

	std::string Client::getPollMessage() const
	{
		assert(initialized);
		assert(options.usePolling);

		nlohmann::json pollJson;
		pollJson["poll"] = true;
		return pollJson.dump();
	}
}
