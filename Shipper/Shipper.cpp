#include <Shipper.h>

Shipper::Shipper() {}

Shipper::~Shipper() {}

const String getEventName(String msg) {
	return msg.substring(2, msg.indexOf("\"", 2));
}

const String getEventPayload(String msg) {
	String result = msg.substring(msg.indexOf(",", 0) + 2, msg.length() - 2);
	if(result.startsWith("\"")) {
		result.remove(0, 1);
	}
	if(result.endsWith("\"")) {
		result.remove(result.length() - 1);
	}
	return result;
}

void Shipper::handleEvent(uint8_t * payload) {
	String msg = String((char*)payload);
	trigger(getEventName(msg).c_str(), getEventPayload(msg).c_str(), getEventPayload(msg).length());
}

void Shipper::loop(SocketIOclient &socket) {
	for(auto packet=_packets.begin(); packet != _packets.end();) {
		if(socket.sendEVENT(*packet)) {
			//SOCKETIOCLIENT_DEBUG("[SIoC] packet \"%s\" emitted\n", packet->c_str());
			packet = _packets.erase(packet);
		} else {
			++packet;
		}
	}
}

void Shipper::on(const char* event, std::function<void (const char * payload, size_t length)> func) {
	_events[event] = func;
}

void Shipper::emit(const char* event, const char * payload) {
	_doc.clear();
	JsonArray _ms = _doc.to<JsonArray>();
	
	// add evnet name
    // Hint: socket.on('event_name', ....
	_ms.add(event);
	
	// add payload (parameters) for the event
	JsonObject _params = _ms.createNestedObject();
	
	StaticJsonDocument<1024> params;
	DeserializationError err = deserializeJson(params, payload);
	
	if (err == DeserializationError::Ok) {
		// Get a reference to the root object
		JsonObject obj = params.as<JsonObject>();
		for (JsonObject::iterator it=obj.begin(); it!=obj.end(); ++it) {
			_params[it->key()] = it->value();
		}
	} else {
		params["Error"] = "Don't deserialize form payload.";
		params["payload"] = payload;
	}
	
	// JSON to String (serializion)
	String message;
	serializeJson(_ms, message);
	
	//SOCKETIOCLIENT_DEBUG("[SIoC] add packet %s\n", message.c_str());
	_packets.push_back(message);
}

void Shipper::emit(String event, String payload) {
	_doc.clear();
	JsonArray _ms = _doc.to<JsonArray>();
	
	// add evnet name
    // Hint: socket.on('event_name', ....
	_ms.add(event);
	
	// add payload (parameters) for the event
	JsonObject _params = _ms.createNestedObject();
	
	StaticJsonDocument<1024> params;
	DeserializationError err = deserializeJson(params, payload);
	
	if (err == DeserializationError::Ok) {
		// Get a reference to the root object
		JsonObject obj = params.as<JsonObject>();
		for (JsonObject::iterator it=obj.begin(); it!=obj.end(); ++it) {
			_params[it->key()] = it->value();
		}
	} else {
		params["Error"] = "Don't deserialize form payload.";
		params["payload"] = payload;
	}
	
	// JSON to String (serializion)
	String message;
	serializeJson(_ms, message);
	
	//SOCKETIOCLIENT_DEBUG("[SIoC] add packet %s\n", message.c_str());
	_packets.push_back(message);
}

void Shipper::trigger(const char* event, const char * payload, size_t length) {
	auto e = _events.find(event);
	if(e != _events.end()) {
		//SOCKETIOCLIENT_DEBUG("[SIoC] trigger event %s\n", event);
		e->second(payload, length);
	} else {
		SOCKETIOCLIENT_DEBUG("[SIoC] event %s not found. %d events available\n", event, _events.size());
	}
}