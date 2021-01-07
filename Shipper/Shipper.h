/**
 * SocketIoClient.h
 *
 *  Created on: Dec 27, 2020
 *      Author: Nghia
 */
 
#ifndef SHIPPER_H_
#define SHIPPER_H_

#include <Arduino.h>
#include <map>
#include <vector>
#include "SocketIOclient.h"
#include <ArduinoJson.h>

#define SOCKETIOCLIENT_DEBUG(...) Serial.printf(__VA_ARGS__);
//#define SOCKETIOCLIENT_DEBUG(...)


class Shipper {
protected:
	std::vector<String> _packets;
	std::map<String, std::function<void (const char * payload, size_t length)>> _events;
	StaticJsonDocument<1024> _doc;
	void trigger(const char* event, const char * payload, size_t length);
public:
	Shipper(void);
    virtual ~Shipper(void);

	void loop(SocketIOclient &socket);
	void on(const char* event, std::function<void (const char * payload, size_t length)>);
	void emit(const char* event, const char * payload = NULL);
	void emit(String event, String payload);
	void handleEvent(uint8_t * payload);
};

#endif /* SHIPPER_H_ */