
#include "StreamCore.h"
#include "StreamBusInterface.h"

#include "devEK9000.h"

class drvEL600X : public StreamBusInterface {
public:
	drvEL600X(Client* cl, devEK9000* dev, devEK9000Terminal* term);
	~drvEL600X();
	
	bool lockRequest(unsigned long lockTimeout_ms) override;
	bool unlock() override;
	bool writeRequest(const void* output, size_t size, unsigned long writeTimeout_ms) override;
	bool readRequest(unsigned long replyTimeout_ms, unsigned long readTimeout_ms, ssize_t expectedLength, bool async) override;
	bool supportsAsyncRead() override;
	bool supportsEvent() override;
	bool acceptEvent(unsigned long mask, unsigned long timeout_ms) override;
	bool connectRequest(unsigned long timeout_ms) override;
	bool disconnectRequest() override;
	void finish() override;
	
	static StreamBusInterface* getBusInterface(Client* client, const char* busname, int addr, const char* param);
	
protected:
	devEK9000* m_device;
	devEK9000Terminal* m_term;
};

RegisterStreamBusInterface(drvEL600X);

StreamBusInterface* drvEL600X::getBusInterface(Client *client, const char *busname, int addr, const char *param) {

	devEK9000* pdev = devEK9000::FindDevice(busname);
	if (!pdev) {
		epicsStdoutPrintf("getBusInterface: could not find device '%s'\n", busname);
		return nullptr;
	}
	
	if (addr < 0 || addr > pdev->m_numTerms) {
		epicsStdoutPrintf("getBusInterface: invalid terminal index %d, must be 0<x<=%d\n", addr, pdev->m_numTerms);
		return nullptr;
	}
	
	char paramStr[256];
	strncpy(paramStr, param, sizeof(paramStr));
	paramStr[sizeof(paramStr)-1] = 0;
	
	const char* paramName = strtok(paramStr, "=");
	const char* paramValue = strtok(paramStr, "=");
	
	if (!paramName || !paramValue || strcmp(paramName, "channel") != 0) {
		epicsStdoutPrintf("getBusInterface: channel number must be specified as a parameter. i.e. channel=1\n");
		return nullptr;
	}

	int channel = 0;
	if (epicsParseInt32(paramValue, &channel, 10, NULL) != 0) {
		epicsStdoutPrintf("getBusInterface: channel is not a valid integer\n");
		return nullptr;
	}
	if (channel < 1) {
		epicsStdoutPrintf("getBusInterface: channel must be >= 1\n");
		return nullptr;
	}
	return new drvEL600X(client, pdev, pdev->TerminalByIndex(addr));
}

drvEL600X::drvEL600X(Client* cl, devEK9000* dev, devEK9000Terminal* term) :
	StreamBusInterface(cl),
	m_device(dev),
	m_term(term)
{
}

drvEL600X::~drvEL600X() {
}

bool drvEL600X::lockRequest(unsigned long lockTimeout_ms) {
	return false;
}

bool drvEL600X::unlock() {
	return false;
}

bool drvEL600X::writeRequest(const void* output, size_t size, unsigned long writeTimeout_ms) {
	return false;
}

bool drvEL600X::readRequest(unsigned long replyTimeout_ms, unsigned long readTimeout_ms, ssize_t expectedLength, bool async) {
	return false;
}

bool drvEL600X::supportsAsyncRead() {
	return true; // we can use getEK9000IO
}

bool drvEL600X::supportsEvent() {
	return false;
}

bool drvEL600X::acceptEvent(unsigned long mask, unsigned long timeout_ms) {
	return false;
}

bool drvEL600X::connectRequest(unsigned long timeout_ms) {
	return false;
}

bool drvEL600X::disconnectRequest() {
	return false;
}

void drvEL600X::finish() {
	
}
