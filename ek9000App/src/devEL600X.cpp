
#include "StreamCore.h"
#include "StreamBusInterface.h"

#include "asynDriver.h"
#include "devEK9000.h"
#include "ekUtil.h"
#include "terminals.g.h"
#include <algorithm>
#include <sys/select.h>

// Enable for write queuing optimization (avoids lock contention but adds potential latency)
//#define WRITE_QUEUE

#pragma pack(1)
struct pdo_el600x_t {
	struct {
		uint8_t transmit_req : 1;
		uint8_t recv_accepted : 1;
		uint8_t init_req : 1;
		uint8_t send_cont : 1;
		uint8_t _r0 : 4;
	} status;
	uint8_t out_len;
	uint8_t buf[22];
};
#pragma pack()

DEFINE_SINGLE_CHANNEL_INPUT_PDO(pdo_el600x_t, EL6002);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(pdo_el600x_t, EL6002);

class drvEL600X : public StreamBusInterface {
public:
	drvEL600X(Client* cl, devEK9000* dev, devEK9000Terminal* term);
	~drvEL600X();
	
	bool lockRequest(unsigned long lockTimeout_ms) OVERRIDE;
	bool unlock() OVERRIDE;
	bool writeRequest(const void* output, size_t size, unsigned long writeTimeout_ms) OVERRIDE;
	bool readRequest(unsigned long replyTimeout_ms, unsigned long readTimeout_ms, ssize_t expectedLength, bool async) OVERRIDE;
	bool supportsAsyncRead() OVERRIDE;
	bool supportsEvent() OVERRIDE;
	bool acceptEvent(unsigned long mask, unsigned long timeout_ms) OVERRIDE;
	bool connectRequest(unsigned long timeout_ms) OVERRIDE;
	bool disconnectRequest() OVERRIDE;
	void finish() OVERRIDE;
	
	static StreamBusInterface* getBusInterface(Client* client, const char* busname, int addr, const char* param);
	
protected:
	devEK9000* m_device;
	devEK9000Terminal* m_term;

	char m_readBuf[2048]; // 2kb buffer should be more than enough
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
	return m_device->lock() == asynSuccess;
}

bool drvEL600X::unlock() {
	return m_device->unlock() == asynSuccess;
}

bool drvEL600X::writeRequest(const void* output, size_t size, unsigned long writeTimeout_ms) {
	uint64_t startMs = util::time_ms();

	asynPrint(m_device->GetAsynUser(), ASYN_TRACEIO_DRIVER, "drvEL600X::writeRequest: output=%p, size=%zu, timeout=%lu\n",
		output, size, writeTimeout_ms);

	// Split the transmission based on what we can transfer at once
	// unfortunately this means we'll spend quite a bit of time doing I/O, so a higher-than-normal write timeout will be required
	// Another concern is lock contention, if we're doing I/O for ~50ms and polling every ~100ms, we're going to end up with some overlap.
	// an optimization for this could be batching write requests and servicing them after poll returns. In that case, all other output
	// records should undergo the same fate. This would also open the door to write combining, if we have a significant number of writes
	// queued at the same time.
	size_t numTransmissions = size / sizeof(pdo_el600x_t::buf);
	for (size_t iTr = 0; iTr < numTransmissions; ++iTr) {
		size_t thisSize = iTr < (numTransmissions-1) ? sizeof(pdo_el600x_t::buf) : size;

		pdo_el600x_t pdo;
		memset(&pdo, 0, sizeof(pdo));

		pdo.out_len = thisSize;
		pdo.status.transmit_req = 1;
		
		memcpy(pdo.buf, static_cast<const char*>(output) + iTr * sizeof(pdo_el600x_t::buf), thisSize);

		int status = m_device->doEK9000IO(1, m_term->m_outputStart, STRUCT_SIZE_TO_MODBUS_SIZE(sizeof(pdo_el600x_t)), 
			reinterpret_cast<uint16_t*>(&pdo));

		asynPrint(m_device->GetAsynUser(), ASYN_TRACEIO_DRIVER,
			"drvEL600X::writeRequest: write fragment %zu/%zu, fragSize=%zu, status=%s\n",
			iTr, numTransmissions, thisSize, devEK9000::ErrorToString(status));

		if (status != EK_EOK) {
			writeCallback(StreamIoFault);
			return false;
		}

		if ((util::time_ms() - startMs) >= writeTimeout_ms) {
			writeCallback(StreamIoTimeout);
			return false;
		}
	}

	asynPrint(m_device->GetAsynUser(), ASYN_TRACEIO_DRIVER, "drvEL600X::writeRequest: completed transfer\n");
	return true;
}

bool drvEL600X::readRequest(unsigned long replyTimeout_ms, unsigned long readTimeout_ms, ssize_t expectedLength, bool async) {
	uint64_t startMs = util::time_ms();

	asynPrint(m_device->GetAsynUser(), ASYN_TRACEIO_DRIVER, 
		"drvEL600X:readRequest: replyTimeout=%lu, readTimeout=%lu, expectLen=%ld, asyn=%s\n",
		replyTimeout_ms, readTimeout_ms, expectedLength, async ? "true" : "false");

	size_t bufOff = 0;

	const size_t numTrans = expectedLength / sizeof(pdo_el600x_t::buf);
	for (size_t nTi = 0; nTi < numTrans; ++nTi) {
		const size_t thisSize = nTi < (numTrans-1) ? sizeof(pdo_el600x_t::buf) : expectedLength;

		pdo_el600x_t pdo;

		int status = m_device->doModbusIO(0, MODBUS_READ_INPUT_REGISTERS, m_term->m_inputStart, 
			reinterpret_cast<uint16_t*>(&pdo), STRUCT_SIZE_TO_MODBUS_SIZE(thisSize));

		asynPrint(m_device->GetAsynUser(), ASYN_TRACEIO_DRIVER, 
			"drvEL600X:readRequest: read fragment %zu/%zu, size=%zu, status=%s\n",
			nTi, numTrans, thisSize, devEK9000::ErrorToString(status));

		if ((bufOff + pdo.out_len) > sizeof(m_readBuf)) {
			LOG_ERROR(m_device, "buffer overflow, sizeof(m_readBuf) == %zu\n", sizeof(m_readBuf));
			readCallback(StreamIoFault);
			return false;
		}

		// Copy into the staging buffer
		memcpy(m_readBuf + bufOff, pdo.buf, pdo.out_len);
		bufOff += pdo.out_len;

		if (status != EK_EOK) {
			readCallback(StreamIoFault);
			return false;
		}

		if (util::time_ms() - startMs >= readTimeout_ms) {
			readCallback(StreamIoTimeout);
			return false;
		}
	}

	asynPrint(m_device->GetAsynUser(), ASYN_TRACEIO_DRIVER, "drvEL600X:readRequest: read completed\n");
	return true;
}

bool drvEL600X::supportsAsyncRead() {
	return false; // For now, no. Can be implemented using getEK9000IO or queued IO requests
}

bool drvEL600X::supportsEvent() {
	return false;
}

bool drvEL600X::acceptEvent(unsigned long mask, unsigned long timeout_ms) {
	return false;
}

bool drvEL600X::connectRequest(unsigned long timeout_ms) {
	uint64_t startMs = util::time_ms();
	const uint64_t sleepDur = std::clamp<uint64_t>(timeout_ms / 100, 5, 50);
	
	// This is probably not necessary, but keep asking if we're connected before returning
	while (!m_device->VerifyConnection()) {
		epicsThreadSleep(sleepDur);
		if (util::time_ms() - startMs >= timeout_ms) {
			connectCallback(StreamIoTimeout);
			return false;
		}
	}
	connectCallback();
	return true;
}

bool drvEL600X::disconnectRequest() {
	disconnectCallback();
	return true; // no-op, we're always connected!
}

void drvEL600X::finish() {
	
}
