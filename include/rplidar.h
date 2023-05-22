#ifndef RPLIDAR_H
#define RPLIDAR_H

#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/uart.h>
#include <hardware/irq.h>
#include <stdio.h>
#include <time.h>
#include <vector>

#define RPLIDAR_DEBUG 0
#define RPLIDAR_DEBUG_FUNCS 0

#define DEBUG_PRINT(...) if (RPLIDAR_DEBUG) printf(__VA_ARGS__)

#define RPLIDAR_DEFAULT_INTERFACE uart0
#define RPLIDAR_DEFAULT_BAUDRATE 115200
#define RPLIDAR_DEFAULT_TX_PIN 0
#define RPLIDAR_DEFAULT_RX_PIN 1

#define RPLIDAR_REQUEST_START_BYTE 0xA5
#define RPLIDAR_RESPONSE_START_BYTE1 0xA5
#define RPLIDAR_RESPONSE_START_BYTE2 0x5A

#define RPLIDAR_DEFAULT_MOTOR_PIN 22
#define RPLIDAR_DEFAULT_MOTOR_DUTY_CYCLE 100 // %
#define RPLIDAR_DEFAULT_MOTOR_FREQUENCY 10000 // Hz

#define RPLIDAR_BUFFER_SIZE 4096

namespace RPLidar {

enum ResponseMode {
	NoResp = -1,	// No response required
	Single = 0,		// Single request-single response mode
	Multi = 1		// Single request-multiple response mode
	// 2 an 3 are reserved for future use
};

struct Command {
	uint8_t value;
	bool requiresPayload;
	ResponseMode responseMode;
};

const Command CMD_STOP =			{ 0x25, false, NoResp };
const Command CMD_RESET =			{ 0x40, false, NoResp };
const Command CMD_SCAN =			{ 0x20, false, Multi };
const Command CMD_EXPRESS_SCAN =	{ 0x82, true, Multi };
const Command CMD_FORCE_SCAN =		{ 0x21, false, Multi };
const Command CMD_GET_INFO =		{ 0x50, false, Single };
const Command CMD_GET_HEALTH =		{ 0x52, false, Single };
const Command CMD_GET_SAMPLERATE =	{ 0x59, false, Single };

class RPLidar {
public:
	RPLidar(uint baudrate = RPLIDAR_DEFAULT_BAUDRATE, uint txPin = RPLIDAR_DEFAULT_TX_PIN, uint rxPin = RPLIDAR_DEFAULT_RX_PIN, uint motorPin = RPLIDAR_DEFAULT_MOTOR_PIN, uart_inst_t* interface = RPLIDAR_DEFAULT_INTERFACE);
	~RPLidar();

	void setMotorDutyCycle(uint8_t dutyCycle = RPLIDAR_DEFAULT_MOTOR_DUTY_CYCLE);
	void setMotorFrequency(uint frequency = RPLIDAR_DEFAULT_MOTOR_FREQUENCY);

	bool startScan();
	void stopScan();
	void reset();
	void getDeviceInfo(uint8_t &model, uint16_t &firmware, uint8_t &hardware, uint8_t *serialNumber);
	void getHealth(uint8_t &status, uint16_t &error);
	void getSampleRate();
	
	void startExpressScan();
	void forceScan();

	void processData();
	
	void getScan(uint16_t* distances);
	// void getScan(std::vector<uint16_t> &distances, std::vector<uint16_t> &angles, std::vector<uint8_t> &qualities);

#if RPLIDAR_DEBUG_FUNCS
	void debugPrintBuffer();
#endif

private:
	static void uartIRQHandler();

	void sendCommand(Command cmd, uint8_t *payload = nullptr, uint8_t payloadSize = 0);
	
	void waitForResponse();
	bool verifyResponseStartBytes();

	void waitForData(uint dataBytes);

	static RPLidar *_singleton;

	uint _motorPin;
	uint8_t _motorDutyCycle = RPLIDAR_DEFAULT_MOTOR_DUTY_CYCLE;
	uint _motorFrequency = RPLIDAR_DEFAULT_MOTOR_FREQUENCY;

	uart_inst_t* _interface;

	bool isScanning = false;
	bool isExpressScanning = false;
	bool isForcingScan = false;
	uint scanPacketOffset = 0;

	bool waitingForResponse = false;
	bool responseReceived = false;

	bool waitingForData = false;
	uint dataBytesRemaining = 0;
	bool dataReceived = false;

	uint8_t dataBuffer[RPLIDAR_BUFFER_SIZE];
	uint16_t dataBufferIndex = 0;

	uint8_t processBuffer[RPLIDAR_BUFFER_SIZE];
	uint16_t processBufferIndex = 0;
	uint16_t processBufferLen = 0;

	uint8_t response[7];
	
	uint8_t data[256];
	
	uint16_t _distances[360];

	// std::vector<uint16_t> _distances;
	// std::vector<uint16_t> _angles;
	// std::vector<uint8_t> _qualities;
	
	// std::vector<uint16_t> new_distances;
	// std::vector<uint16_t> new_angles;
	// std::vector<uint8_t> new_qualities;
};

} // namespace RPLidar

#endif // RPLIDAR_H