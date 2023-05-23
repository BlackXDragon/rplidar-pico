#include "rplidar.h"

namespace RPLidar {

RPLidar* RPLidar::_singleton = nullptr;

RPLidar::RPLidar(uint baudrate, uint txPin, uint rxPin, uint motorPin, uart_inst_t* interface) {
	DEBUG_PRINT("[RPLidar] constructor\n");

	if (_singleton != nullptr) {
		return;
	}
	_singleton = this;
	
	DEBUG_PRINT("[RPLidar] Singleton set\n");

	_interface = interface;
	_motorPin = motorPin;

	gpio_set_function(_motorPin, GPIO_FUNC_PWM);

	setMotorFrequency();

	uint slice_num = pwm_gpio_to_slice_num(_motorPin);
	pwm_set_enabled(slice_num, true);

	DEBUG_PRINT("[RPLidar] PWM enabled\n");

	uart_init(interface, baudrate);
	gpio_set_function(txPin, GPIO_FUNC_UART);
	gpio_set_function(rxPin, GPIO_FUNC_UART);

	uart_set_hw_flow(interface, false, false);

	uart_set_format(interface, 8, 1, UART_PARITY_NONE);

	uart_set_fifo_enabled(interface, false);

	int uart_irq = interface == uart0 ? UART0_IRQ : UART1_IRQ;

	irq_set_exclusive_handler(uart_irq, uartIRQHandler);
	irq_set_enabled(uart_irq, true);

	uart_set_irq_enables(interface, true, false);

	DEBUG_PRINT("[RPLidar] UART initialized\n");

	// reset();

	// uint8_t model;
	// uint16_t firmware;
	// uint8_t hardware;
	// uint8_t serialNumber[16];
	// getDeviceInfo(model, firmware, hardware, serialNumber);
	// printf("RPLidar model: %d\n", model);
	// printf("RPLidar firmware: %d\n", firmware);
	// printf("RPLidar hardware: %d\n", hardware);
	// printf("RPLidar serial number: ");
	// for (int i = 0; i < 16; i++) {
	// 	printf("%02x", serialNumber[i]);
	// }

	uint8_t status;
	uint16_t error;
	getHealth(status, error);
	printf("RPLidar health status: %d\n", status);
	if (status != 0) {
		printf("RPLidar error code: %d\n", error);
		pwm_set_enabled(slice_num, false);
		return;
	}
}

RPLidar::~RPLidar() {
	stopScan();

	uart_deinit(_interface);

	irq_set_enabled(_interface == uart0 ? UART0_IRQ : UART1_IRQ, false);

	uart_set_irq_enables(_interface, false, false);
}

void RPLidar::setMotorDutyCycle(uint8_t dutyCycle) {
	_motorDutyCycle = dutyCycle;

	// Calculate the number of cycles for the duty cycle from the frequency
	uint16_t cycles = (125e6 / _motorFrequency) * _motorDutyCycle / 100;
	// uint16_t cycles = 6250;

	// Set the gpio level to 1 when the counter is less than the number of cycles
	pwm_set_gpio_level(_motorPin, cycles - 1);
	const uint LED_PIN = 2;
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	gpio_put(LED_PIN, true);
}

void RPLidar::setMotorFrequency(uint frequency) {
	// Verify that the frequency is less than 125MHz / (2**16)
	if (frequency < 1907) {
		const uint LED_PIN = PICO_DEFAULT_LED_PIN;
		gpio_init(LED_PIN);
		gpio_set_dir(LED_PIN, GPIO_OUT);
		gpio_put(LED_PIN, true);
		return;
	}

	_motorFrequency = frequency;
	uint slice_num = pwm_gpio_to_slice_num(_motorPin);
	// uint slice_num = 3;

	// Calculate the wrap value from the frequency
	uint16_t wrap = 125e6 / _motorFrequency;
	// uint16_t wrap = 12500;

	if (wrap >= 0xffff) {
		// The wrap value is too large
		const uint LED_PIN = PICO_DEFAULT_LED_PIN;
		gpio_init(LED_PIN);
		gpio_set_dir(LED_PIN, GPIO_OUT);
		gpio_put(LED_PIN, true);
		return;
	}

	// Set the wrap value
	pwm_set_wrap(slice_num, wrap);

	setMotorDutyCycle();
}

bool RPLidar::startScan() {
	sendCommand(CMD_SCAN);

	waitForResponse();

	// Check if the scan started successfully
	if (!verifyResponseStartBytes()) {
		return false;
	}
	if (response[2] != 0x05 || response[5] != 0x40 || response[6] != 0x81) {
		return false;
	}

	isScanning = true;

	DEBUG_PRINT("[RPLidar] Scan started\n");
	return true;
}

void RPLidar::stopScan() {
	if (!isScanning || isForcingScan || isExpressScanning) {
		return;
	}

	sendCommand(CMD_STOP);

	sleep_ms(10);

	isScanning = false;

	DEBUG_PRINT("[RPLidar] Scan stopped\n");
}

void RPLidar::reset() {
	DEBUG_PRINT("[RPLidar] Resetting\n");
	sendCommand(CMD_RESET);
	sleep_ms(1000);
	dataBufferIndex = 0;
	DEBUG_PRINT("[RPLidar] Reset\n");
}

void RPLidar::getDeviceInfo(uint8_t &model, uint16_t &firmware, uint8_t &hardware, uint8_t *serialNumber) {
	DEBUG_PRINT("[RPLidar] Getting device info\n");
	sendCommand(CMD_GET_INFO);

	waitForResponse();

	// Check if the device info was received successfully
	if (!verifyResponseStartBytes()) {
		return;
	}
	if (response[2] != 0x14 || response[6] != 0x04) {
		return;
	}

	// Wait for 20 bytes of data
	waitForData(20);

	model = data[0];

	firmware = data[1] | (data[2] << 8);

	hardware = data[3];

	for (int i = 0; i < 16; i++) {
		serialNumber[i] = data[4 + i];
	}
}

void RPLidar::getHealth(uint8_t &status, uint16_t &error) {
	DEBUG_PRINT("[RPLidar] Getting health\n");
	sendCommand(CMD_GET_HEALTH);

	waitForResponse();

	// Check if the health was received successfully
	if (!verifyResponseStartBytes()) {
		return;
	}
	if (response[2] != 0x03 || response[5] != 0x40 || response[6] != 0x81) {
		return;
	}

	// Wait for 3 bytes of data
	waitForData(3);

	status = data[0];

	error = data[1] | (data[2] << 8);
}

void RPLidar::getSampleRate() {
	sendCommand(CMD_GET_SAMPLERATE);
}

void RPLidar::startExpressScan() {
	sendCommand(CMD_EXPRESS_SCAN);
}

void RPLidar::forceScan() {
	sendCommand(CMD_FORCE_SCAN);
}

void RPLidar::processData() {
	if (!(isScanning || isForcingScan || isExpressScanning))
		return;
	if (!newScan)
		return;
	
	newScan = false;

	// Verify that the first byte is start of scan
	if (!((dataBuffer[0] & 0x01) && !((dataBuffer[0] & 0x02) >> 1) && (dataBuffer[1] & 0x01))) {
		DEBUG_PRINT("[RPLidar] Invalid start of scan\n");
		return;
	}

	// DEBUG_PRINT("[RPLidar] Processing data\n");
	processBufferIndex = 0;
	while (processBufferLen - processBufferIndex > 4) {
		// Verify the checkbits
		if ((dataBuffer[processBufferIndex] & 0x01) == !((dataBuffer[processBufferIndex] & 0x02) >> 1) && (dataBuffer[processBufferIndex + 1] & 0x01)) {
			_qualities.clear();
			_angles.clear();
			_distances.clear();
			_qualities.push_back(dataBuffer[processBufferIndex] >> 2);
			_angles.push_back(((dataBuffer[processBufferIndex + 1] >> 1) | (dataBuffer[processBufferIndex + 2] << 7)) / 64);
			_distances.push_back((dataBuffer[processBufferIndex + 3] | (dataBuffer[processBufferIndex + 4])) / 4000.0);
			processBufferIndex += 5;
		} else {
			// Erroneous data
			DEBUG_PRINT("[RPLidar] Invalid data: %#X %#X\n", dataBuffer[processBufferIndex], dataBuffer[processBufferIndex + 1]);
		}
	}
}

void RPLidar::getScan(std::vector<float> &distances, std::vector<uint16_t> &angles) {
	std::vector<uint8_t> temp;
	getScan(distances, angles, temp);
}

void RPLidar::getScan(std::vector<float> &distances, std::vector<uint16_t> &angles, std::vector<uint8_t> &qualities) {
	// distances.clear();
	// angles.clear();
	// qualities.clear();
	// std::copy(_distances.begin(), _distances.end(), std::back_inserter(distances));
	// std::copy(_angles.begin(), _angles.end(), std::back_inserter(angles));
	// std::copy(_qualities.begin(), _qualities.end(), std::back_inserter(qualities));
	distances = _distances;
	angles = _angles;
	qualities = _qualities;
	// DEBUG_PRINT("[RPLidar] getScan: %d %d\n", _distances.size(), _angles.size());
	// DEBUG_PRINT("[RPLidar] %d %d\n", distances.size(), angles.size());
}

#if RPLIDAR_DEBUG_FUNCS
void RPLidar::debugPrintBuffer() {
	for (int i = 0; i < dataBufferIndex; i++) {
		printf("%#X ", dataBuffer[i]);
	}
	printf("\n");
}

void RPLidar::debugPrintLength() {
	printf("Distances len: %d\n", new_distances.size());
}
#endif

void RPLidar::uartIRQHandler() {
	RPLidar *rpl = _singleton;
	// DEBUG_PRINT("[RPLidar] UART IRQ\n");
	while (uart_is_readable(rpl->_interface)) {

        rpl->dataBuffer[rpl->dataBufferIndex++] = uart_getc(rpl->_interface);

		// DEBUG_PRINT("[RPLidar] Received byte: %#X\n", rpl->dataBuffer[rpl->dataBufferIndex - 1]);

		if (rpl->dataBufferIndex > (RPLIDAR_BUFFER_SIZE - 1)) {
			// DEBUG_PRINT("[RPLidar] Buffer overflow\n");
			rpl->dataBufferIndex = 0;
			if (rpl->isScanning) {
				std::copy(rpl->dataBuffer + rpl->processBufferLen, rpl->dataBuffer + RPLIDAR_BUFFER_SIZE, rpl->dataBuffer);
				rpl->dataBufferIndex = RPLIDAR_BUFFER_SIZE - rpl->processBufferLen;
			}
		}
		
		if (rpl->waitingForResponse && rpl->dataBufferIndex == 7 && rpl->dataBuffer[0] == 0xA5 && rpl->dataBuffer[1] == 0x5A) {
			// Received a response
			std::copy(rpl->dataBuffer, rpl->dataBuffer + 7, rpl->response);
			rpl->waitingForResponse = false;
			rpl->responseReceived = true;
			rpl->dataBufferIndex = 0;
			return;
		}

		if (rpl->waitingForData) {
			rpl->dataBytesRemaining--;
			if (rpl->dataBytesRemaining == 0) {
				// Received all the data
				std::copy(rpl->dataBuffer, rpl->dataBuffer + rpl->dataBufferIndex, rpl->data);
				rpl->waitingForData = false;
				rpl->dataReceived = true;
				rpl->dataBufferIndex = 0;
			}
			return;
		}

		if (rpl->isScanning) {
			switch (rpl->scanPacketOffset) {
				case 0:
					if ((rpl->dataBuffer[rpl->dataBufferIndex - 1] & 0x01) == !((rpl->dataBuffer[rpl->dataBufferIndex - 1] & 0x02) >> 1)) {
						// DEBUG_PRINT("[RPLidar] New packet start\n");
						rpl->scanPacketOffset = 1;
						if (rpl->dataBufferIndex >= 5) {
							if (rpl->dataBuffer[rpl->dataBufferIndex - 6] & 0x01) {
								// New scan
								DEBUG_PRINT("[RPLidar] New scan %d\n", rpl->dataBufferIndex - 6);
								std::copy(rpl->dataBuffer, rpl->dataBuffer + rpl->dataBufferIndex - 6, rpl->processBuffer);
								rpl->processBufferLen = rpl->dataBufferIndex - 6;
								rpl->newScan = true;
								std::copy(rpl->dataBuffer + rpl->dataBufferIndex - 6, rpl->dataBuffer + rpl->dataBufferIndex, rpl->dataBuffer);
								rpl->dataBufferIndex = 6;
							}
						}
					}
					break;
				case 1:
					if (rpl->dataBuffer[rpl->dataBufferIndex - 1] & 0x01)
						rpl->scanPacketOffset = 2;
					else
						rpl->scanPacketOffset = 0;
					break;
				default:
					rpl->scanPacketOffset++;
					rpl->scanPacketOffset %= 5;
					break;
			}
		}
    }
}

void RPLidar::sendCommand(Command cmd, uint8_t *payload, uint8_t payloadSize) {
	DEBUG_PRINT("[RPLidar] Sending command: ");
	if (cmd.requiresPayload && payload == nullptr)
		return;

	uint8_t packet[cmd.requiresPayload ? payloadSize + 4 : 2];

	packet[0] = RPLIDAR_REQUEST_START_BYTE;
	packet[1] = cmd.value;
	if (cmd.requiresPayload) {
		packet[2] = payloadSize;
		std::copy(payload, payload + payloadSize, packet + 3);
		uint8_t checksum = 0;
		for (int i = 0; i < payloadSize + 2; i++) {
			checksum ^= packet[i];
		}
		packet[payloadSize + 3] = checksum;
	}

	for (int i = 0; i < (cmd.requiresPayload ? payloadSize + 4 : 2); i++) {
		DEBUG_PRINT("%#X ", packet[i]);
	}
	DEBUG_PRINT("\n");

	uart_write_blocking(_interface, packet, cmd.requiresPayload ? payloadSize + 4 : 2);
}

void RPLidar::waitForResponse() {
	DEBUG_PRINT("[RPLidar] Waiting for response\n");
	waitingForResponse = true;
	while (!responseReceived) {
		sleep_ms(1);
		// Debug print the data buffer till the index
		#ifdef RPLIDAR_DEBUG
		for (int i = 0; i < dataBufferIndex; i++) {
			DEBUG_PRINT("%#X ", dataBuffer[i]);
		}
		DEBUG_PRINT("\n");
		#endif
	}
	responseReceived = false;
}

bool RPLidar::verifyResponseStartBytes() {
	return response[0] == RPLIDAR_RESPONSE_START_BYTE1 && response[1] == RPLIDAR_RESPONSE_START_BYTE2;
}

void RPLidar::waitForData(uint dataBytes) {
	dataBytesRemaining = dataBytes;
	waitingForData = true;
	while (!dataReceived) {
		sleep_ms(1);
	}
	dataReceived = false;
}

} // namespace RPLidar