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
	if (!(isScanning || isForcingScan || isExpressScanning)) {
		return;
	}
	if (dataBufferIndex < 5) {
		return;
	}
	processBufferLen = dataBufferIndex;
	std::copy(dataBuffer, dataBuffer + processBufferLen, processBuffer);
	dataBufferIndex = 0;
	// DEBUG_PRINT("[RPLidar] %d\n", processBufferLen);
	processBufferIndex = 0;
	while (processBufferLen - processBufferIndex > 4) {
		// DEBUG_PRINT("[RPLidar] %d\n", processBufferLen - processBufferIndex);
		if (processBuffer[processBufferIndex + 1] & 0x01 && (((processBuffer[processBufferIndex + 0] & 0x02) >> 1) == !(processBuffer[processBufferIndex + 0] & 0x01))) {
			// if (processBuffer[processBufferIndex + 0] & 0x01) {
			// 	// New scan
			// 	DEBUG_PRINT("[RPLidar] New scan\n");
			// }
			// New datapoint
			int angle = ((processBuffer[processBufferIndex + 1] >> 1) | (processBuffer[processBufferIndex + 2] >> 1) << 7 | (processBuffer[processBufferIndex + 2] & 0x80 << 6)) / 64;
			if (angle < 0 || angle > 360) {
				// Invalid angle
				// DEBUG_PRINT("[RPLidar] Invalid angle\n");
				processBufferIndex += 5;
				continue;
			}
			_distances[angle] = (processBuffer[processBufferIndex + 3] | (processBuffer[processBufferIndex + 4] << 8)) / 4;
			// DEBUG_PRINT("[RPLidar] New datapoint %d\n", new_distances.size());
			processBufferIndex += 5;
		} else {
			// Invalid data
			// DEBUG_PRINT("[RPLidar] Invalid data\n");
			// DEBUG_PRINT("[RPLidar] bytes: %#X %#X %#X %#X %#X\n", processBuffer[processBufferIndex + 0], processBuffer[processBufferIndex + 1], processBuffer[processBufferIndex + 2], processBuffer[processBufferIndex + 3], processBuffer[processBufferIndex + 4]);
			// for (int i = processBufferIndex; i < processBufferIndex + 5; i++) {
			// 	DEBUG_PRINT(" %#X", processBuffer[i]);
			// }
			// DEBUG_PRINT("\n");
			processBufferIndex++;
		}
	}
}

void RPLidar::getScan(uint16_t* distances) {
	std::copy(_distances, _distances + 359, distances);
}

// void RPLidar::getScan(std::vector<uint16_t> &distances, std::vector<uint16_t> &angles, std::vector<uint8_t> &qualities) {
// 	// distances.clear();
// 	// angles.clear();
// 	// qualities.clear();
// 	// std::copy(_distances.begin(), _distances.end(), std::back_inserter(distances));
// 	// std::copy(_angles.begin(), _angles.end(), std::back_inserter(angles));
// 	// std::copy(_qualities.begin(), _qualities.end(), std::back_inserter(qualities));
// 	distances = _distances;
// 	angles = _angles;
// 	qualities = _qualities;
// 	DEBUG_PRINT("[RPLidar] getScan: %d %d\n", _distances.size(), _angles.size());
// 	// DEBUG_PRINT("[RPLidar] %d %d\n", distances.size(), angles.size());
// }

#if RPLIDAR_DEBUG_FUNCS
void RPLidar::debugPrintBuffer() {
	for (int i = 0; i < dataBufferIndex; i++) {
		printf("%#X ", dataBuffer[i]);
	}
	printf("\n");
}
#endif

void RPLidar::uartIRQHandler() {
	// DEBUG_PRINT("[RPLidar] UART IRQ\n");
	while (uart_is_readable(_singleton->_interface)) {

        _singleton->dataBuffer[_singleton->dataBufferIndex++] = uart_getc(_singleton->_interface);

		// DEBUG_PRINT("[RPLidar] Received byte: %#X\n", _singleton->dataBuffer[_singleton->dataBufferIndex - 1]);

		if (_singleton->dataBufferIndex > (RPLIDAR_BUFFER_SIZE - 1)) {
			// DEBUG_PRINT("[RPLidar] Buffer overflow\n");
			_singleton->dataBufferIndex = 0;
			if (_singleton->isScanning) {
				std::copy(_singleton->dataBuffer + _singleton->processBufferLen, _singleton->dataBuffer + RPLIDAR_BUFFER_SIZE, _singleton->dataBuffer);
				_singleton->dataBufferIndex = RPLIDAR_BUFFER_SIZE - _singleton->processBufferLen;
			}
		}
		
		if (_singleton->waitingForResponse && _singleton->dataBufferIndex == 7 && _singleton->dataBuffer[0] == 0xA5 && _singleton->dataBuffer[1] == 0x5A) {
			// Received a response
			std::copy(_singleton->dataBuffer, _singleton->dataBuffer + 7, _singleton->response);
			_singleton->waitingForResponse = false;
			_singleton->responseReceived = true;
			_singleton->dataBufferIndex = 0;
			return;
		}

		if (_singleton->waitingForData) {
			_singleton->dataBytesRemaining--;
			if (_singleton->dataBytesRemaining == 0) {
				// Received all the data
				std::copy(_singleton->dataBuffer, _singleton->dataBuffer + _singleton->dataBufferIndex, _singleton->data);
				_singleton->waitingForData = false;
				_singleton->dataReceived = true;
				_singleton->dataBufferIndex = 0;
			}
			return;
		}

		// _singleton->processBufferIndex = 0;
		// if (_singleton->isScanning) {
		// 	DEBUG_PRINT("[RPLidar] Scanning %d\n", _singleton->dataBufferIndex);
		// 	// bool processed = false;
		// 	while (_singleton->dataBufferIndex > 4) {
		// 		DEBUG_PRINT("[RPLidar] dbi %d\n", _singleton->dataBufferIndex);
		// 		if (_singleton->dataBuffer[_singleton->processBufferIndex + 1] & 0x01 && (((_singleton->dataBuffer[_singleton->processBufferIndex + 0] & 0x02) >> 1) == !(_singleton->dataBuffer[_singleton->processBufferIndex + 0] & 0x01))) {
		// 			// processed = true;
		// 			if (_singleton->dataBuffer[_singleton->processBufferIndex + 0] & 0x01) {
		// 				// New scan
		// 				_singleton->_qualities.clear();
		// 				_singleton->_distances.clear();
		// 				_singleton->_angles.clear();
		// 				std::copy(_singleton->new_qualities.begin(), _singleton->new_qualities.end(), std::back_inserter(_singleton->_qualities));
		// 				std::copy(_singleton->new_distances.begin(), _singleton->new_distances.end(), std::back_inserter(_singleton->_distances));
		// 				std::copy(_singleton->new_angles.begin(), _singleton->new_angles.end(), std::back_inserter(_singleton->_angles));
		// 				// Print the lengths of new_distances and _distances
		// 				DEBUG_PRINT("[RPLidar] %d %d\n", _singleton->new_distances.size(), _singleton->_distances.size());
		// 				_singleton->new_qualities.clear();
		// 				_singleton->new_distances.clear();
		// 				_singleton->new_angles.clear();
		// 				DEBUG_PRINT("[RPLidar] New scan\n");
		// 				DEBUG_PRINT("[RPLidar] bytes: %#X %#X %#X %#X %#X\n", _singleton->dataBuffer[_singleton->processBufferIndex + 0], _singleton->dataBuffer[_singleton->processBufferIndex + 1], _singleton->dataBuffer[_singleton->processBufferIndex + 2], _singleton->dataBuffer[_singleton->processBufferIndex + 3], _singleton->dataBuffer[_singleton->processBufferIndex + 4]);
		// 				DEBUG_PRINT("[RPLidar] lens: %d %d\n", _singleton->processBufferLen, _singleton->processBufferIndex);
		// 				if (_singleton->processBufferLen > 10)
		// 					DEBUG_PRINT("[RPLidar] next bytes: %#X %#X %#X %#X %#X\n", _singleton->dataBuffer[_singleton->processBufferIndex + 5], _singleton->dataBuffer[_singleton->processBufferIndex + 6], _singleton->dataBuffer[_singleton->processBufferIndex + 7], _singleton->dataBuffer[_singleton->processBufferIndex + 8], _singleton->dataBuffer[_singleton->processBufferIndex + 9]);
		// 			}
		// 			// New datapoint
		// 			_singleton->new_qualities.push_back(_singleton->dataBuffer[_singleton->processBufferIndex + 0] >> 2);
		// 			_singleton->new_angles.push_back(((_singleton->dataBuffer[_singleton->processBufferIndex + 1] >> 1) | (_singleton->dataBuffer[_singleton->processBufferIndex + 2] >> 1) << 7 | (_singleton->dataBuffer[_singleton->processBufferIndex + 2] & 0x80 << 6)) / 64);
		// 			_singleton->new_distances.push_back((_singleton->dataBuffer[_singleton->processBufferIndex + 3] | (_singleton->dataBuffer[_singleton->processBufferIndex + 4] << 8)) / 4);
		// 			DEBUG_PRINT("[RPLidar] New datapoint %d\n", _singleton->new_distances.size());
		// 			_singleton->processBufferIndex += 5;
		// 		} else {
		// 			// Invalid data
		// 			DEBUG_PRINT("[RPLidar] Invalid data\n");
		// 			DEBUG_PRINT("[RPLidar] bytes: %#X %#X %#X %#X %#X\n", _singleton->dataBuffer[_singleton->processBufferIndex + 0], _singleton->dataBuffer[_singleton->processBufferIndex + 1], _singleton->dataBuffer[_singleton->processBufferIndex + 2], _singleton->dataBuffer[_singleton->processBufferIndex + 3], _singleton->dataBuffer[_singleton->processBufferIndex + 4]);
		// 			// for (int i = processBufferIndex; i < processBufferIndex + 5; i++) {
		// 			// 	DEBUG_PRINT(" %#X", dataBuffer[i]);
		// 			// }
		// 			// DEBUG_PRINT("\n");
		// 			_singleton->processBufferIndex++;
		// 		}
		// 	}
		// 	// if (processed) {
		// 	// 	// Remove processed data
		// 	// 	std::copy(_singleton->dataBuffer + _singleton->processBufferIndex, _singleton->dataBuffer + _singleton->dataBufferIndex, _singleton->dataBuffer);
		// 	// 	_singleton->dataBufferIndex -= _singleton->processBufferIndex;
		// 	// }
		// }


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