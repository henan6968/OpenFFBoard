/*
 * VescUART.cpp
 *
 *  VESC motor driver over UART (USART3 / motor_uart).
 *
 *  Frame format (VESC-bldc/comm/packet.c):
 *      0x02 | len | payload[len] | crc16_hi | crc16_lo | 0x03
 *  CRC16 is CCITT-FALSE (poly 0x1021, init 0x0000, no reflection, no final xor,
 *  computed over the payload only) and matches VESC-bldc/util/crc.c crc16().
 */

#include "target_constants.h"
#ifdef VESC_UART
#include "VescUART.h"
#include "ClassIDs.h"
#include "cppmain.h"

// *****    static initializers for the two concrete instances    *****

bool VescUART_1::inUse = false;

ClassIdentifier VescUART_1::info = {
	 .name = "VESC UART 1" ,
	 .id = CLSID_MOT_VESCUART0
};

bool VescUART_1::isCreatable() {
	return !VescUART_1::inUse;
}

const ClassIdentifier VescUART_1::getInfo() {
	return info;
}

bool VescUART_2::inUse = false;

ClassIdentifier VescUART_2::info = {
	 .name = "VESC UART 2" ,
	 .id = CLSID_MOT_VESCUART1
};

bool VescUART_2::isCreatable() {
	return !VescUART_2::inUse;
}

const ClassIdentifier VescUART_2::getInfo() {
	return info;
}

// *****    				 VescUART						 *****

/*
 * instance is 0 based: 0 -> motor driver slot 13, 1 -> slot 14.
 *
 * The command handler is registered as "vescuart", deliberately NOT "vesc": the
 * CAN driver already owns that class name and its instance ids would collide
 * (VescCAN uses address 0,1,2), which would make vesc.N.* resolve to whichever
 * handler was registered first.
 *
 * UARTDevice reserves motor_uart here, but Axis::setDrvType() constructs the new
 * driver BEFORE destroying the old one, so the previous owner may still hold the
 * port at this point. acquirePort() is therefore retried from Run() and from
 * every transmit until it succeeds; until then the driver stays quiet.
 */
VescUART::VescUART(uint8_t instance) :
		CommandHandler("vescuart", CLSID_MOT_VESCUART0, instance),
		UARTDevice(motor_uart),
		Thread("VESCUART", VESCUART_THREAD_MEM, VESCUART_THREAD_PRIO),
		instance(instance) {

	if (instance == 0) {
		this->flashAddrs = VescUARTFlashAddrs( { ADR_VESCUART1_DATA, ADR_VESCUART1_OFFSET });
	} else {
		this->flashAddrs = VescUARTFlashAddrs( { ADR_VESCUART2_DATA, ADR_VESCUART2_OFFSET });
	}

	restoreFlash();
	acquirePort();

	registerCommands();
	threadStarted = this->Start();
}

VescUART::~VescUART() {
	// Stop the thread that would keep talking, then let UARTDevice hand the port
	// back. Deliberately NO transmit here: Axis::setDrvType() destroys this object
	// from inside a critical section, and a blocking UART write there would mask
	// interrupts for up to the HAL timeout. Clearing activeMotor is enough - the
	// VESC cuts the motor itself when its 1s command watchdog expires.
	this->runThread = false;
	this->activeMotor = false;
	this->lastTorque = 0.0;
	this->torqueQueued = false;
	this->state = VescUARTState::VESC_STATE_UNKNOWN;

	// The thread checks runThread at its loop boundary and is never blocked for
	// longer than one tick - it has either already exited Run() or is parked in
	// Delay(), so suspending it here is race free.
	if (threadStarted) {
		this->Suspend();
	}
}

/**
 * Take ownership of motor_uart: reserve it, force our line settings and arm the
 * byte receive interrupt.
 *
 * Safe to call repeatedly. It is retried from Run() and from sendPacket() because
 * the port may still belong to MotorSimplemotion (or to the sibling VescUART
 * instance) when this object is constructed - Axis::setDrvType() creates the new
 * driver before destroying the old one.
 */
void VescUART::acquirePort() {
	if (uartport == nullptr) {
		return;
	}
	if (uartport->reservePort(this)) {
		configurePort();
		uartport->registerInterrupt();
	}
}

/**
 * Configure motor_uart for the VESC. 8N1 at 460800 baud, which is what the F407
 * target already sets up in MX_USART3_UART_Init(). reconfigurePort() is a no-op
 * when the settings match, so this only corrects a port another driver changed.
 */
void VescUART::configurePort() {
	UART_InitTypeDef uartconf;
	uartconf.BaudRate = VESCUART_BAUDRATE;
	uartconf.WordLength = UART_WORDLENGTH_8B;
	uartconf.StopBits = UART_STOPBITS_1;
	uartconf.Parity = UART_PARITY_NONE;
	uartconf.Mode = UART_MODE_TX_RX;
	uartconf.HwFlowCtl = UART_HWCONTROL_NONE;
	uartconf.OverSampling = UART_OVERSAMPLING_16;
	uartport->reconfigurePort(uartconf);

}

/**
 * The port semaphore is taken by sendPacket() and must be held across the whole
 * build+transmit so txBuf cannot be clobbered by the other thread. UARTPort calls
 * these two hooks around every transfer, so they have to be no-ops here or the
 * semaphore would be taken twice. MotorSimplemotion does the same thing.
 */
void VescUART::startUartTransfer(UARTPort* port, bool transmit) {
	(void) port;
	(void) transmit;
}

void VescUART::endUartTransfer(UARTPort* port, bool transmit) {
	(void) port;
	(void) transmit;
}

// *****    MotorDriver impl    *****

/**
 * Torque command. power is full signed 16 bit, +-1.0 maps onto the VESC
 * "Motor Current Max" setting.
 *
 * Nothing is blocking here on purpose: the packet is queued for the driver
 * thread, so the 1kHz FFB loop can never be held up by the UART.
 */
void VescUART::turn(int16_t power) {
	float torque = ((float) power / (float) 0x7fff);
	if (fabsf(torque - lastTorque) > 0.01f) {
		pulseErrLed();
	}
	lastTorque = torque;
	queueTorque(torque);
}

void VescUART::stopMotor() {
	// Send the safe state while the VESC is still known to be reachable, only
	// then mark the motor inactive (setTorqueRel() refuses to transmit otherwise).
	this->torqueQueued = false;
	this->setTorqueRel(0.0);
	this->lastTorque = 0.0;
	this->activeMotor = false;
}

void VescUART::startMotor() {
	this->activeMotor = true;
}

bool VescUART::motorReady() {
	return state == VescUARTState::VESC_STATE_READY;
}

Encoder* VescUART::getEncoder() {
	if (useEncoder)
		return static_cast<Encoder*>(this);
	else
		return MotorDriver::getEncoder();
}

bool VescUART::hasIntegratedEncoder() {
	return useEncoder;
}

EncoderType VescUART::getEncoderType() {
	if (useEncoder)
		return EncoderType::absolute;
	return MotorDriver::getEncoder()->getEncoderType();
}

// *****    Encoder impl    *****

/**
 * Called from the 1kHz FFB loop via Axis::getEncAngle(). Must never block and
 * must never touch the UART: it only raises a flag that Run() picks up on its
 * next tick, then returns the cached angle immediately.
 *
 * Raising the flag unconditionally is harmless - askRotorPos() re-checks
 * useEncoder, the link state, whether a request is already in flight and the
 * rate limit, and it coalesces repeated requests.
 */
float VescUART::getPos_f() {
	posRequest = true;
	return lastPos - posOffset;
}

int32_t VescUART::getPos() {
	return getCpr() * getPos_f();
}

uint32_t VescUART::getCpr() {
	// cpr is not a real encoder count here. The VESC reports degrees with 1e-5
	// resolution, so one turn is exactly 1e6 "counts" which keeps setPos() and
	// the offset arithmetic in clean integer degrees.
	return 1000000;
}

/**
 * Used to centre the wheel. pos is in cpr units, i.e. 1e6 per turn.
 */
void VescUART::setPos(int32_t pos) {
	posOffset = lastPos - ((float) pos / (float) getCpr());
	saveFlashOffset();
}

/**
 * Absolute encoder angle in degrees (0..360) turned into a signed multi turn
 * position. Only called from the USART3 interrupt.
 *
 * A delta of more than half a turn is interpreted as a wrap around, exactly like
 * the CAN based driver does it. Note the module of the resulting position is the
 * same as the raw angle, which is what makes the centring arithmetic above work.
 */
void VescUART::decodeEncoderPosition(float newPos) {
	// First sample after power up or after a link loss: take it as the reference.
	// Without this the very first angle is compared against a prevPos360 of 0 and
	// any angle in the upper half of the range is mistaken for a wrap, which
	// shifts the whole axis by a full turn.
	if (!posValid) {
		prevPos360 = newPos;
		mtPos = 0;
		lastPos = newPos / 360.0f;
		posValid = true;
		encCount++;
		return;
	}

	float delta = newPos - prevPos360;

	if (fabsf(delta) > 180.0f) {
		// if delta is negative the wheel turned CCW, so decrease the turn count
		// Wrapped around the 0/360 boundary. The parentheses around the ternary are
		// load bearing: '+=' binds tighter than '?:', so without them this parses as
		// (mtPos += (delta > 0)) ? -1.0f : 1.0f and sends the turn count the wrong way.
		mtPos += ((delta > 0) ? -1.0f : 1.0f);
	}

	lastPos = (newPos / 360.0f) + mtPos;

	// getPos() multiplies by getCpr() = 1e6, so anything past ~2147 degrees of
	// travel would overflow the int32 that getPos() returns. Clamp instead of
	// wrapping the sign, which would look like a huge jump to the axis.
	if (lastPos > VESCUART_POS_LIMIT) lastPos = VESCUART_POS_LIMIT;
	else if (lastPos < -VESCUART_POS_LIMIT) lastPos = -VESCUART_POS_LIMIT;
	prevPos360 = newPos;

	encCount++;
}

// *****    PersistentStorage impl    *****

void VescUART::restoreFlash() {
	uint16_t dataFlash = 0;

	if (Flash_Read(flashAddrs.data, &dataFlash)) {
		this->useEncoder = (dataFlash >> 3) & 0x1;
	}

	if (Flash_Read(flashAddrs.offset, &dataFlash)) {
		this->posOffset = (float) ((int16_t) dataFlash / 10000.0);

		this->posOffset = this->posOffset - (int) this->posOffset; // Remove the multi-turn value
		bool moreThanHalfTurn = fabs(this->posOffset) > 0.5 ? 1 : 0;
		if (moreThanHalfTurn) {
			// if delta is neg, turn is CCW, decrement multi turn pos... else increment it
			this->posOffset += (this->posOffset > 0) ? -1.0 : 1.0;
		}
	}
}

void VescUART::saveFlash() {
	uint16_t dataFlash = 0;
	dataFlash |= (this->useEncoder & 0x1) << 3;  // set the encoder use in bit 3
	Flash_Write(flashAddrs.data, dataFlash);

	saveFlashOffset();
}

void VescUART::saveFlashOffset() {
	// store the -180..180 offset
	float storedOffset = this->posOffset - (int) this->posOffset; // Remove the multi-turn value
	bool moreThanHalfTurn = fabs(storedOffset) > 0.5 ? 1 : 0;
	if (moreThanHalfTurn) {
		storedOffset += (storedOffset > 0) ? -1.0 : 1.0;
	}

	uint16_t dataFlash = ((int16_t) (storedOffset * 10000) & 0xFFFF);
	Flash_Write(flashAddrs.offset, dataFlash);
}

// *****    UART receive    *****

/*
 * Feed one byte into the receive state machine. Runs in the USART3 interrupt, so:
 * no floating point beyond the trivial position arithmetic, no UART access, no
 * semaphores, no allocation, no blocking.
 *
 * UARTPort::uartRxComplete() does NOT re-arm the receive interrupt itself - with
 * UART_BUF_SIZE == 1 it delivers exactly one byte and then disarms - so every exit
 * path has to call registerInterrupt() again. UART_CommandInterface and
 * MotorSimplemotion do the same.
 *
 * This is a port of VESC-bldc/comm/packet.c packet_process_byte(); the decoder
 * itself lives in parseRxBuffer().
 */
void VescUART::uartRcv(char& buf) {
	rxBytes++;
	if (uartport != nullptr) {
		uartport->registerInterrupt();
	}

	// rxLen can never reach VESCUART_RX_SIZE: the parser either completes a frame
	// (at most 2 + 255 + 3 = 260 bytes) or returns for more data, and every byte it
	// rejects is dropped in the same call. The bound is asserted rather than
	// handled so a future change to the framing rules cannot silently overrun.
	if (rxLen >= VESCUART_RX_SIZE) {
		rxLen = 0;
		bytesLeft = 0;
	}

	rxBuf[rxLen++] = (uint8_t) buf;

	// Still filling a payload we already know the size of?
	if (bytesLeft > 1) {
		bytesLeft--;
		return;
	}

	parseRxBuffer();
}

/*
 * Try to decode frames from the head of rxBuf until the next one is incomplete.
 *
 * On a failure the read pointer advances by exactly one byte, mirroring
 * try_decode_packet(), so a frame that starts in the middle of noise is still
 * found. A byte is only ever examined once: every rejection path drops that byte
 * before looking again, so the same garbage is never rescanned.
 *
 * work caps the slides performed by one call so a burst of junk cannot monopolise
 * the USART3 interrupt; the remainder is drained by the bytes that follow, at one
 * slide each. The buffer reset in uartRcv() is the hard backstop.
 */
void VescUART::parseRxBuffer() {
	int work = VESCUART_RX_WORK;

	while (rxLen > 0) {
		bool isLen8b = (rxBuf[0] == 0x02);
		bool isLen16b = (rxBuf[0] == 0x03);
		unsigned int dataStart = rxBuf[0]; // 0x02 -> 2, 0x03 -> 3

		if (!isLen8b && !isLen16b) {
			// No valid start byte
			memmove(rxBuf, rxBuf + 1, --rxLen);
			formatErrors++;
			if (--work <= 0) {
				return;
			}
			continue;
		}

		if (rxLen < dataStart) {
			bytesLeft = dataStart - rxLen; // Need the length field
			return;
		}

		unsigned int payloadLen = 0;
		if (isLen8b) {
			payloadLen = rxBuf[1];
			if (payloadLen < 1) {
				// Zero length packets are not supported
				memmove(rxBuf, rxBuf + 1, --rxLen);
				formatErrors++;
				if (--work <= 0) {
					return;
				}
				continue;
			}
		} else {
			payloadLen = ((unsigned int) rxBuf[1] << 8) | (unsigned int) rxBuf[2];
			if (payloadLen < 255) {
				// A shorter packet must use less length bytes
				memmove(rxBuf, rxBuf + 1, --rxLen);
				formatErrors++;
				if (--work <= 0) {
					return;
				}
				continue;
			}
		}

		if (payloadLen > VESCUART_MAX_PL_LEN) {
			// Longer than anything we ever ask for
			memmove(rxBuf, rxBuf + 1, --rxLen);
			formatErrors++;
			if (--work <= 0) {
				return;
			}
			continue;
		}

		unsigned int frameLen = payloadLen + dataStart + 3;
		if (rxLen < frameLen) {
			bytesLeft = frameLen - rxLen; // Rest of the frame is still in flight
			return;
		}

		if (rxBuf[dataStart + payloadLen + 2] != 0x03) {
			// Invalid stop byte
			memmove(rxBuf, rxBuf + 1, --rxLen);
			formatErrors++;
			if (--work <= 0) {
				return;
			}
			continue;
		}

		uint16_t crcCalc = crc16(rxBuf + dataStart, payloadLen);
		uint16_t crcRx = ((uint16_t) rxBuf[dataStart + payloadLen] << 8)
				| (uint16_t) rxBuf[dataStart + payloadLen + 1];

		if (crcCalc != crcRx) {
			memmove(rxBuf, rxBuf + 1, --rxLen);
			crcErrors++;
			if (--work <= 0) {
				return;
			}
			continue;
		}

		// Valid frame: consume it, then keep parsing whatever is behind it
		bytesLeft = 0;
		work = VESCUART_RX_WORK;
		handlePacket(rxBuf + dataStart, payloadLen);
		memmove(rxBuf, rxBuf + frameLen, rxLen - frameLen);
		rxLen -= frameLen;
	}

	bytesLeft = 1; // Waiting for the next start byte
}

/**
 * CRC-16/CCITT-FALSE, bit for bit identical to VESC-bldc/util/crc.c crc16().
 * Kept as a bitwise implementation instead of a 512 byte table: at the maximum
 * reply rate this costs well under 1% of the F407 core and saves the RAM.
 */
uint16_t VescUART::crc16(const uint8_t* buf, uint32_t len) {
	uint16_t crc = 0;
	for (uint32_t i = 0; i < len; i++) {
		crc ^= (uint16_t) buf[i] << 8;
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000) ? (uint16_t) ((crc << 1) ^ 0x1021) : (uint16_t) (crc << 1);
		}
	}
	return crc;
}

/**
 * Handle one CRC checked payload. Runs in the USART3 interrupt.
 */
void VescUART::handlePacket(const uint8_t* payload, uint16_t len) {
	if (len < 1) {
		return;
	}

	// Any valid packet from the VESC is proof of life
	lastVescResponse = HAL_GetTick();

	VescUARTCmd command = (VescUARTCmd) (payload[0] & 0xFF);
	const uint8_t* data = payload + 1;
	int32_t datalen = (int32_t) len - 1;

	switch (command) {

	case VescUARTCmd::COMM_FW_VERSION: {
		if (datalen < 2) {
			break;
		}
		int32_t ind = 0;
		uint8_t fw_major = data[ind++];
		uint8_t fw_minor = data[ind++];

		// Hardware name is a NUL terminated string right after the version
		uint8_t nameLen = 0;
		while ((ind + nameLen) < datalen && data[ind + nameLen] != 0 && nameLen < (sizeof(hwName) - 1)) {
			nameLen++;
		}
		if (nameLen) {
			memcpy(hwName, data + ind, nameLen);
		}
		hwName[nameLen] = 0;
		ind += nameLen;
		if (ind < datalen) {
			ind++; // Skip the NUL terminator
		}

		// UUID + pairing flag, then FW_TEST_VERSION_NUMBER
		ind += 12;
		if (ind < datalen) {
			ind++; // pairing_done
		}

		uint8_t isTestFw = 0;
		if (ind < datalen) {
			isTestFw = data[ind++];
		}

		fwMajor = fw_major;
		fwMinor = fw_minor;

		bool compatible = false;
		if (!isTestFw)
			compatible = ((fw_major << 8) | fw_minor) >= (FW_MIN_RELEASE >> 8);
		else
			compatible = ((fw_major << 16) | (fw_minor << 8) | isTestFw) >= FW_MIN_RELEASE;

		this->state = compatible ? VescUARTState::VESC_STATE_COMPATIBLE : VescUARTState::VESC_STATE_INCOMPATIBLE;
		break;
	}

	case VescUARTCmd::COMM_GET_VALUES_SELECTIVE: {
		// The request this answers is finished, whether or not it parsed - without
		// this the 50ms reply timeout in Run() would gate the poll rate to ~20Hz.
		telemetryPending = false;

		if (datalen < 4) {
			break;
		}
		int32_t ind = 0;
		uint32_t mask = decodeUint32(data, &ind);

		// Fields are appended in ascending mask bit order, so this order is fixed.
		if (mask & ((uint32_t) 1 << 8)) {
			if ((datalen - ind) >= 2) {
				voltage = decodeFloat16(data, 1e1, &ind); // Input voltage in V
			}
		}
		if (mask & ((uint32_t) 1 << 15)) {
			if ((datalen - ind) >= 1) {
				vescErrorFlag = data[ind++];

				// The VESC answered, so the link is up. This is what makes the
				// difference between READY and ERROR.
				if (vescErrorFlag) {
					state = VescUARTState::VESC_STATE_ERROR;
				} else {
					state = VescUARTState::VESC_STATE_READY;
				}
			}
		}
		if (mask & ((uint32_t) 1 << 16)) {
			if ((datalen - ind) >= 4) {
				float pos = decodeFloat32(data, 1e6, &ind); // Encoder angle 0..360 deg
				decodeEncoderPosition(pos);

				uint32_t period = HAL_GetTick() - encStartPeriod;
				if (period >= VESCUART_ENCRATE_WINDOW_MS) {
					encRate = encCount / (period / 1000.0f);
					encCount = 0;
					encStartPeriod = HAL_GetTick();
				}
			}
		}
		break;
	}

	case VescUARTCmd::COMM_ROTOR_POSITION: {
		// Not requested by this driver (see the header) but tolerated in case a
		// VESC is configured to push its display angle periodically.
		if (datalen >= 4) {
			int32_t ind = 0;
			float pos = decodeInt32(data, &ind) / 100000.0f;
			decodeEncoderPosition(pos);
		}
		break;
	}

	default:
		break;
	}
}

// *****    UART transmit    *****

/**
 * Build and transmit one frame. Must not be called from an interrupt or while
 * the port semaphore is already held.
 *
 * transmit() blocks until the shift register is empty, which is ~130us for the
 * longest packet we send at 460800 baud. All callers are either the driver
 * thread or a short command handler.
 */
void VescUART::sendPacket(uint8_t cmd, const uint8_t* payload, uint8_t len) {
	if (uartport == nullptr) {
		return;
	}

	uint16_t total = (uint16_t) len + 1;
	if (total == 0 || total > (VESCUART_TX_SIZE - 5)) {
		return; // Cannot be framed
	}

	// txBuf is shared between the driver thread and the command thread. The port
	// semaphore inside transmit() only covers the shift-out, so the semaphore has
	// to be held across the whole build or two threads interleave in this buffer.
	// A short timeout keeps this non-blocking: dropping a telemetry request or a
	// keepalive is harmless, the next tick retries it.
	// Blocking take is fine here: with getPos_f() no longer transmitting, the only
	// contenders are this driver's own thread and a command thread, and a frame is
	// ~350us at 460800 baud. Nothing on the 1kHz FFB path can end up waiting.
	if (!uartport->takeSemaphore(true, portMAX_DELAY)) {
		return;
	}

	memset(txBuf, 0, VESCUART_TX_SIZE);

	int32_t ind = 0;
	txBuf[ind++] = 0x02;			// 8 bit length marker
	txBuf[ind++] = (char) total;	// payload length

	txBuf[ind++] = (char) cmd;
	if (payload != nullptr && len > 0) {
		memcpy(txBuf + ind, payload, len);
		ind += len;
	}

	uint16_t crc = crc16((const uint8_t*) txBuf + 2, total);
	txBuf[ind++] = (char) (crc >> 8);
	txBuf[ind++] = (char) (crc & 0xFF);
	txBuf[ind++] = 0x03;			// Stop byte

	if (uartport->transmit(txBuf, (uint16_t) ind, 100)) {
		txPackets++;
	} else {
		txFailures++;
	}
	uartport->giveSemaphore(true);
}

/**
 * COMM_SET_CURRENT_REL (84): [84][int32_be(torque * 1e5)], +-1.0 == Motor Current Max.
 */
void VescUART::setTorqueRel(float torque) {
	if (!activeMotor || !motorReady()) {
		return;
	}

	uint8_t buffer[4];
	int32_t index = 0;
	encodeFloat32(buffer, torque, 1e5, &index);
	sendPacket((uint8_t) VescUARTCmd::COMM_SET_CURRENT_REL, buffer, sizeof(buffer));
}

/**
 * COMM_FW_VERSION (0): no payload, triggers the handshake reply.
 */
void VescUART::getFirmwareInfo() {
	sendPacket((uint8_t) VescUARTCmd::COMM_FW_VERSION, nullptr, 0);
}

/**
 * COMM_GET_VALUES_SELECTIVE (50): ask for input voltage, fault code and the
 * encoder angle in a single round trip.
 */
void VescUART::askGetValue() {
	uint8_t buffer[4];
	int32_t index = 0;
	encodeUint32(buffer, (uint32_t) VESCUART_SELECTIVE_MASK, &index);
	sendPacket((uint8_t) VescUARTCmd::COMM_GET_VALUES_SELECTIVE, buffer, sizeof(buffer));
}

/**
 * Request a fresh set of selective values (angle + voltage + fault).
 *
 * All the gating lives here because this is the only place that starts a
 * position request. It is called from the FFB loop (getPos_f), from the driver
 * and from the forceposread command, and it never blocks:
 *
 *  - without useEncoder there is no angle to fetch
 *  - only in READY/ERROR, an UNKNOWN VESC must not be flooded
 *  - at most one request in flight, so replies cannot be mixed up
 *  - the request is deferred while a torque packet is queued so traction
 *    control and keepalive always win (the flag, not the value: a queued
 *    zero-torque packet is just as urgent)
 *  - throttled to one request per VESCUART_POS_INTERVAL_MS (500Hz)
 */
void VescUART::askRotorPos() {
	if (!useEncoder || uartport == nullptr) {
		return;
	}
	if (state != VescUARTState::VESC_STATE_READY && state != VescUARTState::VESC_STATE_ERROR) {
		return;
	}
	if (telemetryPending || torqueQueued) {
		return;
	}
	if (HAL_GetTick() - lastTelemetryRequest < VESCUART_POS_INTERVAL_MS) {
		return;
	}
	doAskGetValue();
}

/**
 * Send one selective-values request and mark it as in flight. Callers are
 * askRotorPos() (the throttled FFB path) and the forceposread command.
 */
void VescUART::doAskGetValue() {
	askGetValue();
	telemetryPending = true;
	lastTelemetryRequest = HAL_GetTick();
}

/**
 * Queue a torque update for the driver thread. Returns immediately.
 */
void VescUART::queueTorque(float torque) {
	pendingTorque = torque;
	torqueQueued = true;
}

void VescUART::registerCommands() {
	CommandHandler::registerCommands();
	registerCommand("errorflags", VescUART_commands::errorflags, "VESC fault code (0 = ok)", CMDFLAG_GET);
	registerCommand("vescstate", VescUART_commands::vescstate, "VESC state (0=UNKNOWN 1=INCOMPATIBLE 2=PONG 3=COMPATIBLE 4=READY 5=ERROR)", CMDFLAG_GET);
	registerCommand("voltage", VescUART_commands::voltage, "VESC input voltage in mV", CMDFLAG_GET);
	registerCommand("encrate", VescUART_commands::encrate, "Angle replies per second", CMDFLAG_GET);
	registerCommand("pos", VescUART_commands::pos, "VESC position", CMDFLAG_GET);
	registerCommand("torque", VescUART_commands::torque, "Current VESC torque request", CMDFLAG_GET);
	registerCommand("forceposread", VescUART_commands::forceposread, "Force a position update", CMDFLAG_GET);
	registerCommand("useencoder", VescUART_commands::useencoder, "Use the VESC encoder for the axis", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("offset", VescUART_commands::offset, "Get or set the encoder offset", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("crcerrors", VescUART_commands::crcerrors, "CRC error count", CMDFLAG_GET);
	registerCommand("uarterrors", VescUART_commands::uarterrors, "Frame/overrun error count", CMDFLAG_GET);
	registerCommand("fwversion", VescUART_commands::fwversion, "VESC firmware version (major<<8)|minor", CMDFLAG_GET);
	registerCommand("hwname", VescUART_commands::hwname, "VESC hardware name", CMDFLAG_GET | CMDFLAG_STR_ONLY);
	// Plain CMDFLAG_GET, not CMDFLAG_GET | CMDFLAG_DEBUG: executeCommands() validates
	// cmdDef->flags & (GET|SET|INFOSTRING|GETADR|SETADR) and CMDFLAG_DEBUG is not in
	// that mask, so a debug-only flag combination silently never produces a reply.
	registerCommand("protostats", VescUART_commands::protostats, "UART protocol statistics (crc:frame)", CMDFLAG_GET);
	registerCommand("txcount", VescUART_commands::txcount, "Packets transmitted (ok:refused)", CMDFLAG_GET);
	registerCommand("rxcount", VescUART_commands::rxcount, "Raw bytes received from the VESC", CMDFLAG_GET);
}

CommandStatus VescUART::command(const ParsedCommand& cmd, std::vector<CommandReply>& replies) {

	switch (static_cast<VescUART_commands>(cmd.cmdId)) {

	case VescUART_commands::errorflags:
		if (cmd.type == CMDtype::get)
			replies.emplace_back((uint32_t) vescErrorFlag);
		break;

	case VescUART_commands::vescstate:
		if (cmd.type == CMDtype::get)
			replies.emplace_back((uint32_t) this->state);
		break;

	case VescUART_commands::voltage:
		if (cmd.type == CMDtype::get)
			replies.emplace_back((int32_t) (voltage * 1000.0f));
		break;

	case VescUART_commands::encrate:
		if (cmd.type == CMDtype::get)
			replies.emplace_back((uint32_t) this->encRate);
		break;

	case VescUART_commands::pos:
		if (cmd.type == CMDtype::get) {
			// 1e4 scale, like the other position commands. A 1e9 scale would overflow
			// int32 past ~2.1 turns, which VESCUART_POS_LIMIT allows.
			replies.emplace_back((int32_t) ((lastPos - posOffset) * 10000));
		}
		break;

	case VescUART_commands::torque:
		if (cmd.type == CMDtype::get)
			replies.emplace_back((int32_t) (lastTorque * 10000));
		break;

	case VescUART_commands::txcount:
		if (cmd.type == CMDtype::get) {
			replies.emplace_back((uint32_t) this->txPackets);
			replies.emplace_back((uint32_t) this->txFailures);
		}
		break;

	case VescUART_commands::rxcount:
		if (cmd.type == CMDtype::get)
			replies.emplace_back((uint32_t) this->rxBytes);
		break;

	case VescUART_commands::forceposread:
		if (cmd.type == CMDtype::get) {
			if (uartport != nullptr && uartport->isOwnedBy(this)
					&& state >= VescUARTState::VESC_STATE_COMPATIBLE) {
				doAskGetValue();
			}
		}
		break;

	case VescUART_commands::useencoder:
		if (cmd.type == CMDtype::get) {
			replies.emplace_back(useEncoder ? 1 : 0);
		} else if (cmd.type == CMDtype::set) {
			useEncoder = cmd.val != 0;
		}
		break;

	case VescUART_commands::offset:
		if (cmd.type == CMDtype::get) {
			replies.emplace_back((int32_t) (posOffset * 10000));
		} else if (cmd.type == CMDtype::set) {
			posOffset = (float) cmd.val / 10000.0;
			this->saveFlashOffset();
		}
		break;

	case VescUART_commands::crcerrors:
		if (cmd.type == CMDtype::get)
			replies.emplace_back((uint32_t) crcErrors);
		break;

	case VescUART_commands::uarterrors:
		if (cmd.type == CMDtype::get)
			replies.emplace_back((uint32_t) formatErrors);
		break;

	case VescUART_commands::fwversion:
		if (cmd.type == CMDtype::get)
			replies.emplace_back((uint32_t) (((uint32_t) fwMajor << 8) | fwMinor));
		break;

	case VescUART_commands::hwname:
		if (cmd.type == CMDtype::get)
			replies.emplace_back(std::string(hwName));
		break;


	case VescUART_commands::protostats:
		if (cmd.type == CMDtype::get) {
			replies.emplace_back((int64_t) crcErrors, (int64_t) formatErrors);
		}
		break;

	default:
		return CommandStatus::NOT_FOUND;
	}

	return CommandStatus::OK;
}

/*
 * Housekeeping loop.
 *
 * The CAN based driver polls every 500ms, but this one needs a much faster tick:
 * the encoder angle is not pushed by the VESC over UART, so it has to be pulled.
 * getPos_f() only raises posRequest; this thread performs the transmit, so no UART
 * access ever happens on the 1kHz FFB path. It is also the only place that sends
 * queued torque packets, the keepalive and the handshake.
 */
void VescUART::Run() {
	this->lastTorqueSent = HAL_GetTick();

	while (runThread) {
		// Housekeeping tick: sleep exactly one RTOS tick per iteration. Everything
		// guarded below is timestamp comparisons plus at most one 11 byte transmit,
		// so this is cheap; the gate then paces the angle poll. A free-running loop
		// would poll marginally faster but would burn a whole core at priority 25.
		Delay(1);
		uint32_t now = HAL_GetTick();
		if (now - lastTick < VESCUART_TICK_MS) {
			continue;
		}
		lastTick = now;

		// Axis::setDrvType() builds the new driver before destroying the old one, so
		// motor_uart may still have been owned by the previous driver at
		// construction time. isReserved() is not enough here: it is also true when
		// somebody ELSE holds the port, which is exactly the case that needs the
		// retry.
		if (uartport != nullptr && !uartport->isOwnedBy(this)) {
			acquirePort();
		}

		// getPos_f() asked for a fresh angle on the FFB path. Sending it from here
		// keeps every UART access out of the 1kHz loop.
		if (posRequest) {
			posRequest = false;
			askRotorPos();
		}

		// ---- 1. queued torque has priority over everything else ----
		if (torqueQueued) {
			float torque = pendingTorque;
			torqueQueued = false;
			if (motorReady() && activeMotor) {
				setTorqueRel(torque);
				lastTorqueSent = now;
			}
		}

		// ---- 2. handshake, then compatibility gate ----
		if (state == VescUARTState::VESC_STATE_UNKNOWN) {
			if (now - lastFwRequest >= VESCUART_KEEPALIVE_MS) {
				getFirmwareInfo();
				lastFwRequest = now;
			}
		} else if (state >= VescUARTState::VESC_STATE_COMPATIBLE) {
			// ---- 3. telemetry heartbeat: keeps the angle and voltage alive ----
			if (!telemetryPending && (!useEncoder || (now - lastTelemetryRequest >= VESCUART_KEEPALIVE_MS))) {
				askGetValue();
				telemetryPending = true;
				lastTelemetryRequest = now;
			}

			// ---- 4. VESC watchdog keepalive ----
			if (lastTorque != 0.0 && activeMotor && state == VescUARTState::VESC_STATE_READY
					&& (now - lastTorqueSent >= VESCUART_KEEPALIVE_MS)) {
				setTorqueRel(lastTorque);
				lastTorqueSent = now;
			}
		}

		// ---- 5. replies that never came ----
		if (telemetryPending && (now - lastTelemetryRequest > VESCUART_REPLY_TIMEOUT_MS)) {
			telemetryPending = false;
		}

		// ---- 6. a READY/ERROR VESC that stopped talking is no longer trusted ----
		// lastVescResponse is re-read from the ISR right before the store: a reply
		// that landed in between has just set the state back to READY, and this
		// transition would otherwise declare a healthy link dead.
		uint32_t lastResponse = lastVescResponse;
		if ((state == VescUARTState::VESC_STATE_READY || state == VescUARTState::VESC_STATE_ERROR)
				&& (now - lastResponse > VESCUART_TIMEOUT_MS)
				&& (HAL_GetTick() - lastVescResponse > VESCUART_TIMEOUT_MS)) {
			state = VescUARTState::VESC_STATE_UNKNOWN;
			telemetryPending = false;
			// Safety: the link is gone, do not leave the last torque standing
			lastTorque = 0.0;
			pulseErrLed();
		}

		// ---- 7. an inactive motor can never keep a stale commanded torque ----
		if (!activeMotor && lastTorque != 0.0) {
			lastTorque = 0.0;
		}
	}
}

// *****    little endian helpers (VESC-bldc/util/buffer.c is big endian)    *****

void VescUART::encodeInt32(uint8_t* buffer, int32_t number, int32_t* index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void VescUART::encodeUint32(uint8_t* buffer, uint32_t number, int32_t* index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void VescUART::encodeFloat32(uint8_t* buffer, float number, float scale, int32_t* index) {
	encodeInt32(buffer, (int32_t) (number * scale), index);
}

uint32_t VescUART::decodeUint32(const uint8_t* buffer, int32_t* index) {
	uint32_t res = ((uint32_t) buffer[*index]) << 24
			| ((uint32_t) buffer[*index + 1]) << 16
			| ((uint32_t) buffer[*index + 2]) << 8
			| ((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

int32_t VescUART::decodeInt32(const uint8_t* buffer, int32_t* index) {
	int32_t res = (int32_t) decodeUint32(buffer, index);
	return res;
}

int16_t VescUART::decodeInt16(const uint8_t* buffer, int32_t* index) {
	int16_t res = (int16_t) (((uint16_t) buffer[*index] << 8)
			| ((uint16_t) buffer[*index + 1]));
	*index += 2;
	return res;
}

float VescUART::decodeFloat16(const uint8_t* buffer, float scale, int32_t* index) {
	return (float) decodeInt16(buffer, index) / scale;
}

float VescUART::decodeFloat32(const uint8_t* buffer, float scale, int32_t* index) {
	return (float) decodeInt32(buffer, index) / scale;
}

#endif /* VESC_UART */
