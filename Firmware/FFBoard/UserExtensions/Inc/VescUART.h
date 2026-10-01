/*
 * VescUART.h
 *
 *  VESC motor driver over UART (USART3 / motor_uart).
 *
 *  Protocol reference: VESC-bldc/comm/packet.c (framing) and VESC-bldc/util/crc.c (CRC16).
 *  See docs/32_UART驱动实现规格.md for the full byte level specification.
 *
 *  This driver talks to a completely unmodified VESC:
 *    COMM_FW_VERSION           (0)  handshake / compatibility check
 *    COMM_GET_VALUES_SELECTIVE (50) input voltage + fault + encoder position
 *    COMM_SET_CURRENT_REL      (84) torque, +-1.0 = Motor Current Max
 *
 *  NOTE about the encoder position: the VESC has no incoming handler for
 *  COMM_ROTOR_POSITION (22) - commands_send_rotor_pos() is only ever called by the
 *  CAN poll path (comm_can.c:2019) and by the periodic display thread (main.c:179),
 *  so sending [22] over UART never produces a reply. The angle is therefore pulled
 *  with COMM_GET_VALUES_SELECTIVE mask bit 16 (float32 degrees at 1e6), which the
 *  same round trip also uses to fetch the input voltage (bit 8) and fault (bit 15).
 *  That is exactly the field the working CAN driver already uses for its bit 16.
 *
 *  That also means useEncoder must be true for the angle to be polled at all -
 *  there is no second, independent source for it.
 *
 *  Command namespace: this driver registers as "vescuart", NOT "vesc". The CAN
 *  driver already owns "vesc" and its instance ids would otherwise collide.
 */

#ifndef USEREXTENSIONS_SRC_VESCUART_H_
#define USEREXTENSIONS_SRC_VESCUART_H_

#include "MotorDriver.h"
#include "cpp_target_config.h"
#include "UART.h"
#include "Encoder.h"
#include "thread.hpp"
#include "CommandHandler.h"
#include "PersistentStorage.h"
#include <math.h>

#ifdef VESC_UART

// Thread memory/priority. Same priority as the CAN variant.
#define VESCUART_THREAD_MEM 640
#define VESCUART_THREAD_PRIO 25 // Must be higher than main thread

// tx buffer for the longest frame we send (COMM_GET_VALUES_SELECTIVE, 11 bytes).
// Slack is kept for debugging and to leave room for a longer request later.
#define VESCUART_TX_SIZE 96
// rx working buffer: one byte of history plus the maximum possible single frame
// (2 start/length + 255 payload + 2 crc + 1 stop = 260). Anything longer is
// dropped the same way VESC's packet_process_byte() drops it.
#define VESCUART_RX_SIZE 264
// Max resync slides performed by a single parseRxBuffer() call. Bounds the time
// spent inside the USART3 interrupt; leftover junk is drained by the bytes that
// follow, at one slide each. The buffer reset on overflow is the hard backstop.
#define VESCUART_RX_WORK 32
// Longest payload we accept. Replies we ask for are <= 12 bytes, the firmware
// version reply is ~65. Anything beyond the 8 bit framing limit is rejected.
#define VESCUART_MAX_PL_LEN 255
#define VESCUART_HWNAME_SIZE 32
// F407 side: always USART3 on PB10 = TX / PB11 = RX (motor_uart).
//
// VESC side: the important part is the SPEED, and it depends on which socket the
// three wires land on. configurePort() reconfigures motor_uart to
// VESCUART_BAUDRATE, so this value must be whatever that socket runs at:
//
//   VESC "UART2" socket  = HW_UART_P_DEV = SD4 = UART4 (PC10/PC11)
//       speed is HW_UART_P_BAUD, hardcoded 115200 (hw_mksesc_75_100_v2_core.h:185)
//       it is only enabled when appconf.permanent_uart_enabled is true
//       (app.c:169, app_uartcomm.c:215-219 otherwise parks the pins as inputs)
//   VESC COMM header     = HW_UART_DEV = SD3 = USART3 (PB10/PB11)
//       speed is appconf.app_uart_baudrate, i.e. whatever VESC Tool shows
//
// Both sockets run the same commands_process_packet() pipeline
// (app_uartcomm.c:102-107), so only the baud rate differs.
//
// Current wiring: the 8-pin COMM header (USART3, PB10/PB11), so the speed is
// whatever the VESC's appconf.app_uart_baudrate is - changed 2026-10-01.
//
// The driver does not hardcode one speed: it tries these in turn until the VESC
// answers (see Run()). A two sided setting like this is otherwise a trap - a
// mismatch means a dead link with no way to tell the driver anything, and the fix
// would be a reflash. Trying them means either end can be changed first, and the
// speed can later be changed from VESC Tool alone.
#define VESCUART_BAUDRATE 115200
#define VESCUART_BAUD_CANDIDATES 3
extern const uint32_t VESCUART_BAUD_TABLE[VESCUART_BAUD_CANDIDATES];
// Handshake attempts at one speed before moving to the next.
#define VESCUART_BAUD_PROBE 2


// Talk to the VESC at least this often while the motor is enabled, its own
// watchdog cuts the motor after 1000ms without any command.
#define VESCUART_KEEPALIVE_MS 500
// A READY/ERROR VESC that does not answer within this time falls back to UNKNOWN.
#define VESCUART_TIMEOUT_MS 1000
// Minimum spacing between two encoder position requests (500Hz). One full
// request/reply round trip is ~350us at 460800 baud, so this is very relaxed.
//
// The real limit is not this gate but the VESC's own reply latency: measured on
// this build the link settles at 333Hz (3ms) no matter how short the gate is.
#define VESCUART_POS_INTERVAL_MS 2
// Torque packets are only queued when the request moved by at least this
// fraction of full scale.
//
// Why: the FFB loop runs at 1kHz and used to queue a packet for *every* change,
// including one-count changes. A torque packet is 10 bytes = 0.87ms of wire time
// at 115200 baud, so a 1kHz stream needs 10kB/s out of the 11.5kB/s this link
// has - the transmit calls then block the driver thread for most of every tick
// and the encoder polling plus the keepalive get squeezed. 0.5% of full scale is
// 0.04Nm on this wheel, below the VESC's own current measurement noise, and cuts
// the packet rate by roughly an order of magnitude while the wheel is moving.
#define VESCUART_TORQUE_DEADBAND 0.005f
// A deadbanded update can be missed (semaphore timeout, a link dropout, the
// driver being restarted). Resend whenever the value on the VESC has been out of
// date for this long, which bounds the staleness and re-arms its watchdog.
#define VESCUART_TORQUE_REFRESH_MS 50
// A telemetry request that got no reply is retried after this time.
#define VESCUART_REPLY_TIMEOUT_MS 50
// Driver thread tick. getPos_f() only raises a flag and this thread performs the
// transmit, which keeps every UART access out of the 1kHz FFB loop.
//
// This value is the angle poll ceiling: the position request is only serviced on
// a tick boundary, so a tick has to be at least as short as
// VESCUART_POS_INTERVAL_MS or the rate collapses to 1/tick. Measured with a 10ms
// tick the link settled at exactly 100Hz instead of the intended 500Hz, i.e. 10ms
// of angle latency - far too much for force feedback.
//
// 1ms gives the full 500Hz: at a 2ms tick the next request can only start on the
// following tick boundary, so the rate halves to 250Hz (measured). Per-tick work is
// a handful of HAL_GetTick() comparisons plus at most one 11 byte transmit.
#define VESCUART_TICK_MS 1
// |position| is clamped to this many turns. getCpr() is 1e6, so 2147 turns would
// already overflow the int32 that getPos() returns.
#define VESCUART_POS_LIMIT 2000.0f
// Position heartbeat (encrate) is computed over this window.
#define VESCUART_ENCRATE_WINDOW_MS 1000

// Fields requested in one COMM_GET_VALUES_SELECTIVE round trip:
//   bit 8  input voltage  float16(volts, 1e1)
//   bit 15 fault code     uint8
//   bit 16 encoder angle  float32(deg, 1e6)
// VESC appends them in ascending bit order and re-reads the mask with a fresh
// cursor for every field, so the reply layout is mask, voltage, fault, angle.
#define VESCUART_SELECTIVE_MASK ((uint32_t)((1u << 16) | (1u << 15) | (1u << 8)))
// The same request plus the average q-axis current (bit 5, float32 amperes at 1e2).
// That is the torque producing current and the one worth comparing against the
// commanded torque. Bit 2 is NOT usable for this: it is
// mc_interface_read_reset_avg_motor_current(), the filtered *magnitude* of the
// current vector, unsigned - a signed command compared against it looks like noise.
// Four extra bytes per reply cost poll rate, so this is only used while
// vescart.0.monitorcurrent is on. Bit 5 sorts before bits 8/15/16, so it is parsed
// before them.
#define VESCUART_SELECTIVE_MASK_CURRENT ((uint32_t)((1u << 16) | (1u << 15) | (1u << 8) | (1u << 5)))

#define FW_MIN_RELEASE ((5 << 16) | (3 << 8) | 51)

// Deliberately distinct from VescCAN's VescState/VescCmd: both headers are
// included by MotorDriver.cpp, so identical enum names would not compile.
enum class VescUARTState : uint32_t {
	VESC_STATE_UNKNOWN = 0,
	VESC_STATE_INCOMPATIBLE = 1,
	VESC_STATE_PONG = 2,
	VESC_STATE_COMPATIBLE = 3,
	VESC_STATE_READY = 4,
	VESC_STATE_ERROR = 5
};

enum class VescUARTCmd : uint8_t {
	COMM_FW_VERSION = 0,
	COMM_ROTOR_POSITION = 22,
	COMM_GET_VALUES_SELECTIVE = 50,
	COMM_SET_CURRENT_REL = 84
};

// EEPROM addresses for this driver instance
struct VescUARTFlashAddrs {
	uint16_t data = ADR_VESCUART1_DATA;
	uint16_t offset = ADR_VESCUART1_OFFSET;
};

/**
 * VESC over UART.
 *
 * The motor_uart port is shared with MotorSimplemotion - UARTPort::reservePort()
 * guarantees only one of the two can own it, so both may be compiled in but only
 * one may be selected as axis.X.drvtype at any time.
 */
class VescUART : public MotorDriver,
		public Encoder,
		public PersistentStorage,
		public CommandHandler,
		public UARTDevice,
		public cpp_freertos::Thread {
public:
	VescUART(uint8_t instance);
	virtual ~VescUART();

	// MotorDriver impl
	void turn(int16_t power) override;		//!< Torque, 0x7fff == +-Motor Current Max
	void stopMotor() override;
	void startMotor() override;
	bool motorReady() override;
	Encoder* getEncoder() override;
	bool hasIntegratedEncoder() override;
	EncoderType getEncoderType() override;

	// Encoder impl
	float getPos_f() override;				//!< Non blocking, returns the cached angle
	int32_t getPos() override;
	void setPos(int32_t pos) override;
	uint32_t getCpr() override;

	// PersistentStorage impl
	void saveFlash() override;
	void restoreFlash() override;

	// UARTDevice impl - runs in the USART3 interrupt, keep it short
	void uartRcv(char& buf) override;

	// UARTDevice impl - the tx semaphore is managed by sendPacket()
	void startUartTransfer(UARTPort* port, bool transmit) override;
	void endUartTransfer(UARTPort* port, bool transmit) override;

	// Thread impl - all UART traffic is transmitted here
	void Run() override;

	// CommandHandler impl
	void registerCommands();
	CommandStatus command(const ParsedCommand& cmd, std::vector<CommandReply>& replies) override;
	std::string getHelpstring() override { return "VESC over UART interface"; }

private:
	enum class VescUART_commands : uint32_t {
		errorflags, vescstate, voltage, encrate, pos, torque, forceposread, useencoder, offset,
		crcerrors, uarterrors, fwversion, hwname, protostats, txcount, rxcount,
		current, monitorcurrent, baud
	};

	uint8_t instance;

	// ---- live state, volatile because written from the USART3 interrupt ----
	volatile VescUARTState state = VescUARTState::VESC_STATE_UNKNOWN;
	volatile uint8_t vescErrorFlag = 0;
	volatile float voltage = 0;				//!< VESC input voltage in V
	volatile float motorCurrent = 0;		//!< VESC average q-axis (torque) current in A (monitoring only)
	bool monitorCurrent = false;			//!< also ask for the current in the angle poll
	volatile float lastPos = 0;				//!< multi turn position, signed turns
	volatile float prevPos360 = 0;			//!< previous raw 0..360 angle
	volatile bool posValid = false;			//!< prevPos360 has been seeded by a real sample
	volatile float mtPos = 0;				//!< turn counter
	volatile uint32_t encCount = 0;			//!< position replies since encStartPeriod
	volatile uint32_t encStartPeriod = 0;
	volatile float encRate = 0;				//!< position replies per second
	volatile uint32_t lastVescResponse = 0;	//!< tick of the last valid VESC packet
	volatile uint32_t crcErrors = 0;
	volatile uint32_t formatErrors = 0;
	volatile uint32_t txPackets = 0;		//!< packets put on the wire
	volatile uint32_t txFailures = 0;		//!< transmits the HAL refused
	volatile uint32_t rxBytes = 0;			//!< raw bytes seen by the USART
	char hwName[VESCUART_HWNAME_SIZE] = {0};
	volatile uint8_t fwMajor = 0;
	volatile uint8_t fwMinor = 0;

	// ---- receive state machine (VESC-bldc/comm/packet.c packet_process_byte) ----
	uint8_t rxBuf[VESCUART_RX_SIZE] = {0};
	uint16_t rxLen = 0;
	int16_t bytesLeft = 0;

	// ---- transmit bookkeeping, only touched with the tx semaphore held ----
	char txBuf[VESCUART_TX_SIZE] = {0};
	volatile bool telemetryPending = false;	//!< a selective values request is in flight
	volatile uint32_t lastTelemetryRequest = 0;
	volatile uint32_t lastFwRequest = 0;
	volatile bool torqueQueued = false;		//!< set by the FFB thread in turn()
	volatile float pendingTorque = 0;		//!< torque waiting for the driver thread
	volatile float lastSentTorque = 2.0f;	//!< torque of the last packet that went out (2.0 = none yet)
	uint32_t lastTorqueTx = 0;				//!< tick of the last torque packet
	volatile bool posRequest = false;		//!< getPos_f() wants a fresh angle

	// ---- encoder section ----
	bool useEncoder = true;
	float posOffset = 0;					//!< in turns, centres the wheel
	volatile float lastTorque = 0;
	bool activeMotor = false;
	uint32_t lastTorqueSent = 0;
	uint32_t lastTick = 0;					//!< last VESCUART_TICK_MS boundary
	volatile uint8_t baudIndex = 0;			//!< which candidate is in use
	uint8_t baudFailures = 0;				//!< handshakes that timed out at this speed

	volatile bool runThread = true;			//!< cleared by the destructor before suspending
	bool threadStarted = false;				//!< Start() succeeded, so Suspend() is safe

	VescUARTFlashAddrs flashAddrs;

	// protocol
	static uint16_t crc16(const uint8_t* buf, uint32_t len);
	void acquirePort();						//!< reserve + configure + arm the RX interrupt
	void configurePort();
	bool sendPacket(uint8_t cmd, const uint8_t* payload, uint8_t len);
	void handlePacket(const uint8_t* payload, uint16_t len);
	void parseRxBuffer();
	void decodeEncoderPosition(float newPos);

	// commands, all transmit only
	void getFirmwareInfo();
	void askGetValue();
	void askRotorPos();
	void doAskGetValue();
	bool setTorqueRel(float torque);

	// helpers
	void queueTorque(float torque);
	void saveFlashOffset();
	void encodeInt32(uint8_t* buffer, int32_t number, int32_t* index);
	void encodeUint32(uint8_t* buffer, uint32_t number, int32_t* index);
	void encodeFloat32(uint8_t* buffer, float number, float scale, int32_t* index);
	uint32_t decodeUint32(const uint8_t* buffer, int32_t* index);
	int32_t decodeInt32(const uint8_t* buffer, int32_t* index);
	int16_t decodeInt16(const uint8_t* buffer, int32_t* index);
	float decodeFloat16(const uint8_t* buffer, float scale, int32_t* index);
	float decodeFloat32(const uint8_t* buffer, float scale, int32_t* index);
};

/**
 * Instance 1 of the VESC UART driver. Motor driver slot 13, commands vescuart.0.*
 */
class VescUART_1 : public VescUART {
public:
	VescUART_1() : VescUART{0} { inUse = true; }
	~VescUART_1() { inUse = false; }

	static ClassIdentifier info;
	static bool inUse;

	static bool isCreatable();
	const ClassIdentifier getInfo();
};

/**
 * Instance 2 of the VESC UART driver. Motor driver slot 14, commands vescuart.1.*
 */
class VescUART_2 : public VescUART {
public:
	VescUART_2() : VescUART{1} { inUse = true; }
	~VescUART_2() { inUse = false; }

	static ClassIdentifier info;
	static bool inUse;

	static bool isCreatable();
	const ClassIdentifier getInfo();
};

#endif /* VESC_UART */
#endif /* USEREXTENSIONS_SRC_VESCUART_H_ */
