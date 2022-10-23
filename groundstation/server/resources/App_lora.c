/*!
 *
 * @code
 *   _____          __  __ ______
 *  / ____|   /\   |  \/  |  ____|
 * | |  __   /  \  | \  / | |__   ___  _ __
 * | | |_ | / /\ \ | |\/| |  __| / _ \| '_ \
 * | |__| |/ ____ \| |  | | |___| (_) | | | |
 *  \_____/_/    \_\_|  |_|______\___/|_| |_|
 *
 * @endcode
 */
/**
 * @file    App_Lora.c
 * @author  Fabian Franz
 * @date 	23.09.2022
 * @brief   LoRa TX/RX functionalities for the GCU
 */

// ------------------------------------- //
// Includes ---------------------------- //
// ------------------------------------- //
#include "App_lora.h"
#include "App_Debug.h"

// ------------------------------------- //
// Defines ----------------------------- //
// ------------------------------------- //
// Debug
#define DEBUB true
#define UART_PORT HUART1_AND_2
// LoRa
#define RF_FREQUENCY                                868000000 /* Hz */
#define TX_OUTPUT_POWER                             14        /* dBm */
#define LORA_BANDWIDTH                              2         /* Hz */ // 0 = 125 kHz, 1 = 250 kHz, 2 = 500 kHz
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */

// ------------------------------------- //
// Variables --------------------------- //
// ------------------------------------- //
void (*volatile eventReceptor)(pingPongFSM_t *const fsm);
PacketParams_t packetParams;  // TODO: this is lazy, better put in fsm type

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250,
		LORA_BW_500 };
/**
 * This array keeps all the known master devices.
 * The
 */
//{lot number, wafer number, x/y position}
const uint32_t known_masters[][3] = { { 0x410022, 0x5056500E, 0x20383756 } };
const uint32_t known_slaves[][3] = { { 0x4C0019, 0x5056500B, 0x20383756 } };

// ------------------------------------- //
// Function Implementations ------------ //
// ------------------------------------- //
// Main and Init ----------------------- //
void app_lora_init(pingPongFSM_t *const fsm) {
	ModulationParams_t modulationParams;
	// Initialize the hardware (SPI bus, TCXO control, RF switch)
	SUBGRF_Init(RadioOnDioIrq);

	// Use DCDC converter if `DCDC_ENABLE` is defined in radio_conf.h
	// "By default, the SMPS clock detection is disabled and must be enabled before enabling the SMPS." (6.1 in RM0453)
	SUBGRF_WriteRegister(SUBGHZ_SMPSC0R,
			(SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
	SUBGRF_SetRegulatorMode();

	// Use the whole 256-byte buffer for both TX and RX
	SUBGRF_SetBufferBaseAddress(0x00, 0x00);

	// Set RF interface parameters
	SUBGRF_SetRfFrequency(RF_FREQUENCY);
	SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
	SUBGRF_SetStopRxTimerOnPreambleDetect(false);
	SUBGRF_SetPacketType(PACKET_TYPE_LORA);
	SUBGRF_WriteRegister( REG_LR_SYNCWORD,
			( LORA_MAC_PRIVATE_SYNCWORD >> 8) & 0xFF);
	SUBGRF_WriteRegister( REG_LR_SYNCWORD + 1,
	LORA_MAC_PRIVATE_SYNCWORD & 0xFF);

	// Modulation parameters
	modulationParams.PacketType = PACKET_TYPE_LORA;
	modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
	modulationParams.Params.LoRa.CodingRate =
			(RadioLoRaCodingRates_t) LORA_CODINGRATE;
	modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
	modulationParams.Params.LoRa.SpreadingFactor =
			(RadioLoRaSpreadingFactors_t) LORA_SPREADING_FACTOR;
	SUBGRF_SetModulationParams(&modulationParams);

	// Packet parameters
	packetParams.PacketType = PACKET_TYPE_LORA;
	packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
	packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
	packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
	packetParams.Params.LoRa.PayloadLength = 0xFF;
	packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
	SUBGRF_SetPacketParams(&packetParams);

	// WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
	// RegIqPolaritySetup @address 0x0736
	SUBGRF_WriteRegister(0x0736, SUBGRF_ReadRegister(0x0736) | (1 << 2));

	// Finally, set the finite state machine parameters
	init_state(fsm);
	fsm->rxTimeout = 2000; 		// TODO: calculate from parameters
	fsm->rxMargin = 0;   		// TODO: determine
	fsm->tx_counter = 0;
	fsm->payload_state = MOTION; 	// TODO: to be determined in fsm events
}
void app_lora_main() {
	// local variables
	uint32_t rnd = 0;
	pingPongFSM_t fsm;

	// init LoRa hardware
	app_lora_init(&fsm);

	// Set interrupt masks for fsm to NONE
	SUBGRF_SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE,
			IRQ_RADIO_NONE);
	rnd = SUBGRF_GetRandom();
	fsm.randomDelay = rnd >> 22; // [0, 1023] ms

	// If enabled, print all application informations
#ifdef DEBUG
	print_app_info(&fsm);
#endif

	// Sync and start to receipt
	HAL_Delay(fsm.randomDelay);
	// Set interrupt masks for fsm
	SUBGRF_SetDioIrqParams(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
			IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR, IRQ_RADIO_NONE,
			IRQ_RADIO_NONE);
	SUBGRF_SetSwitch(RFO_HP, RFSWITCH_RX);
	SUBGRF_SetRx(fsm.rxTimeout << 6);
	fsm.subState = SSTATE_RX;

	// main while loop
	while (1) {
		eventReceptor = NULL;
		while (eventReceptor == NULL)
			;
		eventReceptor(&fsm);
		// TODO: read uplink parameters
	}
}

// Function Tools ---------------------- //
void init_state(pingPongFSM_t *const fsm) {
	uint32_t lot, wafer, pos = 0; // UIDs
	uint8_t m_size; 		 // Size of 2. dimension of known_masters
	uint8_t s_size; 		 // Size of 2. dimension of known_slaves

	get_UIDs(&lot, &wafer, &pos);
	/**
	 * Size of the second dimension known masters or slaves
	 * is determined by the size of uint32_t (4) and the 3 different UIDs.
	 */
	m_size = sizeof(known_masters) / (sizeof(uint32_t) * 3);
	s_size = sizeof(known_slaves) / (sizeof(uint32_t) * 3);

	for (uint8_t i = 0; i < m_size; i++)
		if ((lot == known_masters[i][0]) && (wafer == known_masters[i][1])
				&& (pos == known_masters[i][2])) {
			fsm->state = STATE_MASTER;
			return;
		}
	for (uint8_t i = 0; i < s_size; i++)
		if ((lot == known_slaves[i][0]) && (wafer == known_slaves[i][1])
				&& (pos == known_slaves[i][2])) {
			fsm->state = STATE_SLAVE;
			return;
		}
	/*
	 * 	If we reach here, there is no slave or master UID
	 * 	known for this particular board.
	 */
	fsm->state = STATE_NULL;
}
void get_UIDs(uint32_t *lot, uint32_t *wafer, uint32_t *pos) {
	*lot = HAL_GetUIDw0();
	*wafer = HAL_GetUIDw1();
	*pos = HAL_GetUIDw2();
}
uint8_t payload_size(payload_state_t txState) {
	uint8_t size = 0;

	switch (txState) {
	case SYNC:
		break;
	case MOTION:
		size = sizeof(motion_payload_t);
		break;
	case ENVIRONMENT:
		size = sizeof(environment_payload_t);
		break;
	case UPLINK:
		size = sizeof(uplink_payload_t);
		break;
	case TRACKING:
		size = sizeof(tracking_payload_t);
		break;
	default:
		break;
	}
	return size;
}

// ------------------------------------- //
// Test and Debug Functions ------------ //
// ------------------------------------- //
void print_app_info(pingPongFSM_t *const fsm) {
	println(UART_PORT, "-------------------------------------------------");
	nprintf(UART_PORT, "App Version: %s\n", LORA_APP_VERSION);
	print(UART_PORT, "Mode: ");
	if (fsm->state == STATE_MASTER)
		println(UART_PORT, "MASTER");
	if (fsm->state == STATE_SLAVE)
		println(UART_PORT, "SLAVE");
	if (fsm->state == STATE_NULL) {
		println(UART_PORT, "!!! NOT KNOWN !!!");
	}
	nprintf(UART_PORT, "Random Delay: %u ms\n", fsm->randomDelay);
	nprintf(UART_PORT, "Rx Timeout: %u ms\n", fsm->rxTimeout);
	nprintf(UART_PORT, "Rx Margin: %u ms\n", fsm->rxMargin);
	print_chip_info();
	println(UART_PORT, "-------------------------------------------------");
}
void print_chip_info() {
	uint32_t lot, wafer, pos;

	get_UIDs(&lot, &wafer, &pos);
	nprintf(UART_PORT, "Lot = %u\n", lot);
	nprintf(UART_PORT, "Lot = %u\n", wafer);
	nprintf(UART_PORT, "Lot = %u\n", pos);
}
uint16_t get_random(){
	uint32_t rnd = SUBGRF_GetRandom();
	rnd = rnd >> 22; // [0, 1023] ms
	return rnd;
}
void put_test_data(pingPongFSM_t *const fsm) {
	nprintf(UART_PORT, "Write %u as test data in all arguments.\n");
	fsm->payload_state = MOTION;
	static uint32_t iterator=0;
	iterator ++;

	fsm->motion_payload.id = get_random();
	fsm->motion_payload.time = 0;
	fsm->motion_payload.roll = get_random();
	fsm->motion_payload.pitch = get_random();
	fsm->motion_payload.yaw = get_random();
	fsm->motion_payload.sun = 0;
	fsm->motion_payload.status = iterator;
	fsm->motion_payload.reserved[0] = 0;
	fsm->motion_payload.reserved[1] = get_random();
	fsm->motion_payload.reserved[2] = get_random();
}
void print_received_data(pingPongFSM_t *const fsm) {
	switch (fsm->payload_state) {
	case SYNC:
		break;
	case MOTION:
		fsm->arg_iterator = 0;
		get_motion_arguments(fsm);
		print_motion_data(fsm);
		get_motion_arguments(fsm);
		print_motion_data(fsm);
		break;
	case ENVIRONMENT:
		print_environment_data(fsm);
		break;
	case UPLINK:
		print_uplink_data(fsm);
		break;
	case TRACKING:
		print_tracking_data(fsm);
		break;
	default:
		break;
	}
}
void print_motion_data(pingPongFSM_t *const fsm) {
	println(UART_PORT, "[MOTION]");
	nprintf(UART_PORT, "ID=%u\n", fsm->motion_payload.id);
	nprintf(UART_PORT, "Time=%u\n", fsm->motion_payload.time);
	nprintf(UART_PORT, "Roll=%u\n", fsm->motion_payload.roll);
	nprintf(UART_PORT, "Pitch=%u\n", fsm->motion_payload.pitch);
	nprintf(UART_PORT, "Yaw=%u\n", fsm->motion_payload.yaw);
	nprintf(UART_PORT, "Sun=%u\n", fsm->motion_payload.sun);
	nprintf(UART_PORT, "Status=%u\n", fsm->motion_payload.status);
	nprintf(UART_PORT, "Reserved0=%u\n", fsm->motion_payload.reserved[0]);
	nprintf(UART_PORT, "Reserved1=%u\n", fsm->motion_payload.reserved[1]);
	nprintf(UART_PORT, "Reserved2=%u\n", fsm->motion_payload.reserved[2]);
	println(UART_PORT, "[END]");
}
void print_environment_data(pingPongFSM_t *const fsm) {
	println(UART_PORT, "[ENVIRONMENT]");
	nprintf(UART_PORT, "ID=%u\n", fsm->environment_payload.id);
	nprintf(UART_PORT, "Time=%u\n", fsm->environment_payload.time);
	nprintf(UART_PORT, "Longitude=%u\n", fsm->tracking_payload.longitude);
	nprintf(UART_PORT, "Latitude=%u\n", fsm->tracking_payload.latitude);
	nprintf(UART_PORT, "Pressure=%u\n", fsm->environment_payload.pressure);
	nprintf(UART_PORT, "Gas_Concentration=%u\n",
			fsm->environment_payload.gas_concentration);
	nprintf(UART_PORT, "Servo=%u\n", fsm->environment_payload.servo);
	nprintf(UART_PORT, "Temperature=%u\n", fsm->environment_payload.temp);
	nprintf(UART_PORT, "Temp=%u\n", fsm->environment_payload.v_bat);
	nprintf(UART_PORT, "Status=%u\n", fsm->environment_payload.status);
	nprintf(UART_PORT, "Reserved0=%u\n", fsm->environment_payload.reserved[0]);
	nprintf(UART_PORT, "Reserved1=%u\n", fsm->environment_payload.reserved[1]);
	nprintf(UART_PORT, "Reserved2=%u\n", fsm->environment_payload.reserved[2]);
	println(UART_PORT, "[END]");
}
// TODO: not needed
void print_uplink_data(pingPongFSM_t *const fsm) {
	println(UART_PORT, "[UPLINK]");
	nprintf(UART_PORT, "ID=%u\n", fsm->uplink_payload.cmd_id);
	nprintf(UART_PORT, "Arg_Index=%u\n", fsm->uplink_payload.arg_index);
	nprintf(UART_PORT, "Arg_Value=%u\n", fsm->uplink_payload.arg_val);
	println(UART_PORT, "[END]");
}
void print_tracking_data(pingPongFSM_t *const fsm) {
	println(UART_PORT, "[TRACKING]");
	nprintf(UART_PORT, "Longitude=%u\n", fsm->tracking_payload.longitude);
	nprintf(UART_PORT, "Latitude=%u\n", fsm->tracking_payload.latitude);
	println(UART_PORT, "[END]");
}

// ------------------------------------- //
// Packet Functions -------------------- //
// ------------------------------------- //
// Interpretation ---------------------- //
void get_motion_arguments(pingPongFSM_t *const fsm) {
	// Reset the argument iterator, to start with the first one.
	fsm->motion_payload.id = (ID_TYPE) get_argument(fsm, sizeof(ID_TYPE));
	fsm->motion_payload.time = (TIME_TYPE) get_argument(fsm, sizeof(TIME_TYPE));
	fsm->motion_payload.roll = (ROLL_TYPE) get_argument(fsm, sizeof(ROLL_TYPE));
	fsm->motion_payload.pitch = (PITCH_TYPE) get_argument(fsm,
			sizeof(PITCH_TYPE));
	fsm->motion_payload.yaw = (YAW_TYPE) get_argument(fsm, sizeof(YAW_TYPE));
	fsm->motion_payload.sun = (SUN_TYPE) get_argument(fsm, sizeof(SUN_TYPE));
	fsm->motion_payload.status = (STATUS_TYPE) get_argument(fsm,
			sizeof(STATUS_TYPE));
	// Get arguments for reserved bytes.
	for (uint8_t i = 0; i < MOTION_RESERVED_SIZE; i++)
		fsm->motion_payload.reserved[i] = (RESERVED_TYPE) get_argument(fsm,
				sizeof(RESERVED_TYPE));
}
void get_environment_arguments(pingPongFSM_t *const fsm) {
	// Reset the argument iterator, to start with the first one.
	fsm->arg_iterator = 0;
	fsm->environment_payload.id = (ID_TYPE) get_argument(fsm, sizeof(ID_TYPE));
	fsm->environment_payload.time = (TIME_TYPE) get_argument(fsm,
			sizeof(TIME_TYPE));
	fsm->environment_payload.longitude = (LONGITUDE_TYPE) get_argument(fsm,
			sizeof(LONGITUDE_TYPE));
	fsm->environment_payload.latitude = (LATITUDE_TYPE) get_argument(fsm,
			sizeof(LATITUDE_TYPE));
	fsm->environment_payload.pressure = (PRESSURE_TYPE) get_argument(fsm,
			sizeof(PRESSURE_TYPE));
	fsm->environment_payload.gas_concentration =
			(GAS_CONCENTRATION_TYPE) get_argument(fsm,
					sizeof(GAS_CONCENTRATION_TYPE));
	fsm->environment_payload.servo = (SERVO_TYPE) get_argument(fsm,
			sizeof(SERVO_TYPE));
	fsm->environment_payload.temp = (TEMP_TYPE) get_argument(fsm,
			sizeof(TEMP_TYPE));
	fsm->environment_payload.v_bat = (V_BAT_TYPE) get_argument(fsm,
			sizeof(V_BAT_TYPE));
	fsm->environment_payload.status = (STATUS_TYPE) get_argument(fsm,
			sizeof(STATUS_TYPE));
	// Get arguments for reserved bytes.
	for (uint8_t i = 0; i < ENVIRONMENT_RESERVED_SIZE; i++)
		fsm->environment_payload.reserved[i] = (RESERVED_TYPE) get_argument(fsm,
				sizeof(RESERVED_TYPE));
}
void get_uplink_arguments(pingPongFSM_t *const fsm) {
	// Reset the argument iterator, to start with the first one.
	fsm->arg_iterator = 0;
	fsm->uplink_payload.cmd_id = (ID_TYPE) get_argument(fsm, sizeof(ID_TYPE));
	fsm->uplink_payload.arg_index = (ARG_INDEX_TYPE) get_argument(fsm,
			sizeof(ARG_INDEX_TYPE));
	fsm->uplink_payload.arg_val = (ARG_VALUE_TYPE) get_argument(fsm,
			sizeof(ARG_VALUE_TYPE));
}
void get_tracking_arguments(pingPongFSM_t *const fsm) {
	// Reset the argument iterator, to start with the first one.
	fsm->arg_iterator = 0;
	fsm->tracking_payload.longitude = (LONGITUDE_TYPE) get_argument(fsm,
			sizeof(LONGITUDE_TYPE));
	fsm->tracking_payload.latitude = (LATITUDE_TYPE) get_argument(fsm,
			sizeof(LATITUDE_TYPE));
}
uint32_t get_argument(pingPongFSM_t *const fsm, uint8_t arg_size) {
	uint32_t arg = 0;
	uint8_t i;

	for (i = 0; i < arg_size; i++) {
		arg |= (((uint32_t)(fsm->rxBuffer[fsm->arg_iterator + (arg_size - i - 1)])) << (8 * i));
	}
	fsm->arg_iterator += 1;
	return arg;
}

// Preparation ------------------------ //
void prepare_payload(pingPongFSM_t *const fsm) {
	memset(fsm->txBuffer, 0, sizeof(fsm->txBuffer));
	switch (fsm->payload_state) {
	case SYNC:
		break;
	case MOTION:
		// Payload will be 2 motion measurement iterations (22 Byte)
		fsm->arg_iterator = 0;
		put_motion_arguments(fsm);
		put_motion_arguments(fsm);
		break;
	case ENVIRONMENT:
		put_environment_arguments(fsm);
		break;
	case UPLINK:
		put_uplink_arguments(fsm);
		break;
	case TRACKING:
		put_tracking_arguments(fsm);
		break;
	default:
		break;
	}
	nprintf(UART_PORT, "Size of TX Buffer: %u \n", sizeof(motion_payload_t));
}
void put_motion_arguments(pingPongFSM_t *const fsm) {
	// Reset the argument iterator, to start with the first one.
	put_argument(fsm, (uint32_t) fsm->motion_payload.id, sizeof(ID_TYPE));
	put_argument(fsm, (uint32_t) fsm->motion_payload.time, sizeof(TIME_TYPE));
	put_argument(fsm, (uint32_t) fsm->motion_payload.roll, sizeof(ROLL_TYPE));
	put_argument(fsm, (uint32_t) fsm->motion_payload.pitch, sizeof(PITCH_TYPE));
	put_argument(fsm, (uint32_t) fsm->motion_payload.yaw, sizeof(YAW_TYPE));
	put_argument(fsm, (uint32_t) fsm->motion_payload.sun, sizeof(SUN_TYPE));
	put_argument(fsm, (uint32_t) fsm->motion_payload.status,
			sizeof(STATUS_TYPE));
	// Put arguments for reserved bytes.
	for (uint8_t i = 0; i < MOTION_RESERVED_SIZE; i++)
		put_argument(fsm, (uint32_t) fsm->motion_payload.reserved[i],
				sizeof(RESERVED_TYPE));
}
void put_environment_arguments(pingPongFSM_t *const fsm) {
	// Reset the argument iterator, to start with the first one.
	fsm->arg_iterator = 0;
	put_argument(fsm, (uint32_t) fsm->environment_payload.id, sizeof(ID_TYPE));
	put_argument(fsm, (uint32_t) fsm->environment_payload.time,
			sizeof(TIME_TYPE));
	put_argument(fsm, (uint32_t) fsm->environment_payload.longitude,
			sizeof(LONGITUDE_TYPE));
	put_argument(fsm, (uint32_t) fsm->environment_payload.latitude,
			sizeof(LATITUDE_TYPE));
	put_argument(fsm, (uint32_t) fsm->environment_payload.pressure,
			sizeof(PRESSURE_TYPE));
	put_argument(fsm, (uint32_t) fsm->environment_payload.gas_concentration,
			sizeof(GAS_CONCENTRATION_TYPE));
	put_argument(fsm, (uint32_t) fsm->environment_payload.servo,
			sizeof(SERVO_TYPE));
	put_argument(fsm, (uint32_t) fsm->environment_payload.temp,
			sizeof(TEMP_TYPE));
	put_argument(fsm, (uint32_t) fsm->environment_payload.v_bat,
			sizeof(V_BAT_TYPE));
	put_argument(fsm, (uint32_t) fsm->environment_payload.status,
			sizeof(STATUS_TYPE));
	// Put arguments for reserved bytes.
	for (uint8_t i = 0; i < ENVIRONMENT_RESERVED_SIZE; i++)
		put_argument(fsm, (uint32_t) fsm->environment_payload.reserved[i],
				sizeof(RESERVED_TYPE));
}
void put_uplink_arguments(pingPongFSM_t *const fsm) {
	// Reset the argument iterator, to start with the first one.
	fsm->arg_iterator = 0;
	put_argument(fsm, (uint32_t) fsm->uplink_payload.cmd_id, sizeof(ID_TYPE));
	put_argument(fsm, (uint32_t) fsm->uplink_payload.arg_index,
			sizeof(ARG_INDEX_TYPE));
	put_argument(fsm, (uint32_t) fsm->uplink_payload.arg_val,
			sizeof(ARG_VALUE_TYPE));
}
void put_tracking_arguments(pingPongFSM_t *const fsm) {
	// Reset the argument iterator, to start with the first one.
	fsm->arg_iterator = 0;
	put_argument(fsm, (uint32_t) fsm->tracking_payload.longitude,
			sizeof(LONGITUDE_TYPE));
	put_argument(fsm, (uint32_t) fsm->tracking_payload.latitude,
			sizeof(LATITUDE_TYPE));
}
void put_argument(pingPongFSM_t *const fsm, uint32_t arg, uint8_t arg_size) {
	uint8_t i;
	for (i = 0; i < arg_size; i++) {
		fsm->txBuffer[fsm->arg_iterator + (arg_size - i - 1)] = (uint8_t)(arg >> (8 * i));
	}
	fsm->arg_iterator += 1;
}

// ------------------------------------- //
// Event Functions --------------------- //
// ------------------------------------- //
void RadioOnDioIrq(RadioIrqMasks_t radioIrq) {
	switch (radioIrq) {
	case IRQ_TX_DONE:
		eventReceptor = eventTxDone;
		break;
	case IRQ_RX_DONE:
		eventReceptor = eventRxDone;
		break;
	case IRQ_RX_TX_TIMEOUT:
		if (SUBGRF_GetOperatingMode() == MODE_TX) {
			eventReceptor = eventTxTimeout;
		} else if (SUBGRF_GetOperatingMode() == MODE_RX) {
			eventReceptor = eventRxTimeout;
		}
		break;
	case IRQ_CRC_ERROR:
		eventReceptor = eventRxError;
		break;
	default:
		break;
	}
}
void eventTxDone(pingPongFSM_t *const fsm) {
#ifdef DEBUG
	println(UART_PORT, "TX done");
#endif
	switch (fsm->state) {
	case STATE_MASTER:
		enterMasterRx(fsm);
		fsm->subState = SSTATE_RX;
		// TODO: enter masterRx after 25 motion and 1 environment packet iterations
		break;
	case STATE_SLAVE:
		enterSlaveTx(fsm);
		fsm->subState = SSTATE_TX;
		// TODO: enter slaveRx after 25 motion and 1 environment packet iterations
		break;
	default:
		break;
	}
}
void eventRxDone(pingPongFSM_t *const fsm) {
	//TODO: Analyze the amount of motion + environment packets
	// (from payload counter) + stop the time
	// https://github.com/eewiki/stm32wl_radioDriver_pingPong/blob/main/main_lora.c
#ifdef DEBUG
	println(UART_PORT, "RX done");
#endif
	switch (fsm->state) {
	case STATE_MASTER:
		transitionRxDone(fsm); // TODO: not suitable anymore
		enterMasterRx(fsm);
		fsm->subState = SSTATE_RX;
		// TODO: enter masterRx after 25 motion and 1 environment packet iterations
		break;
	case STATE_SLAVE:
		transitionRxDone(fsm); // TODO: not suitable anymore
		enterSlaveTx(fsm);
		// TODO: enter slaveTx after 25 motion and 1 environment packet iterations
		fsm->subState = SSTATE_TX;
		break;
	default:
		break;
	}
}
void eventTxTimeout(pingPongFSM_t *const fsm) {
#ifdef DEBUG
	println(UART_PORT, "TX timeout");
#endif
	switch (fsm->state) {
	case STATE_MASTER:
		enterMasterRx(fsm);
		fsm->subState = SSTATE_RX;
		break;
	case STATE_SLAVE:
		enterSlaveTx(fsm);
		fsm->subState = SSTATE_TX;
		break;
	default:
		break;
	}
}
void eventRxTimeout(pingPongFSM_t *const fsm) {
#ifdef DEBUG
	println(UART_PORT, "RX timeout");
#endif
	switch (fsm->state) {
	case STATE_MASTER:
		HAL_Delay(fsm->rxMargin);
		enterMasterRx(fsm);
		fsm->subState = SSTATE_RX;
		break;
	case STATE_SLAVE:
		enterSlaveTx(fsm);
		fsm->subState = SSTATE_TX;
		break;
	default:
		break;
	}
}
void eventRxError(pingPongFSM_t *const fsm) {
#ifdef DEBUG
	println(UART_PORT, "RX error");
#endif
	switch (fsm->state) {
	case STATE_MASTER:
		switch (fsm->subState) {
		case SSTATE_RX:
			HAL_Delay(fsm->randomDelay);
			enterMasterTx(fsm);
			fsm->subState = SSTATE_TX;
			break;
		default:
			break;
		}
		break;
	case STATE_SLAVE:
		switch (fsm->subState) {
		case SSTATE_RX:
			enterSlaveRx(fsm);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}
void enterMasterRx(pingPongFSM_t *const fsm) {
#ifdef DEBUG
	println(UART_PORT, "Enter master RX");
#endif
	SUBGRF_SetDioIrqParams(
			IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
			IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
			IRQ_RADIO_NONE, IRQ_RADIO_NONE);
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
	packetParams.Params.LoRa.PayloadLength = 0xFF;
	SUBGRF_SetPacketParams(&packetParams);
	SUBGRF_SetRx(fsm->rxTimeout << 6);
}
void enterSlaveRx(pingPongFSM_t *const fsm) {
#ifdef DEBUG
	println(UART_PORT, "Enter slave RX");
#endif
	SUBGRF_SetDioIrqParams(
			IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
			IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
			IRQ_RADIO_NONE, IRQ_RADIO_NONE);
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
	packetParams.Params.LoRa.PayloadLength = 0xFF;
	SUBGRF_SetPacketParams(&packetParams);
	SUBGRF_SetRx(fsm->rxTimeout << 6);
}
void enterMasterTx(pingPongFSM_t *const fsm) {
#ifdef DEBUG
	println(UART_PORT, "Enter Master Tx");
	put_test_data(fsm);
#endif
	// prepare the massage to send over the LoRa modem
	prepare_payload(fsm);
	HAL_Delay(fsm->rxMargin);
	// prepare the SubGHz interface
	SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
			IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
	// Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
	SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
	// TODO: "TX_BUFFER_SIZE" has to be replaced by strlen()
	packetParams.Params.LoRa.PayloadLength = TX_BUFFER_SIZE;
	SUBGRF_SetPacketParams(&packetParams);
	SUBGRF_SendPayload((uint8_t*) fsm->txBuffer, TX_BUFFER_SIZE, 0);
}
void enterSlaveTx(pingPongFSM_t *const fsm) {
#ifdef DEBUG
	println(UART_PORT, "Enter Slave Tx");
	put_test_data(fsm);
#endif
	// prepare the massage to send over the LoRa modem
	prepare_payload(fsm);
	HAL_Delay(fsm->rxMargin);
	// prepare the SubGHz interface
	SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
			IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
	// Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
	SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
	// TODO: "TX_BUFFER_SIZE" has to be replaced by strlen()
	packetParams.Params.LoRa.PayloadLength = TX_BUFFER_SIZE;
	SUBGRF_SetPacketParams(&packetParams);
	SUBGRF_SendPayload((uint8_t*) fsm->txBuffer, TX_BUFFER_SIZE, 0);
}
// tODO: transition is not a suitable name anymore,
// cause transition only happens after 50 motion payloads
void transitionRxDone(pingPongFSM_t *const fsm) {
	PacketStatus_t packetStatus;
#ifdef DEBUG
	println(UART_PORT, "Transition RX done");
#endif
	// Workaround 15.3 in DS.SX1261-2.W.APP (because following RX w/ timeout sequence)
	SUBGRF_WriteRegister(0x0920, 0x00);
	SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

	SUBGRF_GetPayload((uint8_t*) fsm->rxBuffer, &fsm->rxSize, 0xFF);
	SUBGRF_GetPacketStatus(&packetStatus);

#ifdef DEBUG
	println(UART_PORT, "-------------------------------------------------");
	print_received_data(fsm);
	nprintf(UART_PORT, "RSSI: %d, SNR: %d \n", packetStatus.Params.LoRa.RssiPkt,
			packetStatus.Params.LoRa.SnrPkt);
	println(UART_PORT, "-------------------------------------------------");
#endif
}
// ------------------------------------- //
