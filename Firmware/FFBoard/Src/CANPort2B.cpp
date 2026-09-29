/*
 * CANPort2B.cpp
 *
 *  Created on: 21.09.2023
 *      Author: Yannick
 */

#include "CANPort2B.h"
#include <cstdio>
#include <cstring>
#if defined(CANTYPE_2B)
ClassIdentifier CANPort_2B::info = {
	.name = "Can port",
	.id = CLSID_CANPORT,
	.visibility = ClassVisibility::visible};

//CANPort::CANPort(const CANPortHardwareConfig<uint32_t>& presets,uint8_t instance) : presets(presets),CommandHandler("can", CLSID_CANPORT, instance){
//
//}

CANPort_2B::CANPort_2B(CAN_HandleTypeDef &hcan,const CANPortHardwareConfig& presets,const OutputPin* silentPin,uint8_t instance) : CANPort(presets),CommandHandler("can", CLSID_CANPORT, instance), hcan(&hcan),silentPin(silentPin) {
	//HAL_CAN_Start(this->hcan);
	restoreFlashDelayed();
#ifdef CAN_COMMANDS_DISABLED_IF_NOT_USED
	this->setCommandsEnabled(false);
#endif
	registerCommands();
}
//CANPort_2B::CANPort(CAN_HandleTypeDef &hcan,const OutputPin* silentPin) : CommandHandler("can", CLSID_CANPORT, 0), hcan(&hcan), silentPin(silentPin) {
//	//HAL_CAN_Start(this->hcan);
//	restoreFlash();
//	registerCommands();
//}

void CANPort_2B::registerCommands(){
	CommandHandler::registerCommands();
	registerCommand("speed", CanPort_commands::speed, "CAN speed preset (! for list)", CMDFLAG_GET|CMDFLAG_SET|CMDFLAG_INFOSTRING);
	registerCommand("send", CanPort_commands::send, "Send CAN frame. Adr&Value required", CMDFLAG_SETADR);
	registerCommand("len", CanPort_commands::len, "Set length of next frames", CMDFLAG_SET|CMDFLAG_GET);
	// --- TrueGrip CAN 诊断 ---
	registerCommand("test", CanPort_commands::test, "DIAG: send 1 frame, wait 300ms, report TX result", CMDFLAG_GET);
	registerCommand("txresult", CanPort_commands::txresult, "DIAG: dump MCR/MSR/TSR/ESR", CMDFLAG_GET);
	registerCommand("status", CanPort_commands::status, "DIAG: one-line CAN status", CMDFLAG_GET);
	registerCommand("fastsend", CanPort_commands::fastsend, "DIAG: send 1 frame, poll TSR every 1ms", CMDFLAG_GET);
	registerCommand("pins", CanPort_commands::pins, "DIAG: dump GPIOD MODER/ODR/IDR for PD0/PD1/PD4", CMDFLAG_GET);
	registerCommand("loopback", CanPort_commands::loopback, "DIAG: switch CAN1 to internal loopback mode", CMDFLAG_GET);
	registerCommand("normal", CanPort_commands::normal, "DIAG: switch CAN1 back to normal mode", CMDFLAG_GET);
	registerCommand("txactivity", CanPort_commands::txactivity, "DIAG: send 200 frames, count PD1 low samples", CMDFLAG_GET);
	registerCommand("clocks", CanPort_commands::clocks, "DIAG: dump RCC enable regs + CAN BTR/MCR", CMDFLAG_GET);
	registerCommand("onetx", CanPort_commands::onetx, "DIAG: clear bus-off then send exactly ONE frame", CMDFLAG_GET);
	registerCommand("brklo", CanPort_commands::brklo, "DIAG: hold CAN TX pin LOW ~600ms (static dominant) and report", CMDFLAG_GET);
	registerCommand("rxtest", CanPort_commands::rxtest, "DIAG: sample CAN1_RX pin while sending", CMDFLAG_GET);
	registerCommand("selftest", CanPort_commands::selftest, "DIAG: internal loopback TX/RX test, transceiver bypassed", CMDFLAG_GET);
	registerCommand("listen", CanPort_commands::listen, "DIAG: listen-only 10s, report whether peer transmits (REC)", CMDFLAG_GET);
	registerCommand("vescping", CanPort_commands::vescping, "DIAG: send VESC COMM_PING_CAN frames (VESC must answer)", CMDFLAG_GET);
}

void CANPort_2B::saveFlash(){
	if(this->getCommandHandlerInfo()->instance != 0){
		return; // Only first instance can save
	}
	uint16_t data = (this->speedPreset & 0b111); // set the baudrate in 0..2 bit
	Flash_Write(ADR_CANCONF1, data);
}

void CANPort_2B::restoreFlash(){
	if(this->getCommandHandlerInfo()->instance != 0){
		return; // Only first instance can save
	}
	uint16_t data;
	if(Flash_Read(ADR_CANCONF1, &data)){
		setSpeedPreset(data & 0b111);
	}
}


CANPort_2B::~CANPort_2B() {
	// removes all filters
	for (uint8_t i = 0; i < canFilters.size(); i++){
		canFilters[i].FilterActivation = false;
		HAL_CAN_ConfigFilter(this->hcan, &canFilters[i]);
	}
	canFilters.clear();
	HAL_CAN_Stop(this->hcan);
	if(silentPin){
		silentPin->set(); // set pin high to disable
	}
}

/**
 * Signals that this port is being used.
 * Increments the user counter
 */
//void CANPort_2B::takePort(){
//	if(portUsers++ == 0){
//		start();
//	}
//}

/**
 * Signals that the port is not needed anymore.
 * Decrements the user counter
 */
//void CANPort_2B::freePort(){
//	if(portUsers>0){
//		portUsers--;
//	}
//
//	if(portUsers == 0){
//		stop();
//	}
//}

/**
 * Enables the can port
 */
bool CANPort_2B::start(){
	setSilentMode(false);
	active = true;
#ifdef CAN_COMMANDS_DISABLED_IF_NOT_USED
	this->setCommandsEnabled(true);
#endif
	//CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_TX_MAILBOX_EMPTY);
	setSpeedPreset(this->speedPreset); // Set preset again for a safe state
	return HAL_CAN_Start(this->hcan) == HAL_OK;
}

/**
 * Disables the can port
 */
bool CANPort_2B::stop(){
	setSilentMode(true);
	active = false;
#ifdef CAN_COMMANDS_DISABLED_IF_NOT_USED
	this->setCommandsEnabled(false);
#endif
	return HAL_CAN_Stop(this->hcan) == HAL_OK;
}


uint32_t CANPort_2B::getSpeed(){
	return presetToSpeed(speedPreset);
}

uint8_t CANPort_2B::getSpeedPreset(){
	return (speedPreset);
}


/**
 * Changes the speed of the CAN port to a preset
 */
void CANPort_2B::setSpeedPreset(uint8_t preset){
	if(preset > 5)
		return;
	speedPreset = preset;

	configSem.Take();
	HAL_CAN_Stop(this->hcan);
	HAL_CAN_AbortTxRequest(hcan, txMailboxes);
	this->hcan->Instance->BTR = presets.getPreset(preset).init;
	HAL_CAN_ResetError(hcan);

	HAL_CAN_Start(this->hcan);
	configSem.Give();
}

/**
 * Changes the speed of the CAN port in bits/s
 * Must match a preset speed
 */
void CANPort_2B::setSpeed(uint32_t speed){
	uint8_t preset = speedToPreset(speed);
	setSpeedPreset(preset);
}



/**
 * Sets the can port passive if a transceiver with silent mode is available
 */
void CANPort_2B::setSilentMode(bool silent){
	this->silent = silent;
	if(silentPin){
		silentPin->write(silent);
	}
}

/**
 * Transmits a CAN frame on this port
 * Wraps the internal transmit function
 */
bool CANPort_2B::sendMessage(CAN_tx_msg& msg){
	return this->sendMessage(&msg.header,msg.data,nullptr);
}

void CANPort_2B::abortTxRequests(){
	HAL_CAN_AbortTxRequest(hcan, txMailboxes);
}

/**
 * Transmits a CAN frame with separate data and header settings
 */
bool CANPort_2B::sendMessage(CAN_msg_header_tx *pHeader, uint8_t aData[],uint32_t *pTxMailbox){

	header.DLC = pHeader->length;
	header.IDE = pHeader->extId ? CAN_ID_EXT : CAN_ID_STD;
	header.RTR = pHeader->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
	header.StdId = pHeader->extId ? 0 : pHeader->id;
	header.ExtId = pHeader->extId ? pHeader->id : 0;

	if(this->silent)
		setSilentMode(false);
	uint32_t mailbox;
	if(pTxMailbox == nullptr){
		pTxMailbox = &mailbox;
	}

	if(!HAL_CAN_GetTxMailboxesFreeLevel(hcan) && HAL_GetTick() - lastSentTime > sendTimeout){
		// Mailbox full and nothing has been sent successfully for some time. Abort previous requests if timeout reached
		HAL_CAN_AbortTxRequest(hcan, txMailboxes);
	}
//
	if(!HAL_CAN_GetTxMailboxesFreeLevel(hcan)){ // Only wait if no mailbox is free
		isWaitingFlag = true;
		if(!takeSemaphore(sendTimeout)){
			isWaitingFlag = false;
			return false;
		}

	}

	//this->isTakenFlag = true;
	if (HAL_CAN_AddTxMessage(this->hcan, &header, aData, pTxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
		if(isWaitingFlag)
			giveSemaphore();
	  return false;
	}
	txMailboxes |= *pTxMailbox;
//	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)) // Give back semaphore immediately if mailboxes are still free
//		giveSemaphore();
	return true;
}

void CANPort_2B::canTxCpltCallback(CANPort *port,uint32_t mailbox){
	if(port == this){
		lastSentTime = HAL_GetTick();
		txMailboxes &= ~mailbox;
		if(isWaitingFlag)
			giveSemaphore();
	}
}

void CANPort_2B::canTxAbortCallback(CANPort *port,uint32_t mailbox){
	if(port == this){
		txMailboxes &= ~mailbox;
		if(isWaitingFlag)
			giveSemaphore();
	}
}

void CANPort_2B::canErrorCallback(CANPort *port, uint32_t code){
	if(
		port == this &&
		code & (HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_ALST2 | HAL_CAN_ERROR_TX_TERR0 | HAL_CAN_ERROR_TX_TERR1 | HAL_CAN_ERROR_TX_TERR2)
	){
		giveSemaphore();
	}
}


/**
 * TrueGrip 诊断用：在环回自测期间抓取收到的报文
 */
void CANPort_2B::canRxPendCallback(CANPort *port,CAN_rx_msg& msg){
	if(port != this || !testMode){
		return;
	}
	rxcnt++;
	if(rxlen == 0){
		rxid = msg.header.id;
		rxlen = msg.header.length;
		for(uint32_t i = 0; i < msg.header.length && i < 8; i++){
			rxbuf[i] = msg.data[i];
		}
		rxflag = true;
	}
}

/**
 * Adds a filter to the can handle
 * Returns a free bank id if successfull and -1 if all banks are full
 * Use the returned id to disable the filter again
 */
int32_t CANPort_2B::addCanFilter(CAN_filter filter){

	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterActivation = filter.active;
	sFilterConfig.FilterBank = 0; // Init new
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterFIFOAssignment = filter.buffer == 0 ? CAN_RX_FIFO0 : CAN_RX_FIFO1;


	if (!filter.extid) {
	    // Standard 11-bit ID -> bits 31:21
	    sFilterConfig.FilterIdHigh     = (filter.filter_id << 5) & 0xFFFF;
	    sFilterConfig.FilterIdLow      = 0;
	    sFilterConfig.FilterMaskIdHigh = (filter.filter_mask << 5) & 0xFFFF;
	    sFilterConfig.FilterMaskIdLow  = 0;
	} else {
	    // Extended 29-bit ID -> bits 31:3
	    sFilterConfig.FilterIdHigh     = (filter.filter_id >> 13) & 0xFFFF;
	    sFilterConfig.FilterIdLow      = (filter.filter_id << 3) & 0xFFF8;
	    sFilterConfig.FilterMaskIdHigh = (filter.filter_mask >> 13) & 0xFFFF;
	    sFilterConfig.FilterMaskIdLow  = (filter.filter_mask << 3) & 0xFFF8;

	    sFilterConfig.FilterIdLow     |= 0x04; // IDE = 1
	    sFilterConfig.FilterMaskIdLow |= 0x04; // force IDE bit to match
	}


	configSem.Take();
	int32_t lowestId = 0;// sFilterConfig.FilterFIFOAssignment == CAN_RX_FIFO0 ? 0 : slaveFilterStart;
	int32_t highestId = slaveFilterStart;// sFilterConfig.FilterFIFOAssignment == CAN_RX_FIFO0 ? slaveFilterStart : 29;
	int32_t foundId = -1;

	for(uint8_t id = lowestId; id < highestId ; id++ ){
		bool foundExisting = false;
		for(CAN_FilterTypeDef filter : canFilters){
			if(id == filter.FilterBank
//					&& filter.FilterIdHigh == sFilterConfig.FilterIdHigh && filter.FilterIdLow == sFilterConfig.FilterIdLow &&filter.FilterFIFOAssignment == sFilterConfig.FilterFIFOAssignment &&
//					&& filter.FilterMaskIdHigh == sFilterConfig.FilterMaskIdHigh && filter.FilterMaskIdLow == sFilterConfig.FilterMaskIdLow
//					&& filter.FilterMode == sFilterConfig.FilterMode && filter.FilterScale == sFilterConfig.FilterScale
					)
			{
				foundExisting = true;
				break;
			}
		}
		foundId = id;
		if(!foundExisting){
			break;
		}

	}
	if(foundId < highestId){
		if(sFilterConfig.FilterBank == 0)
			sFilterConfig.FilterBank = foundId;
		if (HAL_CAN_ConfigFilter(this->hcan, &sFilterConfig) == HAL_OK){
			canFilters.push_back(sFilterConfig);
		}
	}
	configSem.Give();
	return foundId;
}

/**
 * Disables a can filter
 * Use the id returned by the addCanFilter function
 */
void CANPort_2B::removeCanFilter(uint8_t filterId){
	configSem.Take();
	for (uint8_t i = 0; i < canFilters.size(); i++){
		if(canFilters[i].FilterBank == filterId){
			canFilters[i].FilterActivation = false;
			HAL_CAN_ConfigFilter(this->hcan, &canFilters[i]);
			canFilters.erase(canFilters.begin()+i);
			break;
		}
	}
	configSem.Give();
}


CommandStatus CANPort_2B::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){

	switch(static_cast<CanPort_commands>(cmd.cmdId)){

	case CanPort_commands::speed:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->speedPreset);
		}else if(cmd.type == CMDtype::set){
			setSpeedPreset(cmd.val);
		}else if(cmd.type == CMDtype::info){
			for(uint8_t i = 0; i<presets.presets.size();i++){
				replies.emplace_back(std::string(presets.presets[i].name)  + ":" + std::to_string(i));
			}
		}
	break;

	case CanPort_commands::send:
	{
		if(cmd.type == CMDtype::setat){
			if(!active){
				start(); // If port is not used activate port at first use
			}
			CAN_tx_msg msg;
			memcpy(msg.data,&cmd.val,8);
			msg.header.id = cmd.adr;
			msg.header.length = nextLen;
			sendMessage(msg);
		}else{
			return CommandStatus::NOT_FOUND;
		}
		break;
	}
	case CanPort_commands::len:
		handleGetSet(cmd, replies, nextLen);
		nextLen = std::min<uint32_t>(nextLen,8);
		break;

	// ================= TrueGrip CAN 诊断 =================
	case CanPort_commands::txresult:
	{
		uint32_t mcr = hcan->Instance->MCR;
		uint32_t msr = hcan->Instance->MSR;
		uint32_t tsr = hcan->Instance->TSR;
		uint32_t esr = hcan->Instance->ESR;
		uint32_t tec = esr & 0xFF;
		uint32_t rec = (esr >> 8) & 0xFF;
		uint32_t lec = (esr >> 4) & 0x7;
		char buf[220];
		snprintf(buf, sizeof(buf),
			"MCR=0x%08lX INRQ=%lu SLEEP=%lu MSR=0x%08lX INAK=%lu SLAK=%lu "
			"TSR=0x%08lX TXOK=%lu%lu%lu TME=%lu%lu%lu ESR=0x%08lX TEC=%lu REC=%lu LEC=%lu BOFF=%lu",
			(unsigned long)mcr, (unsigned long)(mcr & 1), (unsigned long)((mcr >> 1) & 1),
			(unsigned long)msr, (unsigned long)(msr & 1), (unsigned long)((msr >> 1) & 1),
			(unsigned long)tsr,
			(unsigned long)(tsr & 1), (unsigned long)((tsr >> 8) & 1), (unsigned long)((tsr >> 16) & 1),
			(unsigned long)((tsr >> 26) & 1), (unsigned long)((tsr >> 27) & 1), (unsigned long)((tsr >> 28) & 1),
			(unsigned long)esr, (unsigned long)tec, (unsigned long)rec, (unsigned long)lec,
			(unsigned long)((esr >> 2) & 1));
		replies.emplace_back(std::string(buf));
		replies.emplace_back((int64_t)tec);
		break;
	}

	case CanPort_commands::test:
	{
		if(!active){
			start();
		}
		CAN_tx_msg msg;
		memset(&msg, 0, sizeof(msg));
		msg.header.id = 0x123;
		msg.header.length = 8;
		msg.header.extId = false;
		for(int i = 0; i < 8; i++){
			msg.data[i] = 0;
		}
		bool ok = sendMessage(msg);
		HAL_Delay(300);
		uint32_t tsr2 = hcan->Instance->TSR;
		uint32_t esr2 = hcan->Instance->ESR;
		uint32_t tec2 = esr2 & 0xFF;
		uint32_t lec2 = (esr2 >> 4) & 0x7;
		int txok = ((tsr2 & 1) || ((tsr2 >> 8) & 1) || ((tsr2 >> 16) & 1)) ? 1 : 0;
		const char* verdict = txok ? "ACK_OK_transceiver_bus_peer_good"
			: (tec2 ? "NO_ACK_transceiver_drives_bus_nobody_answers"
			         : "NO_TX_frame_never_left_controller");
		char buf[240];
		snprintf(buf, sizeof(buf), "sendMsg=%d TXOK=%d TEC=%lu LEC=%lu -> %s",
			ok ? 1 : 0, txok, (unsigned long)tec2, (unsigned long)lec2, verdict);
		replies.emplace_back(std::string(buf));
		replies.emplace_back((int64_t)txok);
		break;
	}

	case CanPort_commands::fastsend:
	{
		if(!active){ start(); }
		// 发送前清错误，这样读到的 LEC 就是【本次发送】的结果
		HAL_CAN_ResetError(hcan);
		uint32_t tsr0 = hcan->Instance->TSR;
		CAN_tx_msg msg;
		memset(&msg, 0, sizeof(msg));
		msg.header.id = 0x123;
		msg.header.length = 8;
		msg.header.extId = false;
		bool ok = sendMessage(msg);
		// 关键修正：先给硬件 2ms 完成发送，再采样 TSR
		int txok = 0; uint32_t first_ms = 0;
		for(uint32_t t = 0; t < 100; t++){
			HAL_Delay(2);
			uint32_t tsr = hcan->Instance->TSR;
			if((tsr & 1) || ((tsr >> 8) & 1) || ((tsr >> 16) & 1)){ txok = 1; first_ms = t * 2; break; }
		}
		uint32_t esr = hcan->Instance->ESR;
		char buf[240];
		snprintf(buf, sizeof(buf),
			"ok=%d TXOK=%d at=%lums TSR0=0x%08lX TSR=0x%08lX TEC=%lu REC=%lu LEC=%lu BOFF=%lu",
			ok ? 1 : 0, txok, (unsigned long)first_ms,
			(unsigned long)tsr0, (unsigned long)hcan->Instance->TSR,
			(unsigned long)(esr & 0xFF), (unsigned long)((esr >> 8) & 0xFF),
			(unsigned long)((esr >> 4) & 0x7), (unsigned long)((esr >> 2) & 1));
		replies.emplace_back(std::string(buf));
		replies.emplace_back((int64_t)txok);
		break;
	}

	case CanPort_commands::onetx:
	{
		// 彻底清 bus-off：Stop -> 清 INAK -> ResetError -> Start
		HAL_CAN_Stop(hcan);
		HAL_CAN_ResetError(hcan);
		HAL_CAN_Start(hcan);
		HAL_CAN_ResetError(hcan);
		uint32_t mcr0 = hcan->Instance->MCR;
		uint32_t esr0 = hcan->Instance->ESR;
		uint32_t tsr0 = hcan->Instance->TSR;
		// 只发一帧
		CAN_tx_msg msg;
		memset(&msg, 0, sizeof(msg));
		msg.header.id = 0x123;
		msg.header.length = 8;
		msg.header.extId = false;
		bool ok = sendMessage(msg);
		// 采样：立刻 / 1ms / 2ms / 5ms；重点看 TEC 和 TSR 的 TXOK/TXERR
		uint32_t t1 = hcan->Instance->TSR;
		uint32_t e1 = hcan->Instance->ESR;
		HAL_Delay(1); uint32_t t2 = hcan->Instance->TSR; uint32_t e2 = hcan->Instance->ESR;
		HAL_Delay(2); uint32_t t3 = hcan->Instance->TSR; uint32_t e3 = hcan->Instance->ESR;
		HAL_Delay(5); uint32_t t4 = hcan->Instance->TSR; uint32_t e4 = hcan->Instance->ESR;
		uint32_t tec4 = e4 & 0xFF;
		uint32_t tsrAny = t1 | t2 | t3 | t4;
		int ackOk = (tsrAny & 1) || ((tsrAny >> 8) & 1) || ((tsrAny >> 16) & 1);
		char buf[300];
		snprintf(buf, sizeof(buf),
			"ok=%d before:MCR=0x%08lX TSR=0x%08lX ESR=0x%08lX | t0:TSR=0x%08lX ESR=0x%08lX | t1:TSR=0x%08lX ESR=0x%08lX | t3:TSR=0x%08lX ESR=0x%08lX | t8:TSR=0x%08lX TEC=%lu LEC=%lu BOFF=%lu",
			ok ? 1 : 0,
			(unsigned long)mcr0, (unsigned long)tsr0, (unsigned long)esr0,
			(unsigned long)t1, (unsigned long)e1,
			(unsigned long)t2, (unsigned long)e2,
			(unsigned long)t3, (unsigned long)e3,
			(unsigned long)t4, (unsigned long)(e4 & 0xFF), (unsigned long)((e4 >> 4) & 7), (unsigned long)((e4 >> 2) & 1));
		replies.emplace_back(std::string(buf));
		if(ackOk){
			replies.emplace_back(std::string("VERDICT=BUS_OK: 收到 ACK -> 模块+总线+对端 全部正常"));
		}else if(tec4 >= 8){
			replies.emplace_back(std::string("VERDICT=NO_ACK: 帧发出去了但没人应答 -> 收发器/总线/对端 三者之一"));
		}else{
			replies.emplace_back(std::string("VERDICT=NO_TX: 帧没被硬件处理（TEC=0）-> 控制器没真正上线"));
		}
		replies.emplace_back((int64_t)ackOk);
		break;
	}

	case CanPort_commands::clocks:
	{
		uint32_t apb1 = RCC->APB1ENR;
		uint32_t ahb1 = RCC->AHB1ENR;
		uint32_t btr  = hcan->Instance->BTR;
		uint32_t mcr  = hcan->Instance->MCR;
		uint32_t msr  = hcan->Instance->MSR;
		uint32_t gmoder = GPIOD->MODER;
		uint32_t gafrl = GPIOD->AFR[0];
		char buf[280];
		snprintf(buf, sizeof(buf),
			"APB1ENR=0x%08lX CAN1EN=%lu | AHB1ENR=0x%08lX GPIODEN=%lu | BTR=0x%08lX | MCR=0x%08lX INRQ=%lu SLEEP=%lu | MSR=0x%08lX INAK=%lu SLAK=%lu | PD0MODER=%lu PD1MODER=%lu AFR0=0x%08lX",
			(unsigned long)apb1, (unsigned long)((apb1 >> 25) & 1),
			(unsigned long)ahb1, (unsigned long)((ahb1 >> 3) & 1),
			(unsigned long)btr, (unsigned long)mcr,
			(unsigned long)(mcr & 1), (unsigned long)((mcr >> 1) & 1),
			(unsigned long)msr, (unsigned long)(msr & 1), (unsigned long)((msr >> 1) & 1),
			(unsigned long)(gmoder & 3), (unsigned long)((gmoder >> 2) & 3),
			(unsigned long)gafrl);
		replies.emplace_back(std::string(buf));
		replies.emplace_back((int64_t)((apb1 >> 25) & 1));
		break;
	}

	case CanPort_commands::txactivity:
	{
		// 连续发送，每次发完立刻采样 PD1 的 IDR，统计低电平次数
		if(!active){ start(); }
		HAL_CAN_ResetError(hcan);
		int lowCnt = 0, total = 0;
		int lowAfterSend = 0;
		for(int i = 0; i < 200; i++){
			uint32_t tsr_before = hcan->Instance->TSR;
			CAN_tx_msg msg;
			memset(&msg, 0, sizeof(msg));
			msg.header.id = 0x123;
			msg.header.length = 8;
			msg.header.extId = false;
			sendMessage(msg);
			// 采样多次，抓 TX 拉低的瞬间
			for(int k = 0; k < 30; k++){
				if((GPIOD->IDR & (1u << 1)) == 0){ lowCnt++; }
				total++;
			}
			if((GPIOD->IDR & (1u << 1)) == 0){ lowAfterSend++; }
			HAL_CAN_ResetError(hcan);
		}
		char buf[240];
		snprintf(buf, sizeof(buf),
			"200 frames sent. PD1(IDR) low samples: %d / %d (%.2f%%). low-after-send: %d/200. final IDR=0x%08lX",
			lowCnt, total, (total ? 100.0 * lowCnt / total : 0.0), lowAfterSend,
			(unsigned long)GPIOD->IDR);
		replies.emplace_back(std::string(buf));
		replies.emplace_back((int64_t)lowCnt);
		break;
	}

	case CanPort_commands::loopback:
	{
		HAL_CAN_Stop(hcan);
		hcan->Instance->BTR |= (1u << 30);   // LBKM: loopback mode
		HAL_CAN_Start(hcan);
		HAL_CAN_ResetError(hcan);
		// 重新装过滤器（HAL_CAN_Stop 会清掉）
		for(CAN_FilterTypeDef f : canFilters){
			if(f.FilterActivation){ HAL_CAN_ConfigFilter(hcan, &f); }
		}
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
		replies.emplace_back(std::string("CAN1 -> LOOPBACK mode (internal, transceiver bypassed)"));
		break;
	}

	case CanPort_commands::normal:
	{
		HAL_CAN_Stop(hcan);
		hcan->Instance->BTR &= ~(1u << 30);  // clear LBKM
		HAL_CAN_Start(hcan);
		HAL_CAN_ResetError(hcan);
		replies.emplace_back(std::string("CAN1 -> NORMAL mode"));
		break;
	}

	case CanPort_commands::pins:
	{
		uint32_t moder = GPIOD->MODER;
		uint32_t odr   = GPIOD->ODR;
		uint32_t idr   = GPIOD->IDR;
		char buf[200];
		snprintf(buf, sizeof(buf),
			"MODER PD0=%lu PD1=%lu PD4=%lu | ODR PD0=%lu PD1=%lu PD4=%lu | IDR PD0=%lu PD1=%lu PD4=%lu",
			(unsigned long)(moder & 3), (unsigned long)((moder >> 2) & 3), (unsigned long)((moder >> 8) & 3),
			(unsigned long)(odr & 1), (unsigned long)((odr >> 1) & 1), (unsigned long)((odr >> 4) & 1),
			(unsigned long)(idr & 1), (unsigned long)((idr >> 1) & 1), (unsigned long)((idr >> 4) & 1));
		replies.emplace_back(std::string(buf));
		break;
	}

	case CanPort_commands::status:
	{
		uint32_t mcr = hcan->Instance->MCR;
		uint32_t msr = hcan->Instance->MSR;
		uint32_t esr = hcan->Instance->ESR;
		char buf[200];
		snprintf(buf, sizeof(buf),
			"active=%d silent=%d speed=%lu INRQ=%lu SLEEP=%lu INAK=%lu BOFF=%lu TEC=%lu REC=%lu",
			active ? 1 : 0, silent ? 1 : 0, (unsigned long)getSpeed(),
			(unsigned long)(mcr & 1), (unsigned long)((mcr >> 1) & 1),
			(unsigned long)(msr & 1), (unsigned long)((esr >> 2) & 1),
			(unsigned long)(esr & 0xFF), (unsigned long)((esr >> 8) & 0xFF));
		replies.emplace_back(std::string(buf));
		break;
	}
	// ================= TrueGrip CAN 隔离测试 =================
	// 目的：不做"发一帧看有没有 ACK"这种混在一起的测试，
	//       而是分别验证 ①MCU 的 CAN 控制器 ②模块的收发器驱动总线 的能力。
	case CanPort_commands::brklo:
	{
		// 让 CAN1_TX(PD1) 持续输出【静态低电平】（= 收发器持续显性，等效极长显性脉冲）。
		// 好处：万用表直流档能稳定读数，不再受"几微秒跳变"影响。
		// 同时读 CAN1_RX(PD0)：收发器好的话它必须跟着变低。
		HAL_CAN_Stop(hcan);
		HAL_CAN_AbortTxRequest(hcan, txMailboxes);
		uint32_t idr0 = GPIOD->IDR;
		GPIO_InitTypeDef gi = {0};
		gi.Pin = GPIO_PIN_1;
		gi.Mode = GPIO_MODE_OUTPUT_PP;
		gi.Pull = GPIO_NOPULL;
		gi.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOD, &gi);

		uint32_t lowCnt = 0;
		for(int i = 0; i < 200; i++){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // TX 拉低
			if((GPIOD->IDR & GPIO_PIN_0) == 0){ lowCnt++; }
			HAL_Delay(3); // 共约 600ms 静态显性
		}
		uint32_t idr1 = GPIOD->IDR;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); // 恢复隐性

		// 恢复 CAN 复用功能 + 重新初始化 CAN（MspInit 会把 PD0/PD1 设回 AF9）
		HAL_CAN_Init(hcan);
		setSpeedPreset(this->speedPreset); // 内部含 Stop/ResetError/Start
		HAL_CAN_ResetError(hcan);
		HAL_Delay(20);
		uint32_t esr = hcan->Instance->ESR;

		int ok = (lowCnt == 0) ? 1 : 0;
		char buf[300];
		snprintf(buf, sizeof(buf),
			"TX_PIN_LOW 600ms -> RX(PD0) low samples=%lu/200 | IDR before=0x%08lX after=0x%08lX (PD0=%lu PD1=%lu) | after-test TEC=%lu LEC=%lu",
			(unsigned long)lowCnt, (unsigned long)idr0, (unsigned long)idr1,
			(unsigned long)(idr1 & 1), (unsigned long)((idr1 >> 1) & 1),
			(unsigned long)(esr & 0xFF), (unsigned long)((esr >> 4) & 7));
		replies.emplace_back(std::string(buf));
		if(ok){
			replies.emplace_back(std::string("VERDICT=TXRX_OK: 模块收发器正常(能驱动总线且 RX 回传给 MCU)，不要再换模块"));
		}else{
			replies.emplace_back(std::string("VERDICT=TXRX_FAIL: MCU 把 TX 拉低时 RX 没跟着低 -> 模块/CTX/CRX 接线/收发器 有问题"));
		}
		replies.emplace_back((int64_t)ok);
		break;
	}

	case CanPort_commands::rxtest:
	{
		// 采样 CAN1_RX(PD0)。发送时若总线正确显性，RX 会被拉低。
		if(!active){ start(); }
		HAL_CAN_ResetError(hcan);
		uint32_t rxidle = (GPIOD->IDR & GPIO_PIN_0) ? 1 : 0;
		uint32_t lows = 0, frames = 0;
		for(uint32_t i = 0; i < 16; i++){
			CAN_tx_msg msg;
			memset(&msg, 0, sizeof(msg));
			msg.header.id = 0x123;
			msg.header.length = 8;
			msg.header.extId = false;
			if(sendMessage(msg)){ frames++; }
			for(uint32_t k = 0; k < 64; k++){
				if((GPIOD->IDR & GPIO_PIN_0) == 0){ lows++; }
			}
			HAL_CAN_ResetError(hcan); // 防 bus-off 干扰
		}
		uint32_t esr = hcan->Instance->ESR;
		char buf[280];
		snprintf(buf, sizeof(buf),
			"idle RX=%lu (1=隐性/正常) | %lu frames, RX low samples=%lu/1024 | TEC=%lu REC=%lu LEC=%lu BOFF=%lu",
			(unsigned long)rxidle, (unsigned long)frames, (unsigned long)lows,
			(unsigned long)(esr & 0xFF), (unsigned long)((esr >> 8) & 0xFF),
			(unsigned long)((esr >> 4) & 7), (unsigned long)((esr >> 2) & 1));
		replies.emplace_back(std::string(buf));
		replies.emplace_back((int64_t)lows);
		break;
	}

	case CanPort_commands::selftest:
	{
		// 内部环回：完全绕过收发器和总线。通了 => MCU 侧无嫌疑，问题一定在外部。
		if(!active){ start(); }
		HAL_CAN_Stop(hcan);
		HAL_CAN_AbortTxRequest(hcan, txMailboxes);
		memset(&rxbuf, 0, sizeof(rxbuf));
		rxlen = 0;
		rxflag = false;
		testMode = true;

		bool initOk = (HAL_CAN_Init(hcan) == HAL_OK);
		hcan->Instance->BTR |= CAN_BTR_LBKM; // 内部环回（需要先处于初始化模式，HAL_CAN_Init 已进入）
		// 放行所有报文
		CAN_FilterTypeDef f;
		memset(&f, 0, sizeof(f));
		f.FilterBank = 27;
		f.FilterMode = CAN_FILTERMODE_IDMASK;
		f.FilterScale = CAN_FILTERSCALE_32BIT;
		f.FilterIdHigh = 0; f.FilterIdLow = 0;
		f.FilterMaskIdHigh = 0; f.FilterMaskIdLow = 0;
		f.FilterFIFOAssignment = CAN_RX_FIFO0;
		f.FilterActivation = ENABLE;
		bool fltOk = (HAL_CAN_ConfigFilter(hcan, &f) == HAL_OK);
		bool startOk = (HAL_CAN_Start(hcan) == HAL_OK);
		bool notifOk = (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK);
		HAL_CAN_ResetError(hcan);

		CAN_tx_msg msg;
		memset(&msg, 0, sizeof(msg));
		msg.header.id = 0x123;
		msg.header.length = 8;
		msg.header.extId = false;
		for(int i = 0; i < 8; i++){ msg.data[i] = (uint8_t)(0xA0 + i); }
		bool sent = sendMessage(msg);

		bool txok = false;
		for(int t = 0; t < 50; t++){
			uint32_t tsr = hcan->Instance->TSR;
			if((tsr & 1) || ((tsr >> 8) & 1) || ((tsr >> 16) & 1)){ txok = true; break; }
			HAL_Delay(1);
		}
		for(int t = 0; t < 20 && rxlen == 0; t++){ HAL_Delay(1); }
		uint32_t esr = hcan->Instance->ESR;

		testMode = false;
		HAL_CAN_Stop(hcan);
		hcan->Instance->BTR &= ~CAN_BTR_LBKM;
		HAL_CAN_Init(hcan);
		// 恢复真实过滤器
		for(CAN_FilterTypeDef cf : canFilters){
			if(cf.FilterActivation){ HAL_CAN_ConfigFilter(hcan, &cf); }
		}
		setSpeedPreset(this->speedPreset);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_TX_MAILBOX_EMPTY);

		char buf[320];
		snprintf(buf, sizeof(buf),
			"init=%d flt=%d start=%d notif=%d TXOK=%d RXmsg=%lu RXlen=%lu RXid=0x%lX rxbuf[0]=0x%02X | ESR TEC=%lu LEC=%lu",
			initOk ? 1 : 0, fltOk ? 1 : 0, startOk ? 1 : 0, notifOk ? 1 : 0,
			txok ? 1 : 0, (unsigned long)rxcnt, (unsigned long)rxlen,
			(unsigned long)rxid, (unsigned)rxbuf[0],
			(unsigned long)(esr & 0xFF), (unsigned long)((esr >> 4) & 7));
		replies.emplace_back(std::string(buf));
		// 注意：HAL 的 TxMailboxCompleteCallback 会把 TXOK 清掉，
		// 所以"发成功"的可靠判据是【环回收到了内容正确的帧】，而不是 TXOK=1
		if(rxlen > 0 && rxbuf[0] == 0xA0){
			replies.emplace_back(std::string("VERDICT=MCU_OK: 内部环回 TX+RX 都正常 -> MCU/时钟/波特率/过滤器/中断 全部无罪，问题在外部链路"));
		}else if(txok){
			replies.emplace_back(std::string("VERDICT=MCU_TX_OK_RX_FAIL: 能发不能收 -> 接收中断/过滤器/FIFO 有问题"));
		}else{
			replies.emplace_back(std::string("VERDICT=MCU_FAIL: 内部环回没能自发自收 -> CAN 控制器状态有问题，与模块无关"));
		}
		replies.emplace_back((int64_t)((rxlen > 0 && rxbuf[0] == 0xA0) ? 1 : 0));
		break;
	}
	case CanPort_commands::listen:
	{
		// 只听模式（SILM=1, LBKM=0）：完全不发送、不应答任何帧，只观察。
		// 用途：对端(VESC)若在广播状态帧，REC 会涨；REC=0 说明对端一句话都没说。
		// 这一步把"F407 发不出去"和"对端不在总线上"彻底分开。
		if(!active){ start(); }
		HAL_CAN_Stop(hcan);
		HAL_CAN_AbortTxRequest(hcan, txMailboxes);
		HAL_CAN_Init(hcan);
		hcan->Instance->BTR |= CAN_BTR_SILM;    // SILM=1 只听；不动 LBKM
		CAN_FilterTypeDef f;
		memset(&f, 0, sizeof(f));
		f.FilterBank = 26;
		f.FilterMode = CAN_FILTERMODE_IDMASK;
		f.FilterScale = CAN_FILTERSCALE_32BIT;
		f.FilterIdHigh = 0; f.FilterIdLow = 0;
		f.FilterMaskIdHigh = 0; f.FilterMaskIdLow = 0;
		f.FilterFIFOAssignment = CAN_RX_FIFO0;
		f.FilterActivation = ENABLE;
		HAL_CAN_ConfigFilter(hcan, &f);
		HAL_CAN_Start(hcan);
		HAL_CAN_ResetError(hcan);

		// 只停留 1.5 秒就返回 —— 避免长时间阻塞把 USB CDC 卡死
		//
		// ⚠️ TrueGrip 修正（见 docs/27_CAN不通_硬证据定位.md §7.2）：
		// 原实现把 rec1 在**进入观察窗口之前**就读了（HAL_Delay 之后立刻读 ESR），
		// 于是 rec1 永远是 0，VERDICT 恒为 PEER_SILENT —— 这个诊断命令此前毫无价值。
		// 现在改成：先记录起点，走完 1.5s 观察窗口后再记录终点，用差值判断。
		uint32_t recBefore = (hcan->Instance->ESR >> 8) & 0xFFu;
		uint32_t f0 = hcan->Instance->RF0R & 3u;
		HAL_Delay(1500);
		uint32_t rec1 = (hcan->Instance->ESR >> 8) & 0xFFu;
		uint32_t tec1 = hcan->Instance->ESR & 0xFFu;
		uint32_t f1 = hcan->Instance->RF0R & 3u;
		uint32_t fsr1 = hcan->Instance->RF0R >> 3;
		int sawTraffic = (rec1 != recBefore) || (f1 != f0);
		(void)sawTraffic;

		// 还原：退出只听、禁用临时过滤器、恢复正常
		HAL_CAN_Stop(hcan);
		hcan->Instance->BTR &= ~CAN_BTR_SILM;
		f.FilterBank = 26;
		f.FilterActivation = DISABLE;
		HAL_CAN_Init(hcan);
		HAL_CAN_ConfigFilter(hcan, &f);
		for(CAN_FilterTypeDef cf : canFilters){
			if(cf.FilterActivation){ HAL_CAN_ConfigFilter(hcan, &cf); }
		}
		setSpeedPreset(this->speedPreset);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_TX_MAILBOX_EMPTY);

		char buf[280];
		snprintf(buf, sizeof(buf),
			"listen-only 1.5s: REC %lu -> %lu (delta %lu) | FIFO0 pending %lu -> %lu | FSR=0x%lX | TEC=%lu",
			(unsigned long)recBefore, (unsigned long)rec1, (unsigned long)(rec1 - recBefore),
			(unsigned long)f0, (unsigned long)f1,
			(unsigned long)fsr1, (unsigned long)tec1);
		replies.emplace_back(std::string(buf));
		if(rec1 != recBefore || f1 != f0){
			replies.emplace_back(std::string("VERDICT=PEER_ALIVE: 对端确实在发帧 -> 模块RX通、对端在发、波特率一致"));
		}else{
			replies.emplace_back(std::string("VERDICT=PEER_SILENT: 本次窗口内没收到帧 -> 对端没在总线上发言(或收发器待机). 可连续调用累积观察"));
		}
		replies.emplace_back((int64_t)(rec1 - recBefore));
		break;
	}
	case CanPort_commands::vescping:
	{
		// 让 F407 扮演 VESC Tool 去 ping VESC。
		// 依据（厂商固件 comm_can.c comm_can_ping()）：
		//   buffer[0] = appconf->controller_id;
		//   comm_can_transmit_eid_replace(id | ((uint32_t)CAN_PACKET_PING << 8), buffer, 1, true, 0);
		// 即扩展 ID = (17<<8)|id，payload = 发送方自己的 controller_id。
		if(!active){ start(); }
		// 关键：先彻底清 bus-off。bus-off 的节点【发不出帧】，
		// 不清的话这个测试就是无效的（我上一次就踩了这个坑）。
		HAL_CAN_Stop(hcan);
		HAL_CAN_ResetError(hcan);
		HAL_CAN_Start(hcan);
		HAL_CAN_ResetError(hcan);
		HAL_Delay(5);
		uint32_t recBefore = (hcan->Instance->ESR >> 8) & 0xFF;
		uint32_t mcrOn = hcan->Instance->MCR;
		uint32_t boffBefore = (hcan->Instance->ESR >> 2) & 1;

		CAN_tx_msg msg;
		memset(&msg, 0, sizeof(msg));
		msg.header.id = ((uint32_t)17 << 8) | 108u;   // CAN_PACKET_PING | VESC controller_id
		msg.header.length = 1;
		msg.header.extId = true;
		msg.data[0] = 108;
		sendMessage(msg);
		HAL_Delay(2);
		CAN_tx_msg msg2;
		memset(&msg2, 0, sizeof(msg2));
		msg2.header.id = ((uint32_t)17 << 8) | 0u;
		msg2.header.length = 1;
		msg2.header.extId = true;
		msg2.data[0] = 108;
		sendMessage(msg2);

		uint32_t tsrAfterSend = hcan->Instance->TSR;
		uint32_t recAfter = recBefore;
		for(int i = 0; i < 20; i++){
			HAL_Delay(20);
			uint32_t r = (hcan->Instance->ESR >> 8) & 0xFF;
			if(r > recAfter){ recAfter = r; }
		}

		char buf[300];
		uint32_t tecAfter = hcan->Instance->ESR & 0xFF;
		snprintf(buf, sizeof(buf),
			"cleared busoff(was %lu) MCR=0x%08lX | sent 2x PING eid=(17<<8)|id payload=108 | TSR=0x%08lX | REC %lu -> %lu | TEC=%lu LEC=%lu BOFF=%lu",
			(unsigned long)boffBefore, (unsigned long)mcrOn,
			(unsigned long)tsrAfterSend,
			(unsigned long)recBefore, (unsigned long)recAfter,
			(unsigned long)tecAfter,
			(unsigned long)((hcan->Instance->ESR >> 4) & 7),
			(unsigned long)((hcan->Instance->ESR >> 2) & 1));
		replies.emplace_back(std::string(buf));
		if(recAfter > recBefore){
			replies.emplace_back(std::string("VERDICT=PEER_REPLIED: 对端回了帧"));
		}else if(tecAfter >= 8){
			replies.emplace_back(std::string("VERDICT=NO_REPLY: 对端回了 0 帧，且我方发送报错(TEC>=8) -> 无人 ACK"));
		}else{
			replies.emplace_back(std::string("VERDICT=SENT_NO_REPLY: 帧已发出(TEC=0)但对端不回 -> VESC 不在运行/不在总线上"));
		}
		replies.emplace_back((int64_t)recAfter);
		break;
	}
	// =====================================================
	default:
		return CommandStatus::NOT_FOUND;
	}
	return CommandStatus::OK;
}
#endif
