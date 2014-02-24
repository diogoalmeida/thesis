/*
 * Copyright (c) 2010, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice, this list
 * 	  of conditions and the following disclaimer.
 *
 * 	- Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation and/or other
 *	  materials provided with the distribution.
 *
 * 	- Neither the name of the KTH Royal Institute of Technology nor the names of its
 *    contributors may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */
/**
 * @author Aziz Khakulov <khakulov@kth.se> * 
 * 
 * @version  $Revision: 1.0 Date: 2011/06/07 $ 
 * @modified 2011/06/07 
 */
#define UARTDEBUG
#define DEBUGRADIOMSG

//#include "printf.h"
#include "TKN154.h"

#include "ServControl_profile.h"

module ServControlP @ safe() {

uses {
	interface Boot;
	interface Leds;
	interface LocalTime<TSymbolIEEE802154> as LocalTime;
  interface Timer<TMilli> as TimerSamples;
	interface GeneralIO as PinA; //channel A

	
	//Serial UART0
	interface UartStream;
	interface Resource as UartResource;

	// For 802.15.4
	interface Packet;
	interface MCPS_DATA;
	interface MLME_RESET;
	interface MLME_START;
	interface MLME_SET;
	interface MLME_GET;
	interface IEEE154Frame as Frame;

#ifndef TKN154_BEACON_DISABLED
	interface MLME_SCAN;
	interface MLME_SYNC;
	interface MLME_BEACON_NOTIFY;
	interface MLME_SYNC_LOSS;
	interface IEEE154BeaconFrame as BeaconFrame;
#endif
}
provides interface Msp430UartConfigure;
}
implementation {
	uint8_t j;

	//Position buffer
	uint8_t posbuff[8];
	uint8_t posbuff_len = 0; 
	
	//Buffer with 8 chained polulu commands => decapitated
	servo_serial_msg7_t plcmd[8];
	servo_serial_msg8_t pl8cmd[8];
	
	uint8_t isShortCommand = 0; //false

	servo_serial_msg7_t servoMsg7;
	servo_serial_msg8_t servoMsg8;
	// variables to send the message
	message_t m_frame;	
	uint8_t msgType;
	uint8_t leng;
	uint8_t motor_val;
	uint8_t busy = 0;
#ifndef TKN154_BEACON_DISABLED
	bool m_wasScanSuccessful;
	void setAddressingFields(uint16_t address);
	ieee154_PANDescriptor_t m_PANDescriptor;
#endif
	msp430_uart_union_config_t msp430_uart_config = {
		{
			utxe : 1,
			urxe : 1,
			ubr : BAUD_RATE_UBR,
			umctl : BAUD_RATE_UMCTL,
			ssel : 0x02,
			pena : 0,
			pev : 0,
			spb : 0,
			clen : 1,
			listen : 0,
			mm : 0,
			ckpl : 0,
			urxse : 0,
			urxeie : 1,
			urxwie : 0
		}
	};
	
	uint8_t getPololuParams(uint8_t isOn,  uint8_t isDirectionInvert, uint8_t range){
		uint8_t param = 0;
		if(isOn) param = param | (1 << 6); //Bit 6 controls on or off
		if(isDirectionInvert) param = param | (1 << 5); //Bit 5 controlls direction
		param = param | (range & 0x1F); //Range for the servo
		return param;
	}
	

	async command msp430_uart_union_config_t* Msp430UartConfigure.getConfig() {
		return &msp430_uart_config;
	}
	
	void init_uart(){
		uint16_t k,p;
		
		
		
		uint8_t par;

		//par = getPololuParams(1,0,7); //On, default dir, Range=7
		par = 0x47;
		//0b01000111;
		atomic
		{
			
			
			for(j = 0; j<8; j++){
				plcmd[j].startByte = PM_START_CHAR;
				plcmd[j].deviceID = PM_POLOLU_ID;
				plcmd[j].comNr = PM_COM_PARAMS;
				plcmd[j].servoNum = j;
				plcmd[j].data = par;
			}

			
			busy = TRUE;
		} 
		isShortCommand = 1;
		call UartResource.request();
	}	

	event void Boot.booted() {
		//printf("ServControl Running! ... \n");
		//printfflush();
		
			
		//Wait before init to allow other systems to come online
		call TimerSamples.startOneShot(10);

		//uartDebugTest();		
	}


	
event void TimerSamples.fired() {	
	call PinA.makeOutput();
	call PinA.set();
	init_uart();
	leng = sizeof(RadioMsg);
	TOSH_MAKE_UTXD0_OUTPUT();
	TOSH_SET_UTXD0_PIN();
	call MLME_RESET.request(TRUE);	
}
	
	/*********************************************************************
	 * S E R I A L   T R A N S M I T T E R
	 *********************************************************************/
	event void UartResource.granted() {
		call Leds.led2Toggle();
		
		if(isShortCommand){
			call UartStream.send((uint8_t *)&plcmd, 40);
			isShortCommand = 0;
		}
		else{
			call UartStream.send((uint8_t *)&pl8cmd, 48);
		}
	}
	
	async event void UartStream.sendDone( uint8_t* buf, uint16_t len, error_t error ) {
		busy = FALSE;

		call UartResource.release();
		
		call MLME_SET.macRxOnWhenIdle(TRUE);
	}
	async event void UartStream.receiveDone( uint8_t* buf, uint16_t len, error_t error ) {}

	async event void UartStream.receivedByte( uint8_t byte ) {
		
	}	
	/*********************************************************************
	 * E N D   S E R I A L   T R A N S M I T T E R
	 *********************************************************************/

	/*********************************************************************
	 * I E E E   8 0 2 . 1 5 . 4
	 *********************************************************************/
	void setAddressingFields(uint16_t address) {
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = address;

		call Frame.setAddressingFields(
				&m_frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security
		);
	}
	void startApp()
	{
#ifndef TKN154_BEACON_DISABLED
		ieee154_phyChannelsSupported_t channelMask;
		uint8_t scanDuration = BEACON_ORDER;
#endif
//		call Leds.led0Off();
//		call Leds.led1Off();
//		call Leds.led2Off();

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(TOS_NODE_ID);
		call MLME_SET.macRxOnWhenIdle(TRUE);
		call MLME_SET.macMaxCSMABackoffs(M);
		call MLME_SET.macMinBE(M_0);
		call MLME_SET.macMaxBE(M_B);
		call MLME_SET.macMaxFrameRetries(N);

#ifndef TKN154_BEACON_DISABLED
		// scan only the channel where we expect the coordinator
		channelMask = ((uint32_t) 1) << RADIO_CHANNEL;

		// we want all received beacons to be signalled 
		// through the MLME_BEACON_NOTIFY interface, i.e.
		// we set the macAutoRequest attribute to FALSE
		call MLME_SET.macAutoRequest(FALSE);
		call MLME_SET.macRxOnWhenIdle(TRUE);
		m_wasScanSuccessful = FALSE;
		call MLME_SCAN.request (
				PASSIVE_SCAN, // ScanType
				channelMask, // ScanChannels
				scanDuration, // ScanDuration
				0x00, // ChannelPage
				0, // EnergyDetectListNumEntries
				NULL, // EnergyDetectList
				0, // PANDescriptorListNumEntries
				NULL, // PANDescriptorList
				0 // security
		);
#endif
		//call UartResource.request();

	}

#ifndef TKN154_BEACON_DISABLED
	event message_t* MLME_BEACON_NOTIFY.indication (message_t* frame)
	{
		// received a beacon frame
		ieee154_phyCurrentPage_t page = call MLME_GET.phyCurrentPage();
		ieee154_macBSN_t beaconSequenceNumber = call BeaconFrame.getBSN(frame);

		if (!m_wasScanSuccessful) {
			// received a beacon during channel scanning
			if (call BeaconFrame.parsePANDescriptor(
							frame, RADIO_CHANNEL, page, &m_PANDescriptor) == SUCCESS) {
				// let's see if the beacon is from our coordinator...
				if (m_PANDescriptor.CoordAddrMode == ADDR_MODE_SHORT_ADDRESS &&
						m_PANDescriptor.CoordPANId == PAN_ID &&
						m_PANDescriptor.CoordAddress.shortAddress == COORDINATOR_ADDRESS) {
					// yes! wait until SCAN is finished, then syncronize to the beacons
					m_wasScanSuccessful = TRUE;
				}
			}
		} else {
			// received a beacon during synchronization, toggle LED2
			if (beaconSequenceNumber & 1)
			//call Leds.led2On();
			else
		//	call Leds.led2Off();
		}

		return frame;
	}

	event void MLME_SCAN.confirm (
			ieee154_status_t status,
			uint8_t ScanType,
			uint8_t ChannelPage,
			uint32_t UnscannedChannels,
			uint8_t EnergyDetectListNumEntries,
			int8_t* EnergyDetectList,
			uint8_t PANDescriptorListNumEntries,
			ieee154_PANDescriptor_t* PANDescriptorList
	)
	{
		if (m_wasScanSuccessful) {
//			call Leds.led0Off();
//			call Leds.led1Off();
//			call Leds.led2On();
			// we received a beacon from the coordinator before
			call MLME_SET.macCoordShortAddress(m_PANDescriptor.CoordAddress.shortAddress);
			call MLME_SET.macPANId(m_PANDescriptor.CoordPANId);
			call MLME_SYNC.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor.ChannelPage, TRUE);

		} else {
//			call Leds.led0On();
//			call Leds.led1On();
//			call Leds.led2Off();

			startApp();
		}

	}

	event void MLME_SYNC_LOSS.indication(
			ieee154_status_t lossReason,
			uint16_t PANId,
			uint8_t LogicalChannel,
			uint8_t ChannelPage,
			ieee154_security_t *security)
	{
		m_wasScanSuccessful = FALSE;
		call Leds.led2Off();
		startApp();
	}
#endif
	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status != IEEE154_SUCCESS) return;

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(TOS_NODE_ID);
		call MLME_SET.macAssociationPermit(FALSE);
		call MLME_SET.macRxOnWhenIdle(TRUE);

		call MLME_START.request(
			PAN_ID, // PANId
			RADIO_CHANNEL, // LogicalChannel
			0, // ChannelPage,
			0, // StartTime,
			BEACON_ORDER, // BeaconOrder
			SUPERFRAME_ORDER, // SuperframeOrder
			TRUE, // PANCoordinator
			FALSE, // BatteryLifeExtension
			FALSE, // CoordRealignment
			0, // CoordRealignSecurity,
			0 // BeaconSecurity
		);

		startApp();
	}
	event void MLME_START.confirm(ieee154_status_t status) {}

	event void MCPS_DATA.confirm (
		message_t *msg,
		uint8_t msduHandle,
		ieee154_status_t status,
		uint32_t timestamp
	) {
	}
	/*************************** Receive functions *************************/
	event message_t* MCPS_DATA.indication (message_t* frame) {
		RadioMsg* mesg;
		uint8_t i;		

		mesg = (RadioMsg *) call Packet.getPayload(frame, leng);
		call Leds.led1Toggle();
		/*
		#ifdef DEBUGRADIOMSG
		printf("Message type: %u \n", mesg->MsgType);
		printf("Data0: %u \n", mesg->data0);		
		printf("Data1: %u \n", mesg->data1);
		printf("Data2: %u \n", mesg->data2);
		printf("Data3: %u \n", mesg->data3);
		printf("Data4: %u \n", mesg->data4);
		printf("Data5: %u \n", mesg->data5);
		printf("Data6: %u \n", mesg->data6);
		printf("Data7: %u \n", mesg->data7);
		printfflush();
		#endif	
		*/
		
		switch(mesg->MsgType)
		{
			case 0:	 
							break;
			//Send 8 bit servo command
			case 1: if(!busy)
					{
						call Leds.led0Toggle();								
						atomic
						{
							for(j = 0; j<8; j++)
							{
								pl8cmd[j].startByte = PM_START_CHAR;
								pl8cmd[j].deviceID = PM_POLOLU_ID;
								pl8cmd[j].comNr = PM_COM_POS_8;
								pl8cmd[j].servoNum = j;
							}
							//MSB in data1
							pl8cmd[0].data1 = ((mesg->data0) & 0x80) >> 7;
							//Lower 7 bits in data2, see polulu manual
							pl8cmd[0].data2 = (mesg->data0) & 0x7F;
							
							pl8cmd[1].data1 = ((mesg->data1) & 0x80) >> 7;
							pl8cmd[1].data2 = (mesg->data1) & 0x7F;
							pl8cmd[2].data1 = ((mesg->data2) & 0x80) >> 7;
							pl8cmd[2].data2 = (mesg->data2) & 0x7F;
							pl8cmd[3].data1 = ((mesg->data3) & 0x80) >> 7;
							pl8cmd[3].data2 = (mesg->data3) & 0x7F;
							pl8cmd[4].data1 = ((mesg->data4) & 0x80) >> 7;
							pl8cmd[4].data2 = (mesg->data4) & 0x7F;
							pl8cmd[5].data1 = ((mesg->data5) & 0x80) >> 7;
							pl8cmd[5].data2 = (mesg->data5) & 0x7F;
							pl8cmd[6].data1 = ((mesg->data6) & 0x80) >> 7;
							pl8cmd[6].data2 = (mesg->data6) & 0x7F;
							pl8cmd[7].data1 = ((mesg->data7) & 0x80) >> 7;
							pl8cmd[7].data2 = (mesg->data7) & 0x7F;
							busy = TRUE;
						} 
						
						call UartResource.request();

					}
					break;
			//Send 7 bit servo command
			case 2: if(!busy)
					{
						call Leds.led0Toggle();								
						atomic
						{
							for(j = 0; j<8; j++)
							{
								plcmd[j].startByte = PM_START_CHAR;
								plcmd[j].deviceID = PM_POLOLU_ID;
								plcmd[j].comNr = PM_COM_POS_7;
								plcmd[j].servoNum = j;
							}
							plcmd[0].data = mesg->data0;
							plcmd[1].data = mesg->data1;
							plcmd[2].data = mesg->data2;
							plcmd[3].data = mesg->data3;
							plcmd[4].data = mesg->data4;
							plcmd[5].data = mesg->data5;
							plcmd[6].data = mesg->data6;
							plcmd[7].data = mesg->data7;
							
							busy = TRUE;
						} 
						
						isShortCommand = 1;
						call UartResource.request();

					}
					break;
			
			default: break;
		}

		call MLME_SET.macRxOnWhenIdle(FALSE);		
		return frame;
	}
	/*********************************************************************
	 * E N D   I E E E   8 0 2 . 1 5 . 4
	 *********************************************************************/
}
