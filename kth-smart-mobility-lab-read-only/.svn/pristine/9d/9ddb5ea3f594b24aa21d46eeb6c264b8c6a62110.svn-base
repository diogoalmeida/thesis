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


   @Mani Amoozadeh
   @modified 2013/03/01
   
   1. Re-arrange the code structure to make it more readable
   2. Send data to UART more efficiently (using plcmd)
   3. Increase the range from [0,127] to [0,255] (more precise movement)
   4. Set 128 as the mid-range values both for steering and motor
 */

#include "printf.h"
#include "TKN154.h"
#include "app_profile.h"

module MCP @ safe() 
{
	uses 
        {
		interface Boot;
		interface Leds;
		interface LocalTime<TSymbolIEEE802154> as LocalTime;
		interface Timer<TMilli> as TimerSamples;
		interface GeneralIO as PinA; //channel A

		// Serial UART0
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

implementation 
{
        // prototype
        void startApp();

	servo_serial_msg8_t plcmd[2];
	uint8_t neutralSent = 0;

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

	msp430_uart_union_config_t msp430_uart_config = 
        {
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

	//----------------------------------------

	servo_serial_msg7_t setParams ( uint8_t OnOff,  uint8_t direction, uint8_t range, uint8_t servoNr)
        {
                servo_serial_msg7_t MsgTmp;

		uint8_t temp = 0;
		uint8_t parameters = 0;		
		temp = OnOff << 6;
		parameters += temp;
		temp = direction << 5;
		parameters += temp;
		temp = parameters + (range & 0x1f);
		parameters = temp & 0x7f;		

		atomic 
                {		      
			MsgTmp.startByte = PM_START_CHAR;
			MsgTmp.deviceID = PM_POLOLU_ID;
			MsgTmp.comNr = PM_COM_PARAMS;
			MsgTmp.servoNum = servoNr;
			MsgTmp.data = parameters;
			msgType = 7;
		}

                return MsgTmp;		
	}

	//----------------------------------------

	void setSpeed (uint8_t speedVal, uint8_t servoNr){
		uint8_t speed;
		speed = speedVal & 0x7f;
		atomic {		
			servoMsg7.startByte = PM_START_CHAR;
			servoMsg7.deviceID = PM_POLOLU_ID;
			servoMsg7.comNr = PM_COM_SPEED;
			servoMsg7.servoNum = servoNr;
			servoMsg7.data = speedVal;
			msgType = 7;
		}		
	}

	//----------------------------------------

	servo_serial_msg7_t setPosition7 (uint8_t posVal, uint8_t servoNr)
        {
	        servo_serial_msg7_t MsgTmp;
		uint8_t pos;
                // bit seven of posval must be clear (refer to servo board manual)
		pos = posVal & 0x7f;

		atomic 
                {		
		    MsgTmp.startByte = PM_START_CHAR;
		    MsgTmp.deviceID = PM_POLOLU_ID;
		    MsgTmp.comNr = PM_COM_POS_7;
		    MsgTmp.servoNum = servoNr;
		    MsgTmp.data = pos;
		    msgType = 7;
                }

                return MsgTmp;		
	}

	//----------------------------------------

	servo_serial_msg8_t setPosition8 (uint16_t posVal, uint8_t servoNr)
        {
	        servo_serial_msg8_t MsgTmp;
		uint8_t posLow;
		uint8_t posHigh;
		uint8_t tmp;
		tmp = posVal & 0x80;
		posHigh = tmp >> 7;
		posLow = posVal & 0x7f;

		atomic 
                {
			MsgTmp.startByte = PM_START_CHAR;
			MsgTmp.deviceID = PM_POLOLU_ID;
			MsgTmp.comNr = PM_COM_POS_8;
			MsgTmp.servoNum = servoNr;
			MsgTmp.data1 = posHigh;
			MsgTmp.data2 = posLow;		
			msgType = 8;
                }

                return MsgTmp;		
	}

	//----------------------------------------

	void setPositionAbs (uint16_t posVal, uint8_t servoNr){
		uint8_t posLow;
		uint8_t posHigh;
		uint8_t tmp;
		tmp = posVal & 0x1f80;
		posHigh = tmp >> 7;
		posLow = posVal & 0x7f;
		atomic {		
			servoMsg8.startByte = PM_START_CHAR;
			servoMsg8.deviceID = PM_POLOLU_ID;
			servoMsg8.comNr = PM_COM_POS_ABS;
			servoMsg8.servoNum = servoNr;
			servoMsg8.data1 = posHigh;
			servoMsg8.data2 = posLow;
			msgType = 8;}		
	}	

	//----------------------------------------

	void setNeutral (uint16_t posVal, uint8_t servoNr){
		uint8_t posLow;
		uint8_t posHigh;
		uint8_t tmp;
		tmp = posVal & 0x1f80;
		posHigh = tmp >> 7;
		posLow = posVal & 0x7f;
		atomic {
			servoMsg8.startByte = PM_START_CHAR;
			servoMsg8.deviceID = PM_POLOLU_ID;
			servoMsg8.comNr = PM_COM_NEUTR;
			servoMsg8.servoNum = servoNr;
			servoMsg8.data1 = posHigh;
			servoMsg8.data2 = posLow;
			msgType = 8;
		}		
	}

	//----------------------------------------

	async command msp430_uart_union_config_t* Msp430UartConfigure.getConfig() 
        {
		return &msp430_uart_config;
	}

	event void Boot.booted() 
        {
		call PinA.makeOutput();
		call PinA.set();
		leng = sizeof(SFMsg);
		TOSH_MAKE_UTXD0_OUTPUT();
		TOSH_SET_UTXD0_PIN();
		call MLME_RESET.request(TRUE);		
		// call TimerSamples.startPeriodic(200);		
	}

	event void TimerSamples.fired() 
        {	
	        // if(call UartResource.isOwner()==TRUE) call Leds.led1Toggle();
		// call UartResource.request();	
	}	


	/*********************************************************************
	 * I E E E   8 0 2 . 1 5 . 4
	 *********************************************************************/

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

	event void MLME_START.confirm(ieee154_status_t status) { }

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
		if (m_wasScanSuccessful) 
                {
			//			call Leds.led0Off();
			//			call Leds.led1Off();
			//			call Leds.led2On();
			// we received a beacon from the coordinator before
			call MLME_SET.macCoordShortAddress(m_PANDescriptor.CoordAddress.shortAddress);
			call MLME_SET.macPANId(m_PANDescriptor.CoordPANId);
			call MLME_SYNC.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor.ChannelPage, TRUE);

		} 
                else 
                {
			//			call Leds.led0On();
			//			call Leds.led1On();
			//			call Leds.led2Off();

			startApp();
		}

	}

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

	void setAddressingFields(uint16_t address) 
        {
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

	event message_t* MCPS_DATA.indication (message_t* frame) 
        {		
		SFMsg* mesg;
		mesg = (SFMsg *) call Packet.getPayload(frame, leng);

                atomic
                {
                    if(!busy)
                    {
                        if(!neutralSent)
                        {
                            plcmd[0] = setPosition8 (128, STEERING_NUMBER);
                            plcmd[1] = setPosition8 (128, MOTOR_NUMBER);

                            neutralSent = TRUE;
                        }
                        else
                        {
                            plcmd[0] = setPosition8 (mesg->steering_val, STEERING_NUMBER);
                            plcmd[1] = setPosition8 (mesg->motor_val, MOTOR_NUMBER);
                        }

		        busy = TRUE;

                        call Leds.led2Toggle();   // toggle the blue LED (means we received sth)
		        call MLME_SET.macRxOnWhenIdle(FALSE);
		        call UartResource.request();
                    }
                }

		return frame;
	}

	event void MCPS_DATA.confirm (
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp
	                ) 
        {

	}

	/*********************************************************************
	 * E N D   I E E E   8 0 2 . 1 5 . 4
	 *********************************************************************/

	/*********************************************************************
	 * S E R I A L   T R A N S M I T T E R
	 *********************************************************************/

	event void UartResource.granted() 
        {
		if(call UartResource.isOwner()==TRUE) 
                    call Leds.led1Toggle();    // toggle the green LED (granted access)

                // plcms size * 7
		call UartStream.send((uint8_t *)&plcmd, 14);
	}

	async event void UartStream.sendDone( uint8_t* buf, uint16_t len, error_t error ) 
        {
	    call UartResource.release();
	    call MLME_SET.macRxOnWhenIdle(TRUE);
	    busy = FALSE;
	}

	async event void UartStream.receiveDone( uint8_t* buf, uint16_t len, error_t error ) { }

	async event void UartStream.receivedByte( uint8_t byte ) { }	

	/*********************************************************************
	 * E N D   S E R I A L   T R A N S M I T T E R
	 *********************************************************************/
}


