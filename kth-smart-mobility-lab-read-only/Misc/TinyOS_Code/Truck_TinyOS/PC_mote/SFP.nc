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

#include "TKN154.h"
#include "app_profile.h"
#include "AM.h"
#include "Serial.h"

module SFP
{
  uses 
  {
    interface Boot;
    interface MCPS_DATA;
    interface MLME_RESET;
    interface MLME_START;
    interface MLME_SET;
    interface MLME_GET;
    interface IEEE154Frame as Frame;
    interface IEEE154BeaconFrame as BeaconFrame;
    interface Leds;
    interface Packet;    
    interface SplitControl as SerialControl;    
    interface AMSend as UartSend[am_id_t id];
    interface Receive as UartReceive[am_id_t id];
    interface Packet as UartPacket;
    interface AMPacket as UartAMPacket;
  }
} 

implementation 
{
  uint8_t leng;
  am_addr_t dest;
  message_t m_frame;
  uint8_t m_payloadLen;
  uint8_t *payloadRegion; 
  ieee154_PANDescriptor_t m_PANDescriptor;
  bool m_ledCount;  
  void startApp();
  task void packetSendTask();

 //----------------------------------------
  void dropBlink() {
    call Leds.led2Toggle();
  }
//----------------------------------------
  void failBlink() {
    call Leds.led0Toggle();
  }
//----------------------------------------  
	void succBlink() {
		call Leds.led1Toggle();
	}
//----------------------------------------	
    event void Boot.booted() 
    {
        leng = sizeof(SensorValues);  	
        m_payloadLen = sizeof(SFMsg);
        payloadRegion = call Packet.getPayload(&m_frame, m_payloadLen);
        call MLME_RESET.request(TRUE);
        call SerialControl.start();
    }
  
    event void SerialControl.startDone(error_t error) 
    {		
        failBlink();
    }
    
    event void SerialControl.stopDone(error_t error) { }
	
    event void MLME_RESET.confirm(ieee154_status_t status)
    {
        if (status != IEEE154_SUCCESS) 
            return;
      
        call MLME_SET.phyTransmitPower(TX_POWER);
        call MLME_SET.macShortAddress(TOS_NODE_ID);
        call MLME_SET.macAssociationPermit(FALSE);
        call MLME_SET.macRxOnWhenIdle(TRUE);

        call MLME_START.request(
                            PAN_ID,               // PANId
                            RADIO_CHANNEL,        // LogicalChannel
                            0,                    // ChannelPage,
                            0,                    // StartTime,
                            BEACON_ORDER,         // BeaconOrder
                            SUPERFRAME_ORDER,     // SuperframeOrder
                            TRUE,                 // PANCoordinator
                            FALSE,                // BatteryLifeExtension
                            FALSE,                // CoordRealignment
                            0,                    // CoordRealignSecurity,
                            0                     // BeaconSecurity
                          );
      
        startApp();
    }

    void startApp()
    {
        ieee154_address_t deviceShortAddress;
        //deviceShortAddress.shortAddress = CLIENT_ADDRESS; // destination
        deviceShortAddress.shortAddress = 0xFFFF;   // destination

        call Frame.setAddressingFields(
                                   &m_frame,                
                                   ADDR_MODE_SHORT_ADDRESS,        // SrcAddrMode,
                                   ADDR_MODE_SHORT_ADDRESS,        // DstAddrMode,
                                   PAN_ID,			   // DstPANId,
                                   &deviceShortAddress, 	   // DstAddr,
                                   NULL                            // security
                                  );  
	succBlink();
    }

    event message_t *UartReceive.receive[am_id_t id](message_t *msg,
						   void *payload,
						   uint8_t len) 
    {
        message_t *ret = msg;
	dest = call UartAMPacket.destination(msg);		
	
        if (len == sizeof(SFMsg)) 
        {		
	    memcpy(payloadRegion, payload, m_payloadLen);		
	    post packetSendTask();		
	    succBlink();		
	}
			
	return ret;  
    } 
  
    task void packetSendTask()
    {
    //  call Leds.led1Toggle();
        ieee154_address_t deviceShortAddress;
        deviceShortAddress.shortAddress = (uint8_t) dest;

	call Frame.setAddressingFields(
                                   &m_frame,                
                                   ADDR_MODE_SHORT_ADDRESS,        // SrcAddrMode,
                                   ADDR_MODE_SHORT_ADDRESS,        // DstAddrMode,
                                   PAN_ID,			   // DstPANId,
                                   &deviceShortAddress, 	   // DstAddr,
                                   NULL                            // security
                                  ); 
   
        if (call MCPS_DATA.request (
                                 &m_frame,                         // frame,
                                 m_payloadLen,                     // payloadLength,
                                 0,                                // msduHandle,
                                 TX_OPTIONS_ACK 		   // TxOptions,
                                 ) != IEEE154_SUCCESS)
      
            call Leds.led2Toggle(); //fail!
    }
 
    // Confirm reports the results of a request to transfer a frame
    event void MCPS_DATA.confirm (
                          message_t *msg,
                          uint8_t msduHandle,
                          ieee154_status_t status,
                          uint32_t timestamp
                          )
    {
        if (status == IEEE154_SUCCESS) 
        {
            call Leds.led0Toggle(); 
        }
 
    }






  event message_t* MCPS_DATA.indication (message_t* frame)
   {
   	//SensorValues* mesg;
   	//mesg = (SensorValues *) call Packet.getPayload(frame, leng);
   	//am_addr_t s = call Frame.getSrcAddrMode(frame);
   	//call UartAMPacket.setSource(frame, s);
   	if (call UartSend.send[AM_SENSORVALUES](AM_BROADCAST_ADDR, frame, leng) == SUCCESS) {
			//call Leds.led1Toggle();
	}
    return frame;
   }
	event void MLME_START.confirm(ieee154_status_t status) {}
	event void UartSend.sendDone[am_id_t id](message_t* msg, error_t error) {}
}
