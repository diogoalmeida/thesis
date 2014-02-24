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
 * @author Aziz Khakulov <khakulov@kth.se> 
 *  
 * 
 * @version  $Revision: 1.0 Date: 2011/06/07 $ 
 * @modified 2011/06/07 
 */

#include "printf.h"

#include "ServControl_profile.h"
#include "TKN154.h"

configuration ServControlC {
}
implementation {
	components MainC;
	components new TimerMilliC() as Timer;
	components ServControlP as Enc;
	Enc.Boot -> MainC.Boot;
	Enc.TimerSamples -> Timer;
	components LedsC;
	Enc.Leds -> LedsC;

	//printf
	components PrintfC;
  components SerialStartC;


	components LocalTime62500hzC;
	Enc.LocalTime -> LocalTime62500hzC;

	components new Msp430Uart0C() as UartC;
	Enc.UartStream -> UartC.UartStream;
	Enc.UartResource -> UartC.Resource;
	Enc.Msp430UartConfigure <- UartC.Msp430UartConfigure;
	
#ifdef PIN_DEBUG
	
	components HplMsp430GeneralIOC;
	components new Msp430GpioC() as PinA;
	PinA -> HplMsp430GeneralIOC.Port60;
	Enc.PinA -> PinA;
	
#endif
	/****************************************
	 * 802.15.4
	 *****************************************/
#ifdef TKN154_BEACON_DISABLED
	components Ieee802154NonBeaconEnabledC as MAC;
#else
	components Ieee802154BeaconEnabledC as MAC;
	Enc.MLME_SCAN -> MAC;
	Enc.MLME_SYNC -> MAC;
	Enc.MLME_BEACON_NOTIFY -> MAC;
	Enc.MLME_SYNC_LOSS -> MAC;
	Enc.BeaconFrame -> MAC;
#endif

	Enc.MLME_RESET -> MAC;
	Enc.MLME_SET -> MAC;
	Enc.MLME_GET -> MAC;

	Enc.MLME_START -> MAC;
	Enc.MCPS_DATA -> MAC;
	Enc.Frame -> MAC;
	Enc.Packet -> MAC;
	
}
