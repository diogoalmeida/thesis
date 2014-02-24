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

#ifndef __SERVCONTROL_PROFILE_H
#define __SERVCONTROL_PROFILE_H

enum {
	//Radio message types
	RD_MSG_DIRECT = 0,
	RD_MSG_SETSERVOS = 1,

	//Polulu commands
	PM_COM_PARAMS = 0,
	PM_COM_SPEED = 1,
	PM_COM_POS_7 = 2,
	PM_COM_POS_8 = 3,
	PM_COM_POS_ABS = 4,
	PM_COM_NEUTR = 5,

	//Pololu parameters
	PM_START_CHAR = 0x80,
	PM_POLOLU_ID = 1,
	PM_SERVO_ON = 1,
	PM_SERVO_OFF = 0,
	PM_SERVO_FORWARD = 0,
	PM_SERVO_REVERSE = 1,
	PM_SERVO_RANGE_DEF = 0x0F, //default
	PM_SERVO_FULL_SPEED = 0, //full speed
	PM_SERVO_SLOWEST_SPEED = 1,
	PM_SERVO_FASTEST_SPEED = 0x7f,
	PM_SERVO_NEUTRAL_DEF = 0x0BB8,
	
	
	SLIDING_WINDOW = 65000,
	M_0 = 3, // macMinBE
	M_B = 8, // macMaxBE
	M = 4, //macMaxCSMABackoffs
	N = 2, //macMaxFrameRetries
	UART_QUEUE_LEN = 12,
	AM_SENSORVALUES = 10, 

	// serial comm	
	BAUD_RATE_UBR = 0x006D, // maximum for 12 floats 115200
	BAUD_RATE_UMCTL = 0x44, // maximum for 12 floats 115200
	RADIO_CHANNEL = 0x16,
	PAN_ID = 0x1234,
	BEACON_ORDER = 15,
	SUPERFRAME_ORDER = 15,  
	COORDINATOR_ADDRESS = 0x03,  
	TX_POWER = 0 // in dBm  
};

typedef nx_struct servo_serial_msg7 {
  nx_uint8_t startByte;
  nx_uint8_t deviceID;
  nx_uint8_t comNr;
  nx_uint8_t servoNum;
  nx_uint8_t data;
} servo_serial_msg7_t;

typedef nx_struct servo_serial_msg8 {
  nx_uint8_t startByte;
  nx_uint8_t deviceID;
  nx_uint8_t comNr;
  nx_uint8_t servoNum;
  nx_uint8_t data1;
  nx_uint8_t data2;
} servo_serial_msg8_t;

//Message type 0 	=> direct Polulu command, see manual
//Message type 1x => Set position 7 bit for x servos
//Message type 2x => Set position 8 bit for x servos
typedef nx_struct RadioMsg {
	nx_uint8_t MsgType;	
	nx_uint8_t data0;
	nx_uint8_t data1;
	nx_uint8_t data2;
	nx_uint8_t data3;
	nx_uint8_t data4;
	nx_uint8_t data5;
	nx_uint8_t data6;
	nx_uint8_t data7;
} RadioMsg;
#endif

