/* *******************************************************************
   **** http://www.UAVdesign.net/                                 ****
   **** To build this mex function use:                           ****
   **** in Windows: mex 'udpcomm.c' wsock32.lib                   ****
   **** in Linux:   mex 'udpcomm.c'                               ****
   ******************************************************************* */

#define S_FUNCTION_NAME udpcomm
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include <string.h>

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/un.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#define INFINITE (-1)
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR (-1)
typedef int SOCKET;
#endif


/* Timeout settings */
typedef enum
{
    EReadTimeoutNonblocking	=  0,	/* Always return immediately */
    EReadTimeoutWait	    =  1,	/* Wait for a specified time before returning */
    EReadTimeoutBlocking	=  -1	/* Block until everything is retrieved */
} EReadTimeout;

typedef struct
{
	SOCKET              sock;
    struct sockaddr_in  remote_sin;
    struct timeval      timeout;
    EReadTimeout        timeout_type;
} CUDP, *pCUDP;

static void CUDPCreate(pCUDP S);
static void CUDPDestroy(pCUDP S);
static long CUDPOpen (pCUDP S, const char* HostName, unsigned short LocalPort, unsigned short RemotePort, int MaxPendingConn);
static long CUDPClose (pCUDP S);
static long CUDPWrite (pCUDP S, const void* pData, unsigned int iLen, unsigned long* pdwWritten);
static long CUDPRead (pCUDP S, void* pData, unsigned int iLen, unsigned long* pdwRead);
static long CUDPDataAvailable (pCUDP S);
static long CUDPSetupTimeout (pCUDP S, EReadTimeout timeout_type, int timeout);




#define COMPUTER_COMMAND_OPCODE_USB_CTRL	(0x01)
#define COMPUTER_COMMAND_OPCODE_UDP_CTRL	(0x02)
#define COMPUTER_COMMAND_OPCODE_SEND_FLASH	(0x03)
#define COMPUTER_COMMAND_OPCODE_SEND_FILTER	(0x04)

#define	COMM_API_UNKNOWN  			(0x00)
#define	COMM_API_HELICOPTER_DATA	(0x01)
#define	COMM_API_COMPUTER_DATA	    (0x02)
#define	COMM_API_COMPUTER_COMMANDS  (0x03)
#define COMM_API_READ_PARAMETERS	(0x04)		/* The computer reads the board parameters */
#define COMM_API_FLASH_PARAMETERS	(0x05)		/* The board receives the parameters and writes them to FLASH */
#define COMM_API_READ_FILTER		(0x06)		/* The computer reads the board filter */
#define COMM_API_FLASH_FILTER		(0x07)		/* The board receives the filter coefficients and writes them to FLASH */

#define COMM_MAX_DATA_LENGTH		(2000)

/* Type definitions */
#define uint8		unsigned char
#define int8		char
#define uint16		unsigned short
#define int16       short
#define uint32		unsigned int
#define int32		int

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
#pragma pack(1)
#endif
typedef struct
{
	uint8	opcode;
	uint8	transfer_ctrl;
	uint16	tx_length;
	uint32 	remote_ip;
	uint16	remote_port;
}
#if !(defined(_WIN32) || defined(_WIN64) || defined(__LCC__))
__attribute__ ((__packed__))
#endif
COMPUTER_COMMAND_UDP_CTRL;

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
#pragma pack(1)
#endif
typedef struct
{
	uint8	packet_start;
	uint8	api;
	uint16	length;

}
#if !(defined(_WIN32) || defined(_WIN64) || defined(__LCC__))
__attribute__ ((__packed__))
#endif
COMM_LINK_BEGIN;

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
#pragma pack(1)
#endif
typedef struct
{
	uint8	crc;
	uint8	packet_end;
}
#if !(defined(_WIN32) || defined(_WIN64) || defined(__LCC__))
__attribute__ ((__packed__))
#endif
COMM_LINK_END;

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
#pragma pack()
#endif

typedef struct
{
    pCUDP           UDP;
    unsigned int    _received_packets;
    unsigned int    _received_packets_real;
    unsigned int    _received_overrun_limit;
    unsigned int    _rx_len;
    char*           _rx_buf;
    char*           _rx_buf_old;
    char*           _tx_buf;
    int             _timeout;
    int             _state_rx;
    int             _write_ok;
    int             _already_read;
    int             _handshake;
    unsigned char   _api;
    unsigned char   _crc;
	unsigned short  _length;
} commProtocol, *pcommProtocol;

static void commProtocolCreate(pcommProtocol P);
static void commProtocolDestroy(pcommProtocol P);

static int OpenConnection(pcommProtocol P, const char* Name, unsigned short LocalPort, unsigned short RemotePort, unsigned int handshake);
static int CloseConnection(pcommProtocol P);
static int Send(pcommProtocol P, const char* data, unsigned int len);
static int Receive(pcommProtocol P, char* dest, unsigned int max_len);
static void SetReadTimeout(pcommProtocol P, int timeout);
static void SetReadPacketLength(pcommProtocol P, unsigned int rx_len );
static void SetReadPacketOverrunLimit(pcommProtocol P, unsigned int rx_overrun_limit );
static unsigned int GetReceivedPacketsNumber( pcommProtocol P );

static int MakeErrCode();
static int MakeErrStr(const char* ErrMsg);


static void CUDPCreate(pCUDP S)
{
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    WSADATA wsa_data;
#   endif
    
    S->sock = 0;
    
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    /* Activate the Winsock DLL */
	if (WSAStartup(MAKEWORD(2,2),&wsa_data) != 0)
    {
		/* WSA startup error */
        return;
	}
#   endif
}

static void CUDPDestroy(pCUDP S)
{
    CUDPClose(S);
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    WSACleanup();
#   endif
}

static long CUDPOpen (pCUDP S, const char* HostName, unsigned short LocalPort, unsigned short RemotePort, int MaxPendingConn)
{
    SOCKET sock;
	struct hostent *hent;
	unsigned long remote_addr;
    unsigned long ctl;
    struct sockaddr_in local_sin;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET)
    {
        /* Cannot create socket */
        return MakeErrCode();
    }
    
    if( RemotePort )
    {
        /* Clear the structure so that we don't have garbage around */
        memset(&(S->remote_sin), 0, sizeof(S->remote_sin));
        
        /* Try to find the IP address for the hostname */
        hent = gethostbyname(HostName);
        if (hent != NULL)
        {
            /* Fill in IP address */
            memcpy(&remote_addr, hent->h_addr_list[0], hent->h_length);
            S->remote_sin.sin_addr.s_addr = remote_addr;
        }
        /* If hostname does not exist try as IP address */
        else if ((S->remote_sin.sin_addr.s_addr = inet_addr(HostName))==INADDR_NONE)
        {
            /* If cannot convert IP address then it is an error */
#           if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
            closesocket(sock);
#           else
            close(sock);
#           endif
            return MakeErrStr("Invalid host name or IP address.");
        }

        /* AF means Address Family - same as Protocol Family for now */
        S->remote_sin.sin_family = AF_INET;

        /* Fill in port number in address (careful of byte-ordering) */
        S->remote_sin.sin_port = htons(RemotePort);
    }

    if( LocalPort )
    {
        /* Set up information for bind */
        /* Clear the structure so that we don't have garbage around */
        memset(&local_sin, 0, sizeof(local_sin));

        /* AF means Address Family - same as Protocol Family for now */
        local_sin.sin_family = AF_INET;

        /* Fill in port number in address (careful of byte-ordering) */
        local_sin.sin_port = htons(LocalPort);

        /* Fill in IP address for interface to bind to (INADDR_ANY) */
        local_sin.sin_addr.s_addr = htonl(INADDR_ANY);

        /* Bind to port and interface */
        if (bind(sock, (struct sockaddr *)&local_sin, sizeof(local_sin)) == SOCKET_ERROR)
        {
            /* Cannot bind */
            MakeErrCode();
#           if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
            closesocket(sock);
#           else
            close(sock);
#           endif
            return 1;
        }
    }

    /* Nonblocking mode */
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    ctl = 1;
    ioctlsocket( sock, FIONBIO, &ctl );
#   else
    fcntl(sock, F_SETFL, O_NDELAY);
#   endif
    
    S->sock = sock;
    return 0;
}

static long CUDPClose (pCUDP S)
{
    if(S->sock)
#       if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
        closesocket(S->sock);
#       else
        close(S->sock);
#       endif
	S->sock = 0;
    return 0;
}

static long CUDPWrite (pCUDP S, const void* pData, unsigned int iLen, unsigned long* pdwWritten)
{
    int retval, retval_sel;
    int sock_error;
    fd_set wfds;
    struct timeval timeout;
    
    *pdwWritten = 0;
    FD_ZERO(&wfds);
    
    if( (S->sock) && (iLen > 0) )
    {
        for( ; ; )
        {
            retval = sendto(S->sock, pData, iLen, 0, (struct sockaddr *)&(S->remote_sin), sizeof(S->remote_sin));
            if( retval == SOCKET_ERROR )
            {
#               if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
                sock_error = WSAGetLastError();
                if( sock_error == WSAEWOULDBLOCK )
#               else
                sock_error = errno;
                if( sock_error == EINTR) continue;
                if( (sock_error == EAGAIN) || (sock_error == EWOULDBLOCK) )
#               endif
                {
                    if( S->timeout_type == EReadTimeoutNonblocking ) break;
                    for( ; ; )
                    {
                        FD_SET(S->sock, &wfds);
                        timeout = S->timeout;
                        retval_sel = select(S->sock+1, NULL, &wfds, NULL, (S->timeout_type == EReadTimeoutBlocking ? NULL : &timeout));
#                       if !(defined(_WIN32) || defined(_WIN64) || defined(__LCC__))
                        if( retval_sel == SOCKET_ERROR && errno == EINTR) continue;
#endif
                        if( retval_sel == SOCKET_ERROR ) return MakeErrCode();
                        if( retval_sel == 0 ) return MakeErrStr("Write timeout has occured.");
                        break;
                    }
                }
                else return MakeErrCode();
            }
            else
            {
                *pdwWritten = retval;
                break;
            }
        }
    }
    return 0;
}

static long CUDPRead (pCUDP S, void* pData, unsigned int iLen, unsigned long* pdwRead)
{
    struct sockaddr_in req_sin;
    int recvlen, retval_sel;
    int req_len;
    int sock_error;
    fd_set rfds;
    struct timeval timeout;
    
    *pdwRead = 0;
    FD_ZERO(&rfds);
    
    if( (S->sock) && (iLen > 0) )
    {
        for( ; ; )
        {
            req_len = sizeof(req_sin);
            recvlen=recvfrom(S->sock, pData, iLen, 0, (struct sockaddr *)&req_sin, &req_len);
            if( recvlen == SOCKET_ERROR )
            {
#               if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
                sock_error = WSAGetLastError();
                if( sock_error == WSAEWOULDBLOCK )
#               else
                sock_error = errno;
                if( sock_error == EINTR) continue;
                if( (sock_error == EAGAIN) || (sock_error == EWOULDBLOCK) )
#               endif
                {
                    if( S->timeout_type == EReadTimeoutNonblocking ) break;
                    for( ; ; )
                    {
                        FD_SET(S->sock, &rfds);
                        timeout = S->timeout;
                        retval_sel = select(S->sock+1, &rfds, NULL, NULL, (S->timeout_type == EReadTimeoutBlocking ? NULL : &timeout));
#                       if !(defined(_WIN32) || defined(_WIN64) || defined(__LCC__))
                        if( retval_sel == SOCKET_ERROR && errno == EINTR) continue;
#                       endif
                        if( retval_sel == SOCKET_ERROR ) return MakeErrCode();
                        if( retval_sel == 0 ) return MakeErrStr("Read timeout has occured.");
                        break;
                    }
                }
                else return MakeErrCode();
            }
            else
            {
                *pdwRead = recvlen;
                break;
            }
        }
    }
    return 0;
}

static long CUDPDataAvailable (pCUDP S)
{
    int retval_sel;
    fd_set rfds;
    struct timeval timeout;
    
    if( !S->sock ) return 0;

    FD_ZERO(&rfds);
    for( ; ; )
    {
        FD_SET(S->sock, &rfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        retval_sel = select(S->sock+1, &rfds, NULL, NULL, &timeout);
#       if !(defined(_WIN32) || defined(_WIN64) || defined(__LCC__))
        if( retval_sel == SOCKET_ERROR && errno == EINTR) continue;
#       endif
        if( (retval_sel == SOCKET_ERROR) || (retval_sel == 0) ) return 0;
        return 1;
    }
}

static long CUDPSetupTimeout (pCUDP S, EReadTimeout timeout_type, int timeout)
{
    S->timeout_type = timeout_type;
    S->timeout.tv_sec = timeout / 1000000;
    S->timeout.tv_usec = timeout - S->timeout.tv_sec * 1000000;
    return 0;
}






static int SendPacket(pcommProtocol P, COMM_LINK_BEGIN* pheader, const void* pdata);

static int OpenConnection(pcommProtocol P, const char* HostName, unsigned short LocalPort, unsigned short RemotePort, unsigned int handshake)
{
    COMPUTER_COMMAND_UDP_CTRL command;
    COMM_LINK_BEGIN header;
    int err;
    
    if( (err=CUDPOpen(P->UDP, HostName, LocalPort, RemotePort, 10)) != 0 )
	{		
        return err;
	}
    P->_handshake = handshake;
    
    if( P->_handshake )
    {
        header.packet_start = 0x7E;
        header.length = sizeof(COMPUTER_COMMAND_UDP_CTRL);
        header.api = COMM_API_COMPUTER_COMMANDS;
    
        command.opcode = COMPUTER_COMMAND_OPCODE_UDP_CTRL;
        command.tx_length = P->_rx_len;
        command.transfer_ctrl = (P->_rx_len != 0);
        command.remote_ip = 0;
        command.remote_port = LocalPort;
        if( (err=SendPacket(P, &header, &command)) != 0 ) return err;
    }
    
    return 0;
}

static int CloseConnection(pcommProtocol P)
{
    COMPUTER_COMMAND_UDP_CTRL command;
    COMM_LINK_BEGIN header;
    int err;
    
    if( P->_handshake )
    {
        header.packet_start = 0x7E;
        header.length = sizeof(COMPUTER_COMMAND_UDP_CTRL);
        header.api = COMM_API_COMPUTER_COMMANDS;
    
        command.opcode = COMPUTER_COMMAND_OPCODE_UDP_CTRL;
        command.tx_length = P->_rx_len;
        command.transfer_ctrl = 0x00;
        command.remote_ip = 0;
        command.remote_port = 0;
        if( (err=SendPacket(P, &header, &command)) != 0 ) return err;
    }
    if( (err=CUDPClose(P->UDP)) != 0 ) return err;
    
    return 0;
}

static int Send(pcommProtocol P, const char* data, unsigned int len)
{
    COMM_LINK_BEGIN header;
    int err;
    
    header.packet_start = 0x7E;
    header.length = len;
    header.api = COMM_API_COMPUTER_DATA;
    if( (err=SendPacket(P, &header, data)) != 0 ) return err;

    return 0;
}

static int Receive(pcommProtocol P, char* dest, unsigned int max_len)
{
    COMM_LINK_BEGIN* link_header;
	COMM_LINK_END* link_end;
    unsigned char crc;
    unsigned long read_count;
	unsigned int i;
    int err;
    int read_failed;
    int received_packets;
    
    if( max_len < P->_rx_len ) return MakeErrStr("The received packet is too long.");
    
    received_packets = 0;
    err = 0;
    read_failed = 0;

    for( ; ; )
    {
        err = CUDPRead (P->UDP, P->_rx_buf, P->_rx_len + sizeof(COMM_LINK_BEGIN) + sizeof(COMM_LINK_END), &read_count);
        if( err == 0 )
        {
            if( read_count >= sizeof(COMM_LINK_BEGIN) + sizeof(COMM_LINK_END) )
            {
                read_failed = 0;
                link_header = (COMM_LINK_BEGIN *) P->_rx_buf;
                if( (link_header->api == COMM_API_HELICOPTER_DATA) || (link_header->api == COMM_API_COMPUTER_DATA) )
                {
                    if( (link_header->length == P->_rx_len) || (link_header->length == 0) )
                    {
                        link_end = (COMM_LINK_END *) (P->_rx_buf + sizeof(COMM_LINK_BEGIN) + link_header->length);
                        crc = 0;
                        for( i=0; i<link_header->length; i++ )
                        {
                            crc += P->_rx_buf[sizeof(COMM_LINK_BEGIN) + i];
                        }
                        crc += link_end->crc;
                        if( (crc == 0xFF) && (link_end->packet_end == 0xFF) )
                        {
                            received_packets++;
                            memcpy(dest, P->_rx_buf_old, P->_rx_len);
                            if( link_header->length )
                            {
                                memcpy(P->_rx_buf_old, &(P->_rx_buf[sizeof(COMM_LINK_BEGIN)]), P->_rx_len);
                            }
                            else
                            {
                                memset(P->_rx_buf_old, 0, P->_rx_len);
                            }
                        }
                        else err = MakeErrStr("Bad CRC.");
                    }
                    else err = MakeErrStr("The received packet length is greater than expected.");
                }
            }
            else if( read_count == 0 )
            {
                read_failed = 1;
            }
            else
            {
                err = MakeErrStr("The received packet is too short.");
            }
        }
        if( err ) return err;
        if( read_failed && (P->_timeout >  0) ) return MakeErrStr("Read timeout has occured.");
        
        if( !read_failed && CUDPDataAvailable(P->UDP) )
        {
            continue;
        }
        else if( received_packets || (P->_timeout == 0) )
        {
            if( (received_packets <= 1) || (P->_received_overrun_limit == 0) )
            {
                memcpy(dest, P->_rx_buf_old, P->_rx_len);
            }
            if( P->_received_overrun_limit && (received_packets == 0) && (P->_received_packets_real > 1) )
            {
                P->_received_packets = 1;
            }
            else if( P->_received_overrun_limit && (received_packets > 1) )
            {
                P->_received_packets = received_packets - 1;
            }
            else
            {
                P->_received_packets = received_packets;
            }
            P->_received_packets_real = received_packets;
            return 0;
        }
    }
}

static void SetReadPacketLength(pcommProtocol P, unsigned int rx_len )
{
    unsigned int i;
    P->_rx_len = rx_len;
    if( P->_rx_buf ) free(P->_rx_buf);
    if( P->_rx_buf_old ) free(P->_rx_buf_old);
    P->_rx_buf = (char*) malloc(rx_len + sizeof(COMM_LINK_BEGIN) + sizeof(COMM_LINK_END));
    P->_rx_buf_old = (char*) malloc(rx_len);
    for( i=0; i<P->_rx_len; i++ )
    {
        P->_rx_buf_old[i] = 0;
    }
}

static int SendPacket(pcommProtocol P, COMM_LINK_BEGIN* pheader, const void* pdata)
{
    int i, index;
    unsigned char crc;
    COMM_LINK_END link_end;
    unsigned long dwWritten;
    int err;
    
    if( pheader->length + sizeof(COMM_LINK_BEGIN) + sizeof(COMM_LINK_END) > COMM_MAX_DATA_LENGTH )
    {
        return MakeErrStr("The TX packet size is too large.");
    }
    
    index = 0;
    for( i=0; i<sizeof(COMM_LINK_BEGIN); i++ )
	{
		P->_tx_buf[index++] = ((unsigned char *)pheader)[i];
	}
    crc = 0;
    for( i=0; i<pheader->length; i++ )
	{
		P->_tx_buf[index++] = ((unsigned char *)pdata)[i];
        crc += ((unsigned char *)pdata)[i];
	}
    link_end.crc = 0xFF - crc;
    link_end.packet_end = 0xFF;
    for( i=0; i<sizeof(COMM_LINK_END); i++ )
	{
		P->_tx_buf[index++] = ((unsigned char *)&link_end)[i];
	}
    
    err = CUDPWrite(P->UDP, P->_tx_buf, index, &dwWritten);
    if(dwWritten != index)
    {
        return MakeErrStr("Could not send the full packet.");
    }
    return err;
}

static unsigned int GetReceivedPacketsNumber( pcommProtocol P )
{
    return P->_received_packets;
}

static void SetReadPacketOverrunLimit(pcommProtocol P, unsigned int rx_overrun_limit )
{
    P->_received_overrun_limit = rx_overrun_limit;
}

static void commProtocolCreate(pcommProtocol P)
{
    memset(P, 0, sizeof(commProtocol));
    P->UDP = (pCUDP) malloc(sizeof(CUDP));
    CUDPCreate(P->UDP);
    P->_tx_buf = (char*) malloc(COMM_MAX_DATA_LENGTH);
}

static void commProtocolDestroy(pcommProtocol P)
{
    if( P->_rx_buf )
    {
        free(P->_rx_buf);
        P->_rx_buf = 0;
    }
    if( P->_rx_buf_old )
    {
        free(P->_rx_buf_old);
        P->_rx_buf_old = 0;
    }
    if( P->_tx_buf )
    {
        free(P->_tx_buf);
        P->_tx_buf = 0;
    }
    CUDPDestroy(P->UDP);
    free(P->UDP);
}

static void SetReadTimeout(pcommProtocol P, int timeout)
{
    P->_timeout = timeout;
    if( P->UDP == NULL ) return;
    if( timeout < 0 )
    {
        CUDPSetupTimeout( P->UDP, EReadTimeoutBlocking, 0 );
    }
    else if( timeout == 0 )
    {
        CUDPSetupTimeout( P->UDP, EReadTimeoutNonblocking, 0 );
    }
    else
    {
        CUDPSetupTimeout( P->UDP, EReadTimeoutWait, timeout );
    }
}











#define NPARAMS                 11
#define paramHostName           ssGetSFcnParam(S,0)
#define paramUDPPort_local      ssGetSFcnParam(S,1)
#define paramUDPPort_remote     ssGetSFcnParam(S,2)
#define paramTimeout            ssGetSFcnParam(S,3)
#define paramTXEnable           ssGetSFcnParam(S,4)
#define paramRXEnable           ssGetSFcnParam(S,5)
#define paramReceiveSize        ssGetSFcnParam(S,6)
#define paramRXPacketsBuf       ssGetSFcnParam(S,7)
#define paramRXPacketsNumber    ssGetSFcnParam(S,8)
#define paramHandshake          ssGetSFcnParam(S,9)
#define paramSampleTime         ssGetSFcnParam(S,10)


#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define intval(x) ((int)((x)+0.5))

static char msg[100];

static int MakeErrCode()
{
    int err;
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    err = WSAGetLastError();
    FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM, NULL, err, 0, msg, sizeof(msg), NULL);
#   else
    err = errno;
    MakeErrStr(strerror(err));
#   endif
    return err;
}

static int MakeErrStr(const char* ErrMsg)
{
    int i;
    for(i=0; i<sizeof(msg)-3 && ErrMsg[i]; i++)
    {
        msg[i] = ErrMsg[i];
    }
    msg[i++] = '\r';
    msg[i++] = '\n';
    msg[i]   = 0;
    return 1;
}

/*====================*
 * S-function methods *
 *====================*/

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
/* Function: mdlCheckParameters =============================================
 * Abstract:
 *    Validate our parameters to verify they are okay.
 */
static void mdlCheckParameters(SimStruct *S)
{
    if ( !mxIsChar(paramHostName) )
    {
        ssSetErrorStatus(S, "The Host Name parameter must be a string inside single quotation marks, e.g. '192.168.1.1'.");
        return;
    }

    if ( (!IS_PARAM_DOUBLE(paramUDPPort_local)) || (mxGetNumberOfElements(paramUDPPort_local) != 1) )
    {
        ssSetErrorStatus(S, "Wrong local UDP port.");
        return;
    }
    else if((intval(mxGetPr(paramUDPPort_local)[0]) < 1024) || (intval(mxGetPr(paramUDPPort_local)[0]) > 65535))
    {
        ssSetErrorStatus(S, "Valid UDP ports are between 1024 and 65535.");
        return;
    }
    
    if ( (!IS_PARAM_DOUBLE(paramUDPPort_remote)) || (mxGetNumberOfElements(paramUDPPort_remote) != 1) )
    {
        ssSetErrorStatus(S, "Wrong remote UDP port.");
        return;
    }
    else if((intval(mxGetPr(paramUDPPort_local)[0]) < 1024) || (intval(mxGetPr(paramUDPPort_local)[0]) > 65535))
    {
        ssSetErrorStatus(S, "Valid UDP ports are between 1024 and 65535.");
        return;
    }
    
    if ( (!IS_PARAM_DOUBLE(paramTimeout)) || (mxGetNumberOfElements(paramTimeout) != 1) )
    {
        ssSetErrorStatus(S, "Timeout must be a scalar");
        return;
    } 
    else if (mxGetPr(paramTimeout)[0] < 0)
    {
        ssSetErrorStatus(S, "Timeout must be non-negative");
        return;
    }
    
        if ( (!IS_PARAM_DOUBLE(paramTXEnable)) || (mxGetNumberOfElements(paramTXEnable) != 1) )
    {
        ssSetErrorStatus(S, "TX Enable has to be 0 or 1.");
        return;
    }
    
    if ( (!IS_PARAM_DOUBLE(paramRXEnable)) || (mxGetNumberOfElements(paramRXEnable) != 1) )
    {
        ssSetErrorStatus(S, "RX Enable has to be 0 or 1.");
        return;
    }
    
    if ( (!IS_PARAM_DOUBLE(paramReceiveSize)) || (mxGetNumberOfElements(paramReceiveSize) != 1) )
    {
        ssSetErrorStatus(S, "Wrong receive size.");
        return;
    }

    if ( (intval(mxGetPr(paramTXEnable)[0]) == 0) && ((intval(mxGetPr(paramReceiveSize)[0]) == 0) || (intval(mxGetPr(paramRXEnable)[0]) == 0)) )
    {
        ssSetErrorStatus(S, "TX and RX can not be disabled at the same time.");
        return;
    }
    
    if ( (!IS_PARAM_DOUBLE(paramRXPacketsBuf)) || (mxGetNumberOfElements(paramRXPacketsBuf) != 1) )
    {
        ssSetErrorStatus(S, "The option to leave a packet in the RX buffer has to be 0 or 1.");
        return;
    }
    
    if ( (!IS_PARAM_DOUBLE(paramRXPacketsNumber)) || (mxGetNumberOfElements(paramRXPacketsNumber) != 1) )
    {
        ssSetErrorStatus(S, "The option to show the number of received packets has to be 0 or 1.");
        return;
    }
    
    if ( (!IS_PARAM_DOUBLE(paramHandshake)) || (mxGetNumberOfElements(paramHandshake) != 1) )
    {
        ssSetErrorStatus(S, "The option to activate the handshake has to be 0 or 1.");
        return;
    }
    
    if ( (!IS_PARAM_DOUBLE(paramSampleTime)) || (mxGetNumberOfElements(paramSampleTime) != 1) )
    {
        ssSetErrorStatus(S, "Wrong sample time.");
        return;
    }
    
    if ( !((mxGetPr(paramSampleTime)[0] == -1) || (mxGetPr(paramSampleTime)[0] > 0)) )
    {
        ssSetErrorStatus(S, "The sample time must be inherited (-1) or discrete (a positive number).");
        return;
    }
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{
    int_T param;
    
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S))
    {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
	} 
    else return; /* Parameter mismatch will be reported by Simulink */
#endif

    for( param=0; param<NPARAMS; param++ )
    {
        ssSetSFcnParamTunable(S,param,false);
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if( intval(mxGetScalar(paramTXEnable)) == 0)
    {
        if (!ssSetNumInputPorts(S, 0)) return;
    }
    else
    {
        if (!ssSetNumInputPorts(S, 1)) return;
        ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
        ssSetInputPortDataType(S, 0, SS_UINT8 );
        ssSetInputPortComplexSignal(S, 0, 0);
        ssSetInputPortDirectFeedThrough(S, 0, 0);
        ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
    }
    
    if( (intval(mxGetScalar(paramReceiveSize)) <= 0) || (intval(mxGetScalar(paramRXEnable)) == 0))
    {
        if (!ssSetNumOutputPorts(S, 0)) return;
    }
    else
    {
        if( intval(mxGetScalar(paramRXPacketsNumber)) <= 0)
        {
            if (!ssSetNumOutputPorts(S,1)) return;
        }
        else
        {
            if (!ssSetNumOutputPorts(S,2)) return;
            ssSetOutputPortWidth(S, 1, 1);
            ssSetOutputPortDataType( S, 1, SS_UINT32 );
            ssSetOutputPortComplexSignal(S, 1, 0);
        }
        ssSetOutputPortWidth(S, 0, intval(mxGetScalar(paramReceiveSize)));
        ssSetOutputPortDataType( S, 0, SS_UINT8 );
        ssSetOutputPortComplexSignal(S, 0, 0);
    }
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 1);
    
    /* Reserve place for C objects */
    ssSetNumPWork(S, 3);
    
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                     SS_OPTION_PLACE_ASAP));
}

#if defined(MATLAB_MEX_FILE)
# define MDL_SET_INPUT_PORT_WIDTH
  static void mdlSetInputPortWidth(SimStruct *S, int_T port, int_T inputPortWidth)
  {
      ssSetInputPortWidth(S, port, inputPortWidth);
  }

# define MDL_SET_OUTPUT_PORT_WIDTH
  static void mdlSetOutputPortWidth(SimStruct *S, int_T port,
                                     int_T outputPortWidth)
  {
  }

# define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
  /* Function: mdlSetDefaultPortDimensionInfo ===========================================
   * Abstract:
   *   In case no ports were specified
   */
  static void mdlSetDefaultPortDimensionInfo(SimStruct *S)
  {
      ssSetInputPortWidth(S, 0, 1);
  }
#endif


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, *mxGetPr(paramSampleTime));
	ssSetOffsetTime(S, 0, 0.0);
	ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

/* Function: mdlStart =======================================================
 * Abstract:
 * This function is called once at start of model execution. If you
 * have states that should be initialized once, this is the place
 * to do it.
 */
#define MDL_START
static void mdlStart(SimStruct *S)
{
    pcommProtocol protocol;
    char* host_name;
    int host_len;
    int_T y_len;
    unsigned short local_port;
    unsigned short remote_port;
    unsigned int   handshake;
    int err;
    
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }
    
    /* Store new C object in the pointers vector */
    protocol = (pcommProtocol) malloc(sizeof(commProtocol));
    ssGetPWork(S)[0] = protocol;
    commProtocolCreate(protocol);

    host_len = mxGetM(paramHostName) * mxGetN(paramHostName) + 1;
    host_name = (char*) malloc(host_len);
    ssGetPWork(S)[2] = host_name;
    mxGetString(paramHostName, host_name, host_len);
    
    if( ssGetNumOutputPorts(S) > 0 )
    {
        SetReadPacketLength( protocol, ssGetOutputPortWidth(S, 0) );
        SetReadPacketOverrunLimit(protocol, intval(mxGetPr(paramRXPacketsBuf)[0]));
        SetReadTimeout( protocol, intval(mxGetPr(paramTimeout)[0] * 1000000) );
    }

    if( ssGetNumOutputPorts(S) > 0 )
    {
        y_len = ssGetOutputPortWidth(S, 0);
        ssGetPWork(S)[1] = (uint8_T *) malloc(y_len);
    }
    
    if( ssGetIWork(S)[0] == 0 )
    {
        ssGetIWork(S)[0] = 1;

        local_port  = intval(mxGetPr(paramUDPPort_local )[0]);
        remote_port = intval(mxGetPr(paramUDPPort_remote)[0]);
        handshake   = intval(mxGetPr(paramHandshake )[0]);
        
        if( handshake == 0 )
        {
            if( ssGetNumOutputPorts(S) == 0 ) local_port  = 0;
            if( ssGetNumInputPorts(S)  == 0 ) remote_port = 0;
        }

        if( (err=OpenConnection(protocol, host_name, local_port, remote_port, handshake)) != 0 )
        {
            /* sprintf(msg, "Could not open the UDP port. Error=%d.\r\nLocal Port=%d, Remote Port=%d, Host=%s", err, local_port, remote_port, host_name); */
            ssSetErrorStatus(S,msg);
            return;
        }        
    }

}


/* Function: mdlOutputs =======================================================
 *
 */
#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* Retrieve C object from the pointers vector */
    pcommProtocol protocol = (pcommProtocol) ssGetPWork(S)[0];
    uint8_T *y_buf         = (uint8_T *)     ssGetPWork(S)[1];
    char* host_name        = (char *)        ssGetPWork(S)[2];
    int err;
    uint8_T *py0;
    int_T y_len;
    int i;
    
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }

    if( (ssGetNumOutputPorts(S) > 0) && (!y_buf) )
    {
         ssSetErrorStatus(S,"The output buffer has not been initialized.");
         return;
    }

    /* Read the UDP data */
    if( (ssGetNumOutputPorts(S) > 0) && (ssIsMajorTimeStep(S)) )
    {
        y_buf = (uint8_T *)ssGetPWork(S)[1];
        y_len = ssGetOutputPortWidth(S, 0);
  
        if( !y_buf )
        {
            ssSetErrorStatus(S,"The output buffer has not been initialized.");
            return;
        }
        
        /* Receive the data */
        if( (err=Receive( protocol, (char*) y_buf, y_len )) != 0 )
        {
            /* sprintf(msg,"Could not read from the UDP port. Error=%d", err); */
            ssSetErrorStatus(S,msg);
            return;
        }
    }

    if( ssGetNumOutputPorts(S) > 0 )
    {
        py0 = (uint8_T *)ssGetOutputPortSignal(S,0);
        y_len = ssGetOutputPortWidth(S, 0);
        
        /* Transfer the data from the buffer to the output */
        for( i=0; i<y_len; i++ )
        {
            py0[i] = y_buf[i];
        }
        
        if( intval(mxGetScalar(paramRXPacketsNumber)) > 0)
        {
            *((uint32_T *)ssGetOutputPortSignal(S,1)) = GetReceivedPacketsNumber( protocol );
        }
    }
}


#define MDL_UPDATE
#if defined(MDL_UPDATE)
/* Function: mdlUpdate ======================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Discrete states are typically updated here, but this function is useful
 *    for performing any tasks that should only take place once per
 *    integration step.
 */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    int err;
    uint8_T *pu;
    int_T u_len;
    pcommProtocol protocol;
    UNUSED_ARG(tid); /* not used in single tasking mode */
    
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }
    
    /* Retrieve C object from the pointers vector */
    protocol = (pcommProtocol) ssGetPWork(S)[0];
    
    if( !protocol )
    {
        ssSetErrorStatus(S,"The UDP class has not been initialized.");
        return;
    }
    
    /* Write the data to the serial port */
    if( ssGetNumInputPorts(S) > 0 )
    {
        pu = (uint8_T *)ssGetInputPortSignal(S, 0);
        u_len = ssGetInputPortWidth(S, 0);

        /* Send the data */
        if( (err=Send(protocol, (char*) pu, u_len)) != 0 )
        {
            /* sprintf(msg, "Could not write to the communication port. Error=%d", err); */
            ssSetErrorStatus(S,msg);
            return;
        }
    }
}
#endif /* MDL_UPDATE */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
#define MDL_TERMINATE
static void mdlTerminate(SimStruct *S)
{
    pcommProtocol protocol;
    
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }

    /* Retrieve and destroy C object */
    protocol = ssGetPWork(S)[0];
    
    if( protocol )
    {
        CloseConnection(protocol);
        commProtocolDestroy(protocol);
        free(protocol);
        ssGetPWork(S)[0] = 0;
    }
    
    if( ssGetPWork(S)[1] )
    {
        free( ssGetPWork(S)[1] );
        ssGetPWork(S)[1] = 0;
    }
    if( ssGetPWork(S)[2] )
    {
        free( ssGetPWork(S)[2] );
        ssGetPWork(S)[2] = 0;
    }
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
