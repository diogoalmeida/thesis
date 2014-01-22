/* *******************************************************************
   **** http://www.UAVdesign.net/                                 ****
   **** To build this mex function use:                           ****
   **** mex 'serialcomm.c'                                        ****
   ******************************************************************* */


#define S_FUNCTION_NAME serialcomm
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define SERIAL_NO_OVERLAPPED

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
#define STRICT
#include <windows.h>
#include <tchar.h>

/* Baudrate */
typedef enum
{
    EBaudUnknown = -1,
    EBaud110     = CBR_110,
    EBaud300     = CBR_300,
    EBaud600     = CBR_600,
    EBaud1200    = CBR_1200,
    EBaud2400    = CBR_2400,
    EBaud4800    = CBR_4800,
    EBaud9600    = CBR_9600,
    EBaud14400   = CBR_14400,
    EBaud19200   = CBR_19200,
    EBaud38400   = CBR_38400,
    EBaud56000   = CBR_56000,
    EBaud57600   = CBR_57600,
    EBaud115200  = CBR_115200,
    EBaud128000  = CBR_128000,
    EBaud256000  = CBR_256000,
}
EBaudrate;

/* Data bits (5-8) */
typedef enum
{
    EDataUnknown = -1,
    EData5       =  5,
    EData6       =  6,
    EData7       =  7,
    EData8       =  8
}
EDataBits;

/* Parity scheme */
typedef enum
{
    EParUnknown = -1,
    EParNone    = NOPARITY,
    EParOdd     = ODDPARITY,
    EParEven    = EVENPARITY,
    EParMark    = MARKPARITY,
    EParSpace   = SPACEPARITY
}
EParity;

/* Stop bits */
typedef enum
{
    EStopUnknown = -1,
    EStop1       = ONESTOPBIT,
    EStop1_5     = ONE5STOPBITS,
    EStop2       = TWOSTOPBITS
} 
EStopBits;

#else
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/select.h>

#define INFINITE (-1)
typedef void* LPOVERLAPPED;

/* Baudrate */
typedef enum
{
    EBaud110     = B110,
    EBaud300     = B300,
    EBaud600     = B600,
    EBaud1200    = B1200,
    EBaud2400    = B2400,
    EBaud4800    = B4800,
    EBaud9600    = B9600,
    EBaud19200   = B19200,
    EBaud38400   = B38400,
    EBaud57600   = B57600,
    EBaud115200  = B115200
}
EBaudrate;

/* Data bits (5-8) */
typedef enum
{
    EData5       =  CS5,
    EData6       =  CS6,
    EData7       =  CS7,
    EData8       =  CS8
}
EDataBits;

/* Parity scheme */
typedef enum
{
    EParNone    = 0,
    EParOdd     = PARENB|PARODD,
    EParEven    = PARENB
}
EParity;

/* Stop bits */
typedef enum
{
    EStop1       = 0,
    EStop2       = CSTOPB
} 
EStopBits;


#endif  /* _WIN32 */

/* Handshaking */
typedef enum
{
    EHandshakeUnknown		= -1,
    EHandshakeOff			=  0,	/* No handshaking                  */
    EHandshakeHardware		=  1,	/* Hardware handshaking (RTS/CTS)  */
    EHandshakeSoftware		=  2	/* Software handshaking (XON/XOFF) */
} 
EHandshake;

/* Timeout settings */
typedef enum
{
    EReadTimeoutNonblocking	=  0,	/* Always return immediately */
    EReadTimeoutBlocking	=  -1	/* Block until everything is retrieved */
}
EReadTimeout;
    
/* Attributes */

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
typedef struct
{
	long        m_lLastError;		/* Last serial error */
	HANDLE      m_hFile;			/* File handle */
#ifndef SERIAL_NO_OVERLAPPED
	HANDLE      m_hevtOverlapped;	/* Event handle for internal overlapped operations */
#endif
} CSerial, *pCSerial;

#else

typedef struct
{
	int          m_lLastError;		/* Last serial error */
	int          m_hFile;           /* File handle */
    EReadTimeout timeout;
    long         timeout_sec;
    long         timeout_usec;
} CSerial, *pCSerial;

#endif

/* Construction */
static void CSerialCreate(pCSerial);
static void CSerialDestroy(pCSerial S);

/* Operations */
static long CSerialOpen (pCSerial S, const char* serialDevice, unsigned long dwInQueue, unsigned long dwOutQueue, int fOverlapped);
static long CSerialClose (pCSerial S);
static long CSerialSetup (pCSerial S, EBaudrate eBaudrate, EDataBits eDataBits, EParity eParity, EStopBits eStopBits);
static long CSerialSetMask (pCSerial S, unsigned long dwMask);
static long CSerialSetupHandshaking (pCSerial S, EHandshake eHandshake);
static long CSerialSetupReadTimeouts (pCSerial S, EReadTimeout eReadTimeout);
static long CSerialWrite (pCSerial S, const void* pData, unsigned int iLen, unsigned long* pdwWritten, LPOVERLAPPED lpOverlapped, unsigned long dwTimeout);
static long CSerialRead (pCSerial S, void* pData, unsigned int iLen, unsigned long* pdwRead, LPOVERLAPPED lpOverlapped, unsigned long dwTimeout);
static long CSerialPurge (pCSerial S);


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
	uint8 	opcode;
	uint8	transfer_ctrl;
    uint16	tx_length;
}
#if !(defined(_WIN32) || defined(_WIN64) || defined(__LCC__))
__attribute__ ((__packed__))
#endif
COMPUTER_COMMAND_USB_CTRL;

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
    pCSerial        Ser;
    unsigned int    _received_packets;
    unsigned int    _received_packets_real;
    unsigned int    _received_overrun_limit;
    unsigned int    _rx_len;
    char*           _rx_buf;
    char*           _rx_buf_old;
    int             _timeout;
    int             _state_rx;
    int             _write_ok;
    int             _already_read;
    unsigned char   _api;
    unsigned char   _crc;
	unsigned short  _length;
} commProtocol, *pcommProtocol;

static void commProtocolCreate(pcommProtocol P);
static void commProtocolDestroy(pcommProtocol P);

static int OpenConnection(pcommProtocol P, const char* Name, unsigned int baud);
static int CloseConnection(pcommProtocol P);
static int Send(pcommProtocol P, const char* data, unsigned int len);
static int Receive(pcommProtocol P, char* dest, unsigned int max_len);
static void SetReadTimeout(pcommProtocol P, int timeout);
static void SetReadPacketLength(pcommProtocol P, unsigned int rx_len );
static void SetReadPacketOverrunLimit(pcommProtocol P, unsigned int rx_overrun_limit );
static unsigned int GetReceivedPacketsNumber( pcommProtocol P );






#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)

static void CSerialCreate(pCSerial S)
{
	S->m_lLastError = ERROR_SUCCESS;
	S->m_hFile = 0;
#ifndef SERIAL_NO_OVERLAPPED
	S->m_hevtOverlapped = 0;
#endif
}

static void CSerialDestroy(pCSerial S)
{
	if (S->m_hFile)
	{
		CSerialClose(S);
	}
}

static long CSerialOpen (pCSerial S, const char* serialDevice, unsigned long dwInQueue, unsigned long dwOutQueue, int fOverlapped)
{
	COMMCONFIG commConfig = {0};
    unsigned long dwSize;
    
	S->m_lLastError = ERROR_SUCCESS;
	if (S->m_hFile)
	{
		S->m_lLastError = ERROR_ALREADY_INITIALIZED;
		return S->m_lLastError;
	}

	S->m_hFile = CreateFile(serialDevice,
						   GENERIC_READ|GENERIC_WRITE,
						   0,
						   0,
						   OPEN_EXISTING,
						   fOverlapped?FILE_FLAG_OVERLAPPED:0,
						   0);
	if (S->m_hFile == INVALID_HANDLE_VALUE)
	{
		S->m_hFile = 0;
		S->m_lLastError = GetLastError();
		return S->m_lLastError;
	}

#ifndef SERIAL_NO_OVERLAPPED
	/* Create the event handle for internal overlapped operations (manual reset) */
	if (fOverlapped)
	{
		S->m_hevtOverlapped = CreateEvent(0,1,0,0);
		if (S->m_hevtOverlapped == 0)
		{
			S->m_lLastError = GetLastError();
			CloseHandle(S->m_hFile);
			S->m_hFile = 0;
			return S->m_lLastError;
		}
	}
#endif

	if (dwInQueue || dwOutQueue)
	{
		/* Make sure the queue-sizes are reasonable sized. Win9X systems crash
		   if the input queue-size is zero. Both queues need to be at least
		   16 bytes large. */

		if (!SetupComm(S->m_hFile,dwInQueue,dwOutQueue))
		{
			long lLastError = GetLastError();
			CSerialClose(S);
			S->m_lLastError = lLastError;
			return S->m_lLastError;	
		}
	}

	CSerialSetMask(S, EV_BREAK|EV_ERR|EV_RXCHAR);
	CSerialSetupReadTimeouts(S, EReadTimeoutNonblocking);
	dwSize = sizeof(commConfig);
	commConfig.dwSize = dwSize;
	if (GetDefaultCommConfig(serialDevice,&commConfig,&dwSize))
	{
		SetCommConfig(S->m_hFile,&commConfig,dwSize);
	}
	return S->m_lLastError;
}

static long CSerialClose (pCSerial S)
{
	S->m_lLastError = ERROR_SUCCESS;
	if (S->m_hFile == 0)
	{
		return S->m_lLastError;
	}

#ifndef SERIAL_NO_OVERLAPPED
	if (S->m_hevtOverlapped)
	{
		CloseHandle(S->m_hevtOverlapped);
		S->m_hevtOverlapped = 0;
	}
#endif

	CloseHandle(S->m_hFile);
	S->m_hFile = 0;
	return S->m_lLastError;
}

static long CSerialSetup (pCSerial S, EBaudrate eBaudrate, EDataBits eDataBits, EParity eParity, EStopBits eStopBits)
{
    DCB dcb;
	S->m_lLastError = ERROR_SUCCESS;
	if (S->m_hFile == 0)
	{
		S->m_lLastError = ERROR_INVALID_HANDLE;
		return S->m_lLastError;
	}

    dcb.DCBlength = sizeof(DCB);
	if (!GetCommState(S->m_hFile,&dcb))
	{
		S->m_lLastError = 	GetLastError();
		return S->m_lLastError;
	}

	dcb.BaudRate = (unsigned long) eBaudrate;
	dcb.ByteSize = (BYTE) eDataBits;
	dcb.Parity   = (BYTE) eParity;
	dcb.StopBits = (BYTE) eStopBits;

	dcb.fParity  = (eParity != EParNone);

	if (!SetCommState(S->m_hFile,&dcb))
	{
		S->m_lLastError = GetLastError();
		return S->m_lLastError;
	}

	return S->m_lLastError;
}

static long CSerialSetMask (pCSerial S, unsigned long dwEventMask)
{
	S->m_lLastError = ERROR_SUCCESS;

	if (S->m_hFile == 0)
	{
		S->m_lLastError = ERROR_INVALID_HANDLE;
		return S->m_lLastError;
	}

	if (!SetCommMask(S->m_hFile,dwEventMask))
	{
		S->m_lLastError = GetLastError();
		return S->m_lLastError;
	}

	return S->m_lLastError;
}

static long CSerialSetupHandshaking (pCSerial S, EHandshake eHandshake)
{
    DCB dcb;
    
	S->m_lLastError = ERROR_SUCCESS;
	if (S->m_hFile == 0)
	{
		S->m_lLastError = ERROR_INVALID_HANDLE;
		return S->m_lLastError;
	}

    dcb.DCBlength = sizeof(DCB);
	if (!GetCommState(S->m_hFile,&dcb))
	{
		S->m_lLastError = GetLastError();
    	return S->m_lLastError;
	}

	switch (eHandshake)
	{
	case EHandshakeOff:
		dcb.fOutxCtsFlow = 0;   					/* Disable CTS monitoring */
		dcb.fOutxDsrFlow = 0;   					/* Disable DSR monitoring */
		dcb.fDtrControl = DTR_CONTROL_DISABLE;		/* Disable DTR monitoring */
		dcb.fOutX = 0;  							/* Disable XON/XOFF for transmission */
		dcb.fInX = 0;   							/* Disable XON/XOFF for receiving */
		dcb.fRtsControl = RTS_CONTROL_DISABLE;		/* Disable RTS (Ready To Send) */
		break;

	case EHandshakeHardware:
		dcb.fOutxCtsFlow = 1;   					/* Enable CTS monitoring */
		dcb.fOutxDsrFlow = 0;   					/* Disable DSR monitoring */
		dcb.fDtrControl = DTR_CONTROL_DISABLE;		/* Disable DTR handshaking */
		dcb.fOutX = 0;  							/* Disable XON/XOFF for transmission */
		dcb.fInX = 0;   							/* Disable XON/XOFF for receiving */
		dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;	/* Enable RTS handshaking */
		break;

	case EHandshakeSoftware:
		dcb.fOutxCtsFlow = 0;   					/* Disable CTS (Clear To Send) */
		dcb.fOutxDsrFlow = 0;   					/* Disable DSR (Data Set Ready) */
		dcb.fDtrControl = DTR_CONTROL_DISABLE;		/* Disable DTR (Data Terminal Ready) */
		dcb.fOutX = 1;  							/* Enable XON/XOFF for transmission */
		dcb.fInX = 1;   							/* Enable XON/XOFF for receiving */
		dcb.fRtsControl = RTS_CONTROL_DISABLE;		/* Disable RTS (Ready To Send) */
		break;

	default:
		/* This shouldn't be possible */
		S->m_lLastError = E_INVALIDARG;
		return S->m_lLastError;
	}

	if (!SetCommState(S->m_hFile,&dcb))
	{
		S->m_lLastError = GetLastError();
		return S->m_lLastError;
	}

	return S->m_lLastError;
}

static long CSerialSetupReadTimeouts (pCSerial S, EReadTimeout eReadTimeout)
{
    COMMTIMEOUTS cto;
    
	S->m_lLastError = ERROR_SUCCESS;
	if (S->m_hFile == 0)
	{
		S->m_lLastError = ERROR_INVALID_HANDLE;
		return S->m_lLastError;
	}

	if (!GetCommTimeouts(S->m_hFile,&cto))
	{
		S->m_lLastError = GetLastError();
    	return S->m_lLastError;
	}

	switch (eReadTimeout)
	{
	case EReadTimeoutBlocking:
		cto.ReadIntervalTimeout = 0;
		cto.ReadTotalTimeoutConstant = 0;
		cto.ReadTotalTimeoutMultiplier = 0;
		break;
	case EReadTimeoutNonblocking:
		cto.ReadIntervalTimeout = -1;
		cto.ReadTotalTimeoutConstant = 0;
		cto.ReadTotalTimeoutMultiplier = 0;
		break;
	default:
		cto.ReadIntervalTimeout = 0;
		cto.ReadTotalTimeoutConstant = (unsigned int)eReadTimeout;
		cto.ReadTotalTimeoutMultiplier = 0;
		break;
	}

	if (!SetCommTimeouts(S->m_hFile,&cto))
	{
		S->m_lLastError = GetLastError();
		return S->m_lLastError;
	}

	return S->m_lLastError;
}

static long CSerialWrite (pCSerial S, const void* pData, unsigned int iLen, unsigned long* pdwWritten, LPOVERLAPPED lpOverlapped, unsigned long dwTimeout)
{
    unsigned long dwWritten;
#ifndef SERIAL_NO_OVERLAPPED
    OVERLAPPED ovInternal;
    long lLastError;
#endif
    
	S->m_lLastError = ERROR_SUCCESS;

	if (pdwWritten == 0)
	{
		pdwWritten = &dwWritten;
	}

	*pdwWritten = 0;

	if (S->m_hFile == 0)
	{
		S->m_lLastError = ERROR_INVALID_HANDLE;
		return S->m_lLastError;
	}

#ifndef SERIAL_NO_OVERLAPPED

	if (!S->m_hevtOverlapped && (lpOverlapped || (dwTimeout != INFINITE)))
	{
		S->m_lLastError = ERROR_INVALID_FUNCTION;
		return S->m_lLastError;
	}

	if (!lpOverlapped && S->m_hevtOverlapped)
	{
		memset(&ovInternal,0,sizeof(ovInternal));
		ovInternal.hEvent = S->m_hevtOverlapped;
		lpOverlapped = &ovInternal;
	}

	/* Make sure the overlapped structure isn't busy */

	if (!WriteFile(S->m_hFile,pData,iLen,pdwWritten,lpOverlapped))
	{
		lLastError = GetLastError();
		if (lLastError != ERROR_IO_PENDING)
		{
			S->m_lLastError = lLastError;
			return S->m_lLastError;
		}

		/* We need to block if the client didn't specify an overlapped structure */
		if (lpOverlapped == &ovInternal)
		{
			switch (WaitForSingleObject(lpOverlapped->hEvent,dwTimeout))
			{
			case WAIT_OBJECT_0:
				/* The overlapped operation has completed */
				if (!GetOverlappedResult(S->m_hFile,lpOverlapped,pdwWritten,0))
				{
					S->m_lLastError = GetLastError();
					return S->m_lLastError;
				}
				break;

			case WAIT_TIMEOUT:
				/* Cancel the I/O operation */
				CancelIo(S->m_hFile);

				/* The operation timed out. Set the internal error code and quit */
				S->m_lLastError = ERROR_TIMEOUT;
				return S->m_lLastError;

			default:
				/* Set the internal error code */
				S->m_lLastError = GetLastError();
				return S->m_lLastError;
			}
		}
	}
	else
	{
		/* The operation completed immediatly. Just to be sure
		   we'll set the overlapped structure's event handle. */
		if (lpOverlapped)
			SetEvent(lpOverlapped->hEvent);
	}

#else

	if (!WriteFile(S->m_hFile,pData,iLen,pdwWritten,0))
	{
		S->m_lLastError = GetLastError();
		return S->m_lLastError;
	}

#endif

	return S->m_lLastError;
}

static long CSerialRead (pCSerial S, void* pData, unsigned int iLen, unsigned long* pdwRead, LPOVERLAPPED lpOverlapped, unsigned long dwTimeout)
{
    unsigned long dwRead;
#ifndef SERIAL_NO_OVERLAPPED
    OVERLAPPED ovInternal;
    long lLastError;
#endif
    
	S->m_lLastError = ERROR_SUCCESS;

	if (pdwRead == 0)
	{
		pdwRead = &dwRead;
	}

	*pdwRead = 0;

	if (S->m_hFile == 0)
	{
		S->m_lLastError = ERROR_INVALID_HANDLE;
		return S->m_lLastError;
	}

#ifndef SERIAL_NO_OVERLAPPED

	/* Check if an overlapped structure has been specified */
	if (!S->m_hevtOverlapped && (lpOverlapped || (dwTimeout != INFINITE)))
	{
		S->m_lLastError = ERROR_INVALID_FUNCTION;
		return S->m_lLastError;
	}

	/* Wait for the event to happen */
	if (lpOverlapped == 0)
	{
		memset(&ovInternal,0,sizeof(ovInternal));
		ovInternal.hEvent = S->m_hevtOverlapped;
		lpOverlapped = &ovInternal;
	}

	/* Make sure the overlapped structure isn't busy */
	
	if (!ReadFile(S->m_hFile,pData,iLen,pdwRead,lpOverlapped))
	{
		lLastError = GetLastError();

		/* Overlapped operation in progress is not an actual error */
		if (lLastError != ERROR_IO_PENDING)
		{
			S->m_lLastError = lLastError;
			return S->m_lLastError;
		}

		/* We need to block if the client didn't specify an overlapped structure */
		if (lpOverlapped == &ovInternal)
		{
			/* Wait for the overlapped operation to complete */
			switch (WaitForSingleObject(lpOverlapped->hEvent,dwTimeout))
			{
			case WAIT_OBJECT_0:
				/* The overlapped operation has completed */
				if (!GetOverlappedResult(S->m_hFile,lpOverlapped,pdwRead,0))
				{
					S->m_lLastError = GetLastError();
					return S->m_lLastError;
				}
				break;

			case WAIT_TIMEOUT:
				/* Cancel the I/O operation */
				CancelIo(S->m_hFile);

				/* The operation timed out. Set the internal error code and quit */
				S->m_lLastError = ERROR_TIMEOUT;
				return S->m_lLastError;

			default:
				/* Set the internal error code */
				S->m_lLastError = GetLastError();
				return S->m_lLastError;
			}
		}
	}
	else
	{
		/* The operation completed immediatly. Just to be sure
		   we'll set the overlapped structure's event handle. */
		if (lpOverlapped)
			SetEvent(lpOverlapped->hEvent);
	}

#else
	
	if (!ReadFile(S->m_hFile,pData,iLen,pdwRead,0))
	{
		S->m_lLastError = GetLastError();
		return S->m_lLastError;
	}

#endif

	return S->m_lLastError;
}

static long CSerialPurge (pCSerial S)
{
	S->m_lLastError = ERROR_SUCCESS;

	if (S->m_hFile == 0)
	{
		S->m_lLastError = ERROR_INVALID_HANDLE;
		return S->m_lLastError;
	}

	if (!PurgeComm(S->m_hFile, PURGE_TXCLEAR | PURGE_RXCLEAR))
	{
		S->m_lLastError = GetLastError();
	}
	
	return S->m_lLastError;
}

#else

static void CSerialCreate(pCSerial S)
{
	S->m_lLastError = 0;
	S->m_hFile      = -1;
    S->timeout      = EReadTimeoutNonblocking;
    S->timeout_sec  = 0;
    S->timeout_usec = 0;
}

static void CSerialDestroy(pCSerial S)
{
	/* If the device is already closed, then we don't need to do anything. */
	if (S->m_hFile != -1)
	{
		/* Close implicitly */
		CSerialClose(S);
	}
}

static long CSerialOpen (pCSerial S, const char* serialDevice, unsigned long dwInQueue, unsigned long dwOutQueue, int fOverlapped)
{
	/* Non-blocking reads is default */
    S->m_hFile = open(serialDevice, O_RDWR | O_NOCTTY | O_NDELAY);
    if (S->m_hFile == -1)
    {
        /* could not open the port */
        S->m_lLastError = errno;
    }
    return S->m_lLastError;
}

static long CSerialClose (pCSerial S)
{
	if (S->m_hFile != -1)
    {
        if (close(S->m_hFile) == -1)
        {
            S->m_lLastError = errno;
        }
        S->m_hFile = -1;
    }
    return S->m_lLastError;
}

static long CSerialSetup (pCSerial S, EBaudrate eBaudrate, EDataBits eDataBits, EParity eParity, EStopBits eStopBits)
{
    struct termios serial_attr;
    
    if (S->m_hFile == -1)
    {
        return S->m_lLastError;
    }
    
    S->m_lLastError = 0;
    
    memset(&serial_attr, 0, sizeof(serial_attr));
    tcgetattr(S->m_hFile, &serial_attr);
    
    cfsetispeed(&serial_attr, eBaudrate);
    cfsetospeed(&serial_attr, eBaudrate);

    serial_attr.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
    serial_attr.c_cflag |= (eDataBits | eParity | eStopBits);
    serial_attr.c_cflag |= CLOCAL;          /* Ignore modem control lines. */
    serial_attr.c_cflag |= CREAD;           /* Enable the receiver. */
    
    serial_attr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    serial_attr.c_iflag |= IGNBRK;          /* Ignore BREAK condition on input. */
    serial_attr.c_iflag |= IGNPAR;          /* Ignore framing errors and parity errors. */
    serial_attr.c_iflag &= ~INPCK;
    if (eParity != EParNone)
    {
        serial_attr.c_iflag |= INPCK;
    }
    
    serial_attr.c_oflag &= ~OPOST;
    
    if( tcsetattr(S->m_hFile, TCSANOW, &serial_attr) == -1 )
    {
        S->m_lLastError = errno;
    }
    
    return S->m_lLastError;
}

static long CSerialSetupHandshaking (pCSerial S, EHandshake eHandshake)
{
    struct termios serial_attr;
    
    if (S->m_hFile == -1)
    {
        return S->m_lLastError;
    }
    
    S->m_lLastError = 0;
    
    memset(&serial_attr, 0, sizeof(serial_attr));
    tcgetattr(S->m_hFile, &serial_attr);
    
    switch (eHandshake)
    {
        default:
        case EHandshakeOff:
            /* serial_attr.c_cflag &= ~CRTSCTS; */
            serial_attr.c_iflag &= ~(IXON | IXOFF | IXANY);
        
        case EHandshakeHardware:
            /* serial_attr.c_cflag |= CRTSCTS; */
            serial_attr.c_iflag &= ~(IXON | IXOFF | IXANY);
            
        case EHandshakeSoftware:
            /* serial_attr.c_cflag &= ~CRTSCTS; */
            serial_attr.c_iflag |= IXON | IXOFF | IXANY;
    }
    
    if( tcsetattr(S->m_hFile, TCSANOW, &serial_attr) == -1 )
    {
        S->m_lLastError = errno;
    }
    
    return S->m_lLastError;
}

/*
static long CSerialSetupReadTimeouts (pCSerial S, EReadTimeout eReadTimeout)
{
	struct termios serial_attr;
    
    if (S->m_hFile == -1)
    {
        return S->m_lLastError;
    }
    
    S->m_lLastError = 0;
    
    memset(&serial_attr, 0, sizeof(serial_attr));
    tcgetattr(S->m_hFile, &serial_attr);
    
	switch (eReadTimeout)
	{
        case EReadTimeoutBlocking:
            if (fcntl(S->m_hFile, F_SETFL, 0) == -1)
            {
                S->m_lLastError = errno;
            }
            serial_attr.c_cc[VTIME] = 0;
            serial_attr.c_cc[VMIN]  = 0;
            break;
        case EReadTimeoutNonblocking:
            if (fcntl(S->m_hFile, F_SETFL, O_NDELAY) == -1)
            {
                S->m_lLastError = errno;
            }
            break;
        default:
            if (fcntl(S->m_hFile, F_SETFL, 0) == -1)
            {
                S->m_lLastError = errno;
            }
            serial_attr.c_cc[VTIME] = (unsigned int)eReadTimeout / 100 + 1;
            serial_attr.c_cc[VMIN]  = 0;
            break;
	}
    
    if( !S->m_lLastError )
    if( tcsetattr(S->m_hFile, TCSANOW, &serial_attr) == -1 )
    {
        S->m_lLastError = errno;
    }
    
    return S->m_lLastError;
}
 */

static long CSerialSetupReadTimeouts (pCSerial S, EReadTimeout eReadTimeout)
{
	S->timeout = eReadTimeout;
    switch (eReadTimeout)
	{
        case EReadTimeoutBlocking:
            S->timeout_sec  = -1;
            S->timeout_usec = 0;
            break;
        case EReadTimeoutNonblocking:
            S->timeout_sec  = 0;
            S->timeout_usec = 0;
            break;
        default:
            S->timeout_sec  = (unsigned int)eReadTimeout / 1000;
            S->timeout_usec = ((unsigned int)eReadTimeout - 1000 * S->timeout_sec) * 1000;
            break;
	}
    
    return 0;
}

static long CSerialWrite (pCSerial S, const void* pData, unsigned int iLen, unsigned long* pdwWritten, LPOVERLAPPED lpOverlapped, unsigned long dwTimeout)
{
	struct timeval tv;
    struct timeval* ptv;
    fd_set fds;
    const char* buf = pData;
    int remaining = iLen;
    int write_count = 0;
    int retval;
    
    if (S->m_hFile == -1)
    {
        return S->m_lLastError;
    }
    
    S->m_lLastError = 0;
    
    ptv = (S->timeout == EReadTimeoutBlocking ? NULL : &tv);
    
    tv.tv_sec  = 1;
    tv.tv_usec = 0;
        
    while (remaining)
    {
        FD_ZERO(&fds);
        FD_SET(S->m_hFile, &fds);
        while( (retval = select(S->m_hFile+1, NULL, &fds, NULL, ptv)) == -1 && errno == EINTR) continue;
        if (retval == -1)
        {
            S->m_lLastError = errno;
            break;
        }
        else if (retval != 1)
        {
            /* timeout */
            S->m_lLastError = EBUSY;
            break;
        }
        
        while( (retval = write(S->m_hFile, buf, remaining)) == -1 && errno == EINTR) continue;
        if (retval == -1)
        {
            if (errno != EAGAIN)
            {
                S->m_lLastError = errno;
                break;
            }
        }
        else
        {
            write_count += retval;
            buf += retval;
            remaining -= retval;
        }
    }
    
    if (pdwWritten) *pdwWritten = write_count;
    return S->m_lLastError;
}

static long CSerialRead (pCSerial S, void* pData, unsigned int iLen, unsigned long* pdwRead, LPOVERLAPPED lpOverlapped, unsigned long dwTimeout)
{
    struct timeval tv;
    struct timeval* ptv;
    fd_set fds;
    char* buf = pData;
    int remaining = iLen;
    int read_count = 0;
    int retval;
    
    if (S->m_hFile == -1)
    {
        return S->m_lLastError;
    }
    
    S->m_lLastError = 0;
    
    ptv = (S->timeout == EReadTimeoutBlocking ? NULL : &tv);
    
    tv.tv_sec  = S->timeout_sec;
    tv.tv_usec = S->timeout_usec;
        
    while (remaining)
    {
        FD_ZERO(&fds);
        FD_SET(S->m_hFile, &fds);
        while( (retval = select(S->m_hFile+1, &fds, NULL, NULL, ptv)) == -1 && errno == EINTR) continue;
        if (retval == -1)
        {
            S->m_lLastError = errno;
            break;
        }
        else if (retval != 1)
        {
            /* timeout */
            break;
        }
        
        while( (retval = read(S->m_hFile, buf, remaining)) == -1 && errno == EINTR) continue;
        if (retval == -1)
        {
            if (errno != EAGAIN)
            {
                S->m_lLastError = errno;
                break;
            }
        }
        else
        {
            read_count += retval;
            buf += retval;
            remaining -= retval;
        }
    }
    
    if (pdwRead) *pdwRead = read_count;
    return S->m_lLastError;
}

static long CSerialPurge (pCSerial S)
{
	int retval;
    if (S->m_hFile == -1)
    {
        return S->m_lLastError;
    }
    
    while( (retval = tcflush(S->m_hFile, TCIOFLUSH)) == -1 && errno == EINTR) continue;
    if( retval == -1 )
    {
        S->m_lLastError = errno;
    }
    
    return S->m_lLastError;
}

#endif






static int SendPacket(pcommProtocol P, COMM_LINK_BEGIN* pheader, const void* pdata);

static int OpenConnection(pcommProtocol P, const char* Name, unsigned int baud)
{
    COMPUTER_COMMAND_USB_CTRL command;
    COMM_LINK_BEGIN header;
    char buf[64];
    int i;
    int err;
    
    if( (err=CSerialOpen(P->Ser, Name, 4096, 4096, 0)) != 0 )
	{		
        return err;
	}
    
    CSerialSetup( P->Ser, (EBaudrate) baud, EData8, EParNone, EStop1);
    CSerialSetupHandshaking( P->Ser, EHandshakeOff );
	CSerialSetupReadTimeouts( P->Ser, EReadTimeoutNonblocking );
    
    for( i=0; i<64; i++ )
    {
        buf[i]=0;
    }
    for( i=0; i<=(COMM_MAX_DATA_LENGTH/64)+1; i++ )
    {
        if( (err=CSerialWrite(P->Ser, buf, 64, 0, 0, INFINITE)) != 0 ) return err;
    }
    
    header.packet_start = 0x7E;
    header.length = sizeof(COMPUTER_COMMAND_USB_CTRL);
    header.api = COMM_API_COMPUTER_COMMANDS;
    
    command.opcode = COMPUTER_COMMAND_OPCODE_USB_CTRL;
    command.tx_length = P->_rx_len;
    command.transfer_ctrl = 0x00;

    if( (err=SendPacket(P, &header, &command)) != 0 ) return err;
#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    Sleep(300);
#else
    usleep(300000);
#endif
    CSerialPurge(P->Ser);
    
    if( P->_rx_len )
    {
        command.transfer_ctrl = 0x01;
        if( (err=SendPacket(P, &header, &command)) != 0 ) return err;
    }    

    return 0;
}

static int CloseConnection(pcommProtocol P)
{
    COMPUTER_COMMAND_USB_CTRL command;
    COMM_LINK_BEGIN header;
    char buf[64];
    int i;
    int err;
    
    for( i=0; i<64; i++ )
    {
        buf[i]=0;
    }
    for( i=0; i<=(COMM_MAX_DATA_LENGTH/64)+1; i++ )
    {
        if( (err=CSerialWrite(P->Ser, buf, 64, 0, 0, INFINITE)) != 0 ) return err;
    }
    
    header.packet_start = 0x7E;
    header.length = sizeof(COMPUTER_COMMAND_USB_CTRL);
    header.api = COMM_API_COMPUTER_COMMANDS;
    
    command.opcode = COMPUTER_COMMAND_OPCODE_USB_CTRL;
    command.tx_length = P->_rx_len;
    command.transfer_ctrl = 0x00;
    if( (err=SendPacket(P, &header, &command)) != 0 ) return err;
#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    Sleep(300);
#else
    usleep(300000);
#endif
    CSerialPurge(P->Ser);
    if( (err=CSerialClose(P->Ser)) != 0 ) return err;
    
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
	unsigned long read_count;
	unsigned int i;
	unsigned char ch;
	int temp;
	unsigned char buf[32];
    int ret_value;
    int err;
    int read_failed;
    int received_packets;
    int timeout_enabled;
    
    if( max_len < P->_rx_len ) return -2;
    
    received_packets = 0;
    ret_value = 0;
    err = 0;
    read_failed = 0;
    timeout_enabled = 0;

    for( ; ; )
    {
        switch( P->_state_rx )
		{
			case 0:
                /* Wait for the start of packet (0x7E) */
				if ( (err=CSerialRead(P->Ser, &ch, 1, &read_count, 0, INFINITE)) != 0 ) break;
                if ( read_failed = (read_count != 1) ) break;
                
                if( ch == 0x7E )
				{
					P->_crc = 0;
					P->_state_rx++;
				}
                else err = -3;
				break;

			case 1:
                if ( (err=CSerialRead(P->Ser, &P->_api, 1, &read_count, 0, INFINITE)) != 0 ) break;
                if ( read_failed = (read_count != 1) ) break;
                P->_state_rx++;
                break;

            case 2:
                if ( (err=CSerialRead(P->Ser, &P->_length, 1, &read_count, 0, INFINITE)) != 0 ) break;
                if ( read_failed = (read_count != 1) ) break;
                P->_state_rx++;
                break;

            case 3:
                if ( (err=CSerialRead(P->Ser, ((unsigned char *)&P->_length)+1, 1, &read_count, 0, INFINITE)) != 0 ) break;
                if ( read_failed = (read_count != 1) ) break;
                if( P->_length > COMM_MAX_DATA_LENGTH )
				{
					err = -4;
                    break;
				}
				else
				{
					P->_state_rx++;
				}
                P->_write_ok = 0;
                P->_already_read = 0;
				break;
			
			case 4:
                if( P->_api == COMM_API_HELICOPTER_DATA )
				{
                    if( P->_length == P->_rx_len )
					{
                        if ( (err=CSerialRead(P->Ser, P->_rx_buf+P->_already_read, P->_rx_len-P->_already_read, &read_count, 0, INFINITE)) != 0 ) break;
                        P->_already_read += read_count;
                        if ( read_failed = (P->_already_read != P->_rx_len) ) break;
						for( i=0; i<P->_rx_len; i++ )
						{
							P->_crc += P->_rx_buf[i];
						}
                        P->_write_ok = 1;
					}
                    else if ( P->_length == 0 )
                    {
                        for( i=0; i<P->_rx_len; i++ )
						{
							P->_rx_buf[i] = 0;
						}
                        P->_write_ok = 1;
                    }
                    else
                    {
                        err = -5;
                        break;
                    }
				}
                P->_already_read = 0;
                P->_state_rx++;
                break;

            case 5:
				if( P->_write_ok == 0 )
				{
					/* discard the data */
					do
					{
						temp = P->_length - P->_already_read;
						if( temp > 32 ) temp = 32;
                        if ( (err=CSerialRead(P->Ser, buf, temp, &read_count, 0, INFINITE)) != 0 ) break;
                        P->_already_read += read_count;
                        for( i=0; i<read_count; i++ )
						{
							P->_crc += buf[i];
						}
                        if ( read_failed = (read_count != temp) ) break;
					} while( P->_already_read < P->_length );
                    if( read_failed ) break;
				}
				P->_state_rx++;
				break;

			case 6:
				/* read the crc */
                if ( (err=CSerialRead(P->Ser, &ch, 1, &read_count, 0, INFINITE)) != 0 ) break;
                if ( read_failed = (read_count != 1) ) break;
				P->_crc += ch;
                P->_state_rx++;
                break;
                
            case 7:
                /* read the end of packet */
				if ( (err=CSerialRead(P->Ser, &ch, 1, &read_count, 0, INFINITE)) != 0 ) break;
                if ( read_failed = (read_count != 1) ) break;
                P->_state_rx = 0;

				if( (P->_crc == 0xFF) && (ch == 0xFF) )
				{
                    received_packets++;
                    for( i=0; i<P->_rx_len; i++ )
                    {
                        dest[i] = P->_rx_buf_old[i];
                        P->_rx_buf_old[i] = P->_rx_buf[i];
                    }
                    break;
				}
                else err = -6;
                break;

			default:
                P->_state_rx = 0;
				err = -7;
		}
        
        if( err != 0 )
        {
            return err;
        }
        
        if( timeout_enabled )
        {
            CSerialSetupReadTimeouts( P->Ser, EReadTimeoutNonblocking );
            timeout_enabled = 0;
            if( read_failed ) return -1;
        }
        
        if( read_failed )
        {
            if( received_packets || (P->_timeout == 0) )
            {
                if( (received_packets <= 1) || (P->_received_overrun_limit == 0) )
                for( i=0; i<P->_rx_len; i++ )
                {
                    dest[i] = P->_rx_buf_old[i];
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
            else
            {
                if( P->_timeout < 0 )
                {
                    CSerialSetupReadTimeouts( P->Ser, EReadTimeoutBlocking );
                }
                else
                {
                    CSerialSetupReadTimeouts( P->Ser, (EReadTimeout) P->_timeout );
                }
                timeout_enabled = 1;
            }
        }
    }
}

static void SetReadTimeout(pcommProtocol P, int timeout)
{
    P->_timeout = timeout;
}

static void SetReadPacketLength(pcommProtocol P, unsigned int rx_len )
{
    unsigned int i;
    P->_rx_len = rx_len;
    if( P->_rx_buf ) free(P->_rx_buf);
    if( P->_rx_buf_old ) free(P->_rx_buf_old);
    P->_rx_buf = (char*) malloc(rx_len);
    P->_rx_buf_old = (char*) malloc(rx_len);
    for( i=0; i<P->_rx_len; i++ )
    {
        P->_rx_buf_old[i] = 0;
    }
}

static int SendPacket(pcommProtocol P, COMM_LINK_BEGIN* pheader, const void* pdata)
{
    int i;
    unsigned char crc;
    COMM_LINK_END link_end;
    char buf[COMM_MAX_DATA_LENGTH];
    int buf_length = 0;
    int err;
    
    if( sizeof(COMM_LINK_BEGIN) + pheader->length + sizeof(COMM_LINK_END) > COMM_MAX_DATA_LENGTH )
    {
        return -20;
    }
    crc = 0;
    for( i=0; i<pheader->length; i++ )
    {
        crc += ((char *)pdata)[i];
    }
    link_end.crc = 0xFF - crc;
    link_end.packet_end = 0xFF;
    memcpy(buf, pheader, sizeof(COMM_LINK_BEGIN));
    buf_length += sizeof(COMM_LINK_BEGIN);
    memcpy(buf+buf_length, pdata, pheader->length);
    buf_length += pheader->length;
    memcpy(buf+buf_length, &link_end, sizeof(COMM_LINK_END));
    buf_length += sizeof(COMM_LINK_END);
    
    if( (err=CSerialWrite(P->Ser, buf, buf_length, 0, 0, INFINITE)) != 0 ) return err;
    return 0;
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
    P->Ser = (pCSerial) malloc(sizeof(CSerial));
    CSerialCreate(P->Ser);
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
    CSerialDestroy(P->Ser);
    free(P->Ser);
}






#define NPARAMS                 9
#define paramCOM_port           ssGetSFcnParam(S,0)
#define paramBaudRate           ssGetSFcnParam(S,1)
#define paramTimeout            ssGetSFcnParam(S,2)
#define paramTXEnable           ssGetSFcnParam(S,3)
#define paramRXEnable           ssGetSFcnParam(S,4)
#define paramReceiveSize        ssGetSFcnParam(S,5)
#define paramRXPacketsBuf       ssGetSFcnParam(S,6)
#define paramRXPacketsNumber    ssGetSFcnParam(S,7)
#define paramSampleTime         ssGetSFcnParam(S,8)


#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define intval(x) ((int)((x)+0.5))

static char msg[80];

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
    if ( !mxIsChar(paramCOM_port) || (mxGetNumberOfElements(paramCOM_port)) < 1 )
    {
        ssSetErrorStatus(S, "The Communication Interface parameter must be a nonempty string.");
        return;
    }
    
    if ( (!IS_PARAM_DOUBLE(paramBaudRate)) || (mxGetNumberOfElements(paramBaudRate) != 1) )
    {
        ssSetErrorStatus(S, "Wrong baud rate.");
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
    ssSetNumDiscStates(S, 1);

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
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_PLACE_ASAP));
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
    char port_name[101];
    int port_len;
    int_T y_len;
    int err;

    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }

    /* Store new C object in the pointers vector */
    protocol = (pcommProtocol) malloc(sizeof(commProtocol));
    ssGetPWork(S)[0] = protocol;
    commProtocolCreate(protocol);

    port_len = mxGetM(paramCOM_port) * mxGetN(paramCOM_port) + 1;
    if( port_len  > sizeof(port_name) ) port_len = sizeof(port_name);
    mxGetString(paramCOM_port, port_name, port_len);

    if( ssGetNumOutputPorts(S) > 0 )
    {
        SetReadPacketLength( protocol, ssGetOutputPortWidth(S, 0) );
        SetReadPacketOverrunLimit(protocol, intval(mxGetPr(paramRXPacketsBuf)[0]));
        SetReadTimeout( protocol, intval(mxGetPr(paramTimeout)[0] * 1000) );
    }

    if( ssGetNumOutputPorts(S) > 0 )
    {
        y_len = ssGetOutputPortWidth(S, 0);
        ssGetPWork(S)[1] = (uint8_T *) malloc(y_len);
    }
    
    if( ssGetIWork(S)[0] == 0 )
    {
        ssGetIWork(S)[0] = 1;
        if( (err=OpenConnection(protocol, port_name, intval(mxGetPr(paramBaudRate)[0]))) != 0 )
        {
            sprintf(msg, "Could not open the COM port. Error=%d, COM=%s", err, port_name);
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
    /* Retrieve C++ object from the pointers vector */
    pcommProtocol protocol = (pcommProtocol) ssGetPWork(S)[0];
    uint8_T *y_buf         = (uint8_T *)ssGetPWork(S)[1];
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
    
    /* Read the serial port data */
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
            sprintf(msg,"Could not read from the communication port. Error=%d", err);
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
        ssSetErrorStatus(S,"The Serial class has not been initialized.");
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
            sprintf(msg, "Could not write to the communication port. Error=%d", err);
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
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
