#include <stdio.h>
#include <memory.h>
#include <unistd.h>
#include <stdarg.h>
#include "RFT_COM.h"

//#define RFT_DEBUG_PRINT

void DEBUG_PRINT(const char* _formatString, ...)
{
#ifdef RFT_DEBUG_PRINT
	char svBuffer[1024];
	va_list args;
	va_start(args, _formatString);
	vsprintf( svBuffer, _formatString, args );
	va_end(args);
	printf("%s", svBuffer );
#endif	
}


CRT_RFT_UART::CRT_RFT_UART( void )
{
	initialize_variables();		
}

void CRT_RFT_UART::initialize_variables(void)
{
	m_bConnected = false;		        // Open/Closed indicating flag

	m_nCurrMode = CMD_NONE;
	m_bIsRcvd_Response_Pkt = false;

	m_nRcvdBufferIdx = 0;
	
	memset(m_rcvdBuff, 0, sizeof(unsigned char)*RFT_UART_RX_BUFF_SIZE);
	memset(m_txBuff, 0, sizeof(unsigned char)*RFT_UART_TX_BUFF_SIZE);
}


CRT_RFT_UART::~CRT_RFT_UART( void )
{
	closePort( );
}


bool CRT_RFT_UART::openPort( char *devName, int rate, int byteSize )
{
	if( m_bConnected )
		return true;
	
	if( !createComPort( devName ) )
	{
		DEBUG_PRINT("com port creation error\n");
		return false;
	}

	m_bConnected = setupConnection( rate, byteSize );
	if (!m_bConnected )
	{
		return m_bConnected;
	}

	DEBUG_PRINT("com port creation success...\n");
	return m_bConnected;
}

bool CRT_RFT_UART::createComPort( char *devName )
{
	char szPort[100];

	sprintf( szPort, "%s", devName );

	DEBUG_PRINT( szPort );
	// NON-Blocking Mode File Open..
	fd = open( szPort, O_RDWR | O_NOCTTY | O_NDELAY);

	if( fd == -1 )
		return false;
	else
		fcntl( fd, F_SETFL, O_NDELAY );

	return true;
}

bool CRT_RFT_UART::setupConnection(int dwBaudrate, int bByteSize )
{
	struct termios oldtio, newtio;

	tcgetattr(fd,&oldtio); /* save current serial port settings */
	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	newtio.c_cflag = dwBaudrate | bByteSize | CLOCAL | CREAD;
	tcsetattr(fd,TCSANOW,&newtio);

	return ( true );
}

bool CRT_RFT_UART::closePort( void )
{
  if( !m_bConnected )
		return true;

	if( m_bConnected )
	{
		// set connected flag to false
		m_bConnected = false;
		// disable event notification and wait for thread to halt

		close( fd );
	}
	
	return (true);
}

bool CRT_RFT_UART::isConnected( void )
{
	return (m_bConnected);
}


bool CRT_RFT_UART::readWorker( void )        // read worker.
{
	//DEBUG_PRINT("RFT UART IF READ THREAD START...\n");
	char buff[BUFF_SIZE];
	int nReceived = 0;
	
	if( m_bConnected )
	{
		nReceived = 0;
		nReceived = read( fd, buff, BUFF_SIZE );

		if( nReceived <= 0 )
		{
			//usleep( 1000 ); // just sleep.
			//continue;
			//DEBUG_PRINT("There is no received\n");
			return false;
		}
		
		// data copy
		for( int i = 0; i < nReceived; i++)
		{
			m_rcvdBuff[m_nRcvdBufferIdx + i] = buff[i];
		}
		m_nRcvdBufferIdx += nReceived;

		if (m_nCurrMode == CMD_NONE)
		{
			m_nRcvdBufferIdx = 0;
			DEBUG_PRINT("CMD NONE: Serial data received\n");
			//continue;
			return false;
		}

		// wait packet.... 
		if (m_nRcvdBufferIdx < UART_RESPONSE_PACKET_SIZE)
		{
			DEBUG_PRINT("RECEIVED: %d\n", m_nRcvdBufferIdx);
			//continue;
			return false;
		}

		// find SOP
		int found_idx = -1;
		for (int i = 0; i < m_nRcvdBufferIdx; i++)
		{
			if (m_rcvdBuff[i] == SOP)
			{
				found_idx = i;
				break;
			}
		}

		if (found_idx == -1)
		{
			m_nRcvdBufferIdx = 0;
			DEBUG_PRINT("SOP NOT FOUNDED\n");
			//continue;
			return false;
		}

		// if the index of SOP is not first(0), shift received data....
		if (found_idx != 0) 
		{
			for (int i = 0; i < (m_nRcvdBufferIdx - found_idx); i++)
			{
				m_rcvdBuff[i] = m_rcvdBuff[i + found_idx];
			}
			DEBUG_PRINT("SHIFT\n");
			m_nRcvdBufferIdx = m_nRcvdBufferIdx - found_idx;
		}

		// packet handling....
		if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE )
		{
			return RFT_Data_Handler();		
		}
	}
	else
	{
		return false;
	}
	
	//DEBUG_PRINT("RFT UART IF READ THREAD FINISHED...\n");

}

bool CRT_RFT_UART::RFT_Data_Handler( void )
{
	bool result = false;
	
	switch (m_nCurrMode)
	{
	case CMD_GET_PRODUCT_NAME:
		result = rcvd_ProduectName();
		DEBUG_PRINT("%s\n", m_RFT_IF_PACKET.m_rcvd_product_name );
		break;

	case CMD_GET_SERIAL_NUMBER:
		result = rcvd_SerialNumber();
		DEBUG_PRINT("%s\n", m_RFT_IF_PACKET.m_rcvd_serial_number );
		break;

	case CMD_GET_FIRMWARE_VER:
		result = rcvd_Firmwareversion();
		DEBUG_PRINT("%s\n", m_RFT_IF_PACKET.m_rcvd_firmware_version );
		break;

	case CMD_SET_COMM_BAUDRATE:
		result = rcvd_Response_Set_Comm_Speed();
		break;

	case CMD_GET_COMM_BAUDRATE:
		result = rcvd_CommSpeed();
		break;

	case CMD_SET_FT_FILTER:
		result = rcvd_Response_Set_FT_Filter_Type();
		break;

	case CMD_GET_FT_FILTER:
		result = rcvd_FT_Filter_Type();
		break;

	case CMD_FT_ONCE:
		result = rcvd_FT();
		DEBUG_PRINT("%0.3f, %.03f, %.03f, %0.3f, %.03f, %.03f\r",
			m_RFT_IF_PACKET.m_rcvdForce[0], m_RFT_IF_PACKET.m_rcvdForce[1], m_RFT_IF_PACKET.m_rcvdForce[2],
			m_RFT_IF_PACKET.m_rcvdForce[3], m_RFT_IF_PACKET.m_rcvdForce[4], m_RFT_IF_PACKET.m_rcvdForce[5] );
		break;

	case CMD_FT_CONT:
		result = rcvd_FT();
		DEBUG_PRINT("%0.3f, %.03f, %.03f, %0.3f, %.03f, %.03f\r",
			m_RFT_IF_PACKET.m_rcvdForce[0], m_RFT_IF_PACKET.m_rcvdForce[1], m_RFT_IF_PACKET.m_rcvdForce[2],
			m_RFT_IF_PACKET.m_rcvdForce[3], m_RFT_IF_PACKET.m_rcvdForce[4], m_RFT_IF_PACKET.m_rcvdForce[5] );
		
		break;

	case CMD_FT_CONT_STOP:
		break;

	case CMD_SET_CONT_OUT_FRQ:
		result = rcvd_Response_Set_FT_Cont_Interval();
		break;

	case CMD_GET_CONT_OUT_FRQ:
		result = rcvd_FT_Cont_Interval();
		break;

	case CMD_GET_OVERLOAD_COUNT:
		result = rcvd_FT_Overload_Count();
		break;

	default: 
		break;
	}
	
	return result;
}


bool CRT_RFT_UART::writeBuffer( unsigned char* lpBuffer, int bytesToWrite, int *bytesWritten )
{
	int     currBytesWritten = 0;
	int     totalWritten = 0;
	totalWritten = 0;
	bool result = true;

	// WRITE
	while( (bytesToWrite - totalWritten) > 0 )
	{
		currBytesWritten = write( fd, (lpBuffer + totalWritten), (bytesToWrite - totalWritten) );
		if( currBytesWritten )
		{
			totalWritten += currBytesWritten;
		}
		else
		{
			usleep( 100 );
		}
	}
	*bytesWritten = totalWritten;

	return result;
}


bool CRT_RFT_UART::rqst_ProductName( void )
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_product_name(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_SerialNumber(void)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_serial_name(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}


bool CRT_RFT_UART::rqst_Firmwareverion(void)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_firmware_version(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_CommSpeed(void)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_comm_baudrate(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Filter_Type(void)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_filter_type(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT(void)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_force_once(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Continuous(void)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_force(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Stop(void)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_stop_force_out(m_txBuff);

	m_bIsRcvd_Response_Pkt = true; // FT output stop command don't have response packet.
	m_nCurrMode = CMD_NONE;

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Cont_Interval(void)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_output_frq(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_OverloadCnt(void)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_overload_count(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::set_Comm_Speed(int comm_speed)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_comm_baudrate(m_txBuff, comm_speed);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::set_FT_Filter_Type(int filter_type, int sub_type)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_filter_type(m_txBuff, filter_type, sub_type);


	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}


bool CRT_RFT_UART::set_FT_Cont_Interval(int interval)
{
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_output_frq(m_txBuff, interval);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	DEBUG_PRINT("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::set_FT_Bias(int is_on)
{
	/* Bias setting command don't have response packet
	if (m_nRcvdBufferIdx)
	{
		DEBUG_PRINT("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0; 
	}
	*/

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_bias(m_txBuff, is_on);

	/* Bias setting command don't have response packet
	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];
	*/

	int written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);

	DEBUG_PRINT("written: %d\n", written);
	return result;
}


bool CRT_RFT_UART::rcvd_ProduectName(void)
{
	bool result = false;
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_PRODUCT_NAME))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
			result = true;
		}
	}
	
	return result;
}

bool CRT_RFT_UART::rcvd_SerialNumber(void)
{
	bool result = false;
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_SERIAL_NUMBER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
			result = true;
		}
	}
	return result;
}

bool CRT_RFT_UART::rcvd_Firmwareversion(void)
{
	bool result = false;
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_FIRMWARE_VER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
			result = true;
		}
	}
	return result;
}

bool CRT_RFT_UART::rcvd_CommSpeed(void)
{
	bool result = false;
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_COMM_BAUDRATE))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
			result = true;
		}
	}
	return result;
}

bool CRT_RFT_UART::rcvd_FT_Filter_Type(void)
{
	bool result = false;
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_FT_FILTER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
			result = true;
		}
	}
	return result;
}


bool CRT_RFT_UART::rcvd_FT(void)
{
	bool result = false;
	
	int pkt_size = UART_RESPONSE_PACKET_SIZE;
	int num_of_rcvd_pkt = m_nRcvdBufferIdx / pkt_size;
	int num_of_next_pkt_data = m_nRcvdBufferIdx % pkt_size;
	int transfer_data_size;
	int source_start_idx;

	// if the number of received packet is more than 1, 
	// move the last packet data to first location of buffer to interprete the last received data.
	if (num_of_rcvd_pkt > 1) 
	{
		transfer_data_size = pkt_size + num_of_next_pkt_data;
		source_start_idx = (num_of_rcvd_pkt - 1) * pkt_size;

		for (int i = 0; i < transfer_data_size; i++)
			m_rcvdBuff[i] = m_rcvdBuff[source_start_idx + i];

		DEBUG_PRINT("DATA TRANSFER - BEFORE\n");
	}

	if (m_nRcvdBufferIdx >= pkt_size)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, m_nCurrMode))
		{
			m_bIsRcvd_Response_Pkt = true;

			// if received data size bigger than packet size,
			// shift that to first location of buffer.
			if (num_of_next_pkt_data)
			{
				transfer_data_size = num_of_next_pkt_data;
				source_start_idx = pkt_size;
				for (int i = 0; i < transfer_data_size; i++)
					m_rcvdBuff[i] = m_rcvdBuff[source_start_idx + i];

				DEBUG_PRINT("DATA TRANSFER - AFTER\n");

				m_nRcvdBufferIdx = num_of_next_pkt_data;
			}
			else
			{
				m_nRcvdBufferIdx = 0; 
			}
			
			result = true;
			
		}
	}
	return result;
}



bool CRT_RFT_UART::rcvd_FT_Cont_Interval(void)
{
	bool result = false;
	
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_CONT_OUT_FRQ))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
			result = true;
		}
	}
	return result;
}

bool CRT_RFT_UART::rcvd_Response_Set_Comm_Speed(void)
{
	bool result = false;
	
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_SET_COMM_BAUDRATE))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
			result = true;
		}
	}
	return result;
}

bool CRT_RFT_UART::rcvd_Response_Set_FT_Filter_Type(void)
{
	bool result = false;
	
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_SET_FT_FILTER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
			result = true;
		}
	}
	return result;
}

bool CRT_RFT_UART::rcvd_FT_Overload_Count(void)
{
	bool result = false;
	
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_OVERLOAD_COUNT))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
			result = true;
		}
	}
	return result;
}

bool CRT_RFT_UART::rcvd_Response_Set_FT_Cont_Interval(void)
{
	bool result = false;
	
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_SET_CONT_OUT_FRQ)) // 
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
			result = true;
		}
	}
	return result;
}

//////////////////////////////////////////////////////////////////////////////
// END OF FILE

