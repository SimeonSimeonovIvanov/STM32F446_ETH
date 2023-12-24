/******************************************
 * Title		: Microchip ENCX24J600 Ethernet Interface Driver
 * Author		: Jiri Melnikov
 * Created		: 28.12.2009
 * Version		: 0.1
 * Target MCU	: Atmel AVR series
 *
 * Description	: This driver provides initialization and transmit/receive
 *				functions for the Microchip ENCX24J600 100Mb Ethernet
 *				Controller and PHY. Only supported interface is SPI, no
 *				PSP interface available by now. No security functions are
 *				are supported by now.
 *
 *				This driver is inspired by ENC28J60 driver from Pascal
 *				Stang (2005).
 *
 *				Many lines of code are taken from Microchip's TCP/IP stack.
 * 
 * ****************************************/

#include "cmsis_os.h"
#include "semphr.h"

#include "enc424j600.h"

// Binary constant identifiers for ReadMemoryWindow() and WriteMemoryWindow()
// functions
#define UDA_WINDOW				(0x1)
#define GP_WINDOW				(0x2)
#define RX_WINDOW				(0x4)

extern SPI_HandleTypeDef hspi2;
extern osSemaphoreId_t myBinarySemSpiHandle;

// Internal MAC level variables and flags.
static uint8_t currentBank;
static volatile uint16_t nextPacketPointer;

//static uint16_t enc424j600ReadReg(uint16_t address);
static void enc424j600WriteReg(uint16_t address, uint16_t data);
static void enc424j600ExecuteOp0(uint8_t op);
static void enc424j600BFSReg(uint16_t address, uint16_t bitMask);
//void enc424j600BFCReg(uint16_t address, uint16_t bitMask);
static void enc424j600WriteN(uint8_t op, uint8_t* data, uint16_t dataLen);
static void enc424j600ReadN(uint8_t op, uint8_t* data, uint16_t dataLen);
static uint8_t enc424j600ExecuteOp8(uint8_t op, uint8_t data);
static uint16_t enc424j600ExecuteOp16( uint8_t op, uint16_t data );
static uint32_t enc424j600ExecuteOp32( uint8_t op, uint32_t data );

static void enc424j600MACFlush(void);
static void enc424j600ReadMemoryWindow(uint8_t window, uint8_t *data, uint16_t length);
static void enc424j600WriteMemoryWindow(uint8_t window, uint8_t *data, uint16_t length);

static uint16_t enc424j600ReadPHYReg(uint8_t address);
static void enc424j600WritePHYReg(uint8_t address, uint16_t Data);

/********************************************************************
 * INITIALIZATION
 * ******************************************************************/
void enc424j600Init(uint8_t *mac_addr)
{
	if( xSemaphoreTake( myBinarySemSpiHandle, (TickType_t)portMAX_DELAY ) == pdTRUE )
	{
	//Set default bank
	currentBank = 0;
	unselect_net_chip();

	// Perform a reliable reset
	enc424j600SendSystemReset();

	// Initialize RX tracking variables and other control state flags
	nextPacketPointer = RXSTART;

	// Set up TX/RX/UDA buffer addresses
	enc424j600WriteReg(ETXST, TXSTART);
	enc424j600WriteReg(ERXST, RXSTART);
	enc424j600WriteReg(ERXTAIL, RAMSIZE - 2);
	enc424j600WriteReg(EUDAST, RAMSIZE);
	enc424j600WriteReg(EUDAND, RAMSIZE + 1);

	// Get MAC adress
	uint16_t regValue;
	regValue = enc424j600ReadReg(MAADR1);
	mac_addr[0] = ((uint8_t*) & regValue)[0];
	mac_addr[1] = ((uint8_t*) & regValue)[1];
	regValue = enc424j600ReadReg(MAADR2);
	mac_addr[2] = ((uint8_t*) & regValue)[0];
	mac_addr[3] = ((uint8_t*) & regValue)[1];
	regValue = enc424j600ReadReg(MAADR3);
	mac_addr[4] = ((uint8_t*) & regValue)[0];
	mac_addr[5] = ((uint8_t*) & regValue)[1];

	// If promiscuous mode is set, than allow accept all packets
#ifdef PROMISCUOUS_MODE
	enc424j600WriteReg(ERXFCON,(ERXFCON_CRCEN | ERXFCON_MPEN | ERXFCON_RUNTEN | ERXFCON_UCEN | ERXFCON_NOTMEEN | ERXFCON_MCEN | ERXFCON_BCEN));
#endif

	// Set PHY Auto-negotiation to support 10BaseT Half duplex,
	// 10BaseT Full duplex, 100BaseTX Half Duplex, 100BaseTX Full Duplex,
	// and symmetric PAUSE capability
	enc424j600WritePHYReg(PHANA, PHANA_ADPAUS0 | PHANA_AD10FD | PHANA_AD10 | PHANA_AD100FD | PHANA_AD100 | PHANA_ADIEEE0);

	// Enable RX packet reception
	enc424j600BFSReg(ECON1, ECON1_RXEN);
		xSemaphoreGive( myBinarySemSpiHandle );
	}
}

/********************************************************************
 * UTILS
 * ******************************************************************/
void enc424j600SendSystemReset(void)
{
	//if( xSemaphoreTake( myBinarySemSpiHandle, (TickType_t)portMAX_DELAY ) == pdTRUE )
	{
	volatile uint32_t delay;
	// Perform a reset via the SPI/PSP interface
	do {
		// Set and clear a few bits that clears themselves upon reset.
		// If EUDAST cannot be written to and your code gets stuck in this
		// loop, you have a hardware problem of some sort (SPI or PMP not
		// initialized correctly, I/O pins aren't connected or are
		// shorted to something, power isn't available, etc.)
		//sbi(PORTE, PE7);
		do {
			enc424j600WriteReg(EUDAST, 0x1234);
		} while (enc424j600ReadReg(EUDAST) != 0x1234);
		// Issue a reset and wait for it to complete
		enc424j600BFSReg(ECON2, ECON2_ETHRST);
		currentBank = 0;
		while( (enc424j600ReadReg(ESTAT) & (ESTAT_CLKRDY | ESTAT_RSTDONE | ESTAT_PHYRDY)) != (ESTAT_CLKRDY | ESTAT_RSTDONE | ESTAT_PHYRDY) );

		//_delay_us(300);
		delay = ( 0.0003f * 180000000.0f );
		delay *= 50;
		while( delay-- );

		// Check to see if the reset operation was successful by
		// checking if EUDAST went back to its reset default.  This test
		// should always pass, but certain special conditions might make
		// this test fail, such as a PSP pin shorted to logic high.
	} while( enc424j600ReadReg(EUDAST) != 0x0000u );

	// Really ensure reset is done and give some time for power to be stable
	//_delay_ms(1);
	delay = ( 0.001f * 180000000.0f );
	delay *= 50;
	while(delay--);
		//xSemaphoreGive( myBinarySemSpiHandle );
	}
}

uint16_t enc424j600EventHandler(void)
{
	uint16_t eirVal;
	if( xSemaphoreTake( myBinarySemSpiHandle, (TickType_t)portMAX_DELAY ) == pdTRUE )
	{
		eirVal = enc424j600ReadReg( EIRL );
		if( eirVal )
		{
			if( ( eirVal & EIR_RXABTIF ) || ( eirVal & EIR_PCFULIF ) ) { // buffer overflow
				//printf_P( PSTR("enc424j600EventHandler(): enc424j600ResetReceiver()\n") );
				enc424j600ResetReceiver();
			}
			enc424j600BFCReg( EIRL, eirVal );
		}
		xSemaphoreGive( myBinarySemSpiHandle );
	}
	return eirVal;
}

void enc424j600ResetReceiver(void)
{
	if( xSemaphoreTake( myBinarySemSpiHandle, (TickType_t)portMAX_DELAY ) == pdTRUE )
	{
		enc424j600BFSReg( ECON2, ECON2_RXRST );
		enc424j600BFCReg( ECON2, ECON2_RXRST );
		enc424j600BFCReg( ECON1, ECON1_RXEN );

		nextPacketPointer = RXSTART;

		enc424j600WriteReg( ERXST, RXSTART );
		enc424j600WriteReg(ERXTAIL, RAMSIZE - 2);

		enc424j600BFSReg( ECON1, ECON1_RXEN );

		xSemaphoreGive( myBinarySemSpiHandle );
	}
}

/**
 * Is link connected?
 * @return <char>
 */
char enc424j600MACIsLinked(void)
{
	uint8_t ret = 0;
	//if( xSemaphoreTake( myBinarySemSpiHandle, (TickType_t)portMAX_DELAY ) == pdTRUE )
	{
		ret = ( 0 != ( ESTAT_PHYLNK & enc424j600ReadReg( ESTAT ) ) );
	//	xSemaphoreGive( myBinarySemSpiHandle );
	}
	return ret;
}

/**
 * Is transmission active?
 * @return <char>
 */
char enc424j600MACIsTxReady(void)
{
	return !( ECON1_TXRTS & enc424j600ReadReg(ECON1) );
}
///////////////////////////////////////////////////////////////////////////////
char enc424j600PacketSend( uint8_t* packet, uint16_t len )
{
	if( ECON1_TXRTS & enc424j600ReadReg( ECON1L ) )
	{
		return 0;
	}

	enc424j600WriteReg( EGPWRPT, TXSTART );
	enc424j600WriteMemoryWindow( GP_WINDOW, packet, len );
	enc424j600WriteReg( ETXLEN, len );
	enc424j600MACFlush();

	return 0;
}

char enc424j600PacketBegin(void)
{
	if( ECON1_TXRTS & enc424j600ReadReg( ECON1L ) )
	{
		return 1;
	}

	enc424j600WriteReg( EGPWRPT, TXSTART );

	return 0;
}

char enc424j600PacketSendPart( uint8_t* packet, uint16_t len )
{
	enc424j600WriteMemoryWindow( GP_WINDOW, packet, len );
	return 0;
}

char enc424j600PacketEnd( uint16_t len )
{
	enc424j600WriteReg( ETXLEN, len );
	enc424j600MACFlush();
	return 0;
}
///////////////////////////////////////////////////////////////////////////////
uint16_t enc424j600PacketReceive( uint8_t* packet, uint16_t len )
{
	RXSTATUS statusVector;
	uint16_t newRXTail, eirVal;

	/*estatVal = enc424j600ReadReg(ESTAT);
	if( ESTAT_RXBUSY & estatVal )
	{
		return 0;
	}*/

	eirVal = enc424j600ReadReg(EIR);
	if( !(eirVal & EIR_PKTIF) )
	{
		return 0;
	}

	enc424j600WriteReg( ERXRDPT, nextPacketPointer );
	enc424j600ReadMemoryWindow( RX_WINDOW, (uint8_t*) & nextPacketPointer, sizeof (nextPacketPointer) );
	enc424j600ReadMemoryWindow( RX_WINDOW, (uint8_t*) & statusVector, sizeof (statusVector) );

	len = ( statusVector.bits.ByteCount <= len + 4 ) ? statusVector.bits.ByteCount - 4 : 0;
	enc424j600ReadMemoryWindow( RX_WINDOW, packet, len );

	newRXTail = nextPacketPointer - 2;

	if( RXSTART == nextPacketPointer )
	{
		newRXTail = RAMSIZE - 2;
	}

	enc424j600BFSReg( ECON1, ECON1_PKTDEC );
	enc424j600WriteReg( ERXTAIL, newRXTail );

	return len;
}

uint16_t enc424j600PacketReceiveBegin(void)
{
	RXSTATUS statusVector;
	uint16_t eirVal;

	eirVal = enc424j600ReadReg(EIR);
	if( !(eirVal & EIR_PKTIF) )
	{
		return 0;
	}

	enc424j600WriteReg( ERXRDPT, nextPacketPointer );
	enc424j600ReadMemoryWindow( RX_WINDOW, (uint8_t*) & nextPacketPointer, sizeof (nextPacketPointer) );
	enc424j600ReadMemoryWindow( RX_WINDOW, (uint8_t*) & statusVector, sizeof (statusVector) );

	return statusVector.bits.ByteCount > 4 ? statusVector.bits.ByteCount - 4 : 0;
}

uint16_t enc424j600PacketReceivePart( uint8_t* packet, uint16_t len )
{
	enc424j600WriteReg( ERXRDPT, nextPacketPointer );
	enc424j600ReadMemoryWindow( RX_WINDOW, (uint8_t*) & nextPacketPointer, sizeof (nextPacketPointer) );

	enc424j600ReadMemoryWindow( RX_WINDOW, packet, len );
	return 0;
}

uint16_t enc424j600PacketReceiveEnd( uint16_t len )
{
	uint16_t newRXTail;

	newRXTail = nextPacketPointer - 2;

	if( RXSTART == nextPacketPointer )
	{
		newRXTail = RAMSIZE - 2;
	}

	enc424j600BFSReg( ECON1, ECON1_PKTDEC );
	enc424j600WriteReg( ERXTAIL, newRXTail );

	return len;
}
///////////////////////////////////////////////////////////////////////////////
static void enc424j600MACFlush(void)
{
	uint16_t w;

	// Check to see if the duplex status has changed.  This can
	// change if the user unplugs the cable and plugs it into a
	// different node.  Auto-negotiation will automatically set
	// the duplex in the PHY, but we must also update the MAC
	// inter-packet gap timing and duplex state to match.
	if( EIR_LINKIF & enc424j600ReadReg(EIR) )
	{
		enc424j600BFCReg(EIR, EIR_LINKIF);

		// Update MAC duplex settings to match PHY duplex setting
		w = enc424j600ReadReg( MACON2 );
		if( ESTAT_PHYDPX & enc424j600ReadReg(ESTAT) )
		{
			// Switching to full duplex
			enc424j600WriteReg(MABBIPG, 0x15);
			w |= MACON2_FULDPX;
		} else {
			// Switching to half duplex
			enc424j600WriteReg(MABBIPG, 0x12);
			w &= ~MACON2_FULDPX;
		}
		enc424j600WriteReg(MACON2, w);
	}


	// Start the transmission, but only if we are linked.  Supressing
	// transmissing when unlinked is necessary to avoid stalling the TX engine
	// if we are in PHY energy detect power down mode and no link is present.
	// A stalled TX engine won't do any harm in itself, but will cause the
	// MACIsTXReady() function to continuously return 0, which will
	// ultimately stall the Microchip TCP/IP stack since there is blocking code
	// elsewhere in other files that expect the TX engine to always self-free
	// itself very quickly.
	if( ESTAT_PHYLNK & enc424j600ReadReg(ESTAT) )
	{
		enc424j600BFSReg(ECON1, ECON1_TXRTS);
	}
}

/********************************************************************
 * READERS AND WRITERS
 * ******************************************************************/

static void enc424j600WriteMemoryWindow(uint8_t window, uint8_t *data, uint16_t length)
{
	uint8_t op = RBMUDA;

	if (window & GP_WINDOW)
		op = WBMGP;
	if (window & RX_WINDOW)
		op = WBMRX;

	enc424j600WriteN(op, data, length);
}

static void enc424j600ReadMemoryWindow(uint8_t window, uint8_t *data, uint16_t length)
{
	if (length == 0u)
		return;

	uint8_t op = RBMUDA;

	if (window & GP_WINDOW)
		op = RBMGP;
	if (window & RX_WINDOW)
		op = RBMRX;

	enc424j600ReadN(op, data, length);
}

/**
 * Reads from address
 * @variable <uint16_t> address - register address
 * @return <uint16_t> data - data in register
 */
//static
uint16_t enc424j600ReadReg(uint16_t address)
{
	uint16_t returnValue = 0;
	uint8_t bank;
	// See if we need to change register banks
	bank = ((uint8_t) address) & 0xE0;
	if (bank <= (0x3u << 5))
	{
		//if (bank != currentBank)
		{
			if (bank == (0x0u << 5))
				enc424j600ExecuteOp0(B0SEL);
			else if (bank == (0x1u << 5))
				enc424j600ExecuteOp0(B1SEL);
			else if (bank == (0x2u << 5))
				enc424j600ExecuteOp0(B2SEL);
			else if (bank == (0x3u << 5))
				enc424j600ExecuteOp0(B3SEL);

			currentBank = bank;
		}
		returnValue = enc424j600ExecuteOp16(RCR | (address & 0x1F), 0x0000);
	} else {
		uint32_t returnValue32 = enc424j600ExecuteOp32(RCRU, (uint32_t) address);
		((uint8_t*) & returnValue)[0] = ((uint8_t*) & returnValue32)[1];
		((uint8_t*) & returnValue)[1] = ((uint8_t*) & returnValue32)[2];
	}

	return returnValue;
}

/**
 * Writes to register
 * @variable <uint16_t> address - register address
 * @variable <uint16_t> data - data to register
 */
static void enc424j600WriteReg(uint16_t address, uint16_t data)
{
	uint8_t bank;
	// See if we need to change register banks
	bank = ((uint8_t) address) & 0xE0;
	if (bank <= (0x3u << 5))
	{
		//if (bank != currentBank)
		{
			if (bank == (0x0u << 5))
				enc424j600ExecuteOp0(B0SEL);
			else if (bank == (0x1u << 5))
				enc424j600ExecuteOp0(B1SEL);
			else if (bank == (0x2u << 5))
				enc424j600ExecuteOp0(B2SEL);
			else if (bank == (0x3u << 5))
				enc424j600ExecuteOp0(B3SEL);

			currentBank = bank;
		}
		enc424j600ExecuteOp16(WCR | (address & 0x1F), data);
	} else {
		uint32_t data32 = 0;
		((uint8_t*) & data32)[0] = (uint8_t) address;
		((uint8_t*) & data32)[1] = ((uint8_t*) & data)[0];
		((uint8_t*) & data32)[2] = ((uint8_t*) & data)[1];
		enc424j600ExecuteOp32(WCRU, data32);
	}
}

static uint16_t enc424j600ReadPHYReg(uint8_t address)
{
	uint16_t returnValue = 0;
	// Set the right address and start the register read operation
	enc424j600WriteReg(MIREGADR, 0x0100 | address);
	enc424j600WriteReg(MICMD, MICMD_MIIRD);
	// Loop to wait until the PHY register has been read through the MII
	// This requires 25.6us
	while (enc424j600ReadReg(MISTAT) & MISTAT_BUSY);
	// Stop reading
	enc424j600WriteReg(MICMD, 0x0000);
	// Obtain results and return
	returnValue = enc424j600ReadReg(MIRD);
	return returnValue;
}

static void enc424j600WritePHYReg(uint8_t address, uint16_t Data)
{
	// Write the register address
	enc424j600WriteReg(MIREGADR, 0x0100 | address);
	// Write the data
	enc424j600WriteReg(MIWR, Data);
	// Wait until the PHY register has been written
	while( enc424j600ReadReg(MISTAT) & MISTAT_BUSY );
}

static void enc424j600ReadN(uint8_t op, uint8_t* data, uint16_t dataLen)
{
	select_net_chip();
	HAL_SPI_Transmit( &hspi2, ((uint8_t*)&op), 1, 1000 );
	HAL_SPI_Receive( &hspi2, data, dataLen, 1000 );
	unselect_net_chip();
}

static void enc424j600WriteN(uint8_t op, uint8_t* data, uint16_t dataLen)
{
	select_net_chip();
	HAL_SPI_Transmit( &hspi2, ((uint8_t*)&op), 1, 1000 );
	HAL_SPI_Transmit( &hspi2, data, dataLen, 1000 );
	unselect_net_chip();
}

static void enc424j600BFSReg(uint16_t address, uint16_t bitMask)
{
	uint8_t bank;
	// See if we need to change register banks
	bank = ((int8_t) address) & 0xE0;
	//if (bank != currentBank)
	{
		if (bank == (0x0u << 5))
			enc424j600ExecuteOp0(B0SEL);
		else if (bank == (0x1u << 5))
			enc424j600ExecuteOp0(B1SEL);
		else if (bank == (0x2u << 5))
			enc424j600ExecuteOp0(B2SEL);
		else if (bank == (0x3u << 5))
			enc424j600ExecuteOp0(B3SEL);

		currentBank = bank;
	}
	enc424j600ExecuteOp16(BFS | (address & 0x1F), bitMask);
}

void enc424j600BFCReg(uint16_t address, uint16_t bitMask)
{
	uint8_t bank;

	// See if we need to change register banks
	bank = ((uint8_t) address) & 0xE0;
	//if (bank != currentBank)
	{
		if (bank == (0x0u << 5))
			enc424j600ExecuteOp0(B0SEL);
		else if (bank == (0x1u << 5))
			enc424j600ExecuteOp0(B1SEL);
		else if (bank == (0x2u << 5))
			enc424j600ExecuteOp0(B2SEL);
		else if (bank == (0x3u << 5))
			enc424j600ExecuteOp0(B3SEL);

		currentBank = bank;
	}

	enc424j600ExecuteOp16(BFC | (address & 0x1F), bitMask);
}
/********************************************************************
 * EXECUTES
 * ******************************************************************/

/**
 * Execute SPI operation
 * @variable <uint8_t> op - operation
 */
static void enc424j600ExecuteOp0( uint8_t op )
{
	select_net_chip();
	HAL_SPI_Transmit( &hspi2, ((uint8_t*)&op), 1, 1000 );
	unselect_net_chip();
}

/**
 * Write data to SPI with operation
 * @variable <uint8_t> op - SPI operation
 * @variable <uint8_t> data - data
 */
static uint8_t enc424j600ExecuteOp8( uint8_t op, uint8_t data )
{
	uint8_t returnValue = 0;
	uint8_t buffer[2];

	buffer[0] = data;
	buffer[1] = data>>8;

	select_net_chip();
	HAL_SPI_Transmit( &hspi2, ((uint8_t*)&op), 1, 1000 );
	HAL_SPI_TransmitReceive( &hspi2, buffer, ((uint8_t*)&returnValue), 1, 1000 );
	unselect_net_chip();
	return returnValue;
}

/**
 * Write data to SPI with operation
 * @variable <uint8_t> op - SPI operation
 * @variable <uint16_t> data - data
 */
static uint16_t enc424j600ExecuteOp16( uint8_t op, uint16_t data )
{
	uint16_t returnValue = 0;
	uint8_t buffer[2];

	buffer[0] = data;
	buffer[1] = data>>8;

	select_net_chip();
	HAL_SPI_Transmit( &hspi2, ((uint8_t*)&op), 1, 1000 );
	HAL_SPI_TransmitReceive( &hspi2, buffer, ((uint8_t*)&returnValue), 2, 1000 );
	unselect_net_chip();
	return returnValue;
}

/**
 * Write data to SPI with operation
 * @variable <uint8_t> op - SPI operation
 * @variable <uint32_t> data - data
 */
static uint32_t enc424j600ExecuteOp32( uint8_t op, uint32_t data )
{
	uint32_t returnValue = 0;
	uint8_t buffer[4];

	buffer[0] = data;
	buffer[1] = data>>8;
	buffer[2] = data>>16;
	buffer[3] = data>>24;

	select_net_chip();
	HAL_SPI_Transmit( &hspi2, ((uint8_t*)&op), 1, 1000 );
	HAL_SPI_TransmitReceive( &hspi2, buffer, ((uint8_t*)&returnValue), 3, 100 );
	unselect_net_chip();
	return returnValue;
}
