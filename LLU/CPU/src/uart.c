// standard include
#include "p33Fxxxx.h"
#include <stdio.h>
#include <dsp.h>
#include <pwm12.h>
#include <uart.h>
#include <qei.h>
#include <adc.h>
#include <timer.h>
#include <ports.h>
#include <dma.h>
#include <math.h>
#include <stdlib.h>
#include <libq.h>

#include "def.h"
#include "ptype.h"
#include "var.h"


/*  ***************************************************************************
 *  ***************************************************************************
 *      UART
 *  ***************************************************************************
 *  ***************************************************************************
 */

void Usart1Setting(void)
{   /*---------------------------------------------------------------------------*/
    /* USART1	[6]     			    									     */
    /*---------------------------------------------------------------------------*/
    float BaudRate;
    float BRG;
    // 	Baud Rate = Fcy / ( 4 * (UxBRG + 1) ) with BRGH = 1
    //        value for the U1BRG register rounded to closest integer (+0.5)
    //
    BaudRate = 57600; // desired baud rate
    BRG = (FCY/(4*(BaudRate)))-0.5;

    //...............................................................DMA UART TX
    //  Associate DMA Channel 6 with UART Tx
    DMA6REQ = 0x000c;		// Select UART1 Transmitter
    DMA6PAD = (volatile unsigned int) &U1TXREG;

    DMA6CONbits.SIZE  = 1;	// Transfer bytes
    DMA6CONbits.DIR   = 1;	// Transfer data from RAM to UART
    DMA6CONbits.HALF  = 0;	// Interrupt when all of the data has been moved
    DMA6CONbits.AMODE = 0;	// Register Indirect with Post-Increment
    DMA6CONbits.MODE  = 1;	// One-Shot mode

    // Associate one buffer with Channel 6 for one-shot operation
    DMA6STA = __builtin_dmaoffset(Uart1TxBuff);


        //	Enable DMA Interrupts
    IFS4bits.DMA6IF  = 0;	// Clear DMA Interrupt Flag
    IEC4bits.DMA6IE  = 1;	// Enable DMA interrupt


    //...............................................................DMA UART TX

    //.....................................................................USART
    U1MODEbits.STSEL = 0;		// 1-stop bit
    U1MODEbits.PDSEL = 0;		// No Parity, 8-data bits
    U1MODEbits.ABAUD = 0;		// Autobaud Disabled
    U1MODEbits.RTSMD = 1;		// No flow control
    U1MODEbits.BRGH  = 1;		// Hi speed
    U1BRG = BRG;				// BAUD Rate Setting

    //  Configure UART for DMA transfers
    U1STAbits.UTXISEL0 = 0;		// Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.URXISEL  = 0;		// Interrupt after one RX character is received

    //  Enable UART Rx and Tx
    U1MODEbits.UARTEN  	= 1;	// Enable UART
    U1STAbits.UTXEN 	= 1;	// Enable UART Tx
    IEC4bits.U1EIE      = 0;
    _U1RXIF		= 0;	// Reset RX interrupt flag
    _U1RXIE		= 1;	// Enable RX interrupt

    Uart1RxStatus = 0;
    ChkSum1=0;
    //TxComplete[PORT_COM1]=TRUE; // Porta libera di trasmettere.
    //.....................................................................USART
}

//    void Usart2Setting(void)
//    {   /*---------------------------------------------------------------------------*/
//        /* USART2	[6z]     			    									     */
//        /*---------------------------------------------------------------------------*/
//        float BaudRate2;
//        float BRG2;
//
//        /* 	Baud Rate = Fcy / ( 4 * (UxBRG + 1) ) with BRGH = 1
//                value for the U2BRG register rounded to closest integer (+0.5)
//        */
//        BaudRate2 = 115200; // desired baud rate
//        BRG2 = (FCY/(4*(BaudRate2)))-0.5;
//
//        /*..............................................................DMA UART2 TX */
//        //  Associate DMA Channel 5 with UART2 Tx
//        DMA5REQ = 0x003f;		// Select UART2 Transmitter
//        DMA5PAD = (volatile unsigned int) &U2TXREG;
//
//        DMA5CONbits.SIZE  = 1;	// Transfer bytes
//        DMA5CONbits.DIR   = 1;	// Transfer data from RAM to UART
//        DMA5CONbits.HALF  = 0;	// Interrupt when all of the data has been moved
//        DMA5CONbits.AMODE = 0;	// Register Indirect with Post-Increment
//        DMA5CONbits.MODE  = 1;	// One-Shot mode
//
//        // Associate one buffer with Channel 5 for one-shot operation
//        DMA5STA = __builtin_dmaoffset(Uart2TxBuff);
//
//        //	Enable DMA Interrupts
//        IFS3bits.DMA5IF  = 0;	// Clear DMA Interrupt Flag
//        IEC3bits.DMA5IE  = 1;	// Enable DMA interrupt
//        /*..............................................................DMA UART2 TX */
//
//        /*....................................................................USART2 */
//        U2MODEbits.STSEL = 0;	// 1-stop bit
//        U2MODEbits.PDSEL = 0;	// No Parity, 8-data bits
//        U2MODEbits.ABAUD = 0;	// Autobaud Disabled
//        U2MODEbits.RTSMD = 1;	// No flow control
//        U2MODEbits.BRGH  = 1;	// Hi speed
//        U2BRG = BRG2;			// BAUD Rate Setting
//
//        //  Configure UART2 for DMA transfers
//        U2STAbits.UTXISEL0 = 0;	// Interrupt after one Tx character is transmitted
//        U2STAbits.UTXISEL1 = 0;
//        U2STAbits.URXISEL  = 0;	// Interrupt after one RX character is received
//
//        //  Enable UART2 Rx and Tx
//        U2MODEbits.UARTEN  	= 1;	// Enable UART
//        U2STAbits.UTXEN 	= 1;	// Enable UART Tx
//        IEC4bits.U2EIE 	= 0;
//        _U2RXIF		= 0;	// Reset RX interrupt flag
//        _U2RXIE		= 1;	// Enable RX interrupt
//
//        Uart2RxStatus = 0;
//        ChkSum2=0;
//        //TxComplete[PORT_COM2]=TRUE; // Porta libera di trasmettere.
//        /*.....................................................................USART */
//    }

void Usart2Setting(void)
{   /*---------------------------------------------------------------------------*/
    /* USART2	NO DMA   			    									     */
    /*---------------------------------------------------------------------------*/
    float BaudRate2;
    float BRG2;
    unsigned int i;

    /* 	Baud Rate = Fcy / ( 4 * (UxBRG + 1) ) with BRGH = 1
            value for the U2BRG register rounded to closest integer (+0.5)
    */
    BaudRate2 = 57600; // desired baud rate
    BRG2 = (FCY/(4*(BaudRate2)))-0.5;

    /*....................................................................USART2 */
    U2MODEbits.UARTEN = 0;	// Bit15    TX, RX DISABLED, ENABLE at end of func
                                //          UARTx is disabled; UARTx pins are controlled by
                                //          the corresponding PORT, LAT and TRIS bits
    //U2MODEbits.notimplemented;// Bit14
    U2MODEbits.USIDL = 0;	// Bit13    Continue in Idle

    U2MODEbits.IREN = 0;	// Bit12    No IR translation
    U2MODEbits.RTSMD = 1;	// Bit11    No flow control, UxRTS is in Simplex mode
    //U2MODEbits.notimplemented;// Bit10
    U2MODEbits.UEN = 0;		// Bits8,9  TX,RX enabled, CTS,RTS not
                                //          UxTX and UxRX pins are enabled and used;
                                //          UxCTS, UxRTS and BCLKx pins are controlled by port latches
    U2MODEbits.WAKE = 0;	// Bit7     No Wake up (since we don't sleep here)
    U2MODEbits.LPBACK = 0;	// Bit6     No Loop Back
    U2MODEbits.ABAUD = 0;	// Bit5     No Autobaud (would require sending '55')
    U2MODEbits.URXINV = 0;	// Bit4     IdleState = 1  (for dsPIC)
    U2MODEbits.BRGH  = 1;	// Bit3     Hi speed
    U2MODEbits.PDSEL = 0;	// Bits1,2  8bit, No Parity
    U2MODEbits.STSEL = 0;	// Bit0     One Stop Bit

    U2BRG = BRG2;		// BAUD Rate Setting

    // Load all values in for U1STA SFR
    U2STAbits.UTXISEL1 = 0;	//Bit15     10 = Interrupt is generated when a character is transferred
                                //          to the Transmit Shift register and the transmit buffer becomes empty
                                //          00 = Interrupt after one TX Character is transmitted

    U2STAbits.UTXINV = 0;	//Bit14     N/A, IRDA config
    U2STAbits.UTXISEL0 = 0;	//Bit13     Other half of Bit15
    //U2STAbits.notimplemented = 0;//Bit12
    U2STAbits.UTXBRK = 0;	//Bit11     Disabled
    U2STAbits.UTXEN = 0;	//Bit10     TX pins controlled by periph
    U2STAbits.UTXBF = 0;	//Bit9      *Read Only Bit*
    U2STAbits.TRMT = 0;         //Bit8      *Read Only bit*
    U2STAbits.URXISEL = 0;	//Bits6,7   Int. on character recieved  Interrupt after one RX character is received
    U2STAbits.ADDEN = 0;	//Bit5      Address Detect Disabled
    U2STAbits.RIDLE = 0;	//Bit4      *Read Only Bit*
    U2STAbits.PERR = 0;		//Bit3      *Read Only Bit*
    U2STAbits.FERR = 0;		//Bit2      *Read Only Bit*
    U2STAbits.OERR = 0;		//Bit1      *Read Only Bit*
    U2STAbits.URXDA = 0;	//Bit0      *Read Only Bit*

    IPC7 = 0x4400;	// Mid Range Interrupt Priority level, no urgent reason

    //  Enable UART2 Rx and Tx
    IFS1bits.U2TXIF = 0;	// Clear the Transmit Interrupt Flag
    //IEC1bits.U2TXIE = 1;	// Enable Transmit Interrupts
    IEC1bits.U2TXIE = 1;	// Disable Transmit Interrupts
    IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
    IEC1bits.U2RXIE = 1;	// Enable Recieve Interrupts

    U2MODEbits.UARTEN = 1;	// And turn the peripheral on

    TxNByte_UART2 = 0;
    U2STAbits.UTXEN 	= 1;	// Enable UART Tx
    Uart2RxStatus = 0;
    ChkSum2=0;
    /* wait at least 104 usec (1/9600) before sending first char */
    for(i = 0; i < 4160; i++)
    {
        Nop();
    }
    //TxComplete[PORT_COM2]=TRUE; // Porta libera di trasmettere.
    /*.....................................................................USART */
}


void TxString(unsigned char *Punt, unsigned char NCar, unsigned char Port)	// [18]
{   /*
     * Questa funzione trasmette NCar caratteri del buffer *Punt sulla porta Port
     */
    unsigned char TxCount = 0;


    if( ( Port == 0 ) ) //&& (TxComplete[0]==TRUE))
    {   TxComplete[Port] = FALSE;
        for (TxCount = 0; TxCount < NCar; TxCount++)
        {   //Copio la stringa da tramettere nel buffer associato al DMA6
            Uart1TxBuff[TxCount] = *Punt++;
        }
        DMA6CNT = TxCount-1;	// # of DMA requests
        DMA6CONbits.CHEN  = 1;	// Re-enable DMA Channel
        DMA6REQbits.FORCE = 1;	// Manual mode: Kick-start the first transfer
    }

//    if( ( Port == 1 ) ) //&& (TxComplete[0]==TRUE))
//    {   TxComplete[Port] = FALSE;
//        for (TxCount = 0; TxCount < NCar; TxCount++)
//        {   //Copio la stringa da tramettere nel buffer associato al DMA6
//            Uart2TxBuff[TxCount] = *Punt++;
//        }
//
//        DMA5CNT = TxCount-1;	// # of DMA requests
//        DMA5CONbits.CHEN  = 1;	// Re-enable DMA Channel
//        DMA5REQbits.FORCE = 1;	// Manual mode: Kick-start the first transfer
//    }

    if((Port == 1 ) && (U2STAbits.TRMT == TRUE))
    {   // U2STAbits.TRMT == 1 => Porta libera per trasmettere.
        TxPointer_UART2 = Punt;
        TxNByte_UART2 = NCar;
        TxComplete[Port] = FALSE;
        U2TXREG = *TxPointer_UART2++;
        TxNByte_UART2--;
        //_U2TXIF = 0;                        // interrupt flag reset
        IEC1bits.U2TXIE = 1;                // Enable Transmit Interrupts
    }
}

/* ************************************************************************ */
/*      Vari esempi di uso comodi anche per effettuare il debug             */
/* ************************************************************************ */
/*
 * sprintf(TmpBuff[PORT_COM1][0], "PORT_COM1");
 * sprintf(TmpBuff[PORT_COM2][0], "PORT_COM2");
 * TxString(TmpBuff[PORT_COM1][0], 5, PORT_COM1);
 *
 *
 * TmpBuff[PORT_COM1][0] = 'A';
 * TmpBuff[PORT_COM1][1] = 'B';
 * TmpBuff[PORT_COM1][2] = 'C';
 * TmpBuff[PORT_COM1][3] = 'D';
 * TxString(TmpBuff[PORT_COM1][0], 3, PORT_COM1);
 *
 *
 * TmpBuff[0] = 'A';
 * TmpBuff[1] = 'B';
 * TmpBuff[2] = 'C';
 * TmpBuff[3] = 'D';
 * TxString(TmpBuff, 3, 0);
 * TxString(TmpBuff, 3, 1);
 *
 *
 * sprintf(TmpBuff[PORT_COM1][0], "PORT_COM1");
 * sprintf(TmpBuff[PORT_COM2][0], "PORT_COM2");
 * TxString(TmpBuff[PORT_COM1][0], 9, PORT_COM1);
 * TxString(TmpBuff[PORT_COM2][0], 9, PORT_COM2);
 *
 *
 */


/*  ***************************************************************************
 *  ***************************************************************************
 *      UART Interrupt routine
 *  ***************************************************************************
 *  ***************************************************************************
 */


void _ISR_PSV _DMA6Interrupt(void) {	// DMA for UART1 TX [6d]
    InterruptTest10++;
    _DMA6IF = 0;	// interrupt flag reset
    //TxComplete[PORT_COM1] = TRUE;
    TxComplete[PORT_COM2] = TRUE;
    InterruptTest10--;
}

//void _ISR_PSV _DMA5Interrupt(void)	// DMA for UART2 TX [6zd]
//{   _DMA5IF = 0;	// interrupt flag reset
//    //TxComplete[PORT_COM2] = TRUE;
//    TxComplete[PORT_COM1] = TRUE;
//}

void __attribute__ ((interrupt, no_auto_psv)) _U1ErrInterrupt(void) {
    InterruptTest9++;
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IFS4bits.U1EIF = 0; // Clear the UART1 Error Interrupt Flag
    DISICNT = 0; //re-enable interrupts
    InterruptTest9--;
}

void __attribute__ ((interrupt, no_auto_psv)) _U2ErrInterrupt(void) {
    InterruptTest8++;
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IFS4bits.U2EIF = 0; // Clear the UART2 Error Interrupt Flag
    DISICNT = 0; //re-enable interrupts
    InterruptTest8--;
}

void _ISR_PSV _U1RXInterrupt(void) {	// UART RX [6b]
    InterruptTest7++;
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    _U1RXIF = 0; 	// interrupt flag reset
    ClrWdt();		// [1]
    unsigned char DatoRx;
    DatoRx = ReadUART1();
    if(StatoSeriale[PORT_COM1] == WAIT_MESSAGE)
    {   //  time-out dei dati in ricezione non sia scaduto, altrimenti
        if (!TimerOutRxModbus[PORT_COM1])
        {   FreeRxBuffer(PORT_COM1);
        }
        TimerOutRxModbus[PORT_COM1] = TIME_OUT_MODBUS;
        *RxPointer[PORT_COM1] = DatoRx; //  Salva nel buffer il dato ricevuto
        RxPointer[PORT_COM1]++;
        RxNByte[PORT_COM1]++;
        if (RxNByte[PORT_COM1] >= MODBUS_N_BYTE_RX)     //  se ricevuti più caratteri della lunghezza del buffer
        {   RxPointer[PORT_COM1] = ModbusRxBuff[PORT_COM1]; //  azzera il puntatore di ricezione
            RxNByte[PORT_COM1] = 0;                 //  e il numero di byte ricevuti
        }
    }
    DISICNT = 0; //re-enable interrupts
    InterruptTest7--;
}

void _ISR_PSV _U2RXInterrupt(void) {	// UART2 RX [6zb]
    InterruptTest6++;
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    _U2RXIF = 0; 	// interrupt flag reset
    ClrWdt();		// [1]
    unsigned char DatoRx;
    DatoRx = ReadUART2();
    if(StatoSeriale[PORT_COM2] == WAIT_MESSAGE)
    {   //  time-out dei dati in ricezione non sia scaduto, altrimenti
        if (!TimerOutRxModbus[PORT_COM2])
        {   FreeRxBuffer(PORT_COM2);
        }
        TimerOutRxModbus[PORT_COM2] = TIME_OUT_MODBUS;
        *RxPointer[PORT_COM2] = DatoRx; //  Salva nel buffer il dato ricevuto
        RxPointer[PORT_COM2]++;
        RxNByte[PORT_COM2]++;
        if (RxNByte[PORT_COM2] >= MODBUS_N_BYTE_RX)     //  se ricevuti più caratteri della lunghezza del buffer
        {   RxPointer[PORT_COM2] = ModbusRxBuff[PORT_COM2]; //  azzera il puntatore di ricezione
            RxNByte[PORT_COM2] = 0;                 //  e il numero di byte ricevuti
        }
    }
    DISICNT = 0; //re-enable interrupts

    InterruptTest6--;
}


void _ISR_PSV _U2TXInterrupt(void)  {
    InterruptTest5++;
    _U2TXIF = 0;                        // interrupt flag reset
    unsigned char debugTX;

    IFS1bits.U2TXIF = 0;
    // clear TX interrupt flag

    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles

    ClrWdt();                           // [1]

    if(TxNByte_UART2)
    {   debugTX =  *TxPointer_UART2++;       //  Trasmette il carattere del buffer puntato da TxPointer
        U2TXREG = debugTX;
        TxNByte_UART2 = TxNByte_UART2 - 1;  // TxNByte_UART2--; non pare funzionare
    }
    else
    {
        TxComplete[PORT_COM2] = TRUE;
        IEC1bits.U2TXIE = 0;            //  Disable Transmit Interrupts
    }
    DISICNT = 0; //re-enable interrupts
    InterruptTest5--;
}



