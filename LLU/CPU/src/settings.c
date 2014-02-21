/* ////////////////////////////////////////////////////////////////////////////
** It contains dsPIC settings and initializations
**
** Detailed description are on file "descrEng.txt"
** numbers between brackets, eg.: [1] , are the references to the specific
** decription into the file
/////////////////////////////////////////////////////////////////////////////*/

// standard include
#include "p33Fxxxx.h"
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

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* ports and peripherals registers setting an initialization                 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


// DMA buffers
// ADC buffer, 2 channels (AN0, AN1), 32 bytes each, 2 x 32 = 64 bytes
// Define Message Buffer Length for ECAN1/ECAN2
// #define  MAX_CHNUM	 	5		// Highest Analog input number in Channel Scan
// #define  SAMP_BUFF_SIZE	8		// Size of the input buffer per analog input
// #define  NUM_CHS2SCAN	5		// Number of channels enabled for channel scan
extern unsigned int DmaAdc_A[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(128)));
extern unsigned int DmaAdc_B[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(128)));

extern volatile Motor_t        Motore1;
extern volatile Motor_t        Motore2;


void Settings(void)
{
    //   Watchdog Timer Enable:
    
    
    #ifdef CLOCK_FREQ_10 //{
    // Configure Oscillator to operate the device at 40 Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 10M*32(2*2)=80Mhz for 10 MHz input clock
    PLLFBD=30;					// M=32
    #warning **********************************************************************
    #warning -- compiling for 10MHz oscillator, set MPLAB SIM to 80.0000 MHz ******
    #else

    // Configure Oscillator to operate the device at 39,6288 Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 7.3728M*43(2*2)=79,2576Mhz for 7,3728 MHz input clock
    PLLFBD=41;					// M=43
    #warning **********************************************************************
    #warning -- compiling for 7.3728MHz oscillator, set MPLAB SIM to 79.2576 MHz **
    #endif //}

    CLKDIVbits.PLLPOST=0;		// N1=2
    CLKDIVbits.PLLPRE=0;		// N2=2

    // Disable Watch Dog Timer
    RCONbits.SWDTEN=0;
    RCONbits.BOR = 0;
    RCONbits.POR = 0;
    RCONbits.EXTR = 0;

    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x03);		// Initiate Clock Switch to Primary
                                                // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01);		// Start clock switching

    #ifndef SIM	// [21]
        while(OSCCONbits.COSC != 0b011);	// Wait for Clock switch to occur
        while(OSCCONbits.LOCK!=1) {};		// Wait for PLL to lock
    #endif

#if defined(__dsPIC33FJ64MC802__) || defined(__dsPIC33FJ128MC802__) //{
#warning -- compiling for a 28 pin DSC ****************************************

#ifdef PROTOTYPE //{
	#warning -- compiling for prototype board *************************************

	// Peripheral PIN selection ***********************************
	// Unlock Registers
	//*************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bclr OSCCON, #6 ");
	//************************************************************
	// Configure Input Functions
	//************************************************************
	//***************************
	// Assign IC1 To Pin RP6
	//***************************
	RPINR7bits.IC1R = 6;

	//***************************
	// Assign IC2 To Pin RP10
	//***************************
	RPINR7bits.IC2R = 10;

	//***************************
	// Assign QEA1 To Pin RP6
	//***************************
	RPINR14bits.QEA1R = 6;

	//***************************
	// Assign QEB1 To Pin RP5
	//***************************
	RPINR14bits.QEB1R = 5;

	//***************************
	// Assign QEA2 To Pin RP10
	//***************************
	RPINR16bits.QEA2R = 10;

	//***************************
	// Assign QEB2 To Pin RP11
	//***************************
	RPINR16bits.QEB2R = 11;

	//***************************
	// Assign U1RX To Pin RP8
	//***************************
	RPINR18bits.U1RXR = 8;

	//***************************
	// Assign U2RX To Pin RP7
	//***************************
	RPINR19bits.U2RXR = 7;

	//************************************************************
	// Configure Output Functions
	//************************************************************

	//***************************
	// Assign U1Tx To Pin RP9
	//***************************
	RPOR4bits.RP9R = 3;

	//***************************
	// Assign U2Tx To Pin RP4
	//***************************
	RPOR2bits.RP4R = 5;

	//************************************************************
	// Lock Registers
	//************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bset OSCCON, #6");
	// *********************************** Peripheral PIN selection

	/*-----------------------------------------------------------------------*/
	/* Port	A   			    										     */
	/*-----------------------------------------------------------------------*/
	_TRISA4 = 0;
	/*
	RA4 12 LED1
	*/

	/*-----------------------------------------------------------------------*/
	/* Port	B   			    										     */
	/*-----------------------------------------------------------------------*/
	_TRISB2  = 0;
	_TRISB3	 = 0;

	/*
	RB2  6  H-bridge1 enable
	RB3	 7	H-bridge2 enable
	*/
#endif //}

#ifdef DROIDS //{
#warning -- compiling for 990.011 board **************************************

	// Peripheral PIN selection ***********************************
	// Unlock Registers
	//*************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bclr OSCCON, #6 ");
	//************************************************************
	// Configure Input Functions
	//************************************************************
	//***************************
	// Assign IC1 To Pin RP10
	//***************************
	RPINR7bits.IC1R = 10;

	//***************************
	// Assign IC2 To Pin RP6
	//***************************
	RPINR7bits.IC2R = 6;

	//***************************
	// Assign QEA1 To Pin RP10
	//***************************
	RPINR14bits.QEA1R = 10;

	//***************************
	// Assign QEB1 To Pin RP11
	//***************************
	RPINR14bits.QEB1R = 11;

	//***************************
	// Assign QEA2 To Pin RP6
	//***************************
	RPINR16bits.QEA2R = 6;

	//***************************
	// Assign QEB2 To Pin RP5
	//***************************
	RPINR16bits.QEB2R = 5;

	//***************************
	// Assign U1RX To Pin RP3
	//***************************
	RPINR18bits.U1RXR = 3;

	//***************************
	// Assign U2RX To Pin RP7
	//***************************
	RPINR19bits.U2RXR = 7;

	//************************************************************
	// Configure Output Functions
	//************************************************************

	//***************************
	// Assign U1Tx To Pin RP2
	//***************************
	RPOR1bits.RP2R = 3;
	//***************************
	// Assign U2Tx To Pin RP8
	//***************************
	RPOR4bits.RP8R = 5;

	//************************************************************
	// Lock Registers
	//************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bset OSCCON, #6");
	// *********************************** Peripheral PIN selection

	/*-----------------------------------------------------------------------*/
	/* Port	A   			    										     */
	/*-----------------------------------------------------------------------*/
	_TRISA4 = 0;
	/*
	RA4 12 LED1
	*/

	/*-----------------------------------------------------------------------*/
	/* Port	B   			    										     */
	/*-----------------------------------------------------------------------*/
	_TRISB0  = 0;
	_TRISB1	 = 0;
	_TRISB4	 = 0;
	_TRISB9	 = 0;

	/*
	RB0  4  H-bridge2 enable
	RB1	 5	H-bridge1 enable
	RB4  11 LED2
	RB9  18 DIR 485
	*/
#endif //}
//}

#elif defined(__dsPIC33FJ64MC804__) || defined(__dsPIC33FJ128MC804__) //{
#warning -- compiling for a 44 pin DSC ***************************************


#ifdef DSNAVCON33 //{
	#warning -- compiling for DSNAVCON33 *****************************************
	// Peripheral PIN selection ***********************************
	// Unlock Registers
	//*************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bclr OSCCON, #6 ");
	//************************************************************
	// Configure Input Functions
	//************************************************************
	//***************************
	// Assign IC1 To Pin RP23
	//***************************
	RPINR7bits.IC1R = 23;

	//***************************
	// Assign IC2 To Pin RP25
	//***************************
	RPINR7bits.IC2R = 25;

	//***************************
	// Assign QEA1 To Pin RP23
	//***************************
	RPINR14bits.QEA1R = 23;

	//***************************
	// Assign QEB1 To Pin RP22
	//***************************
	RPINR14bits.QEB1R = 22;

	//***************************
	// Assign QEA2 To Pin RP25
	//***************************
	RPINR16bits.QEA2R = 25;

	//***************************
	// Assign QEB2 To Pin RP24
	//***************************
	RPINR16bits.QEB2R = 24;

	//***************************
	// Assign U1RX To Pin RP5
	//***************************
	RPINR18bits.U1RXR = 5;

	//***************************
	// Assign U2RX To Pin RP20
	//***************************
	RPINR19bits.U2RXR = 20;

	//************************************************************
	// Configure Output Functions
	//************************************************************

	//***************************
	// Assign U1Tx To Pin RP6
	//***************************
	RPOR3bits.RP6R = 3;

	//***************************
	// Assign U2Tx To Pin RP21
	//***************************
	RPOR10bits.RP21R = 5;

	//************************************************************
	// Lock Registers
	//************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bset OSCCON, #6");
	// *********************************** Peripheral PIN selection

	/*-----------------------------------------------------------------------*/
	/* Port	A   			    										     */
	/*-----------------------------------------------------------------------*/
	_TRISA8 = 0;
	/*
	RA8 32 LED1
	*/

	/*-----------------------------------------------------------------------*/
	/* Port	B   			    										     */
	/*-----------------------------------------------------------------------*/
	_TRISA7  = 0;
	_TRISA10 = 0;

	/*
	RA7  13  H-bridge1 enable
	RA10 12	H-bridge2 enable
	*/
#endif //}

#ifdef ROBOCONTROLLER //{
#warning -- compiling for ROBOCONTROLLER *************************************
	// Peripheral PIN selection ***********************************
	// Unlock Registers
	//*************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bclr OSCCON, #6 ");

	//************************************************************
	// Configure Input Functions
	//************************************************************
	//***************************
	// Assign IC1 To Pin RP22
	//***************************
	RPINR7bits.IC1R = 22;		// (Input Capture 1) QEA_1 su RoboController

	//***************************
	// Assign IC2 To Pin RP24
	//***************************
	RPINR7bits.IC2R = 24;		// (Input Capture 2) QEA_2 su RoboController

	//***************************
	// Assign QEA1 To Pin RP22
	//***************************
	RPINR14bits.QEA1R = 22;		// QEA_1 su RoboController

	//***************************
	// Assign QEB1 To Pin RP23
	//***************************
	RPINR14bits.QEB1R = 23;		// QEB_1 su RoboController

	//***************************
	// Assign QEA2 To Pin RP24
	//***************************
	RPINR16bits.QEA2R = 24;		// QEA_2 su RoboController

	//***************************
	// Assign QEB2 To Pin RP25
	//***************************
	RPINR16bits.QEB2R = 25;		// QEB_2 su RoboController

	//***************************
	// Assign U1RX To Pin RP8
	//***************************
	RPINR18bits.U1RXR = 20;		// (UART1 Receive) su RoboController

	//***************************
	// Assign U2RX To Pin RP20
	//***************************
	RPINR19bits.U2RXR = 8;		// (UART2 Receive) su RoboController

	//************************************************************
	// Configure Output Functions
	//************************************************************

	//***************************
	// Assign U1Tx
	//***************************
	RPOR10bits.RP21R = 3;

	//***************************
	// Assign U2Tx
	//***************************
	RPOR4bits.RP8R = 5;

	//************************************************************
	// Lock Registers
	//************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bset OSCCON, #6");
	// *********************************** Peripheral PIN selection

	/*---------------------------------------------------------------------------*/
	/* Port	A   			    											     */
	/*---------------------------------------------------------------------------*/
	_TRISA0 = 0;		// MOTOR_EN1
	_TRISA1 = 0;		// MOTOR_EN2
	_TRISA8 = 0;		// LED1
	_TRISA9 = 0;		// LED2
	_TRISA7 = 0;		// AUX1
	_TRISA10 = 0;		// AUX2

	/*---------------------------------------------------------------------------*/
	/* Port	B   			    											     */
	/*---------------------------------------------------------------------------*/
	//_TRISB2  = 0;
	//_TRISB3  = 0;


	/*---------------------------------------------------------------------------*/
	/* Port	C   			    											     */
	/*---------------------------------------------------------------------------*/
	_TRISC3  = 0;		// DIR RS485
	_TRISC5  = 0;		// TX UART1

#endif //}


#ifdef ROBOCONTROLLERV2 //{
#warning -- compiling for ROBOCONTROLLER V2 or V3 ****************************
	// Peripheral PIN selection ***********************************
	// Unlock Registers
	//*************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bclr OSCCON, #6 ");
            
	//************************************************************
	// Configure Input Functions
	//************************************************************
	//***************************
	// Assign IC1 To Pin RP22
	//***************************
	RPINR7bits.IC1R = 22;		// (Input Capture 1) QEA_1 su RoboController

	//***************************
	// Assign IC2 To Pin RP24
	//***************************
	RPINR7bits.IC2R = 24;		// (Input Capture 2) QEA_2 su RoboController

	//***************************
	// Assign QEA1 To Pin RP22
	//***************************
	RPINR14bits.QEA1R = 22;		// QEA_1 su RoboController

	//***************************
	// Assign QEB1 To Pin RP23
	//***************************
	RPINR14bits.QEB1R = 23;		// QEB_1 su RoboController

	//***************************
	// Assign QEA2 To Pin RP24
	//***************************
	RPINR16bits.QEA2R = 24;		// QEA_2 su RoboController

	//***************************
	// Assign QEB2 To Pin RP25
	//***************************
	RPINR16bits.QEB2R = 25;		// QEB_2 su RoboController

	//***************************
	// Assign U1RX To Pin RP20
	//***************************
	RPINR18bits.U1RXR = 20;		// (UART1 Receive) su RoboController

	//***************************
	// Assign U2RX To Pin RP6
	//***************************
	RPINR19bits.U2RXR = 6;		// (UART2 Receive) su RoboController

	//************************************************************
	// Configure Output Functions
	//************************************************************

	//***************************
	// Assign U1Tx ( RP21 ) U1TX = 00011 = 3
	//***************************
	RPOR10bits.RP21R = 3;

	//***************************
	// Assign U2Tx ( RP5 )
	//***************************
	RPOR2bits.RP5R = 5;

	//************************************************************
	// Lock Registers
	//************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bset OSCCON, #6");
	// *********************************** Peripheral PIN selection

	/*---------------------------------------------------------------------------*/
	/* Port	A   			    											     */
	/*---------------------------------------------------------------------------*/
	_TRISA1 = 0;		// MOTOR_EN1
	_TRISA4 = 0;		// MOTOR_EN2
	_TRISA8 = 0;		// LED1
	_TRISA9 = 0;		// LED2
	_TRISA7 = 0;		// AUX1
	_TRISA10 = 0;		// AUX2

	/*---------------------------------------------------------------------------*/
	/* Port	B   			    											     */
	/*---------------------------------------------------------------------------*/
	_TRISB7  = 0;		// DIR RS485 UART2
	//_TRISB5  = 0;		// TX UART2
        /* per debug uso come uscita i pin di SDA e SCL */
        _TRISB8  = 0;		// SDA = Out : Connettore IC2 pin 6
        _TRISB9  = 0;		// SCL = Out : Connettore IC2 pin 5
        _TRISB4  = 0;		// RB4 = Out : Connettore IC2 pin 4




	/*---------------------------------------------------------------------------*/
	/* Port	C   			    											     */
	/*---------------------------------------------------------------------------*/
	_TRISC3  = 0;		// DIR RS485 UART1
	//_TRISC5  = 0;		// TX UART1
	_TRISC2  = 0;		// OUT Float

#endif //}

#else

#error -- dsPIC33FJ not recognized. Accepted only 64/128MC802 or 64/128MC804

#endif //}
#warning *********************************************************************

    /*---------------------------------------------------------------------------*/
    /* PWM	[11]        			    									     */
    /*---------------------------------------------------------------------------*/
    // Holds the value to be loaded into dutycycle register
    unsigned int period;
    // Holds the value to be loaded into special event compare register
    unsigned int sptime;
    // Holds PWM configuration value
    unsigned int config1;
    // Holds the value be loaded into PWMCON1 register
    unsigned int config2;
    // Holds the value to config the special event trigger postscale and dutycycle
    unsigned int config3;

    // Config PWM
    period = 2048;
    // PWM F=19,340Hz counting UP 12bit resolution @ Fcy=39.628 MHz (osc 7.3728MHz)
    // PWM F=19,522Hz counting UP 12bit resolution @ Fcy=39.628 MHz (osc 10MHz)
    sptime = 0x0;
    // 1:1 postscaler, 1:1 prescale, free running mode
    // PWM time base ON, count up
    config1 = 	PWM1_EN & PWM1_IDLE_CON & PWM1_OP_SCALE1 & PWM1_IPCLK_SCALE1 & PWM1_MOD_FREE;

    // PWM1H e PWM1L enabled in complementar mode
    // dsPICs with 3 pairs of PWM pins have one timer only (A)
    config2 = 	PWM1_MOD1_COMP & PWM1_PEN1L & PWM1_PEN1H &
                PWM1_MOD2_COMP & PWM1_PEN2L & PWM1_PEN2H &
                PWM1_PDIS3H & PWM1_PDIS3L;

    config3 = 	PWM1_SEVOPS1 & PWM1_OSYNC_PWM & PWM1_UEN;
    OpenMCPWM1(period, sptime, config1, config2, config3);

    // Dead Time Unit A assigned to both 1 & 2 PWM pairs
    /* SetMCPWM1DeadTimeAssignment(PWM1_DTS1A_UA & PWM1_DTS1I_UA &
                                   PWM1_DTS2A_UA & PWM1_DTS2I_UA);
    */
    P1DTCON2bits.DTS1A = 0;
    P1DTCON2bits.DTS1I = 0;
    P1DTCON2bits.DTS2A = 0;
    P1DTCON2bits.DTS2I = 0;

    // Dead time 100ns = 0.2% of PWM period
    SetMCPWM1DeadTimeGeneration(PWM1_DTA4 & PWM1_DTAPS1);

    // dutycyclereg=1, dutycycle=50% (motore fermo in LAP mode , updatedisable=0
    SetDCMCPWM1(1, 2048, 0);
    SetDCMCPWM1(2, 2048, 0);

    // configure PWM2 pins as a generic I/O
    PWM2CON1bits.PEN1L = 0;
    PWM2CON1bits.PEN1H = 0;
    /*.......................................................................PWM */


    /*---------------------------------------------------------------------------*/
    /* QEI1	[4]           			    									     */
    /*---------------------------------------------------------------------------*/
    /*
    OpenQEI(QEI_MODE_x4_MATCH & QEI_INPUTS_NOSWAP & QEI_IDLE_STOP
            & QEI_NORMAL_IO & QEI_INDEX_RESET_DISABLE,
            QEI_QE_CLK_DIVIDE_1_128 & QEI_QE_OUT_ENABLE & POS_CNT_ERR_INT_DISABLE);
    */

    QEI1CONbits.QEIM 	= 7;	//	QEI_MODE_x4_MATCH
    //QEI1CONbits.QEIM 	= 5;	//	QEI_MODE_x2_MATCH
    QEI1CONbits.SWPAB 	= 1;	//	QEI_INPUTS_SWAP
    QEI1CONbits.QEISIDL	= 1;	//	QEI_IDLE_STOP
    QEI1CONbits.POSRES	= 0;	//	QEI_INDEX_RESET_DISABLE
    QEI1CONbits.PCDOUT	= 0;	//	QEI_NORMAL_IO
    QEI1CONbits.POSRES	= 0;	//	POS_CNT_ERR_INT_DISABLE

    DFLT1CONbits.QECK	= 6;	//	QEI_QE_CLK_DIVIDE_1_128
    DFLT1CONbits.QEOUT	= 1;	//	QEI_QE_OUT_ENABLE

    MAX1CNT = 0xFFFF;
    POS1CNT = 0;


    /* .......................................................................QEI */

    /* --------------------------------------------------------------------------- */
    /* QEI2	[4]                                                                */
    /* --------------------------------------------------------------------------- */
    /*
    OpenQEI(    QEI_MODE_x4_MATCH & QEI_INPUTS_NOSWAP & QEI_IDLE_STOP &
     *          QEI_NORMAL_IO & QEI_INDEX_RESET_DISABLE,
     *          QEI_QE_CLK_DIVIDE_1_128 & QEI_QE_OUT_ENABLE & POS_CNT_ERR_INT_DISABLE);
    */
    QEI2CONbits.QEIM 	= 7;	//	QEI_MODE_x4_MATCH
    //QEI2CONbits.QEIM 	= 5;	//	QEI_MODE_x2_MATCH
    QEI2CONbits.SWPAB 	= 0;	//	QEI_INPUTS_SWAP
    QEI2CONbits.QEISIDL	= 1;	//	QEI_IDLE_STOP
    QEI2CONbits.POSRES	= 0;	//	QEI_INDEX_RESET_DISABLE
    QEI2CONbits.PCDOUT	= 0;	//	QEI_NORMAL_IO
    QEI2CONbits.POSRES	= 0;	//	POS_CNT_ERR_INT_DISABLE

    DFLT2CONbits.QECK	= 6;	//	QEI_QE_CLK_DIVIDE_1_128
    DFLT2CONbits.QEOUT	= 1;	//	QEI_QE_OUT_ENABLE

    MAX2CNT = 0xFFFF;
    POS2CNT = 0;


    /* .......................................................................QEI */

    /* --------------------------------------------------------------------------- */
    /* Input Capture 1                                                             */
    /* --------------------------------------------------------------------------- */
    // Disattivo l'IC prima di configurarlo
    IC1CONbits.ICM= 0b000;		// Disable Input Capture 1 module

                                        //  bit15-14:   Not Implemented
    IC1CONbits.ICSIDL = 1;              //  bit13:      Input Capture Stop in idle
                                        //  bit12-8:    Not Implemented
    IC1CONbits.ICTMR = 1;		//  bit7:       Select Timer2 as the IC1 Time base
    IC1CONbits.ICI = 0b00;		//  bit6-5:     Interrupt on every capture event
    //IC1CONbits.ICI = 0b01;		//  bit6-5:     Interrupt on every second capture event
    //IC1CONbits.ICI = 0b11;		//  bit6-5:     Interrupt on every fourth capture event

                                        //  bit4:       ReadOnly ICOV : InputCapture Overflow Flag
                                        //  bit3:       ReadOnly ICBNE: InputCapture Buffer Empty Flg
    //IC1CONbits.ICM = 0b000;		//  bit2-0:     OFF  mode : Input Capture module turned off
    //IC1CONbits.ICM = 0b001;		//  bit2-0:     2X   mode : Generate capture event on every 1st rising and falling
    IC1CONbits.ICM = 0b011;		//  bit2-0:     1X   mode : Generate capture event on every 1st rising
    //IC1CONbits.ICM = 0b100;		//  bit2-0:     1/4X mode : Generate capture event on every 4th rising
    //IC1CONbits.ICM = 0b101;		//  bit2-0:     1/16X mode : Generate capture event on every 16th rising
                                        //              In caso di motore lento posso cambiare campionamento
                                        //              senza modificare il timer!!!! :)
                                        // Con encoder a risoluzione elevata devo campionare ogni 16 impulsi
                                        // per misurare un tempo decente.
                                        // Abbassando la risoluzione devo campionare ad ogni fronte.

    /* .............................................................Input Capture */

    /* --------------------------------------------------------------------------- */
    /* Input Capture 2                                                             */
    /* --------------------------------------------------------------------------- */
    // Disattivo l'IC prima di configurarlo
    IC2CONbits.ICM= 0b000;		// Disable Input Capture 1 module

                                        //  bit15-14:   Not Implemented
    IC2CONbits.ICSIDL = 1;              //  bit13:      Input Capture Stop in idle
                                        //  bit12-8:    Not Implemented
    IC2CONbits.ICTMR = 0;		//  bit7:       Select Timer3 as the IC1 Time base
    //IC2CONbits.ICTMR = 1;		//  bit7:       Select Timer2 as the IC1 Time base
    IC2CONbits.ICI = 0b00;		//  bit6-5:     Interrupt on every capture event
    //IC2CONbits.ICI = 0b01;		//  bit6-5:     Interrupt on every second capture event
    //IC2CONbits.ICI = 0b11;		//  bit6-5:     Interrupt on every fourth capture event
                                        //  bit4:       ReadOnly ICOV : InputCapture Overflow Flag
                                        //  bit3:       ReadOnly ICBNE: InputCapture Buffer Empty Flg
    //IC2CONbits.ICM = 0b000;		//  bit2-0:     OFF  mode : Input Capture module turned off
    //IC2CONbits.ICM = 0b001;		//  bit2-0:     2X   mode : Generate capture event on every 1st rising and falling
    IC2CONbits.ICM = 0b011;		//  bit2-0:     1X   mode : Generate capture event on every 1st rising
    //IC2CONbits.ICM = 0b100;		//  bit2-0:     1/4X mode : Generate capture event on every 4th rising
    //IC2CONbits.ICM = 0b101;		//  bit2-0:     1/16X mode : Generate capture event on every 16th rising
                                        //              In caso di motore lento posso cambiare campionamento
                                        //              senza modificare il timer!!!! :)
                                        // Con encoder a risoluzione elevata devo campionare ogni 16 impulsi
                                        // per misurare un tempo decente.
                                        // Abbassando la risoluzione devo campionare ad ogni fronte.
    /* .............................................................Input Capture */



#ifndef TIMER_OFF
    /* --------------------------------------------------------------------------- */
    /* Timer 2	& 3                                                                */
    /* --------------------------------------------------------------------------- */
//#define TMR2_VALUE 0xFFFF
    //   PRESCALER 1:1 -> TimeBase_Timer2 = 1/TyCK = ( 1/ (40Mhz/1) ) = 0,025uSec = 25nSec
    //   PRESCALER 1:8 -> TimeBase_Timer2 = 1/TyCK = ( 1/ (40Mhz/8) ) = 0,200uSec = 200nSec
    //  Un periodo di "2000" per esempio significa 2000*25nSec = 50uSec tra due fronti

    // La quadratura non centra con la misura del periodo perchè andiamo a misurare il movimento
    // del pin fisico dell'encoder.

    // Ipotizzando un encoder da 2048CPR calettato su un motore da 10000rpm ( 167 giri al secondo )
    // vuol dire avere 2048 * 167 = 342016 impulsi al secondo ovvero 342Khz
    // 342Khz significano un impulso ogni 0,000002924 Sec, ovvero 2,92uSec

    // Con una base tempi di 25nSec significa un errore dello 0,85%
    // Con una base tempi di 200nSec significa un errore dello 6,84%

    T2CONbits.TON = 0;				// Disable Timer
    T2CONbits.TSIDL = 1;			// Stop in Idle Mode bit
    T2CONbits.TGATE = 0;			// Disable Gated Timer mode
    //T2CONbits.TCKPS = 0b11;			// Select 1:256 Prescaler
    //T2CONbits.TCKPS = 0b10;			// Select 1:64 Prescaler
    //T2CONbits.TCKPS = 0b01;			// Select 1:8 Prescaler
    T2CONbits.TCKPS = 0b00;			// Select 1:1 Prescaler
    T2CONbits.TCS = 0;				// Select internal clock source
    TMR2 = 0x00;				// Clear timer register
    PR2 = TMR2_VALUE;				// Load the period value



    T3CONbits.TON = 0;				// Disable Timer
    T3CONbits.TSIDL = 1;			// Stop in Idle Mode bit
    T3CONbits.TGATE = 0;			// Disable Gated Timer mode
    //T3CONbits.TCKPS = 0b11;			// Select 1:256 Prescaler
    //T3CONbits.TCKPS = 0b10;			// Select 1:64 Prescaler
    //T3CONbits.TCKPS = 0b01;			// Select 1:8 Prescaler
    T3CONbits.TCKPS = 0b00;			// Select 1:1 Prescaler
    T3CONbits.TCS = 0;				// Select internal clock source
    TMR3 = 0x00;				// Clear timer register
    PR3 = TMR3_VALUE;				// Load the period value

    //  La variabile T_Prescaler_TIMER2 contiene il valore del prescaler moltiplicato 
    //  per 25 che è la base tempi in nSec
    //  Il dato è moltiplicato per 10 per coerenza con T_Prescaler_X10_IC
    if(T2CONbits.TCKPS == 0b00 )   Motore1.I_Prescaler_TIMER = 25; // Prescaler 1:1 * 25nSec
    if(T2CONbits.TCKPS == 0b01 )   Motore1.I_Prescaler_TIMER = 200; // Prescaler 1:8 * 25nSec
    if(T2CONbits.TCKPS == 0b10 )   Motore1.I_Prescaler_TIMER = 1600; // Prescaler 1:64 * 25nSec
    if(T2CONbits.TCKPS == 0b11 )   Motore1.I_Prescaler_TIMER = 6400; // Prescaler 1:256 * 25nSec

    if(T3CONbits.TCKPS == 0b00 )   Motore2.I_Prescaler_TIMER = 25; // Prescaler 1:1 * 25nSec
    if(T3CONbits.TCKPS == 0b01 )   Motore2.I_Prescaler_TIMER = 200; // Prescaler 1:8 * 25nSec
    if(T3CONbits.TCKPS == 0b10 )   Motore2.I_Prescaler_TIMER = 1600; // Prescaler 1:64 * 25nSec
    if(T3CONbits.TCKPS == 0b11 )   Motore2.I_Prescaler_TIMER = 6400; // Prescaler 1:256 * 25nSec

//
/*...................................................................Timer 2 */


/*---------------------------------------------------------------------------*/
/* Timer 1	1ms [13]    			    									 */
/*---------------------------------------------------------------------------*/
#ifdef CLOCK_FREQ_10
    #define TMR1_VALUE 40000
#else
    #define TMR1_VALUE 39628
#endif
    // Configuro il timer ma li accendo quando abilito l'ISR
    OpenTimer1( T1_OFF &
                T1_GATE_OFF &
                T1_PS_1_1 &
                T1_SYNC_EXT_OFF &
                T1_SOURCE_INT,
                TMR1_VALUE);

#endif
/*................................................................ ..Timer 1 */
}


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Interrupts setting                                                        */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void ISR_Settings(void)
{

    //  This means that if the IPL is 0, all exceptions at priority Level 1
    //  and above may interrupt the processor.
    //  If the IPL is 7, only hardware traps may interrupt the processor.
    SET_CPU_IPL(0);

    // Codice di attivazione interrupt in InitADC(void)
    _DMA7IF = 0;            //Clear the DMA interrupt flag bit
    _DMA7IE = 1;            //Set the DMA interrupt enable bit
    DMA7CONbits.CHEN = 1;   // Enable DMA
    AD1CON1bits.ADON = 1;   // Turn on the A/D converter

    //-------PWM	[11]
    ConfigIntMCPWM1(PWM1_INT_DIS);

    // INTERRUPT CONFIGURATION : Timer 1
    // ConfigIntTimer1(T1_INT_PRIOR_1 & T1_INT_ON);
    // _T1IP   =   3;
    // _T1IF   =   0;
    // _T1IE   =   1;

    PR1 = TMR1_VALUE;
    IPC0bits.T1IP = 5;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;   // Start Timer 1

    // INTERRUPT CONFIGURATION : Timer 2
    IPC1bits.T2IP = 3;
    IEC0bits.T2IE = 1 ;
    IFS0bits.T2IF = 0;  // interrupt flag reset
    T2CONbits.TON = 1;   // Start Timer 2
    //	ConfigIntTimer2(T2_INT_PRIOR_4 & T2_INT_ON);
    // _T2IP = 1;  // Set Timer 2 Interrupt Priority Level
    // _T2IF = 0;  // interrupt flag reset
    // _T2IE = 1;  // Enable Timer2 interrupt
    
    // INTERRUPT CONFIGURATION : Timer 3
    IPC2bits.T3IP = 3;
    IEC0bits.T3IE = 1 ;
    IFS0bits.T3IF = 0;  // interrupt flag reset
    T3CONbits.TON = 1;   // Start Timer 2
    // _T3IP = 2;  // Set Timer 3 Interrupt Priority Level
    // _T3IF = 0;  // interrupt flag reset
    // _T3IE = 1;  // Enable Timer3 interrupt
    // PR3 = TMR3_VALUE;
    // T3CONbits.TON = 1;   // Start Timer 3
    
    //-------Input Capture 1 [7]
    //ConfigIntCapture1(IC_INT_ON & IC_INT_PRIOR_4);
    IPC0bits.IC1IP = 4;
    IEC0bits.IC1IE = 1;
    IFS0bits.IC1IF = 0;
    // _IC1IE = 1;
    // _IC1IP = 2;
    // _IC1IF = 0; // Clear IC1 Interrupt Status Flag


    //-------Input Capture 2 [7]
    // ConfigIntCapture2(IC_INT_ON & IC_INT_PRIOR_5);
    IPC1bits.IC2IP = 4;
    IEC0bits.IC2IE = 1;
    IFS0bits.IC2IF = 0;
    // _IC2IE = 1;
    // _IC2IP = 2;
    // _IC2IF = 0; // Clear IC2 Interrupt Status Flag

}
