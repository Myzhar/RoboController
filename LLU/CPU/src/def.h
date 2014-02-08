 
#include "modbus.h"
#include "modbus_registers.h"
#include "eeprom.h"

 #define DEBUG 1 // Uso i led per verificare segnali vari...
#define LOGSIZE 300

#define BLOCCO_INTERRUPT_TRA_CICLI_PID 1 // Se attivo abilito il blocco dell'interrupt IC tra due cicli PID
//#define PRESCALER_DINAMICO  1

/* //////////////////////////////////////////////////////////////////////////*/
/*Commons includes and definitions                                           */
/* //////////////////////////////////////////////////////////////////////////*/

// Configuration switches for compilation. Comment the undesired lines [34]****
#define CLOCK_FREQ_10 10

//#define ROBOCONTROLLER 1
#define ROBOCONTROLLERV2 1

//#define SLOW_ENC // uncomment for low CPR encoders

// for debugging purposes------------------------------------------------------
#define NO_FLASH 1
//*****************************************************************************


#ifdef CLOCK_FREQ_10
/*
CLOCK_FREQ = 10 MHz
Fosc = CLOCK_FREQ*M/(N1*N2) = 10M*32/(2*2)=80Mhz
Fcy = Fosc / 2 = 40
*/
#define FCY 40000000
// TCY = 1 / FCY = 25 ns
#else
/*
CLOCK_FREQ = 7,3728 MHz
Fosc = CLOCK_FREQ*M/(N1*N2) = 7.3728M*43/(2*2)=79,2576Mhz
Fcy = Fosc / 2 = 39.6288
*/
#define FCY 39628800
// TCY = 1 / FCY = 25.2341731266 ns

#endif
#define TCY 1/((float)(FCY)
#define _ISR_PSV __attribute__((interrupt, auto_psv))

#define MAX_RX_BUFF 64
#define MAX_TX_BUFF 64

// macro to round a float in a int
//#define FLOAT2INT(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
//#define max(a,b) ((a) > (b) ? (a) : (b))


//#define RX_HEADER_LEN 3					// command string header length (byte)

#define FALSE       0
#define TRUE        1
#define OK          1

#define LED_ON      0
#define LED_OFF     1
#define PIN_ON      1
#define PIN_OFF     0

/* ////////////////////////////////////////////////////////////////////////////
** Included in "dsPID.c", it contains definitions and variables initialization
/////////////////////////////////////////////////////////////////////////////*/

/*---------------------------------------------------------------------------*/
/* Status bits			    											     */
/*---------------------------------------------------------------------------*/

#define RC6Tick(x) (Delay1KTCYx(4*x),Delay100TCYx(4*x),Delay10TCYx(3*x))
#define	DELAY100MS Delay10KTCYx(100)	// 100ms

#if defined(__dsPIC33FJ64MC802__) || defined(__dsPIC33FJ128MC802__)
// -- compiling for a 28 pin DSC ***************************************
	#define LED1 _LATA4
	#define LED2 _LATB4
	#define DIR_COM1 _LATB9

#elif defined(__dsPIC33FJ64MC804__) || defined(__dsPIC33FJ128MC804__)
// -- compiling for a 44 pin DSC ***************************************

#ifdef ROBOCONTROLLER
	#define LED1 _LATA8
	#define LED2 _LATA9
	#define AUX1 _LATA7
	#define AUX2 _LATA10
	#define MOTOR_ENABLE1 _LATA0
	#define MOTOR_ENABLE2 _LATA1
	#define DIR_COM1 _LATC3
#endif

#ifdef ROBOCONTROLLERV2
	#define LED1 _LATA8
	#define LED2 _LATA9
	#define AUX1 _LATA7
	#define AUX2 _LATA10
	#define MOTOR_ENABLE1 _LATA1
	#define MOTOR_ENABLE2 _LATA4
	#define DIR_COM1 _LATC3
	#define DIR_COM2 _LATB7
        /* DEBUG */
        // SDA = Out : Connettore IC2 pin 6
        // SCL = Out : Connettore IC2 pin 5
        // RB4 = Out : Connettore IC2 pin 4
        #define PIN_CN_IC2_6    _LATB9
        #define PIN_CN_IC2_5    _LATB8
        #define PIN_CN_IC2_4    _LATB4

#endif
#else
#error -- dsPIC33FJ not recognized. Accepted only 64/128MC802 or 64/128MC804
#endif

// ADC
//#define ADC_CALC_FLAG VOLbits1.bit6	// enable ADC value average calculus
//#define ADC_OVLD_LIMIT 800			// in mA
//#define ADC_OVLD_TIME	100			// n x 10ms

// Input Capture (speed measurement)
//#define SLOW_ENC        1
#define VEL_MIN_PID 1600 // Approx 50mm/s

//#define TMR2_VALUE 0xFFFF
#define TMR2_VALUE 60000
#define TMR3_VALUE 60000

//#define TIMER2_OVERFLOW_VALUE   0xFFFF

#define OLD_INDICE_STATUSBIT1   Old_Indice_Status_Bit1.bit1 // Serve ad identificare il cambio di modalità

#define IC1_FIRST           VOLbits1.bit7		  // first encoder pulse for IC1
#define IC2_FIRST           VOLbits1.bit8		  // first encoder pulse for IC2

#define RAMP_FLAG1          VARbits1.bit6	// acc/dec-eleration flag
#define RAMP_T_FLAG1        VARbits1.bit7	// acc/dec-eleration direction
#define RAMP_FLAG2          VARbits1.bit1	// acc/dec-eleration flag
#define RAMP_T_FLAG2        VARbits1.bit15	// acc/dec-eleration direction


#define PID1_CALC_FLAG      VOLbits1.bit0	  // PID and speed elaboration flag
#define PID2_CALC_FLAG      VOLbits1.bit1	  // PID and speed elaboration flag

#define IC1_CALC      VOLbits1.bit2
#define IC2_CALC      VOLbits1.bit3
#define R 0							// right index
#define L 1							// right index
//#define KP1 kCoeffs1[0]
//#define KI1 kCoeffs1[1]
//#define KD1 kCoeffs1[2]
//#define KP2 kCoeffs2[0]
//#define KI2 kCoeffs2[1]
//#define KD2 kCoeffs2[2]


#define KP1 kCoeffs1[0]
#define KI1 kCoeffs1[1]
#define KD1 kCoeffs1[2]
#define PID_OUT1 PIDstruct1.controlOutput
#define PID_MES1 PIDstruct1.measuredOutput
#define PID_REF1 PIDstruct1.controlReference

#define KP2 kCoeffs2[0]
#define KI2 kCoeffs2[1]
#define KD2 kCoeffs2[2]
#define PID_OUT2 PIDstruct2.controlOutput
#define PID_MES2 PIDstruct2.measuredOutput
#define PID_REF2 PIDstruct2.controlReference


#define TWOPI       6.2831853072	//  360°
//#define PI          3.1415926536        //  180°
#define HALFPI      1.570796327         //  90°
#define QUARTPI     0.7853981634        //  45°

#define TWOPINSEC   6283185307.2	// 360° ( 2 Pigreco * 10^9  per la conversione da nSec a rad/Sec )
#define PI_x_100000 314159,26536        // Pigreco * 10^5
#define PI_x_16384  51471               // Pigreco * 16384 ( = 3,1415926536 shift a SX di 14 posti )



#define RT_TIMER_FLAG VARbits1.bit10	 // real time timer enable
#define TIMER_OK_FLAG VARbits1.bit9	 // time elapsed
#define SCHEDULER_FLAG VARbits1.bit5 // enable scheduler [32a]
/* Definitions end ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// Define Message Buffer Length for ECAN1/ECAN2
#define  MAX_CHNUM	 		7		// Highest Analog input number in Channel Scan
#define  SAMP_BUFF_SIZE	 		8		// Size of the input buffer per analog input
#define  NUM_CHS2SCAN			5		// Number of channels enabled for channel scan

// Nomi dei canali analogici
#define NUMERO_INGRESSI_ANALOGICI   5           // Numero totale degli ingressi
                                                // analogici disponibili sulla scheda

#define PIC_AN0                     0           // Canale analogico pic AN0...7
#define PIC_AN1                     1           // Canale analogico pic AN0...7
#define PIC_AN2                     2           // Canale analogico pic AN0...7
#define PIC_AN3                     3           // Canale analogico pic AN0...7
#define PIC_AN4                     4           // Canale analogico pic AN0...7
#define PIC_AN5                     5           // Canale analogico pic AN0...7
#define PIC_AN6                     6           // Canale analogico pic AN0...7
#define PIC_AN7                     7           // Canale analogico pic AN0...7

#define ROBOCONTROLLER_AN0          0           // AN0 del micro = VMOT
#define ROBOCONTROLLER_AN1          1           // AN1 del PCB; AN4 del micro
#define ROBOCONTROLLER_AN2          2           // AN2 del PCB; AN5 del micro
#define ROBOCONTROLLER_AN3          3           // AN3 del PCB; AN6 del micro
#define ROBOCONTROLLER_AN4          4           // AN4 del PCB; AN7 del micro

// UART DEFINE
#define PORT_COM1                       0
#define PORT_COM2                       1

//#define DISABLE_TX U1STAbits.UTXEN = 0
//#define ENABLE_TX U1STAbits.UTXEN = 1
//#define DISABLE_T2 U2STAbits.UTXEN = 0
//#define ENABLE_TX2 U2STAbits.UTXEN = 1

#define OFF				0
#define ON				1

#define NUMERO_PORT_SERIALI             2

// Define usate per identificare i motori.
#define MOTORE1                         0
#define MOTORE2                         1
#define MOTOR_ACTIVE                    1
#define MOTOR_DEACTIVE                  0
#define MOTORE_STOP                     2048
#define MOTORE_STOP_PID                 0


#define DEAD_ZONE           50
#define IC_PRESCALER_1     1
#define IC_PRESCALER_4     4
#define IC_PRESCALER_16    16


// Gestione LED
#define SEGNALAZIONELED_TON         25         // Tempo Ton ( decine di mSec ) della segnalazione led
#define SEGNALAZIONELED_TOFF        50         // Tempo Toff ( decine di mSec ) della segnalazione led
#define SEGNALAZIONELED_TPAUSE      150         // Tempo di pausa ( decine di mSec ) tra due sequenze di lampeggi

// Codici di errore gestiti dal LED 1 : ERROR
#define LED_ERRORCODE_00_NOERROR    0   // Nessun errore da gestire
#define LED_ERRORCODE_01_WATCHDOG   1   // Errore comunicazione seriale ( Watchdog comunicazione )
#define LED_ERRORCODE_02_FAILMOTOR1 2   // Motore 1 in fail
#define LED_ERRORCODE_03_FAILMOTOR2 3   // Motore 2 in fail
#define LED_ERRORCODE_04_FAILMOTORI 4   // Tutti i motori in errore
#define LED_ERRORCODE_05_CHECKERRORIOK 5   // Usato all'accensione per segnalare che non vi sono errori di sistema( EEPROM E SIMILI )

// Modalità di funzionamento gestite dal LED 2 : STATUS
#define LED_POWERON_05_POWERTEST    5   // Segnalazioni con LED in accensione
//Segnalazioni di funzionamento gestite sempre "come allarmi"
#define LED_STATUSCODE_01_NORMAL_RUN    1
#define LED_STATUSCODE_02_WATCHDOGFAIL  2



