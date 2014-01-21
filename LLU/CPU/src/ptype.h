#ifndef PTYPE_INC
#define PTYPE_INC


/*
 *          NOTA RELAZIONI PERIFERICHE e VARIABILI
 *     TIMER2 <=> IC1 <=> QEI1 <=> LEFT <=> 1
 *     TIMER3 <=> IC2 <=> QEI2 <=> RIGHT <=> 0
 */



/* //////////////////////////////////////////////////////////////////////////*/
/* Included	in "dsPid_definitions.h", it contains functions prototypes       */
/* //////////////////////////////////////////////////////////////////////////*/

//! Struttura dati usata per i timer Software
//! \sa Funzione GestioneTimerSW() per ulteriori dettagli
//! \sa Funzione _T1Interrupt() per la chiamata della funzione sotto interrupt
typedef struct
{	unsigned	T_flag:1;           //!< Flag che diventa TRUE nel momento in cui il tempo è trascorso
	unsigned int	T_count_time;       //!< Varibile che viene decrementata ogni xmSec per contare il tempo trascorso
	unsigned int	T_initial_value;    //!< Valore con cui caricare T_count_time per un nuovo conteggio
} Timer_t;

//! Struttura dati usata per ls misura del periodo fuori dall'interrupt Capture

//typedef struct
//{   volatile unsigned int    TimeOld;            //!< Tempo del precedente campionamento
//    volatile unsigned int    TimeNew;            //!< Tempo attuale del campionamento
//    volatile unsigned int    NumbreOfOverflow;   //!< Numero di Overflow intercorsi da quando è stato campionato TimeOld
//    volatile unsigned int    DatiDaElaborare;    //!< Va a "1" quando campiono "TimeOld" nell'ISR, torna a "0" quando fuori ISR eseguo il calcolo
//} PeriodMeasure_t;

//! Struttura usata per i motori, contiene tutte le informazioni di controllo relative al motore e
//! al relativo encoder.
typedef struct
{   /* VARIABILI USATE PER INTERFACCIARSI CON L'ESTERNO */
    volatile int            I_GearAxelSpeed;        //!< Velocità asse riduttore in RPM x100
    volatile int            I_MotorAxelSpeed;           // contengono la velocità attuale
    volatile long           L_WheelSpeed;           // Velocità in mm/Sec della ruota
    
    /* VARIABILI INTERNE ALLA ROBOCONTROLLER            */
    volatile unsigned char  UC_Fail;                // Se a "1" il motore è in Fail, azzero il PID e fermo i ponti.
    volatile unsigned char  UC_MotorNumber;         // Indica a quale motore fa riferimento la struttura in uso ( vedi in funzione PID )
    volatile unsigned char  UC_OverFlowCounter;         // contatori overflow Timer 2, uno per ogni input capture
    volatile unsigned char  UC_OverFlowErrorCounter;         // contatori overflow Timer 2, uno per ogni input capture
    volatile unsigned char  UC_ICM_Restart_Value;       //  Valore di ICM con cui riattivare l'interrupt
    volatile unsigned char  UC_First_IC_Interrupt_Done; // indici per arry e controllo primo interrupt
    volatile unsigned char  UC_IC_idx;
    volatile unsigned int   UI_MediaIC[10];             // array per la media
    volatile unsigned int   UI_Old_Capture;             // variabili per il primo interrupt
    volatile unsigned int   UI_Period;                   // contengono la velocità attuale

    volatile int            I_MotorRpmMax;
    volatile int            I_MotorRpmMin;
    volatile int            I_Prescaler_IC;             // Prescaler usato per la misura del tempo tramite l'InputCapture
    volatile int            I_Prescaler_TIMER;      // Prescaler usato dal TIMER2/3, mi serve a calcolare la base tempi
    volatile long           L_RpmConversion;

/*
    NOTE:
    MotoreX.I_MotorAxelSpeed :	RPM asse motore, un motore "standard" ruota nel range 0-10000RPM, il dato è un intero.

    MotoreX.I_GearAxelSpeed  :	RPM uscita riduttore, considerando un rapporto riduzione minimo di 1:100 per motori da 						10000RPM e di 1:10 per motori sotto i 3000RPM il dato varierà al max nel range 0-300
				**** Il dato GearAxelSpeed è un intero moltiplicato per un fattore 100		******
				**** Range 0-30000 con due cifre decimali in visualizzazione			******



*/
    /*  T_FattoreConversioneRPM_1...T_FattoreConversioneRPM_3 :
     *  Sono dei dati calcolati quando vengono modificati i parametri del robot e servono a determinare 
     *  il valore di conversione per ottenere gli RPM dal dato del Timer.
     *  Si calcolano in questo modo:
     *                         60 / (Er/Pr  * Tr)  * 10^9
     *  Er è la risoluzione dell'encoder, ovvero uno dei parametri:
     *          ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT]
     *          ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT]
     * 
     *  Pr è il valore del prescaler moltiplicato per 10 (5-10-40-160), ovvero i define:
     *          #define IC_PRESCALER_1     1
     *          #define IC_PRESCALER_4     4
     *          #define IC_PRESCALER_16    16
     *         
     *  Tr è il periodo di un singolo count del Input Capture in ns, ovvero:
     *          T_Prescaler_TIMER
     * 
     *  10^9 è per la scala temporale visto che lavoriamo in ns  e vogliamo fare il calcolo tramite 
     *          long int senza usare i float.
     * 
     *  Con encoder a 256cpr i tre valori sono:
     *  60/(256*25) * 10^9 = 9375000 // 1:1
     *  60/(64*25) * 10^9 = 37500000 // 1:4
     *  60/(16*25) * 10^9 = 150000000 // 1:16.
     *  
     *  Semplificando il calcolo ottengo:
     *  (60 * 10^9 * Pr) / (Er*Tr)
     *
     *  60 * 10^9 = 60000000000
     *
     *  => T_FattoreConversioneRPM_1 = (60000000000 * IC_PRESCALER_1) / ( ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_XXX] * T_Prescaler_TIMER )
     *
     */
    volatile long   T_FattoreConversioneRPM_1;
    volatile long   T_FattoreConversioneRPM_2;
    volatile long   T_FattoreConversioneRPM_3;

    /*  Variabili usate per le conversioni delle unità di misura*/
    volatile float  FL_Costante_Conversione_Vlin_to_Vang;
} Motor_t;

//! Struttura usata per la gestione dell'InputCapture e le misure di velocità
typedef struct
{   volatile long   OldMeasure;
    volatile int    ErrorCounter;
} InputCapture_t;

typedef struct
{   volatile unsigned int  Anomalie;    // Numero "Anomalie" registrate
    volatile long   LogginArea[1000][2];
//    volatile unsigned int Pointer;
//    volatile long int PosizioneAnomalia[10][10];    // Posizione - Valore
}   Debugger_t;


typedef struct
{   //volatile unsigned char  UC_PidSpeed;       // 0 = Pid a 1mSec, 1 = Pid a 10mSec
    //volatile unsigned char  UC_PidCounter;
    volatile long       Kp;
    volatile long       Ki;
    volatile long       Kd;

    volatile long       Integrale;
    
    volatile long       Sommatoria;     // Sommatoria di errore Proporzionale, Integrale, Derivativo

    volatile long       OldContrValue;  // Valore di controllo all'istante precedente

//    volatile long       Current;        // Dato corrente di velocità del motore... velocità che deve essere nello stesso
                                        //  tipo e ordine di grandezza del Setpoint
    volatile long       Setpoint;       // Dato di velocità da raggiungere e mantenere.
    volatile long       Rampa;          // Dato di velocità in regolazione per raggiungere il Setpoint in base alla Rampa.
    volatile long       RampaStep;      // Incremento ( ad ogni ciclo PID ) del dato di regolazione Rampa per raggiungere il SetPoint.

    volatile long       Errore;         // Errore Corrente
    volatile long       OldError1;      // Errore al T-1
    volatile long       OldError2;      // Errore al T-2


    volatile long int   ContributoProporzionale;
    volatile long int   ContributoIntegrale;
    volatile long int   ContributoDerivativo;

    volatile long       ComponenteFeedForward;

    volatile long       OutPid;
} Pid_t;




//! Struttura usata per gestire la segnalazione del codice di errore tramite i LED
//!
typedef struct
{   volatile unsigned char  UC_Busy;            // Se a "1" significa che è in corso una sequenza di segnalazione
    volatile unsigned char  UC_ErrorCode;
    volatile unsigned char  UC_Fase;            //Mi indica in che fase della segnalazione sono ( Pausa, LedOn, LedOff, FineSequenza )
    volatile unsigned char  UC_BlinkDoned;    // Conta il numero di blink eseguiti (Led On- Led Off )
    volatile unsigned char  UC_Repetition;  // Numero Ripetizioni, 0 = Infinito
    volatile unsigned char  UC_RepetitionDoned;   // Numero Ripetizioni eseguite
    volatile unsigned int   UI_Ton;
    volatile unsigned int   UI_Toff;
    volatile unsigned int   UI_Tpausa;
    volatile unsigned int   UI_Timer;
} Led_t;


struct Bits{
	unsigned bit0:1;
	unsigned bit1:1;
	unsigned bit2:1;
	unsigned bit3:1;
	unsigned bit4:1;
	unsigned bit5:1;
	unsigned bit6:1;
	unsigned bit7:1;
	unsigned bit8:1;
	unsigned bit9:1;
	unsigned bit10:1;
	unsigned bit11:1;
	unsigned bit12:1;
	unsigned bit13:1;
	unsigned bit14:1;
	unsigned bit15:1;
};

//Struttura per accedere ad un dato o come Float o come due Integer ( per salvare il float in EEPROM )
typedef struct
{   union
    {   struct
        {   unsigned int high_part;
            unsigned int low_part;
        };
        float fval;
   };
} fvalue;


void AggiornaDatiVelocita(void);
//void NavigationGuideMode(void);
//void ManualGuideMode(void);
void GestioneSetpoint(void);

void MotorControlEnable(unsigned char Motor, unsigned char Status);
void AggiornaVariabiliModbus(void);
void Usart1Setting(void);
void Usart2Setting(void);
void TxString(unsigned char *Punt, unsigned char NCar, unsigned char Port);

void Settings(void);

void ISR_Settings(void);
void _ISR _INT1Interrupt(void);
void _ISR _U1RXInterrupt(void);
void _ISR _U1TXInterrupt(void);
void _ISR _IC1Interrupt(void);
void _ISR _IC2Interrupt(void);
void _ISR _T1Interrupt(void);
void _ISR _T2Interrupt(void);
void _ISR _CNInterrupt(void);

//unsigned int modul(signed int x);
//int Vlin_to_Vang(long Vlin, int WheelRadius);
float Costante_Conversione_Vlin_to_Vang(unsigned int GearBoxRatio_AXE, unsigned int GearBoxRatio_Motor, unsigned int Wheel_Radius);


//TIMER SOFTWARE
void ReloadTimerSW(volatile Timer_t *Timer);
void GestioneTimerSW(volatile Timer_t *Timer);

//Modbus
unsigned int LeggiWord(unsigned int Address);
unsigned char ScriviWord(unsigned int Address,unsigned int Word);
unsigned char LeggiBit(unsigned int Address);
unsigned char ScriviBit(unsigned int Address,unsigned char Bit);

void ModbusRoutine(unsigned char Port);
void ModbusRxRoutine(unsigned char Code, unsigned char Port);
void ModbusReadBit(unsigned char Port);
void ModbusReadWord(unsigned char Port);
void ModbusWriteSingleBit(unsigned char Port);
void ModbusWriteSingleWord(unsigned char Port);
void ModbusWriteMultipleBits(unsigned char Port);
void ModbusWriteMultipleWords(unsigned char Port);
void ModbusErroreResponse(unsigned char NumeroErrore, unsigned char Port);
unsigned int ModbusCheckCRC16(unsigned char *P,unsigned char NByte);



void FreeRxBuffer(unsigned char Port);
void InizializzaSeriale(unsigned char Port);



// Gestioni analogiche
void InitADC(void);
int LeggiADC(int NumeroIngresso);
void MediaADC(unsigned int * OutputIngressiMediati, unsigned int DmaAdc[][SAMP_BUFF_SIZE]);
unsigned int TaraturaAnalogiche(unsigned int FlagTaratura);


// EPROM.C
void InitializationEEPROM(void);
void RestoreEEPROMData(void);


// Gestione PID
//void InitPid1(void);
//void InitPid2(void);
void Pid1(void);
void Pid2(void);
//void Pid1Calc (void);
//void Pid2Calc (void);
//void Pid_M1(void);
//void Pid_M2(void);
//void Pid(volatile Pid_t *PID);
void Pid(volatile Pid_t *PID, volatile Motor_t *MOTORE);
//void change_prescaler(volatile unsigned int *ICM, volatile Motor_t *Motore);
void InitMotorStructure();
void UpdateMotorStructure();

// Alarm.c
void GestioneWatchdog(void);
void GestioneSicurezzaMotore(void); //volatile Motor_t *MOTORE)

// Led.c
void GestioneAllarmi();
void SetLedErrorCode(   volatile Led_t *LED,
                        unsigned char ErrorCode, unsigned char Ripetizioni,
                        unsigned int Ton, unsigned int Toff, unsigned int Tpausa );
void GestioneLed1ErrorCode(volatile Led_t *LED); //, volatile unsigned int *PORT_LED );
void GestioneLed2ErrorCode(volatile Led_t *LED);

#endif
