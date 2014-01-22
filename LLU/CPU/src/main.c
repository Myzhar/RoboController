 /*!
* \mainpage main.c
* \author   Mauro Soligo -->mauro.soligo@gmail.com<--
* \version 0.0.1
* \date 12/2011
* \details This is a software :)

-------------------------------------------------------------------------------

 \copyright 2011 Mauro Soligo
 mauro.soligo@gmail.com

 RoboController is free software derived from Guido Ottaviano dsPID33
 you can redistribute it and/or modify it under the terms of ......

-------------------------------------------------------------------------------
*/

unsigned char  Ver[] = "RoboController Test V1.0 Mauro Soligo 2011"; // 42+1 char

// standard include
#define VAR_INC     // Solo nel Main per definire le 

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
#include "macro.h"


unsigned int TestBreakpoint;
float a,b,c,d;
long int Valori[1000];
unsigned int IndiceValori = 0;

int main(int argc, char** argv)
{   
//    float SetpointRPM_M1, SetpointRPM_M2;
    Settings();

    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */
    /* Inizializzazione da EEPROM                                                                               */
    /* Recupero i dati salvati in EEPROM ed eseguo un controllo di correttezza su di essi, questo serve         */
    /* principalmente per il default dopo la programmazione                                                     */
    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */
    InitializationEEPROM();

    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */
    /* Inizializzazione variabili                                                                               */
    /*                                                                                                          */
    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */

    LED1 = LED_OFF;
    LED2 = LED_OFF;

    MotorControlEnable(MOTORE1,MOTOR_DEACTIVE);
    MotorControlEnable(MOTORE2,MOTOR_DEACTIVE);
    
    DmaBuffer = 0;
   
    VarModbus[INDICE_ENC1_TICK] = 0;
    VarModbus[INDICE_ENC2_TICK] = 0;
    VarModbus[INDICE_ENC1_PERIOD] = 0;
    VarModbus[INDICE_ENC2_PERIOD] = 0;
    VarModbus[INDICE_ENC1_SPEED] = 0;
    VarModbus[INDICE_ENC2_SPEED] = 0;
    //Motore1.L_WheelSpeed = 0;
    //Motore2.L_WheelSpeed = 0;
    
    VarModbus[INDICE_STATUSBIT1] = 0;
    //VarModbus[INDICE_STATUSBIT1] &= ~(FLG_STATUSBI1_JOYMODE);     // Al reset disattivo la modalità JoyStick
    //VarModbus[INDICE_STATUSBIT1] &= ~(FLG_STATUSBI1_PID_EN);      // Al reset disattivo la modalità PID
    VarModbus[INDICE_STATUSBIT1] |= FLG_STATUSBI1_PID_EN;           // Al reset attivo la modalità PID
    VarModbus[INDICE_STATUSBIT1] |= FLG_STATUSBI1_COMWATCHDOG;      // Al reset attivo il WatchDog sulla comunicazione
//    VarModbus[INDICE_STATUSBIT1] &= ~(FLG_STATUSBI1_COMWATCHDOG); // Al reset disattivo il WatchDog sulla comunicazione
    VarModbus[INDICE_STATUSBIT1] |= FLG_STATUSBI1_EEPROM_SAVE_EN;   // Al reset attivo il Salvataggio automatico dei parametri in EEPROM
    VarModbus[INDICE_FLAG_TARATURA] = 0;
    //VarModbus[INDICE_PWM_CH1] = MOTORE_STOP;                      // Motori fermi all'accensione
    //VarModbus[INDICE_PWM_CH2] = MOTORE_STOP;                      // Motori fermi all'accensione
    VarModbus[INDICE_PWM_CH1] = MOTORE_STOP_PID;                    // Motori fermi all'accensione
    VarModbus[INDICE_PWM_CH2] = MOTORE_STOP_PID;                    // Motori fermi all'accensione

    InitMotorStructure();
    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */

    /* Inizializzo i timer SW */
    Timer1mSec.T_flag = FALSE;
    Timer1mSec.T_initial_value = 1; // 1 = 1mSec
    Timer1mSec.T_count_time = Timer1mSec.T_initial_value;
    Timer10mSec.T_flag = FALSE;
    Timer10mSec.T_initial_value = 10; // 10 = 10mSec
    Timer10mSec.T_count_time = Timer10mSec.T_initial_value;

    /* Inizializzazione porte seriali */
    InizializzaSeriale(PORT_COM1);
    InizializzaSeriale(PORT_COM2);

    InitADC();

    MotorControlEnable(MOTORE1,MOTOR_ACTIVE);
    MotorControlEnable(MOTORE2,MOTOR_ACTIVE);

    //  TEST SEGNALAZIONI LED, sarà da eseguire dopo il controllo dei relativi FLAG...
    SetLedErrorCode( &Led1Segnalazione, LED_ERRORCODE_05_CHECKERRORIOK, 1, SEGNALAZIONELED_TON, SEGNALAZIONELED_TOFF, SEGNALAZIONELED_TPAUSE);
    SetLedErrorCode( &Led2Segnalazione, LED_POWERON_05_POWERTEST, 1, SEGNALAZIONELED_TON*2, SEGNALAZIONELED_TOFF*2, SEGNALAZIONELED_TPAUSE);

    ISR_Settings(); //  Configures and enables ISRs

//    /* DEBUG */
//    SetpointRPM_M1 = Motore1.FL_Costante_Conversione_Vlin_to_Vang * ((float)((int)(100)));
//    SetpointRPM_M2 = Motore2.FL_Costante_Conversione_Vlin_to_Vang * ((float)((int)(0)));
//    if(!Motore1.UC_Fail) PID1.Setpoint = (long)(SetpointRPM_M1);
//    if(!Motore2.UC_Fail) PID2.Setpoint = (long)(SetpointRPM_M2);
//    /* ***** */

    while(1)
    {   // ----------------------  Gestione protocollo ModBus ---------------------- //
        ModbusRoutine(PORT_COM1);
        ModbusRoutine(PORT_COM2);

        // -----------------  PID and speed calculation every 1ms ----------------- //
        // if(PID1_CALC_FLAG) Pid1(); // Ogni 1mSec ricalcola il prescaler, avvia un ciclo
        // if(PID2_CALC_FLAG) Pid2(); // di lettura dell'IC e esegue il PID sul dato prec.

        // ----------------------  Task eseguito ogni 1mSec  ---------------------- //
        if(Timer1mSec.T_flag)
        {   Timer1mSec.T_flag = FALSE;
            GestioneWatchdog();             //  GESTIONE WATCHDOG COMUNICAZIONE
            GestioneSicurezzaMotore();      //  Gestisco situazioni di FAIL dei motori
        }

        // ----------------------  Task eseguito ogni 10mSec  ---------------------- //
        if(Timer10mSec.T_flag)
        {   Timer10mSec.T_flag = FALSE;
//            GestioneWatchdog();             //  GESTIONE WATCHDOG COMUNICAZIONE
//            GestioneSicurezzaMotore();      //  Gestisco situazioni di FAIL dei motori
            GestioneLed1ErrorCode(&Led1Segnalazione);
            GestioneLed2ErrorCode(&Led2Segnalazione);
        }

        GestioneAllarmi();
        GestioneSetpoint();
        AggiornaDatiVelocita();
        AggiornaVariabiliModbus();
    }
    return (EXIT_SUCCESS);
}

/*! \brief Questa funzione prende il dato presente nei registri WORD_PWM_CH1 e WORD_PWM_CH2
 * e lo interpreta in relazione allo stato del bit FLG_STATUSBI1_PID_EN.
 *
 * FLG_STATUSBI1_PID_EN = 0    :
 * Il dato passato alla robocontroller è un puro PWM, con 2048 il relativo motore
 * è fermo, con 0 il motore gira a massima velocità in un senso e con 4096 nel senso opposto.
 *
 * FLG_STATUSBI1_PID_EN = 1    :
 * Nelle word WORD_PWM_CH1 e WORD_PWM_CH2 passo un dato espresso come velocità in mm/Sec.
 * con 32768 la velocità è pari a 0.
 * Se voglio mandare il robot a 1m/Sec dovrò scrivere 32768+1000 = 33768
 * Se voglio mandare il robot a -1m/Sec dovrò scrivere 32768-1000 = 31768
 *
 */
/*!
  \param void
  \return void
*/
void GestioneSetpoint(void)
{   float SetpointRPM_M1, SetpointRPM_M2;

    if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_PID_EN)
    {   //Funzionamento in modalità PID
        
        if(OLD_INDICE_STATUSBIT1 == 1) // Esco dalla modalità PID e passo a quella PWM
        {   OLD_INDICE_STATUSBIT1 = 0;
            // Riinizializzo le variabili
            // ...
        }

        /* **************************************************************** */
        /* ************ SONO IN MODALITA' PID, ROUTINE "MAIN"  ************ */
        /* **************************************************************** */
        // Converto Vlineare in RPM per il PID
        SetpointRPM_M1 = Motore1.FL_Costante_Conversione_Vlin_to_Vang * ((float)((int)VarModbus[INDICE_PWM_CH1]));
        SetpointRPM_M2 = Motore2.FL_Costante_Conversione_Vlin_to_Vang * ((float)((int)VarModbus[INDICE_PWM_CH2]));
    }
    else
    {   /* **************************************************************** */
        /* ************ SONO IN MODALITA' PWM, ROUTINE "MAIN"  ************ */
        /* **************************************************************** */

        // In modalità PWM il dato delle word INDICE_PWM_CHx lo mando direttamente 
        // al modulo PWM del micro perchè rappresenta già un PWM.
        SetDCMCPWM1(1, 2048 + (int)(VarModbus[INDICE_PWM_CH1]) , 0);                // setta il PWM  del motore
        SetDCMCPWM1(2, 2048 + (int)(VarModbus[INDICE_PWM_CH2]) , 0);                // setta il PWM  del motore
        VarModbus[INDICE_RD_PWM_CH1] = VarModbus[INDICE_PWM_CH1];       // aggiorno PWM in lettura
        VarModbus[INDICE_RD_PWM_CH2] = VarModbus[INDICE_PWM_CH2];       // aggiorno PWM in lettura
    }

    // A prescindere che sia in modalità PID o PWM chiamo sempre le funzioni PID
    // che mi servono in ogni caso per il calcolo dei dati di velocità.
    if(!Motore1.UC_Fail) PID1.Setpoint = (long)(SetpointRPM_M1);
    if(!Motore2.UC_Fail) PID2.Setpoint = (long)(SetpointRPM_M2);

}

/*---------------------------------------------------------------------------*/
/* Interrupt Service Routines                                                */
/*---------------------------------------------------------------------------*/
#ifndef TIMER_OFF

    //PIN_CN_IC2_4 = PIN_ON;
    //PIN_CN_IC2_4 = PIN_OFF;

void _ISR_PSV _T1Interrupt(void)	// Timer 1 [13]
{   // Timer 1	1ms
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IFS0bits.T1IF = 0;  // interrupt flag reset
    // cycle 0 actions

    /*
     * Gestione del calcolo del PID, due strade percorribili:
     * Setto dei FLAG PID1_CALC_FLAG e PID2_CALC_FLAG che poi
     * nel main intercetto.
     *
     * Eseguo direttamente il calcolo nell'interrupt a 1mSec
     * richiamando le funzioni Pid1() e Pid2().
     */
    
    //PID1_CALC_FLAG = 1;	// PID1 and speed calculation enabled
    //PID2_CALC_FLAG = 1;	// PID2 and speed calculation enabled

    Pid1(); // Ogni 1mSec ricalcola il prescaler, avvia un ciclo
    Pid2(); // di lettura dell'IC e esegue il PID sul dato prec.

    /* **************************** TIMER SOFTWARE ******************************/
    GestioneTimerSW(&Timer1mSec);
    GestioneTimerSW(&Timer10mSec);
    /* **************************************************************************/

    /* ******************************** MODBUS **********************************/
    if(TimerRitardoModbus[0])  TimerRitardoModbus[0]--;
    if(TimerOutRxModbus[0])    TimerOutRxModbus[0]--;     //time-out dei dati in ricezione
    if(TimerRitardoModbus[1])  TimerRitardoModbus[1]--;
    if(TimerOutRxModbus[1])    TimerOutRxModbus[1]--;     //time-out dei dati in ricezione
    /* ************************************************************************* */

    DISICNT = 0; //re-enable interrupts
}


//    void _ISR_PSV _T3Interrupt(void)
//    {   // Timer 1	1ms
//        _T3IF=0;   		// interrupt flag reset
//
//        /* ****************************  GESTIONE PID  ******************************/
//    #ifdef DEBUG
//        //LED1 = PIN_ON;  LED2 = PIN_ON;
//        //LED1 = PIN_OFF; LED2 = PIN_OFF; // T = 0 uSec, fronte di discesa per sincronizzarmi
//        //LED2 = PIN_ON;
//        //LED2 = ~LED2;
//    #endif
//
//        //__builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
//
//        /* **************************** TIMER SOFTWARE ******************************/
//        GestioneTimerSW(&Timer1mSec);
//        GestioneTimerSW(&Timer10mSec);
//        /* **************************************************************************/
//
//        /* ******************************** MODBUS **********************************/
//        if(TimerRitardoModbus[0])  TimerRitardoModbus[0]--;
//        if(TimerOutRxModbus[0])    TimerOutRxModbus[0]--;     //time-out dei dati in ricezione
//        if(TimerRitardoModbus[1])  TimerRitardoModbus[1]--;
//        if(TimerOutRxModbus[1])    TimerOutRxModbus[1]--;     //time-out dei dati in ricezione
//        /* ************************************************************************* */
//
//        //DISICNT = 0; //re-enable interrupts
//
//
//    #ifdef DEBUG
//        //LED2 = PIN_OFF;
//        //LED1 = PIN_OFF;  LED2 = PIN_ON;
//        //LED2 = ~LED2;
//    #endif
//
//    }

//    // Timer 2 overflow
//    // void _ISR_PSV _T2Interrupt(void) {
//    void __attribute__((interrupt, auto_psv, shadow)) _T2Interrupt(void) {
//        unsigned char i;
//        //LED1 = PIN_ON;  // DEBUG
//        __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
//
//        IFS0bits.T2IF = 0;
//
//        Motore1.UC_OverFlowCounter++;
//        Motore2.UC_OverFlowCounter++;
//
//        // reset Vel M1
//        if (Motore1.UC_OverFlowCounter > 4)
//        {   //Motore1.UC_OverFlowErrorCounter++;
//            Motore1.UC_OverFlowCounter=4;
//            Motore1.I_MotorAxelSpeed=0;
//            for (i=0;i<8;i++)
//            {   Motore1.UI_MediaIC[i] = 0;
//            }
//        }
//
//        //reset Vel M2
//        if (Motore2.UC_OverFlowCounter > 4)
//        {   //Motore2.UC_OverFlowErrorCounter++;
//            Motore2.UC_OverFlowCounter=4;
//            Motore2.I_MotorAxelSpeed=0;
//            for (i=0;i<8;i++)
//            {   Motore2.UI_MediaIC[i] = 0;
//            }
//        }
//        //LED1 = PIN_OFF; // DEBUG
//
//
//        DISICNT = 0; //re-enable interrupts
//        return;
//    }

// Timer 2 overflow
// void _ISR_PSV _T2Interrupt(void) {
void __attribute__((interrupt, auto_psv, shadow)) _T2Interrupt(void) {
    unsigned char i;
//    PIN_CN_IC2_4 = PIN_ON;
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IFS0bits.T2IF = 0;

    Motore1.UC_OverFlowCounter++;
    // reset Vel M1
    if (Motore1.UC_OverFlowCounter > 4)
    {   //Motore1.UC_OverFlowErrorCounter++;
        Motore1.UC_OverFlowCounter=4;
        Motore1.I_MotorAxelSpeed=0;
        for (i=0;i<8;i++)
        {   Motore1.UI_MediaIC[i] = 0;
        }
    }

//    PIN_CN_IC2_4 = PIN_OFF;
    DISICNT = 0; //re-enable interrupts
    return;
}

// Timer 3 overflow
// void _ISR_PSV _T3Interrupt(void) {
void __attribute__((interrupt, auto_psv, shadow)) _T3Interrupt(void) {
    unsigned char i;
    //LED1 = PIN_ON;  // DEBUG
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IFS0bits.T3IF = 0;
    Motore2.UC_OverFlowCounter++;

    //reset Vel M2
    if (Motore2.UC_OverFlowCounter > 4)
    {   //Motore2.UC_OverFlowErrorCounter++;
        Motore2.UC_OverFlowCounter=4;
        Motore2.I_MotorAxelSpeed=0;
        for (i=0;i<8;i++)
        {   Motore2.UI_MediaIC[i] = 0;
        }
    }
    
    //LED1 = PIN_OFF; // DEBUG
    DISICNT = 0; //re-enable interrupts
    return;
}


#endif

/*  ***************************************************************************
 *  ***************************************************************************
 *  ***************************************************************************
 */


//  PIN_CN_IC2_6
//  PIN_CN_IC2_5
//  PIN_CN_IC2_4

//    InputCapture1.ErrorCounter;
//    InputCapture1.OldMeasure;
// void _ISR_PSV _IC1Interrupt(void) {
void __attribute__((interrupt, auto_psv, shadow)) _IC1Interrupt(void) {
    //  PIN_CN_IC2_5 = PIN_ON;      // DEBUG
    //  PIN_CN_IC2_5 = PIN_OFF;     // DEBUG
    //  PIN_CN_IC2_6 = PIN_ON;      // DEBUG
    //  PIN_CN_IC2_6 = PIN_OFF;     // DEBUG
    //  LED1 = PIN_ON;              // DEBUG
    long int tmp = 0;
    unsigned char i;
    unsigned int ActualIC1BUF;
    
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IFS0bits.IC1IF = 0;

    ActualIC1BUF = IC1BUF;
    
    if (Motore1.UC_First_IC_Interrupt_Done == 0)
    {   Motore1.UI_Old_Capture = ActualIC1BUF;  // 1st interrupt, acquire start time
        Motore1.UC_OverFlowCounter = 0;         // reset overflow
        Motore1.UC_First_IC_Interrupt_Done = 1; // next interrupt valid acquire
        Motore1.UC_OverFlowCounter = 0;         // reset overflow
    }
    else
    {   // 2nd interrupt
        tmp  = TMR2_VALUE;

//        /* WARNING : pezo di codice da verificare meglio */
//        if((Motore1.UC_OverFlowCounter <= 0) && IFS0bits.T2IF && (ActualIC1BUF < Motore1.UI_Old_Capture ))
//        {   IFS0bits.T2IF = 0;
//            Motore1.UC_OverFlowCounter++;
//        }
        /* ********************************************** */
        tmp *= Motore1.UC_OverFlowCounter;  // overflow offset
        tmp += ActualIC1BUF;                // capture
        tmp -= Motore1.UI_Old_Capture;      // click period

/*
 * FILTRO MISURE ERRATE : Inizio
 */

        /* FILTRO MISURE ERRATE PER ERRORI LEGATI A SOVRAPPOSIZIONE DI INTERRUPT
         * Nel caso l'evento del timerOverflow avvenga nello stesso istante
         * in cui vi è il fronte di salita del segnale da misurare ( ENCODER )
         * può essere conteggiato in modo errato il numero di Overflow.
         *
         * Provvisoriamente introduciamo un filtro che di fatto intercetta
         * se la nuova misura discosta dalla precedente per una quantità
         * prossima 0xFFFF.
         * Non possiamo usare oxFFFF perchè tra due misure consecutive, a causa
         * dell'accelerazione o decelerazione del motore, vi sarà comunque una
         * certa differenza il cui risultato è sommato al valore errato di Overflow.
         *
         * Se per 3 misure consecutive il dato misurato discosta dal primo dato
         * campionato aggiorno il filtro al nuovo dato.
         * Serve nella malaugurata ipotesi che andassimo a campionare come primo
         * dato un dato errato :)
         * */


        /* Mantengo aggiornato uno storico degli ultimi mille campioni per analisi
         *  -> TestInputCapture1.Anomalie è l'indice 
         *  -> TestInputCapture1.LogginArea[x][0] : Valore della misura letta
         *  -> TestInputCapture1.LogginArea[x][1] : Valore restituito dal filtro nello stesso istante
         */

        if(TestInputCapture1.Anomalie > (LOGSIZE - 1) )  TestInputCapture1.Anomalie = 0;
        else TestInputCapture1.Anomalie++ ;
        TestInputCapture1.LogginArea[TestInputCapture1.Anomalie][0] = tmp; // Registro dato calcolato



        /*      FILTRO:
         *      se una misura è molto diversa da quella precedente il filtro ritorna quella precedente per
         *      un massimo di 3 cicli.
         *      Se la nuova misura "Errata" rimane stabile per 3 cicli allora il filtro si aggiorna,
         *      restituisce la nuova misura e prende questo valore come riferimento.
         */
        if( ((tmp > InputCapture1.OldMeasure+60000) || (tmp < InputCapture1.OldMeasure-60000)) &&
            (InputCapture1.ErrorCounter < 3) )
        {   // Conto quanti errori avvengono consecutivamente
            // oltre 3 errori consecutivi significa che la misura è stabile e aggiorno
            // il filtro al nuovo valore
            InputCapture1.ErrorCounter++;

            /* Per il momento tengo buona la misura precedente. */
            tmp = InputCapture1.OldMeasure;

            //PIN_CN_IC2_5 ^= PIN_ON;      // DEBUG

        }
        else
        {
            InputCapture1.OldMeasure = tmp;
            InputCapture1.ErrorCounter = 0;
        }

        /* -> TestInputCapture1.LogginArea[x][1] : Valore restituito dal filtro nello stesso istante */
        TestInputCapture1.LogginArea[TestInputCapture1.Anomalie][1] = tmp; // Registro dato restituito dal filtro

        /*  In TestInputCapture1.LogginArea mi trovo le ultime lille musure e i relativi valori filtrati restituiti dal 
         *  filtro in ogni istante.
         */
/*
 * FILTRO MISURE ERRATE : Fine
 */

        Motore1.UI_Period = (unsigned int)((long)Motore1.L_RpmConversion/tmp);

        Motore1.UI_MediaIC[Motore1.UC_IC_idx] = Motore1.UI_Period;
        Motore1.UC_IC_idx++;
        if(Motore1.UC_IC_idx > 7) Motore1.UC_IC_idx = 0;

        // media mobile
        tmp = 0;
        for (i=0;i<8;i++)   tmp += Motore1.UI_MediaIC[i]; // Sommatoria degli 8 campioni

        Motore1.I_MotorAxelSpeed = __builtin_divud(tmp,8);

        // CCW or CW
        // Deve essere posizionato dopo il cambio prescaler per evitare di dover gestire il segno.
        if (!QEI1CONbits.UPDN)
        {   Motore1.I_MotorAxelSpeed *= -1;
        }

        Motore1.UC_First_IC_Interrupt_Done = 0;

        IC1CONbits.ICM = 0; // Disable Input Capture 1 module,
                        // re-enabled after PID computation on 1mSec Interrupt Timer
        
    }
    DISICNT = 0; //re-enable interrupts
    Motore1.UC_OverFlowCounter = 0;         // reset overflow
}

// void _ISR_PSV _IC2Interrupt(void) {
void __attribute__((interrupt, auto_psv, shadow)) _IC2Interrupt(void) {
    long int tmp = 0;
    unsigned char i;
    unsigned int ActualIC2BUF;

    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IFS0bits.IC2IF = 0;

    ActualIC2BUF = IC2BUF;

    if (Motore2.UC_First_IC_Interrupt_Done == 0)
    {   Motore2.UI_Old_Capture = ActualIC2BUF;        // 1st interrupt, acquire start time
        Motore2.UC_OverFlowCounter = 0;         // reset overflow
        Motore2.UC_First_IC_Interrupt_Done = 1; // next interrupt valid acquire
        //return;
    }
    else
    {
    // 2nd interrupt
    //tmp  = TMR2_VALUE + 1;
    tmp  = TMR2_VALUE;
    tmp *= Motore2.UC_OverFlowCounter; // overflow offset
    tmp += IC2BUF;   // capture
    tmp -= Motore2.UI_Old_Capture; // click period

    Motore2.UI_Period = (unsigned int)((long)Motore2.L_RpmConversion/tmp);    // Valore istantaneo di periodo.

    Motore2.UI_MediaIC[Motore2.UC_IC_idx] = Motore2.UI_Period;
    Motore2.UC_IC_idx++;
    if(Motore2.UC_IC_idx > 7) Motore2.UC_IC_idx = 0;

    // media mobile
    tmp = 0;
    for (i=0;i<8;i++)   tmp += Motore2.UI_MediaIC[i];   // Sommatoria degli 8 campioni

    Motore2.I_MotorAxelSpeed = __builtin_divud(tmp,8);

    // CCW or CW
    // Deve essere posizionato dopo il cambio prescaler per evitare di dover gestire il segno.
    if (!QEI2CONbits.UPDN)
    {   Motore2.I_MotorAxelSpeed *= -1;
    }

    Motore2.UC_First_IC_Interrupt_Done = 0;

    IC2CONbits.ICM = 0; // Disable Input Capture 1 module,
                        // re-enabled after PID computation on 1mSec Interrupt Timer
        }
    DISICNT = 0; //re-enable interrupts
    Motore1.UC_OverFlowCounter = 0;         // reset overflow
//
//    DISICNT = 0; //re-enable interrupts
//    return;
}


//    // void _ISR_PSV _IC1Interrupt(void) {
//    void __attribute__((interrupt, auto_psv, shadow)) _IC1Interrupt(void) {
//        long int tmp = 0;
//        unsigned char i;
//        unsigned int ActualIC1BUF;
//
//        IFS0bits.IC1IF = 0;
//        ActualIC1BUF = IC1BUF;
//        LED2 = PIN_ON; // DEBUG
//
//        if (Motore1.UC_First_IC_Interrupt_Done == 0)
//        {   //LED1 = PIN_ON;  // DEBUG
//            Motore1.UI_Old_Capture = ActualIC1BUF;        // 1st interrupt, acquire start time
//            Motore1.UC_OverFlowCounter = 0;         // reset overflow
//            Motore1.UC_First_IC_Interrupt_Done = 1; // next interrupt valid acquire
//            //LED1 = PIN_OFF;  // DEBUG
//            return;
//        }
//
//        // 2nd interrupt
//        tmp  = TMR2_VALUE + 1;
//        tmp *= Motore1.UC_OverFlowCounter;  // overflow offset
//        tmp += ActualIC1BUF;                // capture
//        tmp -= Motore1.UI_Old_Capture;      // click period
//
//
//        if(     (Motore1.UC_OverFlowCounter == 0) &&  ( ActualIC1BUF < Motore1.UI_Old_Capture ) )
//        {
//            //LED2 = PIN_ON;  // DEBUG
//            LED1 = PIN_ON;  // DEBUG
//        }
//
//        Motore1.UI_Period = (unsigned int)((long)Motore1.L_RpmConversion/tmp);
//
//        Motore1.UI_MediaIC[Motore1.UC_IC_idx] = Motore1.UI_Period;
//        Motore1.UC_IC_idx++;
//        if(Motore1.UC_IC_idx > 7) Motore1.UC_IC_idx = 0;
//
//        // media mobile
//        tmp = 0;
//        for (i=0;i<8;i++)   tmp += Motore1.UI_MediaIC[i]; // Sommatoria degli 8 campioni
//
//        Motore1.I_MotorAxelSpeed = __builtin_divud(tmp,8);
//
//        // CCW or CW
//        // Deve essere posizionato dopo il cambio prescaler per evitare di dover gestire il segno.
//        if (!QEI1CONbits.UPDN)
//        {   Motore1.I_MotorAxelSpeed *= -1;
//        }
//
//        Motore1.UC_First_IC_Interrupt_Done = 0;
//
//        IC1CONbits.ICM = 0; // Disable Input Capture 1 module,
//                            // re-enabled after PID computation on 1mSec Interrupt Timer
//
//        LED1 = PIN_OFF; // DEBUG
//        LED2 = PIN_OFF; // DEBUG
//        return;
//    }
//
//    // void _ISR_PSV _IC2Interrupt(void) {
//    void __attribute__((interrupt, auto_psv, shadow)) _IC2Interrupt(void) {
//        long int tmp = 0;
//        unsigned char i;
//        unsigned int ActualIC2BUF;
//
//        IFS0bits.IC2IF = 0;
//        ActualIC2BUF = IC2BUF;
//
//        if (Motore2.UC_First_IC_Interrupt_Done == 0)
//        {   Motore2.UI_Old_Capture = ActualIC2BUF;        // 1st interrupt, acquire start time
//            Motore2.UC_OverFlowCounter = 0;         // reset overflow
//            Motore2.UC_First_IC_Interrupt_Done = 1; // next interrupt valid acquire
//            return;
//        }
//
//        // 2nd interrupt
//        tmp  = TMR2_VALUE + 1;
//        tmp *= Motore2.UC_OverFlowCounter; // overflow offset
//        tmp += IC2BUF;   // capture
//        tmp -= Motore2.UI_Old_Capture; // click period
//
//        Motore2.UI_Period = (unsigned int)((long)Motore2.L_RpmConversion/tmp);    // Valore istantaneo di periodo.
//
//        Motore2.UI_MediaIC[Motore2.UC_IC_idx] = Motore2.UI_Period;
//        Motore2.UC_IC_idx++;
//        if(Motore2.UC_IC_idx > 7) Motore2.UC_IC_idx = 0;
//
//        // media mobile
//        tmp = 0;
//        for (i=0;i<8;i++)   tmp += Motore2.UI_MediaIC[i];   // Sommatoria degli 8 campioni
//
//        Motore2.I_MotorAxelSpeed = __builtin_divud(tmp,8);
//
//        // CCW or CW
//        // Deve essere posizionato dopo il cambio prescaler per evitare di dover gestire il segno.
//        if (!QEI2CONbits.UPDN)
//        {   Motore2.I_MotorAxelSpeed *= -1;
//        }
//
//        Motore2.UC_First_IC_Interrupt_Done = 0;
//
//        IC2CONbits.ICM = 0; // Disable Input Capture 1 module,
//                            // re-enabled after PID computation on 1mSec Interrupt Timer
//
//        return;
//    }

void AggiornaDatiVelocita(void)
{   long tmp;

    /*
     *  RPMruota = RPMmotore * ( RapportoRiduzioneMotore:RapportoRiduzioneAsse)
     *
     *  MotoreX.I_MotorAxelSpeed    :	RPM asse motore, un motore "standard" ruota nel range 0-10000RPM, il dato è un intero.
     *  MotoreX.I_GearAxelSpeed     :	RPM uscita riduttore, considerando un rapporto riduzione minimo di 1:100 per motori da
     * 					10000RPM e di 1:10 per motori sotto i 3000RPM il dato varierà al max nel range 0-300
     *					**** Il dato GearAxelSpeed è un intero moltiplicato per un fattore 100		******
     *					**** Range 0-30000 con due cifre decimali in visualizzazione			******
     * I calcoli vengono effettuati passando per un LONG per evitare problemi di overflow nelle moltiplicazioni tra interi
    */
    tmp = __builtin_mulss(Motore1.I_MotorAxelSpeed,ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT]);
    tmp *= 100;   // Per ottenere I_GearAxelSpeed riscalato di un fattore 100
    tmp = __builtin_divsd(tmp,ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT]);
    Motore1.I_GearAxelSpeed = (int)tmp;

    tmp = __builtin_mulss(Motore2.I_MotorAxelSpeed,ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT]);
    tmp *= 100;   // Per ottenere I_GearAxelSpeed riscalato di un fattore 100
    tmp = __builtin_divsd(tmp,ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT]);
    Motore2.I_GearAxelSpeed = (int)tmp;

    /* Calcolo la velocità di rotazione delle ruote espressa in centesimi di radianti al secondo
     * MotoreX.L_WheelSpeed deriva da I_GearAxelSpeed ed è ritornato alla GUI moltiplicato per 100
     *
     * EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_xxxx : Raggio espresso in decimi di millimetro 360 = 3,6Cm
     *  */
    tmp = __builtin_mulss(ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT],Motore1.I_GearAxelSpeed);
    tmp *= TWOPI;
    tmp = __builtin_divsd(tmp,6000);
    Motore1.L_WheelSpeed = tmp;

    tmp = __builtin_mulss(ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT],Motore2.I_GearAxelSpeed);
    tmp *= TWOPI;
    tmp = __builtin_divsd(tmp,6000);
    Motore2.L_WheelSpeed = tmp;
}

void AggiornaVariabiliModbus(void)
{
    /* **************************************************************** */
    /* Aggiorno tutte le variabili per il protocollo modbus, le variabili
     * modbus vengono lette al max ogni 10mSec e quindi posso aggiornarle
     * con calma
     *                                                                  */
    /* **************************************************************** */

    // ----------------------  ADC value average calculus ----------------------//
    VarModbus[INDICE_TENSIONE_ALIM] = LeggiADC( PIC_AN0 ); // AN0 del micro = VMOT
    VarModbus[INDICE_AN1] = LeggiADC( PIC_AN1 ); // AN1 del PCB; AN5 del micro
    VarModbus[INDICE_AN2] = LeggiADC( PIC_AN2 ); // AN2 del PCB; AN5 del micro
    VarModbus[INDICE_AN3] = LeggiADC( PIC_AN3 ); // AN3 del PCB; AN6 del micro
    VarModbus[INDICE_AN4] = LeggiADC( PIC_AN4 ); // AN4 del PCB; AN7 del micro

    /*
     *     NOTA RELAZIONI PERIFERICHE e VARIABILI
     *     TIMER2 <=> IC1 <=> QEI1 <=> LEFT <=> 1
     *     TIMER3 <=> IC2 <=> QEI2 <=> RIGHT <=> 0
     */

    VarModbus[INDICE_PID_ERROR_RIGHT] = (int)PID1.Errore;
    VarModbus[INDICE_PID_ERROR_LEFT] = (int)PID2.Errore;

//    VarModbus[INDICE_ENC1_SPEED] = Motore1.I_MotorAxelSpeed;
//    VarModbus[INDICE_ENC2_SPEED] = Motore2.I_MotorAxelSpeed;

    /* RITORNO LA VELOCITA' ESPRESSA IN mm/Sec A PARTIRE DALLA VELOCITA' ANGOLARE DEL MOTORE*/
    VarModbus[INDICE_ENC1_SPEED] = (int) ( (float)Motore1.I_MotorAxelSpeed / Motore1.FL_Costante_Conversione_Vlin_to_Vang);
    VarModbus[INDICE_ENC2_SPEED] = (int) ( (float)Motore2.I_MotorAxelSpeed / Motore2.FL_Costante_Conversione_Vlin_to_Vang);


    VarModbus[INDICE_ENC1_PERIOD] = Motore1.I_MotorAxelSpeed;
    VarModbus[INDICE_ENC2_PERIOD] = Motore2.I_MotorAxelSpeed;
//    VarModbus[INDICE_ENC1_TICK] =
//    VarModbus[INDICE_ENC2_TICK] =

    VarModbus[INDICE_DEBUG_00] = Motore1.I_Prescaler_IC;    //Motore1.UI_Period;
    VarModbus[INDICE_DEBUG_01] = Motore2.I_Prescaler_IC;    //Motore2.UI_Period;
    VarModbus[INDICE_DEBUG_02] = Motore1.I_MotorAxelSpeed;
    VarModbus[INDICE_DEBUG_03] = Motore2.I_MotorAxelSpeed;
    VarModbus[INDICE_DEBUG_04] = Motore1.I_GearAxelSpeed;
    VarModbus[INDICE_DEBUG_05] = Motore2.I_GearAxelSpeed;
    VarModbus[INDICE_DEBUG_06] = Motore1.L_WheelSpeed;
    VarModbus[INDICE_DEBUG_07] = Motore2.L_WheelSpeed;

}


/*  Data:
 *              Vangolare = K * Vlineare
 *              Vlineare = Vangolare / K
 *
 *  questa funzione mi ritorna il valore di K e va richiamata solo quando modifico i 
 *  parametri e all'accensione della RoboController.
 *
 *  Vedi funzioni:
 *  -   void UpdateMotorStructure()
 *  -   void InitMotorStructure()
 *
 *  La funzione mi ritorna un dato di tipo Float per semplificare i calcoli.
 */

float Costante_Conversione_Vlin_to_Vang(unsigned int GearBoxRatio_AXE, unsigned int GearBoxRatio_Motor, unsigned int Wheel_Radius)
{   float Numeratore;
    float Denominatore;
    float K;

    /*      Costanti nei calcoli:
     *      PI      =       3.1415926535897931159979634685441851615905761718750 definito in dsp.h
     *      60      =       fattore di conversione da minuti a secondi (dato della velocità a RPM)
     *      100000  =       fattore di conversione da metri a centesimi di mm, ovvero dalla velocità
     *                      espressa in m/Sec all'unità di misura per la ruota ( Wheel_Radius).
     *                      Ovviamente vale 1000 se la velocità ci arriva in cm/S.
     *                      Ovviamente vale 100 se la velocità ci arriva in  mm/S.
     *
     *      Essendo la velocità passata alla robocontroller come intero nel range +/- 32768 e accettando come
     *      "limite" una velocità di 32,768 m/Sec (117.9648 Km/h) la Vlineare che arriva dal protocollo modbus
     *      è da considerarsi espressa in millimetri/Secondo.
     *
     *      Premesso questo la formula di conversione diventa:
     *
     *          K=(GearBoxRatio_Motor * 60 * 100)/(GearBoxRatio_AXE * Wheel_Radius * 2 * PI)
     */

    Numeratore = (float)GearBoxRatio_Motor  * 60 * 100;

    Denominatore    = (float)GearBoxRatio_AXE * Wheel_Radius * 2 * PI;
    
    K = Numeratore / Denominatore;
    return K;
}



