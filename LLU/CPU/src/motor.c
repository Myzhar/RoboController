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

#define _MOTOR_FAIL MOTORE->UC_Fail

/*  FUNZIONE DA RICHIAMARE SOLO AL RESET
*/
void InitMotorStructure()
{


    Motore1.L_RpmConversion = Motore1.T_FattoreConversioneRPM_1;
    Motore2.L_RpmConversion = Motore2.T_FattoreConversioneRPM_1;

    Motore1.UC_OverFlowCounter = 1;
    Motore2.UC_OverFlowCounter = 1;

    Motore1.UC_First_IC_Interrupt_Done = 0;
    Motore2.UC_First_IC_Interrupt_Done = 0;
    Motore1.UC_IC_idx = 0;
    Motore2.UC_IC_idx = 0;
    Motore1.UC_ICM_Restart_Value = 3;
    Motore2.UC_ICM_Restart_Value = 3;

    Motore1.UC_MotorNumber = 1;     // Mi serve nelle funzioni a cui passo la struttura come argomento per
    Motore2.UC_MotorNumber = 2;     // sapere su che struttura sto lavorando

    Motore1.UC_Fail = 0;    //  I motori non sono in errore.
    Motore2.UC_Fail = 0;

    UpdateMotorStructure();
}

/* FUNZIONE DA RICHIAMARE OGNI VOLTA CHE VENGONO MODIFICATI DEI PARAMETRI STRUTTURALI DEL ROBOT
*/
void UpdateMotorStructure()
{
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
    Motore1.T_FattoreConversioneRPM_1 = (60000000000 * IC_PRESCALER_1) /
                                        (ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT] * Motore1.I_Prescaler_TIMER);
    Motore1.T_FattoreConversioneRPM_2 = (60000000000 * IC_PRESCALER_4) /
                                        (ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT] * Motore1.I_Prescaler_TIMER);
    Motore1.T_FattoreConversioneRPM_3 = (60000000000 * IC_PRESCALER_16) /
                                        (ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT] * Motore1.I_Prescaler_TIMER);

    Motore2.T_FattoreConversioneRPM_1 = (60000000000 * IC_PRESCALER_1) /
                                        (ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT] * Motore2.I_Prescaler_TIMER);
    Motore2.T_FattoreConversioneRPM_2 = (60000000000 * IC_PRESCALER_4) /
                                        (ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT] * Motore2.I_Prescaler_TIMER);
    Motore2.T_FattoreConversioneRPM_3 = (60000000000 * IC_PRESCALER_16) /
                                        (ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT] * Motore2.I_Prescaler_TIMER);
    
    Motore1.I_MotorRpmMax = ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT];
    Motore1.I_MotorRpmMin = ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT] * -1;

    Motore2.I_MotorRpmMax = ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT];
    Motore2.I_MotorRpmMin = ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT] * -1;
    
    Motore1.FL_Costante_Conversione_Vlin_to_Vang = Costante_Conversione_Vlin_to_Vang(  ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT],
                                                                                    ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT],
                                                                                    ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT]
                                                                                 );
    Motore2.FL_Costante_Conversione_Vlin_to_Vang = Costante_Conversione_Vlin_to_Vang(  ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT],
                                                                                    ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT],
                                                                                    ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT]
                                                                                 );    

}




/*
 *  Funzione da chiamare continuamente sotto Main, dopo aver
 *  richiamato GestioneWatchdog()
 *
 * Gestisce le sicurezze legate ai motori, si occupa di :
 * - verificare eventuali condizioni di allarme
 * - attivare/disattivare il driver del motore
 *
 */
void GestioneSicurezzaMotore(void) //volatile Motor_t *MOTORE)
{
    Motore1.UC_Fail ? MotorControlEnable(MOTORE1,MOTOR_DEACTIVE) : MotorControlEnable(MOTORE1,MOTOR_ACTIVE);
    Motore2.UC_Fail ? MotorControlEnable(MOTORE2,MOTOR_DEACTIVE) : MotorControlEnable(MOTORE2,MOTOR_ACTIVE);
}

/*! \brief Questa funzione abilita/disabilita il driver motori tenendo conto del flag modbus FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY
 * per determinarne la polarità, ovvero se il driver è attivo col pin a "1" o a "0".
 * Esempi:
 * MotorControlEnable(MOTORE1,MOTOR_ACTIVE);
 * MotorControlEnable(MOTORE2,MOTOR_ACTIVE);
 * MotorControlEnable(MOTORE1,MOTOR_DEACTIVE);
 * MotorControlEnable(MOTORE2,MOTOR_DEACTIVE);
 */
/*!
  \param Motor Indica su quale uscita motore agire, può valere MOTORE1 o MOTORE2
  \param Status Indica se attivare (MOTOR_ACTIVE) o disattivare (MOTOR_DEACTIVE) il motore selezionato.
  \return void
*/
void MotorControlEnable(unsigned char Motor, unsigned char Status)
{
    if( Motor == MOTORE1  )
    {   if ( Status == MOTOR_ACTIVE )
        {   // Attivo il motore 1
            if( ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] & FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY)
                MOTOR_ENABLE1 = 1;
            else
                MOTOR_ENABLE1 = 0;
         }
        else
        {   // Disattivo il motore 1
            if( ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] & FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY)
                MOTOR_ENABLE1 = 0;
            else
                MOTOR_ENABLE1 = 1;
         }
    }

    if( Motor == MOTORE2  )
    {   if ( Status == MOTOR_ACTIVE )
        {   // Attivo il motore 2
            if( ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] & FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY)
                MOTOR_ENABLE2 = 1;
            else
                MOTOR_ENABLE2 = 0;
         }
        else
        {   // Disattivo il motore 2
            if( ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] & FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY)
                MOTOR_ENABLE2 = 0;
            else
                MOTOR_ENABLE2 = 1;
         }
    }
}



