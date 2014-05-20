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

#include "limits.h"

// Macro per la semplificazione della lettura del codice
//#define _RAMPA                  PID->Rampa
//#define _STEP_RAMPA             PID->RampaStep
//#define _SETPOINT               PID->Setpoint
//#define _INTEGRALE              PID->Integrale

//#define _KP                     PID->Kp
//#define _KI                     PID->Ki
//#define _KD                     PID->Kd

//#define _CONTR_INTEGRALE        PID->ContributoIntegrale
//#define _CONTR_PROPORZIONALE    PID->ContributoProporzionale
//#define _CONTR_DERIVATIVO       PID->ContributoDerivativo

//#define _ERRORE                 PID->Errore
//#define _ERRORE_PRECEDENTE      PID->OldError1
//#define _ERRORE_PRE_PRECEDENTE  PID->OldError2

//#define _SOMMATORIA             PID->Sommatoria
//#define _OUT_PREC               PID->OldContrValue

//#define _VALORE_ATTUALE         PID->Current

//#define _COMPONENTE_FEEDFORWARD PID->ComponenteFeedForward
//#define _OUT                    PID->OutPid


//#define _MAX_RPM                MOTORE->I_MotorRpmMax
//#define _MIN_RPM                MOTORE->I_MotorRpmMin

//#define _AXELSPEED              MOTORE->I_MotorAxelSpeed

void Pid1(void)
{ //__builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles


    PID1_CALC_FLAG = 0; // Attivato sotto interrupt ogni 1mSec

    /*  *************************************************************************** */
    // Calcolo il prescaler per la prossima misura
    if (Motore1.I_MotorAxelSpeed < 1800) // UI_Period
    {
        Motore1.UC_ICM_Restart_Value = 0b011;
        Motore1.I_Prescaler_IC = 1; //  bit2-0:     Generate capture event on every 1st rising
        Motore1.L_RpmConversion = Motore1.T_FattoreConversioneRPM_1; // every rising edge
    }
    if (Motore1.I_MotorAxelSpeed > 2200 && Motore1.I_MotorAxelSpeed < 3800)
    {
        Motore1.UC_ICM_Restart_Value = 0b100;
        Motore1.I_Prescaler_IC = 4; //  bit2-0:     Generate capture event on every 4th rising
        Motore1.L_RpmConversion = Motore1.T_FattoreConversioneRPM_2; // every 4th rising edge
    }
    if (Motore1.I_MotorAxelSpeed > 4200)
    {
        Motore1.UC_ICM_Restart_Value = 0b101;
        Motore1.I_Prescaler_IC = 16; //  bit2-0:     Generate capture event on every 16th rising
        Motore1.L_RpmConversion = Motore1.T_FattoreConversioneRPM_3; // every 16th rising edge
    }

    //    Motore1.UC_ICM_Restart_Value = 0b100;
    //    Motore1.I_Prescaler_IC = 4;  //  bit2-0:     Generate capture event on every 4th rising
    //    Motore1.L_RpmConversion = Motore1.T_FattoreConversioneRPM_2 ; // every 4th rising edge
    /*  *************************************************************************** */

    if (VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_PID_EN)
    { // In modalità PID calcolo il PID altrimenti sono in modalità PWM e chiamo questa funzione solo per i calcoli della velocità.
        Pid(&PID1, &Motore1); // T = 6.9uSec MEDI misurati (5.8 typ )
        SetDCMCPWM1(1, PID1.OutPid, 0); // setta il PWM  del motore
        VarModbus[INDICE_RD_PWM_CH1] = PID1.OutPid; // aggiorno PWM in lettura
    }
    IC1CONbits.ICM = Motore1.UC_ICM_Restart_Value;
}

void Pid2(void)
{
    PID2_CALC_FLAG = 0; // Attivato sotto interrupt ogni 1mSec
    /*  *************************************************************************** */
    // Calcolo il prescaler per la prossima misura
    if (Motore2.I_MotorAxelSpeed < 1800) // UI_Period
    {
        Motore2.UC_ICM_Restart_Value = 0b011;
        Motore2.I_Prescaler_IC = 1; //  bit2-0:     Generate capture event on every 1st rising
        Motore2.L_RpmConversion = Motore2.T_FattoreConversioneRPM_1; // every rising edge
    }
    if (Motore2.I_MotorAxelSpeed > 2200 && Motore2.I_MotorAxelSpeed < 3800)
    {
        Motore2.UC_ICM_Restart_Value = 0b100;
        Motore2.I_Prescaler_IC = 4; //  bit2-0:     Generate capture event on every 4th rising
        Motore2.L_RpmConversion = Motore2.T_FattoreConversioneRPM_2; // every 4th rising edge
    }
    if (Motore2.I_MotorAxelSpeed > 4200)
    {
        Motore2.UC_ICM_Restart_Value = 0b101;
        Motore2.I_Prescaler_IC = 16; //  bit2-0:     Generate capture event on every 16th rising
        Motore2.L_RpmConversion = Motore2.T_FattoreConversioneRPM_3; // every 16th rising edge
    }

    //    Motore2.UC_ICM_Restart_Value = 0b100;
    //    Motore2.I_Prescaler_IC = 4;  //  bit2-0:     Generate capture event on every 4th rising
    //    Motore2.L_RpmConversion = Motore2.T_FattoreConversioneRPM_2 ; // every 4th rising edge
    /*  *************************************************************************** */

    if (VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_PID_EN)
    { // In modalità PID calcolo il PID altrimenti sono in modalità PWM e chiamo questa funzione solo per i calcoli della velocità.
        Pid(&PID2, &Motore2); // T = 6.9uSec MEDI misurati (5.8 typ )
        SetDCMCPWM1(2, PID2.OutPid, 0); // setta il PWM  del motore
        VarModbus[INDICE_RD_PWM_CH2] = PID2.OutPid; // aggiorno PWM in lettura
    }
    IC2CONbits.ICM = Motore2.UC_ICM_Restart_Value;
}

void Pid(volatile Pid_t *PID, volatile Motor_t *MOTORE)
{
    long L_ScaledSetpoint = PID->Setpoint; // Dato da mantenere/raggiungere ( velocità di crociera ) moltiplicato per 1000
    long L_ScaledProcesso = MOTORE->I_MotorAxelSpeed; // Dato istantaneo ( velocità istantanea ) moltiplicato per 1000

    //    __builtin_disi(0x3FFF); /* disable interrupts, vedere pg 181 di MPLAB_XC16_C_Compiler_UG_52081.pdf */

    /*
     *  PWM varia da 0 a 4095 con centro a 2048, modalita LAP
     *  Tutti i calcoli sono a tre decimali, interi moltiplicati per 1000
     *
     *  _SETPOINT è la velocità in RPM dell'asse motore da raggiungere
     *  _AXELSPEED è la velocità in RPM istantanea dell'asse motore.
     *  
     * Tutto quello che riguarda il riduttore e la ruota viene calcolato a parte.
     *  
     * Da _SETPOINT e _AXELSPEED derivo:
     *      L_ScaledSetpoint  :   Velore di velocità da raggiungere, dato moltiplicato per 1000
     *      L_ScaledProcesso  :   Velore di velocità istantaneo, dato moltiplicato per 1000
     */


    /* ***************************************************************************************
     *  ************************************   __builtin_  ************************************
     *  ***************************************************************************************
     *  I calcoli sono ottimizzati mediante l'utilizzo delle funzioni __builtin_
     *  descritte nel documento "MPLAB_XC16_C_Compiler_UG_52081.pdf" presente nella directory
     *  del compilatore XC16
     *
     *  __builtin_mulss   :   MPLAB_XC16_C_Compiler_UG_52081.pdf pg294
     *  ***************************************************************************************
     */

    int rescaleFact = 10;

    int saturazione; // Indica se il controllo è in saturazione.
    // Da usare per l'anti-windup

    if ((PID->OldContrValue == 4095) || PID->OldContrValue == 1) // Il controllo è saturo
        saturazione = 1;
    else
        saturazione = 0;

    // Rampa è il dato effettivo di "Setpoint" da raggiungere in ciascun ciclo.
    // Tende a raggiungere il valore di SetPoint in base all'ampiezza dello Step.
    if (VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_RAMP_EN)
    { //  Modalità rampa
        if (PID->Rampa < L_ScaledSetpoint)
        { // Rampa in salita

            if ((PID->Rampa > -DEAD_ZONE) && (PID->Rampa < DEAD_ZONE))
            { //  Se _RAMPA cade nell'interno di +/- DEAD_ZONE
                //  Salto all'esterno.
                PID->Rampa = DEAD_ZONE;
            }

            PID->Rampa += PID->RampaStep;
            if (PID->Rampa > L_ScaledSetpoint)
                PID->Rampa = L_ScaledSetpoint;
            if (PID->Rampa > MOTORE->I_MotorRpmMax)
                PID->Rampa = MOTORE->I_MotorRpmMax;
        }


        if (PID->Rampa > L_ScaledSetpoint)
        { //  Rampa in discesa


            if ((PID->Rampa > -DEAD_ZONE) && (PID->Rampa < 0))
            {
                PID->Rampa = -DEAD_ZONE;
            }
            else if ((PID->Rampa >= 0) && (PID->Rampa < DEAD_ZONE))
            {
                PID->Rampa = DEAD_ZONE;
            }
            else if ((PID->Rampa > -DEAD_ZONE) && (PID->Rampa < DEAD_ZONE))
            {
                PidReset(PID, MOTORE);
            }


            PID->Rampa -= PID->RampaStep;
            if (PID->Rampa < L_ScaledSetpoint)
                PID->Rampa = L_ScaledSetpoint;
            if (PID->Rampa < MOTORE->I_MotorRpmMin)
                PID->Rampa = MOTORE->I_MotorRpmMin;
        }
    }
    else
    { // Modalità senza rampa

        // Verifico limiti del setpoint in modalità senza rampa
        if (L_ScaledSetpoint > MOTORE->I_MotorRpmMax)
            L_ScaledSetpoint = MOTORE->I_MotorRpmMax;

        if (L_ScaledSetpoint < MOTORE->I_MotorRpmMin)
            L_ScaledSetpoint = MOTORE->I_MotorRpmMin;

        if ((L_ScaledSetpoint < DEAD_ZONE) & (L_ScaledSetpoint > -DEAD_ZONE))
        {
            L_ScaledSetpoint = 0;
            PidReset(PID, MOTORE); // All'interno della banda morta resetto il PID
        }

        PID->Rampa = L_ScaledSetpoint;
    }

    PID->Errore = (PID->Rampa - L_ScaledProcesso); // calcolo errore tra il setpoint e il Current
    long int resc_ERRORE = __builtin_mulss((int) PID->Errore, rescaleFact);

    //    PID->ComponenteFeedForward = resc_ERRORE * 2;
    //    if (PID->ComponenteFeedForward >  2045 )
    //        PID->ComponenteFeedForward =  2045;    // limiti componente feed forward
    //
    //    if (PID->ComponenteFeedForward < -2045 )
    //        PID->ComponenteFeedForward = -2045;

    // Y[n] = Y[n-1] + P*(X[n] - X[n-1] ) + I*X[n] + D*(X[n] - 2*X[n-1] + X[n-2])

    // CONTRIBUTO PROPORZIONALE
    PID->ContributoProporzionale = PID->Kp * (resc_ERRORE - PID->OldError1);

    // CONTRIBUTO INTEGRALE
    if (saturazione == 1 || // Controllo saturo -> Anti-WindUp
            PID->Ki == 0) // Nessun contributo integrale
        PID->ContributoIntegrale = 0;
    else
        PID->ContributoIntegrale = PID->Ki * resc_ERRORE;

    // CONTRIBUTO DERIVATIVO
    if (PID->Kd == 0) // Nessun contributo derivativo
        PID->ContributoDerivativo = 0;
    else
        PID->ContributoDerivativo = PID->Kd * (resc_ERRORE - 2 * PID->OldError1 + PID->OldError2);

    // Aggiornamento errori
    PID->OldError2 = PID->OldError1;
    PID->OldError1 = resc_ERRORE;

    // Ora che è differenziale ci va il "+=" by Walt
    PID->Sommatoria += (PID->ContributoProporzionale + PID->ContributoIntegrale + PID->ContributoDerivativo); // sommatoria errori

    if (PID->Sommatoria > (LONG_MAX - 1000))
        PID->Sommatoria = LONG_MAX; // limiti pwm

    if (PID->Sommatoria < (LONG_MIN + 1000))
        PID->Sommatoria = LONG_MIN;

    if (PID->Rampa == 0)
    {
        PID->OutPid = 2048;
        PID->Integrale = 0;
        PID->Rampa = 0;
    }
    else
    { // sommatore per il calcolo del reale PWM da inviare al motore
        //_OUT = 2048 + (PID->ComponenteFeedForward + _SOMMATORIA/1000);
        PID->OutPid = 2048 + PID->Sommatoria / (100 * rescaleFact);
    }

    if (PID->OutPid > 4095) PID->OutPid = 4095;
    if (PID->OutPid < 1) PID->OutPid = 1;

    PID->OldContrValue = PID->OutPid;

    //    __builtin_disi(0x0000); /* enable interrupts, vedere pg 181 di MPLAB_XC16_C_Compiler_UG_52081.pdf */
}

void PidReset(volatile Pid_t *PID, volatile Motor_t *MOTORE)
{
    PID->RampaStep = 0;
    PID->Integrale = 0;
    PID->ContributoIntegrale = 0;
    PID->ContributoProporzionale = 0;
    PID->ContributoDerivativo = 0;

    PID->Errore = 0;
    PID->OldError1 = 0;
    PID->OldError2 = 0;

    PID->Sommatoria = 0;
    PID->OldContrValue = 0;

    PID->OutPid = 0;
}
