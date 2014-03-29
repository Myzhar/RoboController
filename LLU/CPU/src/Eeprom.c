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

#include "DEE_Emulation_16-bit.h"
#include "def.h"
#include "ptype.h"
#include "var.h"

void InitializationEEPROM(void)
{   DataEEInit();
    dataEEFlags.val = 0;
    RestoreEEPROMData();    // Carico tutti i dati salvati nell'EEPROM ed eventuali dati di default
}

void RestoreEEPROMData(void)
{
    /* ******************************************************************** */
    /* Recupero tutti i dati salvati in EEPROM, li carico nelle rispettive  */
    /* variabili ed effettuo tutti i controlli di coerenza necessari        */
    /*                                                                      */
    /* ******************************************************************** */

    /*
     * Tutte le funzioni del tipo:
     *      ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] = DataEERead(EEPROM_MODBUS_STATUSBIT2);
     * possono essere sostituite con una unica funzione
     *      for(i=0; i<NUMERO_PARAMETRI_EEPROM; i++)
     *      {   ParametriEEPROM[i] = DataEERead(i);
     *      }
     * che esegue il caricamento di tutti i parametri, ma per ora per motivi di debug
     * trovo più comodo eseguire i singoli caricamenti.
     */

    // La variabile modbus STATUSBIT2 è sotto EEPROM
    ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] = DataEERead(EEPROM_MODBUS_STATUSBIT2);
    if (ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] = 0;
        ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] |= FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY; // Di default l'enable è attivo a "1"
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2], EEPROM_MODBUS_STATUSBIT2);
    }

    /* Ricarico l'ID modbus del dispositivo e ne verifico la coerenza altrimenti lo imposto a "1"               */
    ParametriEEPROM[EEPROM_MODBUS_ADDRESS_SLAVE] = DataEERead(EEPROM_MODBUS_ADDRESS_SLAVE);
    if ((ParametriEEPROM[EEPROM_MODBUS_ADDRESS_SLAVE] == 0) || (ParametriEEPROM[EEPROM_MODBUS_ADDRESS_SLAVE] >= 255))
    {   ParametriEEPROM[EEPROM_MODBUS_ADDRESS_SLAVE] = 1;
        // l'ID errato lo sovrascrivo anche in caso di EEPROM disabilitata per l'utente
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ADDRESS_SLAVE], EEPROM_MODBUS_ADDRESS_SLAVE);
    }

    /* Ricarico il valore del tempo di WatchDog sulla comunicazione.                                            */
    ParametriEEPROM[EEPROM_MODBUS_COMWATCHDOG_TIME] = DataEERead(EEPROM_MODBUS_COMWATCHDOG_TIME);
    if ((ParametriEEPROM[EEPROM_MODBUS_COMWATCHDOG_TIME] <= 100) || (ParametriEEPROM[EEPROM_MODBUS_COMWATCHDOG_TIME] >= 2000))
    {   ParametriEEPROM[EEPROM_MODBUS_COMWATCHDOG_TIME] = 500;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_COMWATCHDOG_TIME], EEPROM_MODBUS_COMWATCHDOG_TIME);
    }
    //Inizializzo il timer per il watchdog sulla comunicazione
    ComunicationWatchDogTimer = ParametriEEPROM[EEPROM_MODBUS_COMWATCHDOG_TIME];

    /* **************************************************************************************************** */
    /* Carico dall'EEPROM i parametri di conversione.                                                       */
    /* Inizializzo sucessivamente tutti i parametri necessari alla conversione utilizzando dei parametri di */
    /* default in caso di taratura non eseguita.                                                            */
    /* **************************************************************************************************** */
    ParametriEEPROM[TARAT_ZERO_RB_AN0] = DataEERead(TARAT_ZERO_RB_AN0);
    ParametriEEPROM[TARAT_ZERO_RB_AN1] = DataEERead(TARAT_ZERO_RB_AN1);
    ParametriEEPROM[TARAT_ZERO_RB_AN2] = DataEERead(TARAT_ZERO_RB_AN2);
    ParametriEEPROM[TARAT_ZERO_RB_AN3] = DataEERead(TARAT_ZERO_RB_AN3);
    ParametriEEPROM[TARAT_ZERO_RB_AN4] = DataEERead(TARAT_ZERO_RB_AN4);
    ParametriEEPROM[TARAT_CONV_RB_AN0] = DataEERead(TARAT_CONV_RB_AN0);
    ParametriEEPROM[TARAT_CONV_RB_AN1] = DataEERead(TARAT_CONV_RB_AN1);
    ParametriEEPROM[TARAT_CONV_RB_AN2] = DataEERead(TARAT_CONV_RB_AN2);
    ParametriEEPROM[TARAT_CONV_RB_AN3] = DataEERead(TARAT_CONV_RB_AN3);
    ParametriEEPROM[TARAT_CONV_RB_AN4] = DataEERead(TARAT_CONV_RB_AN4);

    ParametriEEPROM[TARAT_COSTCONV_AN0_LOW] = DataEERead(TARAT_COSTCONV_AN0_LOW);  // CostanteTaraturaAN0.fval
    ParametriEEPROM[TARAT_COSTCONV_AN0_HIGH] = DataEERead(TARAT_COSTCONV_AN0_HIGH);
    ParametriEEPROM[TARAT_COSTCONV_AN1_LOW] = DataEERead(TARAT_COSTCONV_AN1_LOW);  // CostanteTaraturaAN1.fval
    ParametriEEPROM[TARAT_COSTCONV_AN1_HIGH] = DataEERead(TARAT_COSTCONV_AN1_HIGH);
    ParametriEEPROM[TARAT_COSTCONV_AN2_LOW] = DataEERead(TARAT_COSTCONV_AN2_LOW);  // CostanteTaraturaAN2.fval
    ParametriEEPROM[TARAT_COSTCONV_AN2_HIGH] = DataEERead(TARAT_COSTCONV_AN2_HIGH);
    ParametriEEPROM[TARAT_COSTCONV_AN3_LOW] = DataEERead(TARAT_COSTCONV_AN3_LOW);  // CostanteTaraturaAN3.fval
    ParametriEEPROM[TARAT_COSTCONV_AN3_HIGH] = DataEERead(TARAT_COSTCONV_AN3_HIGH);
    ParametriEEPROM[TARAT_COSTCONV_AN4_LOW] = DataEERead(TARAT_COSTCONV_AN4_LOW);  // CostanteTaraturaAN4.fval
    ParametriEEPROM[TARAT_COSTCONV_AN4_HIGH] = DataEERead(TARAT_COSTCONV_AN4_HIGH);


    if((ParametriEEPROM[TARAT_COSTCONV_AN0_LOW] == 0) && (ParametriEEPROM[TARAT_COSTCONV_AN0_HIGH] == 0))
    {   // Se non è stato tarato il canale analogico uso una impostazione di default per la conversione
        CostanteTaraturaAN0.fval = 1;
    }
    else
    {   CostanteTaraturaAN0.low_part = ParametriEEPROM[TARAT_COSTCONV_AN0_LOW];
        CostanteTaraturaAN0.high_part = ParametriEEPROM[TARAT_COSTCONV_AN0_HIGH];
    }

    if((ParametriEEPROM[TARAT_COSTCONV_AN1_LOW] == 0) && (ParametriEEPROM[TARAT_COSTCONV_AN1_HIGH] == 0))
    {   // Se non è stato tarato il canale analogico uso una impostazione di default per la conversione
        CostanteTaraturaAN1.fval = 1;
    }
    else
    {   CostanteTaraturaAN1.low_part = ParametriEEPROM[TARAT_COSTCONV_AN1_LOW];
        CostanteTaraturaAN1.high_part = ParametriEEPROM[TARAT_COSTCONV_AN1_HIGH];
    }

    if((ParametriEEPROM[TARAT_COSTCONV_AN2_LOW] == 0) && (ParametriEEPROM[TARAT_COSTCONV_AN2_HIGH] == 0))
    {   // Se non è stato tarato il canale analogico uso una impostazione di default per la conversione
        CostanteTaraturaAN2.fval = 1;
    }
    else
    {   CostanteTaraturaAN2.low_part = ParametriEEPROM[TARAT_COSTCONV_AN2_LOW];
        CostanteTaraturaAN2.high_part = ParametriEEPROM[TARAT_COSTCONV_AN2_HIGH];
    }

    if((ParametriEEPROM[TARAT_COSTCONV_AN3_LOW] == 0) && (ParametriEEPROM[TARAT_COSTCONV_AN3_HIGH] == 0))
    {   // Se non è stato tarato il canale analogico uso una impostazione di default per la conversione
        CostanteTaraturaAN3.fval = 1;
    }
    else
    {   CostanteTaraturaAN3.low_part = ParametriEEPROM[TARAT_COSTCONV_AN3_LOW];
        CostanteTaraturaAN3.high_part = ParametriEEPROM[TARAT_COSTCONV_AN3_HIGH];
    }

    if((ParametriEEPROM[TARAT_COSTCONV_AN4_LOW] == 0) && (ParametriEEPROM[TARAT_COSTCONV_AN4_HIGH] == 0))
    {   // Se non è stato tarato il canale analogico uso una impostazione di default per la conversione
        CostanteTaraturaAN4.fval = 1;
    }
    else
    {   CostanteTaraturaAN4.low_part = ParametriEEPROM[TARAT_COSTCONV_AN4_LOW];
        CostanteTaraturaAN4.high_part = ParametriEEPROM[TARAT_COSTCONV_AN4_HIGH];
    }

    /* **************************************************************************************************** */
    /*                                                                                                      */
    /* Carico dall'EEPROM i parametri di Configurazione del Robot                                           */
    /*                                                                                                      */
    /*                                                                                                      */
    /*                                                                                                      */
    /* **************************************************************************************************** */
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WEIGHT] = DataEERead(EEPROM_MODBUS_ROBOT_DIMENSION_WEIGHT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WIDTH] = DataEERead(EEPROM_MODBUS_ROBOT_DIMENSION_WIDTH);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_HEIGHT] = DataEERead(EEPROM_MODBUS_ROBOT_DIMENSION_HEIGHT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_LENGHT] = DataEERead(EEPROM_MODBUS_ROBOT_DIMENSION_LENGHT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WHEELBASE] = DataEERead(EEPROM_MODBUS_ROBOT_DIMENSION_WHEELBASE);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT] = DataEERead(EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT);         // Raggio in Centesimi di mm della ruota ( 3,3Cm = 3300 Cent/mm)
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT] = DataEERead(EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT);       // Raggio in Centesimi di mm della ruota ( 3,3Cm = 3300 Cent/mm)
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT] = DataEERead(EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT);           // Numero di impulsi encoder per rivoluzione della ruota ( già moltiplicato per 4 )
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT] = DataEERead(EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT);         // Numero di impulsi encoder per rivoluzione della ruota ( già moltiplicato per 4 )
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT] = DataEERead(EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT] = DataEERead(EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_LEFT] = DataEERead(EEPROM_MODBUS_ROBOT_MOTOR_IMAX_LEFT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_RIGHT] = DataEERead(EEPROM_MODBUS_ROBOT_MOTOR_IMAX_RIGHT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_LEFT] = DataEERead(EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_LEFT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_RIGHT] = DataEERead(EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_RIGHT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT] = DataEERead(EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT] = DataEERead(EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT] = DataEERead(EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT] = DataEERead(EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT);

    ParametriEEPROM[EEPROM_MODBUS_ROBOT_CHARGED_BATT] = DataEERead(EEPROM_MODBUS_ROBOT_CHARGED_BATT);
    ParametriEEPROM[EEPROM_MODBUS_ROBOT_DISCHARGED_BATT] = DataEERead(EEPROM_MODBUS_ROBOT_DISCHARGED_BATT);

    if (ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] = 0;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2], EEPROM_MODBUS_STATUSBIT2);
    }

    //Impostazione default se valori sono quelli della cella cancellata ( = 65535 )
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WEIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WEIGHT] = 300;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WEIGHT], EEPROM_MODBUS_ROBOT_DIMENSION_WEIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WIDTH] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WIDTH] = 200;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WIDTH], EEPROM_MODBUS_ROBOT_DIMENSION_WIDTH);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_HEIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_HEIGHT] = 100;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_HEIGHT], EEPROM_MODBUS_ROBOT_DIMENSION_HEIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_LENGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_LENGHT] = 200;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_LENGHT], EEPROM_MODBUS_ROBOT_DIMENSION_LENGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WHEELBASE] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WHEELBASE] = 190;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WHEELBASE], EEPROM_MODBUS_ROBOT_DIMENSION_WHEELBASE);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT] = 3300;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT], EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT] = 3300;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT], EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT] = 400; //2048;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT], EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT] = 400; //2048;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT], EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT] = 6750;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT], EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT] = 6750;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT], EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_LEFT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_LEFT] = 580;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_LEFT], EEPROM_MODBUS_ROBOT_MOTOR_IMAX_LEFT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_RIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_RIGHT] = 580;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_RIGHT], EEPROM_MODBUS_ROBOT_MOTOR_IMAX_RIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_LEFT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_LEFT] = 18;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_LEFT], EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_LEFT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_RIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_RIGHT] = 18;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_RIGHT], EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_RIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT] = 1;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT], EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT] = 1;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT], EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT] = 43;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT], EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT] = 43;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT], EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_CHARGED_BATT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_CHARGED_BATT] = 1680; // 16,8V
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_CHARGED_BATT], EEPROM_MODBUS_ROBOT_CHARGED_BATT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DISCHARGED_BATT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_ROBOT_DISCHARGED_BATT] = 1200; //12,0V
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DISCHARGED_BATT], EEPROM_MODBUS_ROBOT_DISCHARGED_BATT);
    }



    /* **************************************************************************************************** */
    /*                                                                                                      */
    /* Carico dall'EEPROM i parametri di Configurazione del PID                                             */
    /*                                                                                                      */
    /*                                                                                                      */
    /*                                                                                                      */
    /* **************************************************************************************************** */
    ParametriEEPROM[EEPROM_MODBUS_PID_P_LEFT] = DataEERead(EEPROM_MODBUS_PID_P_LEFT);
    ParametriEEPROM[EEPROM_MODBUS_PID_I_LEFT] = DataEERead(EEPROM_MODBUS_PID_I_LEFT);
    ParametriEEPROM[EEPROM_MODBUS_PID_D_LEFT] = DataEERead(EEPROM_MODBUS_PID_D_LEFT);
    ParametriEEPROM[EEPROM_MODBUS_PID_P_RIGHT] = DataEERead(EEPROM_MODBUS_PID_P_RIGHT);
    ParametriEEPROM[EEPROM_MODBUS_PID_I_RIGHT] = DataEERead(EEPROM_MODBUS_PID_I_RIGHT);
    ParametriEEPROM[EEPROM_MODBUS_PID_D_RIGHT] = DataEERead(EEPROM_MODBUS_PID_D_RIGHT);
    ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_LEFT] = DataEERead(EEPROM_MODBUS_PID_RAMP_LEFT);
    ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_RIGHT] = DataEERead(EEPROM_MODBUS_PID_RAMP_RIGHT);

    if(ParametriEEPROM[EEPROM_MODBUS_PID_P_LEFT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_PID_P_LEFT] = 100;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_P_LEFT], EEPROM_MODBUS_PID_P_LEFT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_PID_I_LEFT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_PID_I_LEFT] = 0;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_I_LEFT], EEPROM_MODBUS_PID_I_LEFT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_PID_D_LEFT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_PID_D_LEFT] = 0;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_D_LEFT], EEPROM_MODBUS_PID_D_LEFT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_PID_P_RIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_PID_P_RIGHT] = 100;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_P_RIGHT], EEPROM_MODBUS_PID_P_RIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_PID_I_RIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_PID_I_RIGHT] = 0;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_I_RIGHT], EEPROM_MODBUS_PID_I_RIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_PID_D_RIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_PID_D_RIGHT] = 0;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_D_RIGHT], EEPROM_MODBUS_PID_D_RIGHT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_LEFT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_LEFT] = 1;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_LEFT], EEPROM_MODBUS_PID_RAMP_LEFT);
    }
    if(ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_RIGHT] == 65535)
    {   ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_RIGHT] = 1;
        DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_RIGHT], EEPROM_MODBUS_PID_RAMP_RIGHT);
    }

    // Aggiorno le variabili PID toccate dai dati EEPROM
    PID1.Kp = ParametriEEPROM[EEPROM_MODBUS_PID_P_LEFT];
    PID1.Ki = ParametriEEPROM[EEPROM_MODBUS_PID_I_LEFT];
    PID1.Kd = ParametriEEPROM[EEPROM_MODBUS_PID_D_LEFT];

    PID2.Kp = ParametriEEPROM[EEPROM_MODBUS_PID_P_LEFT];
    PID2.Ki = ParametriEEPROM[EEPROM_MODBUS_PID_I_LEFT];
    PID2.Kd = ParametriEEPROM[EEPROM_MODBUS_PID_D_LEFT];

    PID1.RampaStep = ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_LEFT];
    PID2.RampaStep = ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_RIGHT];
}


