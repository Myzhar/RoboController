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


/*
 *  Funzione da chiamare ogni 1mSec ( nel Main )
 *
 *  Verifica se è scaduto o meno il watchdog di comunicazione seriale
 *  e di conseguenza:
 *  - Segnala o meno il Fail ai motori per arrestarli
 *  - Azzera il Setpoint del PID per "ripartire da fermo" se ritorna la comunicazione
 *  -
 */
void GestioneWatchdog(void)
{
     /* GESTIONE WATCHDOG COMUNICAZIONE */
     /*
      * Se la funzione è attiva tramite il flag FLG_STATUSBI1_COMWATCHDOG
      * setta a "1" il segnale MotoreX.UC_Fail in caso di mancanza di comunicazione
      *
      * La funzione non fa altro, questo segnale deve essere opportunamente
      * usato nelle funzioni che controllano i motori
      *
      */
     if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_COMWATCHDOG)
     {
        if( ComunicationWatchDogTimer > 0 )
        {
            ComunicationWatchDogTimer--;
            Motore1.UC_Fail = 0;    //  I motori non sono in errore.
            Motore2.UC_Fail = 0;
        }
        else
        {
            Motore1.UC_Fail = 1;    //  I motori non sono in errore.
            Motore2.UC_Fail = 1;

            /* Azzero il setopoint */
            PID1.Setpoint = 0;      /* Non scrivo nella variabile modbus relativa tanto non aggiorna il dato */
            PID2.Setpoint = 0;      /* fino alla prossima comunicazione.                                     */
        }
     }
     else
     {  //watcdog disattivo... ovvero non scade mai
        ComunicationWatchDogTimer = 10;
     }
}



