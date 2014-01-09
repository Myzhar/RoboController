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


//! Funzione di gestione di Timer Software
/*! Note sull'utilizzo di questa funzione:
    - Inizializzare una struttura dati di tipo Timer_t
    - Richiamare la funzione GestioneTimerSW() a cadenza fissa ( per esempio sotto interrupt )
    passando come argomento la struttura dati creata.
    - Inizializzare Timer_t->T_initial_value con il valore del timer desiderato 
        diviso per la base tempi con cui è chiamata la funzione GestioneTimerSW()
    - Quando il timer scade mette a TRUE il flag T_flag, il Flag va intercettato
    nel programma per ebilitare l'esecuzione delle operazioni da effettuare sotto timer software.
    - Appena entrati nella funzione sotto Timer il flag va messo a FALSE manualmente.
    - Quando diventa FALSE riparte il timer

    Supponendo che:
    - la funzione GestioneTimerSW() sia chiamata ogni 10mSec
    - T_initial_value sia inizializzato a 10
 
Timer->T_initial_value : Imposta il tempo del timer in 10mSec * 10 =  100mSec

Timer->T_flag : Diventa True dopo Timer->T_initial_value, deve essere resettato a mano.

\sa Timer_t
*/
void GestioneTimerSW(volatile Timer_t *Timer)
{   
    if( Timer->T_count_time && !Timer->T_flag )
    {    Timer->T_count_time--;
    }
    else
    {   Timer->T_count_time = Timer->T_initial_value;
    	Timer->T_flag = TRUE;
    }
}

//! Ricaricamento timer
//! Funzione per ricaricare il timer e evitare di finire il conteggio...
//! da usarsi in casi particolari dove sia necessario allungare il tempo del timer
void ReloadTimerSW(volatile Timer_t *Timer)
{
    Timer->T_count_time = Timer->T_initial_value;
}
