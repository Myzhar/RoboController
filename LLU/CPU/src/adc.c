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


void InitADC(void)
{
    // ---------------------------------------------------------------------------
    // A/D converter ( Start Config )
    // ---------------------------------------------------------------------------
    AD1CON1bits.ADSIDL  =   1;                  // stop in idle
    AD1CON1bits.ADDMABM =   0;                  // DMA buffers are built in scatter/gather mode
    AD1CON1bits.AD12B   =   0;                  // 10-bit ADC operation
    AD1CON1bits.FORM    =   0;                  // integer format
    AD1CON1bits.SSRC    =   0b111;              // auto-convert
    AD1CON1bits.SIMSAM  =   0;                  // Samples multiple channels individually in sequence
    AD1CON1bits.ASAM    =   1;                  // ADC Sample Control: Sampling begins immediately after conversion

    AD1CON2bits.VCFG    =   0;                  //Vrefh = AVdd, Vrefl = AVss
    AD1CON2bits.CSCNA   =   1;                  // Scan Input Selections for CH0+ during Sample A bit
    AD1CON2bits.CHPS    =   0;                  // Converts CH0
    AD1CON2bits.SMPI    =   4;                  // In Channel Scanning è il numero di ingressi - 1 ( DS70183C pg 16-43
    AD1CON2bits.ALTS    =   0;                  //

    AD1CON3bits.ADRC    =   0;                  // ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC    =   0b11111;            // 31 Tad auto sample time
    AD1CON3bits.ADCS    =   63;                 // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
                                                // ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us
    AD1CON4bits.DMABL   =   3;                  // Each buffer contains 8 words

    AD1CHS0bits.CH0NB   =   0;                  // don't care -> sample B
    AD1CHS0bits.CH0SB   =   0;                  // don't care -> sample B
    AD1CHS0bits.CH0NA   =   0;                  // CH0 neg -> Vrefl
    AD1CHS0bits.CH0SA   =   1;                  // CH0 pos -> AN1


    AD1CHS0bits.CH0NA   =   0;                  // CH0 neg -> Vrefl
    AD1CSSLbits.CSS0    =   1;                  // Enable AN0 for channel scan - Vmot
    AD1CSSLbits.CSS4    =   1;                  // Enable AN4 for channel scan
    AD1CSSLbits.CSS5    =   1;                  // Enable AN5 for channel scan
    AD1CSSLbits.CSS6    =   1;                  // Enable AN6 for channel scan
    AD1CSSLbits.CSS7    =   1;                  // Enable AN7 for channel scan

    //AD1PCFGH/AD1PCFGL: Port Configuration Register
    AD1PCFGL=0xFFFF;
    AD1PCFGLbits.PCFG0  =   0;                 // AN0 as Analog Input
    AD1PCFGLbits.PCFG4  =   0;                 // AN4 as Analog Input
    AD1PCFGLbits.PCFG5  =   0;                 // AN5 as Analog Input
    AD1PCFGLbits.PCFG6  =   0;                 // AN6 as Analog Input
    AD1PCFGLbits.PCFG7  =   0;                 // AN7 as Analog Input

    IFS0bits.AD1IF      =   0;                 // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE      =   0;                 // Do Not Enable A/D interrupt
    // Spostato in ISR_Settings :   AD1CON1bits.ADON    =   1;  // Turn on the A/D converter
    // Spostato in ISR_Settings :   AD1CON1bits.ADON    =   0;  // Turn off the A/D converter -- DEBUG --

    // ---------------------------------------------------------------------------
    // DMA A/D converter
    // ---------------------------------------------------------------------------

    // ADC connected to DMA 7
    DMA7CONbits.AMODE = 2;			// Configure DMA for Peripheral indirect mode
    DMA7CONbits.MODE  = 2;			// Configure DMA for Continuous Ping-Pong mode
    DMA7PAD = (volatile unsigned int)&ADC1BUF0; // Point DMA to ADC1BUF0
    //DMA7CNT = 127;                            // 64 DMA request
    DMA7CNT = (SAMP_BUFF_SIZE*NUM_CHS2SCAN)-1;
    DMA7REQ = 13;                               // Select ADC1 as DMA Request source
    DMA7STA = __builtin_dmaoffset(DmaAdc_A);
    DMA7STB = __builtin_dmaoffset(DmaAdc_B);
    // Spostato in ISR_Settings :   _DMA7IF = 0;            //Clear the DMA interrupt flag bit
    // Spostato in ISR_Settings :   _DMA7IE = 1;            //Set the DMA interrupt enable bit
    // Spostato in ISR_Settings :   DMA7CONbits.CHEN=1;     // Enable DMA
    // Spostato in ISR_Settings :   DMA7CONbits.CHEN=0;     // Disable DMA

    // ---------------------------------------------------------------------------
    // A/D converter ( End Config )
    // ---------------------------------------------------------------------------



    //In ISR_Settings(void) :
    //
    //    _DMA7IF = 0;            //Clear the DMA interrupt flag bit
    //    _DMA7IE = 1;            //Set the DMA interrupt enable bit
    //    DMA7CONbits.CHEN = 1;   // Enable DMA
    //    AD1CON1bits.ADON = 1;   // Turn on the A/D converter


    _DMA7IF = 0;            //Clear the DMA interrupt flag bit
    _DMA7IE = 1;            //Set the DMA interrupt enable bit
    DMA7CONbits.CHEN = 1;   // Enable DMA
    AD1CON1bits.ADON = 1;   // Turn on the A/D converter

}

int LeggiADC(int NumeroIngresso)
{   float ReturnValue;

    switch(NumeroIngresso)
    {       case    PIC_AN0    :
                    ReturnValue = (float)(IngressiAnalogici[PIC_AN0] - ParametriEEPROM[TARAT_ZERO_RB_AN0]) * CostanteTaraturaAN0.fval; // AN0 del micro = VMOT
                    break;

            case    PIC_AN1    :
                    ReturnValue = (float)(IngressiAnalogici[PIC_AN1] - ParametriEEPROM[TARAT_ZERO_RB_AN1]) * CostanteTaraturaAN1.fval; // AN1 del PCB; AN5 del micro
                    break;

            case    PIC_AN2    :
                    ReturnValue = (float)(IngressiAnalogici[PIC_AN2] - ParametriEEPROM[TARAT_ZERO_RB_AN2]) * CostanteTaraturaAN2.fval; // AN2 del PCB; AN5 del micro
                    break;

            case    PIC_AN3    :
                    ReturnValue = (float)(IngressiAnalogici[PIC_AN3] - ParametriEEPROM[TARAT_ZERO_RB_AN3]) * CostanteTaraturaAN3.fval; // AN3 del PCB; AN6 del micro
                    break;

            case    PIC_AN4    :
                    ReturnValue = (float)(IngressiAnalogici[PIC_AN4] - ParametriEEPROM[TARAT_ZERO_RB_AN4]) * CostanteTaraturaAN4.fval; // AN4 del PCB; AN7 del micro
                    break;

            default    :
                    ReturnValue = 0;
                    break;
    }

    return (int)ReturnValue;
}






void MediaADC(unsigned int * OutputIngressiMediati, unsigned int DmaAdc[][SAMP_BUFF_SIZE])
{   //Calcolo la media di tutti i valori di tutti i canali DmaADC e
    //la salvo nel rispettivo indirizzo delle variabili modbus.
    unsigned int Media[MAX_CHNUM+1]; //MAX_CHNUM];
    unsigned char i=0,k=0;

    for(i=0; i<MAX_CHNUM+1; i++)
    {   Media[i] = 0;
        for(k=0; k< SAMP_BUFF_SIZE; k++) {
            Media[i] += (unsigned int)(DmaAdc)[i][k];
        }
        Media[i] /= (SAMP_BUFF_SIZE); // SAMP_BUFF_SIZE;
    }

    OutputIngressiMediati[ROBOCONTROLLER_AN0]   =  (unsigned int)Media[PIC_AN0]; // AN0 del micro = VMOT
    OutputIngressiMediati[ROBOCONTROLLER_AN1]   =  (unsigned int)Media[PIC_AN4]; // AN1 del PCB; AN4 del micro
    OutputIngressiMediati[ROBOCONTROLLER_AN2]   =  (unsigned int)Media[PIC_AN5]; // AN2 del PCB; AN5 del micro
    OutputIngressiMediati[ROBOCONTROLLER_AN3]   =  (unsigned int)Media[PIC_AN6]; // AN3 del PCB; AN6 del micro
    OutputIngressiMediati[ROBOCONTROLLER_AN4]   =  (unsigned int)Media[PIC_AN7]; // AN4 del PCB; AN7 del micro

}

unsigned int TaraturaAnalogiche(unsigned int FlagTaratura)
{   // unsigned int ParametriEEPROM[NUMERO_PARAMETRI_EEPROM];
    //  case WORD_FLAG_TARATURA         :   return((unsigned int )VarModbus[INDICE_FLAG_TARATURA]);
    //  case WORD_VAL_TAR_FS            :   return((unsigned int )VarModbus[INDICE_VAL_TAR_FS]);

    // A seconda del bit messo a 1 in FlagTaratura eseguo la relativa operazione di taratura.

        //  IngressiAnalogici[ROBOCONTROLLER_AN1]; // AN1 del PCB; AN4 del micro
        //  IngressiAnalogici[ROBOCONTROLLER_AN2]; // AN2 del PCB; AN5 del micro
        //  IngressiAnalogici[ROBOCONTROLLER_AN3]; // AN3 del PCB; AN6 del micro
        //  IngressiAnalogici[ROBOCONTROLLER_AN4]; // AN4 del PCB; AN7 del micro
        //  IngressiAnalogici[ROBOCONTROLLER_AN0]; // AN0 del micro = VMOT

    // Tarature del punto di 0 ( solitamente gli 0mV ma non è detto
    if(FlagTaratura & (unsigned int)(0x0001))
    {   // Taratura riferimento 0 per canale AN0
        ParametriEEPROM[TARAT_ZERO_RB_AN0] = IngressiAnalogici[ROBOCONTROLLER_AN0];
        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN) DataEEWrite(ParametriEEPROM[TARAT_ZERO_RB_AN0], TARAT_ZERO_RB_AN0);
        FlagTaratura &= ~(0x0001);
    }

    if(FlagTaratura & 0x0002)
    {   // Taratura riferimento 0 per canale AN1
        ParametriEEPROM[TARAT_ZERO_RB_AN1] = IngressiAnalogici[ROBOCONTROLLER_AN1];
        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN) DataEEWrite(ParametriEEPROM[TARAT_ZERO_RB_AN1], TARAT_ZERO_RB_AN1);
        FlagTaratura &= ~(0x0002);
    }

    if(FlagTaratura & 0x0004)
    {   // Taratura riferimento 0 per canale AN2
        ParametriEEPROM[TARAT_ZERO_RB_AN2] = IngressiAnalogici[ROBOCONTROLLER_AN2];
        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN) DataEEWrite(ParametriEEPROM[TARAT_ZERO_RB_AN2], TARAT_ZERO_RB_AN2);
        FlagTaratura &= ~(0x0004);
    }

    if(FlagTaratura & 0x0008)
    {   // Taratura riferimento 0 per canale AN3
        ParametriEEPROM[TARAT_ZERO_RB_AN3] = IngressiAnalogici[ROBOCONTROLLER_AN3];
        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN) DataEEWrite(ParametriEEPROM[TARAT_ZERO_RB_AN3], TARAT_ZERO_RB_AN3);
        FlagTaratura &= ~(0x0008);
    }

    if(FlagTaratura & 0x0010)
    {   // Taratura riferimento 0 per canale AN0
        ParametriEEPROM[TARAT_ZERO_RB_AN4] = IngressiAnalogici[ROBOCONTROLLER_AN4];
        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN) DataEEWrite(ParametriEEPROM[TARAT_ZERO_RB_AN4], TARAT_ZERO_RB_AN4);
        FlagTaratura &= ~(0x0010);
    }


 /* Parametri:
  *  K = dato analogico letto dal convertitore
  *  A = Dato letto con ingresso ADC a 0
  *  B = Dato letto con l'ingresso ADC al valore di fondoscala
  *  C = Valore da associare al fondoscala
  *
  *  Esempio, se voglio che K valga 0 con l'ADC che ritorna 25 e valga 30000 con l'ADC
  *  che ritorna 4090 allora:
  *  A = 25
  *  B = 4090
  *  C = 30000
  *
  *  X = (K-A) * (C-0)/(B-A)
  *  Ponendo T = (C-0)/(B-A) = 30000 / (4090-25) = 7,38007...
  *  Con K=A => X= ( K-K)*T = 0*T = 0
  *  Con K=B => X=(B-A)*T=4065*7,38007=29999,98455
  *  Nel caso di ingresso analogico posto a metà tra A e B:
  *      K=A+[(B-A)/2]=2057,5  => X=(K-A)*[(C-0)/(B-A)]=(2057,5-25)*[30000/(4090-25)] = 2032,5*7,380073801 = 14227,5
  *      
  *
  * Ora, nel programa:
  * K = IngressiAnalogici[PIC_AN0]
  * A = ParametriEEPROM[TARAT_ZERO_RB_AN0] ... ParametriEEPROM[TARAT_ZERO_RB_AN4]
  * B = IngressiAnalogici[PIC_AN0] nell'istante della taratura
  * C = VarModbus[INDICE_VAL_TAR_FS] nell'istante della taratura
  *
  * T = ParametriEEPROM[TARAT_CONV_RB_AN0] = (VarModbus[INDICE_VAL_TAR_FS])/(Media[PIC_AN0] - ParametriEEPROM[TARAT_ZERO_RB_AN0])
  * X = (IngressiAnalogici[PIC_AN0] - ParametriEEPROM[TARAT_ZERO_RB_AN0]) * ParametriEEPROM[TARAT_CONV_RB_AN0]
  *
 */

    if(FlagTaratura & 0x0020)
    {   //ParametriEEPROM[TARAT_CONV_RB_AN0] = VarModbus[INDICE_VAL_TAR_FS] / ( IngressiAnalogici[PIC_AN0] - ParametriEEPROM[TARAT_ZERO_RB_AN0] );
        CostanteTaraturaAN0.fval = (float)VarModbus[INDICE_VAL_TAR_FS] / (float)(IngressiAnalogici[PIC_AN0] - ParametriEEPROM[TARAT_ZERO_RB_AN0] );
        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
        {   DataEEWrite(ParametriEEPROM[TARAT_ZERO_RB_AN0], TARAT_ZERO_RB_AN0);
            DataEEWrite(CostanteTaraturaAN0.low_part,TARAT_COSTCONV_AN0_LOW);
            DataEEWrite(CostanteTaraturaAN0.high_part,TARAT_COSTCONV_AN0_HIGH);
        }
        FlagTaratura &= ~(0x0020);
    }

    if(FlagTaratura & 0x0040)
    {   //ParametriEEPROM[TARAT_CONV_RB_AN1] = VarModbus[INDICE_VAL_TAR_FS] / ( IngressiAnalogici[PIC_AN1] - ParametriEEPROM[TARAT_ZERO_RB_AN1] );
        CostanteTaraturaAN1.fval = (float)VarModbus[INDICE_VAL_TAR_FS] / (float)(IngressiAnalogici[PIC_AN1] - ParametriEEPROM[TARAT_ZERO_RB_AN1] );
        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
        {   DataEEWrite(ParametriEEPROM[TARAT_ZERO_RB_AN1], TARAT_ZERO_RB_AN1);
            DataEEWrite(CostanteTaraturaAN1.low_part,TARAT_COSTCONV_AN1_LOW);
            DataEEWrite(CostanteTaraturaAN1.high_part,TARAT_COSTCONV_AN1_HIGH);
        }
        FlagTaratura &= ~(0x0040);
    }

    if(FlagTaratura & 0x0080)
    {   //ParametriEEPROM[TARAT_CONV_RB_AN2] = VarModbus[INDICE_VAL_TAR_FS] / ( IngressiAnalogici[PIC_AN2] - ParametriEEPROM[TARAT_ZERO_RB_AN2] );
        CostanteTaraturaAN2.fval = (float)VarModbus[INDICE_VAL_TAR_FS] / (float)(IngressiAnalogici[PIC_AN2] - ParametriEEPROM[TARAT_ZERO_RB_AN2] );
        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
        {   DataEEWrite(ParametriEEPROM[TARAT_ZERO_RB_AN2], TARAT_ZERO_RB_AN2);
            DataEEWrite(CostanteTaraturaAN2.low_part,TARAT_COSTCONV_AN2_LOW);
            DataEEWrite(CostanteTaraturaAN2.high_part,TARAT_COSTCONV_AN2_HIGH);
        }
        FlagTaratura &= ~(0x0080);
    }

    if(FlagTaratura & 0x0100)
    {   //ParametriEEPROM[TARAT_CONV_RB_AN3] = VarModbus[INDICE_VAL_TAR_FS] / ( IngressiAnalogici[PIC_AN3] - ParametriEEPROM[TARAT_ZERO_RB_AN3] );
        CostanteTaraturaAN3.fval = (float)VarModbus[INDICE_VAL_TAR_FS] / (float)(IngressiAnalogici[PIC_AN3] - ParametriEEPROM[TARAT_ZERO_RB_AN3] );
        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
        {   DataEEWrite(ParametriEEPROM[TARAT_ZERO_RB_AN3], TARAT_ZERO_RB_AN3);
            DataEEWrite(CostanteTaraturaAN3.low_part,TARAT_COSTCONV_AN3_LOW);
            DataEEWrite(CostanteTaraturaAN3.high_part,TARAT_COSTCONV_AN3_HIGH);
        }
        FlagTaratura &= ~(0x0100);
    }

    if(FlagTaratura & 0x0200)
    {   //ParametriEEPROM[TARAT_CONV_RB_AN4] = VarModbus[INDICE_VAL_TAR_FS] / ( IngressiAnalogici[PIC_AN4] - ParametriEEPROM[TARAT_ZERO_RB_AN4] );
        CostanteTaraturaAN4.fval = (float)VarModbus[INDICE_VAL_TAR_FS] / (float)(IngressiAnalogici[PIC_AN4] - ParametriEEPROM[TARAT_ZERO_RB_AN4] );
        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
        {   DataEEWrite(ParametriEEPROM[TARAT_ZERO_RB_AN4], TARAT_ZERO_RB_AN4);
            DataEEWrite(CostanteTaraturaAN4.low_part,TARAT_COSTCONV_AN4_LOW);
            DataEEWrite(CostanteTaraturaAN4.high_part,TARAT_COSTCONV_AN4_HIGH);
        }
        FlagTaratura &= ~(0x0200);
    }

    return(FlagTaratura);
}

/*  ***************************************************************************
 *  ***************************************************************************
 *                  ADC Interrupt Routine
 *  ***************************************************************************
 *  ***************************************************************************
 */

void _ISR_PSV _DMA7Interrupt(void) {	// DMA for ADC
    InterruptTest11++;
    // A seconda di quale buffer è pronto chiamo MediaADC passandogli quello
    // giusto come argomento.
    _DMA7IF = 0;	// interrupt flag reset
    
    if(DmaBuffer == 0)
        MediaADC(&IngressiAnalogici[0], DmaAdc_A);
    else
        MediaADC(&IngressiAnalogici[0], DmaAdc_B);
   
    DmaBuffer ^= 1;
    InterruptTest11--;
}

