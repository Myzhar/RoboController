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

unsigned int LeggiWord(unsigned int Address)
{   //unsigned int Valore;
    //  In lettura mi ritorna il dato associato all'indirizzo modbus, se l'indirizzo
    //  non è implementato mi ritorna il valore dell'indirizzo stesso.
    switch(Address){
        case WORD_TIPO_DISPOSITIVO      :   return((unsigned int )TIPO_DISPOSITIVO);
        case WORD_VERSIONE_FIRMWARE     :   return((unsigned int )VERSIONE_FIRMWARE);
	case WORD_ADDRESS_SLAVE         :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ADDRESS_SLAVE]);
        case WORD_RITARDO_SERIALE       :   return((unsigned int )RITARDO_RISPOSTA_SERIALE);
        case WORD_STATUSBIT1            :   return((unsigned int )VarModbus[INDICE_STATUSBIT1]);
        case WORD_STATUSBIT2            :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2]); //VarModbus[INDICE_STATUSBIT2]);
        case WORD_PWM_CH1               :   return((unsigned int )VarModbus[INDICE_PWM_CH1]);
        case WORD_PWM_CH2               :   return((unsigned int )VarModbus[INDICE_PWM_CH2]);
        case WORD_AN1                   :   return((unsigned int )VarModbus[INDICE_AN1]);
        case WORD_AN2                   :   return((unsigned int )VarModbus[INDICE_AN2]);
        case WORD_AN3                   :   return((unsigned int )VarModbus[INDICE_AN3]);
        case WORD_AN4                   :   return((unsigned int )VarModbus[INDICE_AN4]);
        case WORD_TENSIONE_ALIM         :   return((unsigned int )VarModbus[INDICE_TENSIONE_ALIM]);
        case WORD_COMWATCHDOG_TIME      :   //return((unsigned int )VarModbus[INDICE_COMWATCHDOG_TIME]);
                                            return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_COMWATCHDOG_TIME]);
        case WORD_FLAG_TARATURA         :   return((unsigned int )VarModbus[INDICE_FLAG_TARATURA]);
        case WORD_VAL_TAR_FS            :   return((unsigned int )VarModbus[INDICE_VAL_TAR_FS]);
        case WORD_ENC1_TICK             :   return((unsigned int )VarModbus[INDICE_ENC1_TICK]);
        case WORD_ENC1_PERIOD           :   return((unsigned int )VarModbus[INDICE_ENC1_PERIOD]);
        case WORD_ENC2_TICK             :   return((unsigned int )VarModbus[INDICE_ENC2_TICK]);
        case WORD_ENC2_PERIOD           :   return((unsigned int )VarModbus[INDICE_ENC2_PERIOD]);

        case WORD_RD_PWM_CH1            :   return((unsigned int )VarModbus[INDICE_RD_PWM_CH1]);
        case WORD_RD_PWM_CH2            :   return((unsigned int )VarModbus[INDICE_RD_PWM_CH2]);
        
        case WORD_ENC1_SPEED            :   //return(Motore1.L_WheelSpeed);
                                            return((unsigned int )VarModbus[INDICE_ENC1_SPEED]);
        case WORD_ENC2_SPEED            :   //return(Motore2.L_WheelSpeed);
                                            return((unsigned int )VarModbus[INDICE_ENC2_SPEED]);

/* *****************************************************************************
 WORD MODBUS USATE PER LA CONFIGURAZIONE DEL ROBOT, MAPPATE DALL'INDIRIZZO 200
 ******************************************************************************/
        case WORD_ROBOT_DIMENSION_WEIGHT            :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WEIGHT]);
        case WORD_ROBOT_DIMENSION_WIDTH             :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WIDTH]);
        case WORD_ROBOT_DIMENSION_HEIGHT            :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_HEIGHT]);
        case WORD_ROBOT_DIMENSION_LENGHT            :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_LENGHT]);
        case WORD_ROBOT_DIMENSION_WHEELBASE         :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WHEELBASE]);
        case WORD_ROBOT_WHEEL_RADIUS_LEFT           :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT]);
        case WORD_ROBOT_WHEEL_RADIUS_RIGHT          :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT]);
        case WORD_ROBOT_ENCODER_CPR_LEFT            :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT]);
        case WORD_ROBOT_ENCODER_CPR_RIGHT           :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT]);
        case WORD_ROBOT_MOTOR_RPMMAX_LEFT           :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT]);
        case WORD_ROBOT_MOTOR_RPMMAX_RIGHT          :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT]);
        case WORD_ROBOT_MOTOR_IMAX_LEFT             :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_LEFT]);
        case WORD_ROBOT_MOTOR_IMAX_RIGHT            :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_RIGHT]);
        case WORD_ROBOT_MOTOR_TORQUEMAX_LEFT        :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_LEFT]);
        case WORD_ROBOT_MOTOR_TORQUEMAX_RIGHT       :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_RIGHT]);
        case WORD_ROBOT_GEARBOX_RATIO_AXE_LEFT      :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT]);
        case WORD_ROBOT_GEARBOX_RATIO_AXE_RIGHT     :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT]);
        case WORD_ROBOT_GEARBOX_RATIO_MOTOR_LEFT    :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT]);
        case WORD_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT   :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT]);

/* *****************************************************************************
 WORD MODBUS USATE PER LA CONFIGURAZIONE DEL PID, MAPPATE DALL'INDIRIZZO 250
  ******************************************************************************/
        case WORD_PID_P_LEFT                         :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_PID_P_LEFT]);
        case WORD_PID_I_LEFT                         :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_PID_I_LEFT]);
        case WORD_PID_D_LEFT                         :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_PID_D_LEFT]);
        case WORD_PID_P_RIGHT                        :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_PID_P_RIGHT]);
        case WORD_PID_I_RIGHT                        :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_PID_I_RIGHT]);
        case WORD_PID_D_RIGHT                        :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_PID_D_RIGHT]);

        case WORD_PID_RAMP_LEFT                      :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_LEFT]);
        case WORD_PID_RAMP_RIGHT                     :   return((unsigned int )ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_RIGHT]);
        case WORD_PID_ERROR_LEFT                     :   return((unsigned int )VarModbus[INDICE_PID_ERROR_LEFT]);
        case WORD_PID_ERROR_RIGHT                    :   return((unsigned int )VarModbus[INDICE_PID_ERROR_RIGHT]);


/* *****************************************************************************
 WORD MODBUS USATE PER DEBUG, MAPPATE DALL'INDIRIZZO 60000
  ******************************************************************************/

        case WORD_DEBUG_00               :  return((unsigned int )VarModbus[INDICE_DEBUG_00]);
        case WORD_DEBUG_01               :  return((unsigned int )VarModbus[INDICE_DEBUG_01]);
        case WORD_DEBUG_02               :  return((unsigned int )VarModbus[INDICE_DEBUG_02]);
        case WORD_DEBUG_03               :   return((unsigned int )VarModbus[INDICE_DEBUG_03]);
        case WORD_DEBUG_04               :   return((unsigned int )VarModbus[INDICE_DEBUG_04]);
        case WORD_DEBUG_05               :   return((unsigned int )VarModbus[INDICE_DEBUG_05]);
        case WORD_DEBUG_06               :   return((unsigned int )VarModbus[INDICE_DEBUG_06]);
        case WORD_DEBUG_07               :   return((unsigned int )VarModbus[INDICE_DEBUG_07]);
        case WORD_DEBUG_08               :   return((unsigned int )VarModbus[INDICE_DEBUG_08]);
        case WORD_DEBUG_09               :   return((unsigned int )VarModbus[INDICE_DEBUG_09]);
        case WORD_DEBUG_10               :   return((unsigned int )VarModbus[INDICE_DEBUG_10]);
        case WORD_DEBUG_11               :   return((unsigned int )VarModbus[INDICE_DEBUG_11]);
        case WORD_DEBUG_12               :   return((unsigned int )VarModbus[INDICE_DEBUG_12]);
        case WORD_DEBUG_13               :   return((unsigned int )VarModbus[INDICE_DEBUG_13]);
        case WORD_DEBUG_14               :   return((unsigned int )VarModbus[INDICE_DEBUG_14]);
        case WORD_DEBUG_15               :   return((unsigned int )VarModbus[INDICE_DEBUG_15]);
        case WORD_DEBUG_16               :   return((unsigned int )VarModbus[INDICE_DEBUG_16]);
        case WORD_DEBUG_17               :   return((unsigned int )VarModbus[INDICE_DEBUG_17]);
        case WORD_DEBUG_18               :   return((unsigned int )VarModbus[INDICE_DEBUG_18]);
        case WORD_DEBUG_19               :   return((unsigned int )VarModbus[INDICE_DEBUG_19]);
        
        default                         :   return(Address);
    }
}

unsigned char ScriviWord(unsigned int Address,unsigned int Word)
{   //  Scrive il dato "Word" nella corretta locazione di memoria nell'array VarModbus
    //  oppure esegue per specifici indirizzi operazioni più complesse.
    switch(Address){
        case WORD_ADDRESS_SLAVE         :   //VarModbus[WORD_ADDRESS_SLAVE] = Word;
                                            if( (Word > 0) && (Word < 255) )
                                            {
                                                ParametriEEPROM[EEPROM_MODBUS_ADDRESS_SLAVE] = Word;
                                                if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ADDRESS_SLAVE], EEPROM_MODBUS_ADDRESS_SLAVE);
                                                }
                                            }
                                            break;
        case WORD_STATUSBIT1            :   VarModbus[INDICE_STATUSBIT1] = Word;  break;
        case WORD_STATUSBIT2            :   //VarModbus[INDICE_STATUSBIT2] = Word;  break;
                                            ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] = Word;
                                            if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                            {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2], EEPROM_MODBUS_STATUSBIT2);
                                            }
                                            break;
        


        case WORD_PWM_CH1               :   VarModbus[INDICE_PWM_CH1] = Word;  break;
        case WORD_PWM_CH2               :   VarModbus[INDICE_PWM_CH2] = Word;  break;
        case WORD_COMWATCHDOG_TIME      :   //VarModbus[INDICE_COMWATCHDOG_TIME] = Word;  break;
                                            ParametriEEPROM[EEPROM_MODBUS_COMWATCHDOG_TIME] = Word;
                                            if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                            {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_COMWATCHDOG_TIME], EEPROM_MODBUS_COMWATCHDOG_TIME);
                                            }
                                            break;

        case WORD_FLAG_TARATURA         :   VarModbus[INDICE_FLAG_TARATURA] = TaraturaAnalogiche(Word); break;
        case WORD_VAL_TAR_FS            :   VarModbus[INDICE_VAL_TAR_FS] = Word;  break;
        case WORD_ENC1_TICK             :   VarModbus[INDICE_ENC1_TICK] = Word;  break;
        case WORD_ENC1_PERIOD           :   VarModbus[INDICE_ENC1_PERIOD] = Word;  break;
        case WORD_ENC2_TICK             :   VarModbus[INDICE_ENC2_TICK] = Word;  break;
        case WORD_ENC2_PERIOD           :   VarModbus[INDICE_ENC2_PERIOD] = Word;  break;
        case WORD_ENC1_SPEED            :   //Motore1.L_WheelSpeed = Word; break;
                                            VarModbus[INDICE_ENC1_SPEED] = Word;  break;
        case WORD_ENC2_SPEED            :   //Motore2.L_WheelSpeed = Word; break;
                                            VarModbus[INDICE_ENC2_SPEED] = Word;  break;






/* *****************************************************************************
 WORD MODBUS USATE PER LA CONFIGURAZIONE DEL ROBOT, MAPPATE DALL'INDIRIZZO 200
 ******************************************************************************/
        case WORD_ROBOT_DIMENSION_WEIGHT            :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WEIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WEIGHT], EEPROM_MODBUS_ROBOT_DIMENSION_WEIGHT);
                                                        }
                                                        break;
        case WORD_ROBOT_DIMENSION_WIDTH             :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WIDTH] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WIDTH], EEPROM_MODBUS_ROBOT_DIMENSION_WIDTH);
                                                        }
                                                        break;
        case WORD_ROBOT_DIMENSION_HEIGHT            :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_HEIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_HEIGHT], EEPROM_MODBUS_ROBOT_DIMENSION_HEIGHT);
                                                        }
                                                        break;
        case WORD_ROBOT_DIMENSION_LENGHT            :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_LENGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_LENGHT], EEPROM_MODBUS_ROBOT_DIMENSION_LENGHT);
                                                        }
                                                        break;
        case WORD_ROBOT_DIMENSION_WHEELBASE         :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WHEELBASE] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_DIMENSION_WHEELBASE], EEPROM_MODBUS_ROBOT_DIMENSION_WHEELBASE);
                                                        }
                                                        break;
        case WORD_ROBOT_WHEEL_RADIUS_LEFT           :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT], EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT);
                                                        }
                                                        break;
        case WORD_ROBOT_WHEEL_RADIUS_RIGHT          :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT], EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT);
                                                        }
                                                        break;
        case WORD_ROBOT_ENCODER_CPR_LEFT            :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT], EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT);
                                                        }
                                                        break;
        case WORD_ROBOT_ENCODER_CPR_RIGHT           :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT], EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT);
                                                        }
                                                        break;
        case WORD_ROBOT_MOTOR_RPMMAX_LEFT           :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT], EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT);
                                                        }
                                                        break;
        case WORD_ROBOT_MOTOR_RPMMAX_RIGHT          :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT], EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT);
                                                        }
                                                        break;
        case WORD_ROBOT_MOTOR_IMAX_LEFT             :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_LEFT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_LEFT], EEPROM_MODBUS_ROBOT_MOTOR_IMAX_LEFT);
                                                        }
                                                        break;
        case WORD_ROBOT_MOTOR_IMAX_RIGHT            :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_RIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_IMAX_RIGHT], EEPROM_MODBUS_ROBOT_MOTOR_IMAX_RIGHT);
                                                        }
                                                        break;
        case WORD_ROBOT_MOTOR_TORQUEMAX_LEFT        :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_LEFT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_LEFT], EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_LEFT);
                                                        }
                                                        break;
        case WORD_ROBOT_MOTOR_TORQUEMAX_RIGHT       :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_RIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_RIGHT], EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_RIGHT);
                                                        }
                                                        break;
        case WORD_ROBOT_GEARBOX_RATIO_AXE_LEFT      :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT], EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT);
                                                        }
                                                        break;
        case WORD_ROBOT_GEARBOX_RATIO_AXE_RIGHT     :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT], EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT);
                                                        }
                                                        break;
        case WORD_ROBOT_GEARBOX_RATIO_MOTOR_LEFT    :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT], EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT);
                                                        }
                                                        break;
        case WORD_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT   :   ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT], EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT);
                                                        }
                                                        break;

/* *****************************************************************************
 WORD MODBUS USATE PER LA CONFIGURAZIONE DEL PID, MAPPATE DALL'INDIRIZZO 250
  ******************************************************************************/
        case WORD_PID_P_LEFT                         :   ParametriEEPROM[EEPROM_MODBUS_PID_P_LEFT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_P_LEFT], EEPROM_MODBUS_PID_P_LEFT);
                                                        }
                                                        PID1.Kp = ParametriEEPROM[EEPROM_MODBUS_PID_P_LEFT];
                                                        break;
        case WORD_PID_I_LEFT                         :   ParametriEEPROM[EEPROM_MODBUS_PID_I_LEFT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_I_LEFT], EEPROM_MODBUS_PID_I_LEFT);
                                                        }
                                                        PID1.Ki = ParametriEEPROM[EEPROM_MODBUS_PID_I_LEFT];
                                                        break;
        case WORD_PID_D_LEFT                         :   ParametriEEPROM[EEPROM_MODBUS_PID_D_LEFT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_D_LEFT], EEPROM_MODBUS_PID_D_LEFT);
                                                        }
                                                        PID1.Kd = ParametriEEPROM[EEPROM_MODBUS_PID_D_LEFT];
                                                        break;
        case WORD_PID_P_RIGHT                        :   ParametriEEPROM[EEPROM_MODBUS_PID_P_RIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_P_RIGHT], EEPROM_MODBUS_PID_P_RIGHT);
                                                        }
                                                        PID2.Kp = ParametriEEPROM[EEPROM_MODBUS_PID_P_RIGHT];
                                                        break;
        case WORD_PID_I_RIGHT                        :   ParametriEEPROM[EEPROM_MODBUS_PID_I_RIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_I_RIGHT], EEPROM_MODBUS_PID_I_RIGHT);
                                                        }
                                                        PID2.Ki = ParametriEEPROM[EEPROM_MODBUS_PID_I_RIGHT];
                                                        break;
        case WORD_PID_D_RIGHT                        :   ParametriEEPROM[EEPROM_MODBUS_PID_D_RIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_D_RIGHT], EEPROM_MODBUS_PID_D_RIGHT);
                                                        }
                                                        PID2.Kd = ParametriEEPROM[EEPROM_MODBUS_PID_D_RIGHT];
                                                        break;


        case WORD_PID_RAMP_LEFT                        :   ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_LEFT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_LEFT], EEPROM_MODBUS_PID_RAMP_LEFT);
                                                        }
                                                        PID1.RampaStep = ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_LEFT];
                                                        break;
        case WORD_PID_RAMP_RIGHT                        :   ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_RIGHT] = Word;
                                                        if(VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_SAVE_EN)
                                                        {   DataEEWrite(ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_RIGHT], EEPROM_MODBUS_PID_RAMP_RIGHT);
                                                        }
                                                        PID2.RampaStep = ParametriEEPROM[EEPROM_MODBUS_PID_RAMP_RIGHT];
                                                        break;


/* *****************************************************************************
 WORD MODBUS USATE PER DEBUG, MAPPATE DALL'INDIRIZZO 60000
  ******************************************************************************/

        case WORD_DEBUG_00               :   VarModbus[INDICE_DEBUG_00] = Word;
        case WORD_DEBUG_01               :   VarModbus[INDICE_DEBUG_01] = Word;
        case WORD_DEBUG_02               :   VarModbus[INDICE_DEBUG_02] = Word;
        case WORD_DEBUG_03               :   VarModbus[INDICE_DEBUG_03] = Word;
        case WORD_DEBUG_04               :   VarModbus[INDICE_DEBUG_04] = Word;
        case WORD_DEBUG_05               :   VarModbus[INDICE_DEBUG_05] = Word;
        case WORD_DEBUG_06               :   VarModbus[INDICE_DEBUG_06] = Word;
        case WORD_DEBUG_07               :   VarModbus[INDICE_DEBUG_07] = Word;
        case WORD_DEBUG_08               :   VarModbus[INDICE_DEBUG_08] = Word;
        case WORD_DEBUG_09               :   VarModbus[INDICE_DEBUG_09] = Word;
        case WORD_DEBUG_10               :   VarModbus[INDICE_DEBUG_10] = Word;
        case WORD_DEBUG_11               :   VarModbus[INDICE_DEBUG_11] = Word;
        case WORD_DEBUG_12               :   VarModbus[INDICE_DEBUG_12] = Word;
        case WORD_DEBUG_13               :   VarModbus[INDICE_DEBUG_13] = Word;
        case WORD_DEBUG_14               :   VarModbus[INDICE_DEBUG_14] = Word;
        case WORD_DEBUG_15               :   VarModbus[INDICE_DEBUG_15] = Word;
        case WORD_DEBUG_16               :   VarModbus[INDICE_DEBUG_16] = Word;
        case WORD_DEBUG_17               :   VarModbus[INDICE_DEBUG_17] = Word;
        case WORD_DEBUG_18               :   VarModbus[INDICE_DEBUG_18] = Word;
        case WORD_DEBUG_19               :   VarModbus[INDICE_DEBUG_19] = Word;


        default				:   break;
    }
    return(OK);
}


/*  ***************************************************************************
 *  ***************************************************************************
 *                              Protocollo modbus
 *  ***************************************************************************
 *  ***************************************************************************
 */
/*
void ModbusRoutine(Port);
void ModbusRxRoutine(unsigned char Code, unsigned char Port);
void ModbusReadWord(unsigned char Port);
void SingleWordWriting(unsigned char Port);
void RispostaErrore(unsigned char NumeroErrore, unsigned char Port);
unsigned int CalcolaCheckCRC16(unsigned char *P,unsigned char NByte);
unsigned int LeggiWord(unsigned int Address);
unsigned char ScriviWord(unsigned int Address,unsigned int Word);
void FreeRxBuffer(Port);
void InizializzaSeriale(unsigned char Port);
*/
void ModbusRoutine(unsigned char Port)
{   unsigned char n,ByteAspettati;
    unsigned int Check;
    switch(StatoSeriale[Port]){
        case    WAIT_MESSAGE	:   n = RxNByte[Port];
                                    if (n >= 8)
                                    {   switch(ModbusRxBuff[Port][1]){
                                            case CODE_BITS_READING              :
                                            case CODE_BITS_READING_BIS          :
                                            case CODE_WORDS_READING		:
                                            case CODE_WORDS_READING_BIS		:
                                            case CODE_SINGLE_BIT_WRITING	:
                                            case CODE_SINGLE_WORD_WRITING	:   ByteAspettati = 8; break;
                                            case CODE_MULTIPLE_BITS_WRITING	:
                                            case CODE_MULTIPLE_WORDS_WRITING	:   ByteAspettati = 9 + ModbusRxBuff[Port][6]; break;
                                            default                             :   ByteAspettati = 8; break;
                                        }

                                        if (n == ByteAspettati)
                                        {
                                            Check = CalcolaCheckCRC16(ModbusRxBuff[Port],n-2);
                                            if ((ModbusRxBuff[Port][n-2] == (unsigned char)(Check)) && (ModbusRxBuff[Port][n-1] == (unsigned char)(Check >> 8))	)
                                            {   // Il Ckeck ricevuto è valido


                                                if (    //ModbusRxBuff[Port][0] == INDIRIZZO_DISPOSITIVO ||
                                                        ModbusRxBuff[Port][0] == ParametriEEPROM[EEPROM_MODBUS_ADDRESS_SLAVE] ||
                                                        ModbusRxBuff[Port][0] == 0xFF ||
                                                        ModbusRxBuff[Port][0] == 0x00)
                                                {
                                                    //L'indirizzo a cui è destinato il pacchetto corrisponde o a quello della scheda
                                                    //o a quello di broadcast.
                                                    TimerRitardoModbus[Port] = RITARDO_RISPOSTA_SERIALE;
                                                    ComunicationWatchDogTimer = ParametriEEPROM[EEPROM_MODBUS_COMWATCHDOG_TIME]; //VarModbus[INDICE_COMWATCHDOG_TIME];
                                                    LED2 = PIN_ON;
                                                    
                                                    // Salto all'elaborazione del pacchetto ricevuto.
                                                    StatoSeriale[Port] = WAIT_TX;

                                                }
                                                else
                                                {	FreeRxBuffer(Port);
                                                }
                                            }
                                            else
                                            {	FreeRxBuffer(Port);
                                            }
                                        }
                                    }
                                    break;

	case	WAIT_TX         :   if(TimerRitardoModbus[Port] == 0)
                                        {   // Attendo il timeout, elaboro il dato ricevuto e mi rimetto in ascolto...
                                            ModbusRxRoutine(ModbusRxBuff[Port][1], Port);
                                            StatoSeriale[Port] = WAIT_MESSAGE;
					}
					break;

        case	INIT_COM	:   // Riinizializzo la seriale.
                                    InizializzaSeriale(Port);
                                    break;
    }
}

/*! \brief Determina in base al CODICE MODBUS ricevuto che tipo di
 *         lettura effettuare
 *
 *  Questa funzione è unica per tutti i casi di lettura modbus e determina quale
 *  funzione chiamare per accedere al dato da leggere in relazione al CODICE
 *  MODUBUS ricevuto
 */
/*!
  \param Code codice modbus a cui rispondere.
  \param Port numero che indicha su che porta seriale agire.
  \return void
*/
void ModbusRxRoutine(unsigned char Code, unsigned char Port)
{   switch(Code){
        case CODE_BITS_READING              :
	case CODE_BITS_READING_BIS          :	ModbusReadBit(Port);              break;
        case CODE_WORDS_READING             :
        case CODE_WORDS_READING_BIS         :   ModbusReadWord(Port);             break;
        case CODE_SINGLE_BIT_WRITING        :	ModbusWriteSingleBit(Port);         break;
        case CODE_SINGLE_WORD_WRITING       :	SingleWordWriting(Port);        break;
        case CODE_MULTIPLE_BITS_WRITING     :   MultipleBitsWriting(Port);	break;
	case CODE_MULTIPLE_WORDS_WRITING    :	MultipleWordsWriting(Port);	break;
	default                             :   RispostaErrore(ILLEGAL_FUNCTION_CODE, Port);	break;
    }
}

void ModbusReadBit(unsigned char Port)
{	unsigned int StartAddress,NBit,Check,i;

	StartAddress 	= (unsigned int)(ModbusRxBuff[Port][2] << 8) + ModbusRxBuff[Port][3];
	NBit 			= (unsigned int)(ModbusRxBuff[Port][4] << 8) + ModbusRxBuff[Port][5];

	ModbusTxBuff[Port][0] = ModbusRxBuff[Port][0];
	ModbusTxBuff[Port][1] = ModbusRxBuff[Port][1];
	ModbusTxBuff[Port][2] = ((NBit - 1) / 8) + 1;

	for(i=0; i<ModbusTxBuff[Port][2]; i++)
		ModbusTxBuff[Port][3+i] = 0;

	for(i=0; i<NBit; i++)
	{
		if (LeggiBit(StartAddress + i))
			ModbusTxBuff[Port][3+(i/8)] |= TabMaskBitModbus[i % 8];

	}

	Check = CalcolaCheckCRC16(ModbusTxBuff[Port],(3 + ModbusTxBuff[Port][2]));
	ModbusTxBuff[Port][3+ModbusTxBuff[Port][2]] = Check;
	ModbusTxBuff[Port][4+ModbusTxBuff[Port][2]] = Check >> 8;
	FreeRxBuffer(Port);;
	if (ModbusTxBuff[Port][0])
		TxString(ModbusTxBuff[Port],(3 + ModbusTxBuff[Port][2] + 2),Port);
}

void ModbusReadWord(unsigned char Port)
{   unsigned int StartAddress,NWord,Word,Check,i;
    StartAddress    = ((unsigned int)ModbusRxBuff[Port][2] << 8) + ModbusRxBuff[Port][3];
    NWord           = ((unsigned int)ModbusRxBuff[Port][4] << 8) + ModbusRxBuff[Port][5];

    if (NWord > MAX_WORD_LETTURA_MULTIPLA)
	{
            RispostaErrore(ILLEGAL_FUNCTION_CODE, Port);
	}
	else
	{   ModbusTxBuff[Port][0] = ModbusRxBuff[Port][0];
            ModbusTxBuff[Port][1] = ModbusRxBuff[Port][1];
            ModbusTxBuff[Port][2] = NWord << 1;
            for(i=0; i<NWord; i++)
            {	Word = LeggiWord(StartAddress + i);
            	ModbusTxBuff[Port][3+(i*2)] = Word >> 8;
		ModbusTxBuff[Port][4+(i*2)] = Word;
            }
            Check = CalcolaCheckCRC16(ModbusTxBuff[Port],(3 + (NWord << 1)));
            ModbusTxBuff[Port][3+(i*2)] = Check;
            ModbusTxBuff[Port][4+(i*2)] = Check >> 8;
            FreeRxBuffer(Port);
            if (ModbusTxBuff[Port][0])
            	TxString(ModbusTxBuff[Port],(3 + (NWord << 1) + 2), Port);
	}
}

void ModbusWriteSingleBit(unsigned char Port)
{   unsigned int Address; //,Check;
    unsigned char Bit,Risultato;

    Address = (unsigned int)(ModbusRxBuff[Port][2] << 8) + ModbusRxBuff[Port][3];
    Bit     = ((unsigned int)(ModbusRxBuff[Port][4] << 8) + ModbusRxBuff[Port][5]) == 0xFF00 ? ON : OFF;

    Risultato = ScriviBit(Address,Bit);

    switch(Risultato)
    {   case OK :   FreeRxBuffer(Port);
                    if (ModbusRxBuff[Port][0])
                        TxString(ModbusRxBuff[Port],8,Port);
                    break;
        default :   RispostaErrore(Risultato,Port);
                    break;
	}
}

void SingleWordWriting(unsigned char Port)
{   unsigned int Address,Word;
    unsigned char Risultato;

    Address 	= ((unsigned int)ModbusRxBuff[Port][2] << 8) + ModbusRxBuff[Port][3];
    Word 	= ((unsigned int)ModbusRxBuff[Port][4] << 8) + ModbusRxBuff[Port][5];

    Risultato = ScriviWord(Address,Word);
    switch(Risultato){
        case OK	:   FreeRxBuffer(Port);
                    if (ModbusRxBuff[Port][0])    TxString(ModbusRxBuff[Port],8, Port);
                    break;
        default	:   RispostaErrore(Risultato, Port);
                    break;
    }
}

void MultipleBitsWriting(unsigned char Port)
{
	unsigned int StartAddress,NBit,Check,i;
	unsigned char Risultato,Bit;

	StartAddress 	= (unsigned int)(ModbusRxBuff[Port][2] << 8) + ModbusRxBuff[Port][3];
	NBit 			= (unsigned int)(ModbusRxBuff[Port][4] << 8) + ModbusRxBuff[Port][5];


	for(i=0; i<NBit; i++)
	{
		Bit = (ModbusRxBuff[Port][7+(i/8)] & (TabMaskBitModbus[i % 8])) ? ON : OFF;
		Risultato = ScriviBit(StartAddress + i,Bit);

		switch(Risultato)
		{	case OK		:	break;
			default		:	RispostaErrore(Risultato,Port);
							return;
		}
	}
	for(i=0; i<6; i++)
	{	ModbusTxBuff[Port][i] = ModbusRxBuff[Port][i];
	}
	Check = CalcolaCheckCRC16(ModbusRxBuff[Port],6);
	ModbusTxBuff[Port][6] = Check;
	ModbusTxBuff[Port][7] = Check >> 8;
	FreeRxBuffer(Port);
	if (ModbusTxBuff[Port][0])
		TxString(ModbusTxBuff[Port],8,Port);
}


void MultipleWordsWriting(unsigned char Port)
{
	unsigned int StartAddress,NWord,Word,Check,i;
	unsigned char Risultato;

	StartAddress 	= (unsigned int)(ModbusRxBuff[Port][2] << 8) + ModbusRxBuff[Port][3];
	NWord 			= (unsigned int)(ModbusRxBuff[Port][4] << 8) + ModbusRxBuff[Port][5];

	for(i=0; i<NWord; i++)
	{	Word = (unsigned int)(ModbusRxBuff[Port][7+(i*2)] << 8) + ModbusRxBuff[Port][8+(i*2)];
		Risultato = ScriviWord(StartAddress + i,Word);

		switch(Risultato)
		{	case OK		:	break;
			default		:	RispostaErrore(Risultato, Port);
							return;
		}
	}
	for(i=0; i<6; i++)
	{	ModbusTxBuff[Port][i] = ModbusRxBuff[Port][i];
	}
	Check = CalcolaCheckCRC16(ModbusTxBuff[Port],6);
	ModbusTxBuff[Port][6] = Check;
	ModbusTxBuff[Port][7] = Check >> 8;
	FreeRxBuffer(Port);
	if (ModbusTxBuff[Port][0])
		TxString(ModbusTxBuff[Port],8,Port);
}


void RispostaErrore(unsigned char NumeroErrore, unsigned char Port)
{   unsigned int Check;
    ModbusTxBuff[Port][0] = ModbusRxBuff[Port][0];
    ModbusTxBuff[Port][1] = ModbusRxBuff[Port][1] | 0x80;
    ModbusTxBuff[Port][2] = NumeroErrore;
    Check = CalcolaCheckCRC16(ModbusTxBuff[Port],3);
    ModbusTxBuff[Port][3] = Check;
    ModbusTxBuff[Port][4] = Check >> 8;
    FreeRxBuffer(Port);
    // Rispondo solo se l'indirizzo è diverso da 0.
    if (ModbusTxBuff[Port][0])    TxString(ModbusTxBuff[Port],5, Port);
}

unsigned int CalcolaCheckCRC16(unsigned char *P,unsigned char NByte)
{   unsigned int Check;
    char i,a;

    Check = 0xFFFF;
    for(i=0; i<NByte; i++)
    {	Check ^= *P;
        for(a=0; a<8; a++)
        {   if (Check & 0x0001)
            {   Check = Check >> 1;
        	Check ^= 0xA001;
            }
            else
            {	Check = Check >> 1;
            }
        }
    	P++;
    }
    return(Check);
}

// Funzioni di Gestione della Seriale per protocollo Modbus
void FreeRxBuffer(unsigned char Port)
{   RxPointer[Port] = ModbusRxBuff[Port];	/*	azzera il puntatore di ricezione	*/
    RxNByte[Port] = 0;				/*	e il numero di byte ricevuti		*/
}

void InizializzaSeriale(unsigned char Port)
{   if(Port == PORT_COM1)
    {
        Usart1Setting();
    }
    else
    {
        Usart2Setting();
    }

    FreeRxBuffer(Port);
    TimerOutRxModbus[Port] = TIME_OUT_MODBUS;
    StatoSeriale[Port] = WAIT_MESSAGE;
    // SET_DIR_RX; // Per RS485
}


unsigned char LeggiBit(unsigned int Address)
{       //Mi permette di accedere a "Bit" a ciascuna word modbus
        return((VarModbus[Address / BIT_PER_WORD] & TabMaskBitIO[Address % BIT_PER_WORD]) ? 1 : 0);
}


unsigned char ScriviBit(unsigned int Address,unsigned char Bit)
{   if (Bit)
        VarModbus[Address / BIT_PER_WORD] |= TabMaskBitIO[Address % BIT_PER_WORD];
    else
        VarModbus[Address / BIT_PER_WORD] &=~TabMaskBitIO[Address % BIT_PER_WORD];
	return(OK);
}

