// Define relative ai parametri da salvare in EEPROM
// Indica la posizione nell'array ParametriEEPROM[] che è l'array che poi viene
// salvato a comando e ripristinato all'accensione.
// La stessa define indica anche la posizione in cui salvare il dato in EEPROM.
// Nel DsPIC33 l'EEPROM è organizzata a word.

#define TARAT_ZERO_RB_AN0                   0   // Valore di taratura per lo 0 in ingresso
#define TARAT_ZERO_RB_AN1                   1
#define TARAT_ZERO_RB_AN2                   2
#define TARAT_ZERO_RB_AN3                   3
#define TARAT_ZERO_RB_AN4                   4
#define TARAT_CONV_RB_AN0                   5   // Valore di conversione per l'ingresso
#define TARAT_CONV_RB_AN1                   6
#define TARAT_CONV_RB_AN2                   7
#define TARAT_CONV_RB_AN3                   8
#define TARAT_CONV_RB_AN4                   9

#define TARAT_COSTCONV_AN0_LOW              10  // CostanteTaraturaAN0.fval
#define TARAT_COSTCONV_AN0_HIGH             11
#define TARAT_COSTCONV_AN1_LOW              12  // CostanteTaraturaAN1.fval
#define TARAT_COSTCONV_AN1_HIGH             13
#define TARAT_COSTCONV_AN2_LOW              14  // CostanteTaraturaAN2.fval
#define TARAT_COSTCONV_AN2_HIGH             15
#define TARAT_COSTCONV_AN3_LOW              16  // CostanteTaraturaAN3.fval
#define TARAT_COSTCONV_AN3_HIGH             17
#define TARAT_COSTCONV_AN4_LOW              18  // CostanteTaraturaAN4.fval
#define TARAT_COSTCONV_AN4_HIGH             19

#define EEPROM_MODBUS_ADDRESS_SLAVE         20  // Memorizza l'ID del dispositivo
#define EEPROM_MODBUS_COMWATCHDOG_TIME      21  // Memorizza l'ID del dispositivo

#define EEPROM_MODBUS_STATUSBIT2            22  // La variabile STATUSBIT2 è salvata in EEPROM

//#define WEEL1_RAD                           22  // Raggio in mt della ruota, lo 0. iniziale è sottointeso
//#define WEEL2_RAD                           23  // 65535 = 0.65535mt = 65,535 Cm = 655,35mm
                                                // Oppure... // Raggio in Centesimi di mm della ruota...

//#define WEEL1_CPR                           24  // Numero di impulsi encoder per rivoluzione della ruota
//#define WEEL2_CPR                           25  // Numero di impulsi encoder per rivoluzione della ruota


/* *****************************************************************************
 WORD MODBUS USATE PER LA CONFIGURAZIONE DEL ROBOT, MAPPATE DALL'INDIRIZZO 200
 ******************************************************************************/
#define EEPROM_MODBUS_ROBOT_DIMENSION_WEIGHT            26
#define EEPROM_MODBUS_ROBOT_DIMENSION_WIDTH             27
#define EEPROM_MODBUS_ROBOT_DIMENSION_HEIGHT            28
#define EEPROM_MODBUS_ROBOT_DIMENSION_LENGHT            29
#define EEPROM_MODBUS_ROBOT_DIMENSION_WHEELBASE         30
#define EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_LEFT           31
#define EEPROM_MODBUS_ROBOT_WHEEL_RADIUS_RIGHT          32
#define EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT            33
#define EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT           34
#define EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT           35
#define EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT          36
#define EEPROM_MODBUS_ROBOT_MOTOR_IMAX_LEFT             37
#define EEPROM_MODBUS_ROBOT_MOTOR_IMAX_RIGHT            38
#define EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_LEFT        39
#define EEPROM_MODBUS_ROBOT_MOTOR_TORQUEMAX_RIGHT       40
#define EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_LEFT      41
#define EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_AXE_RIGHT     42
#define EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_LEFT    43
#define EEPROM_MODBUS_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT   44

/* *****************************************************************************
 WORD MODBUS USATE PER LA CONFIGURAZIONE DEL PID, MAPPATE DALL'INDIRIZZO 250
  ******************************************************************************/
#define EEPROM_MODBUS_PID_P_LEFT                         45
#define EEPROM_MODBUS_PID_I_LEFT                         46
#define EEPROM_MODBUS_PID_D_LEFT                         47
#define EEPROM_MODBUS_PID_P_RIGHT                        48
#define EEPROM_MODBUS_PID_I_RIGHT                        49
#define EEPROM_MODBUS_PID_D_RIGHT                        50

#define EEPROM_MODBUS_PID_RAMP_LEFT                      51
#define EEPROM_MODBUS_PID_RAMP_RIGHT                     52

#define EEPROM_MODBUS_ROBOT_CHARGED_BATT                 53
#define EEPROM_MODBUS_ROBOT_DISCHARGED_BATT              54

#define NUMERO_PARAMETRI_EEPROM                          55  // N° totale di parametri in EEPROM

