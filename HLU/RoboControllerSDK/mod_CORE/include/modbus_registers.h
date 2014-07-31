/*! \file */

#define N_PARAMETRI                     42       //!< Dimensione dell'array usato per le variabili modbus

/*  Posizione nell'array VarModbus[], non sono quindi contemplate le costanti e gli altri
 *  dati accessibili da modbus ma non inseriti in VarModbus[] ( tipo i parametri in EERPOM )
 */
#define INDICE_STATUSBIT1               0
//#define INDICE_STATUSBIT2               1     // Spostata in EEPROM
#define INDICE_PWM_CH1                  2
#define INDICE_PWM_CH2                  3
#define INDICE_TENSIONE_ALIM            4
#define INDICE_AN1                      5
#define INDICE_AN2                      6
#define INDICE_AN3                      7
#define INDICE_AN4                      8
#define INDICE_FLAG_TARATURA            9
#define INDICE_VAL_TAR_FS               10  //!< Valore usato in taratura per tare il fondoscala
#define INDICE_ENC1_TICK                11
#define INDICE_ENC1_PERIOD              12
#define INDICE_ENC2_TICK                13
#define INDICE_ENC2_PERIOD              14
#define INDICE_ENC1_SPEED               15
#define INDICE_ENC2_SPEED               16
#define INDICE_RD_PWM_CH1               17
#define INDICE_RD_PWM_CH2               18
#define INDICE_PID_ERROR_LEFT           19
#define INDICE_PID_ERROR_RIGHT          20



#define INDICE_DEBUG_00                 21  //!< Serve per monitorare nella GUI un valore a caso ( modificando il FW )
#define INDICE_DEBUG_01                 22
#define INDICE_DEBUG_02                 23
#define INDICE_DEBUG_03                 24
#define INDICE_DEBUG_04                 25
#define INDICE_DEBUG_05                 26
#define INDICE_DEBUG_06                 27
#define INDICE_DEBUG_07                 28
#define INDICE_DEBUG_08                 29
#define INDICE_DEBUG_09                 30
#define INDICE_DEBUG_10                 31
#define INDICE_DEBUG_11                 32
#define INDICE_DEBUG_12                 33
#define INDICE_DEBUG_13                 34
#define INDICE_DEBUG_14                 35
#define INDICE_DEBUG_15                 36
#define INDICE_DEBUG_16                 37
#define INDICE_DEBUG_17                 38
#define INDICE_DEBUG_18                 39
#define INDICE_DEBUG_19                 40

/* *****************************************************************************
 Flag per agire sui bit delle word modbus WORD_STATUSBIT1
 ******************************************************************************/

///*! \def FLG_JOYMODE
// *  \brief Flag per agire sui bit delle word modbus WORD_STATUSBIT1
// */
//#define FLG_STATUSBI1_JOYMODE                     0x0001  //!< Se "1" la robocontroller funzionerÃ  in modalitÃ  JoyStick

/*! \def FLG_STATUSBI1_PID_EN
 *  \brief Se a "1" attiva la modalitÃ  con PID e nelle word WORD_PWM_CHx si metterÃ  il dato di velocitÃ  da mantenere espresso in mm/Sec
 * mentre se a "0" il dato in WORD_PWM_CHx sarÃ  interpretato come PWM puro ( 0-4096 con motori fermi a 2048 ).
 */
#define FLG_STATUSBI1_PID_EN                     0x0001  //!< Se "1" sarÃ  abilitato il PID.

/*! \def FLG_JOYMODE
 *  \brief Flag per agire sui bit delle word modbus WORD_STATUSBIT1
 */
#define FLG_STATUSBI1_COMWATCHDOG                 0x0002  //!< Se "1" viene abilitato il watchdog

/*! \def FLG_EEPROM_SAVE_EN
 *  \brief Flag per agire sui bit delle word modbus WORD_STATUSBIT1 e attivare il salvataggio automatico dei parametri in EEPROM
 */
#define FLG_STATUSBI1_EEPROM_SAVE_EN              0x0004  //!< Se "1" viene abilitato il salvataggio automatico dei parametri in EEPROM

/*! \def FLG_EEPROM_RAMP_EN
 *  \brief Flag per agire sui bit delle word modbus WORD_STATUSBIT1 e attivare il salvataggio automatico dei parametri in EEPROM
 */
#define FLG_STATUSBI1_EEPROM_RAMP_EN              0x0008  //!< Se "1" viene abilitato il funzionamento RAMPA


/* *****************************************************************************
 Flag per agire sui bit delle word modbus WORD_STATUSBIT2
 WORD_STATUSBIT2 Ã¨ salvata in EEPROM
 ******************************************************************************/

/*! \def FLG_EEPROM_ENCODER_POSITION
 * \brief Se "0" Encoder calettato al motore, se "1" encoder calettato alla ruota
 */
#define FLG_STATUSBI2_EEPROM_ENCODER_POSITION     0x0001  //!< Se "0" Encoder calettato al motore, se "1" encoder calettato alla ruota

/*! \def FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY
 * \brief Controlla la polaritÃ  del segnale di Enable verso il driver del motore. Se "0" il driver Ã¨ attivo basso, se "1" il driver Ã¨ attivo con Enable = "1"
 */
#define FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY     0x0002  //!< Controlla la polaritÃ  del segnale di Enable verso il driver del motore. Se "0" il driver Ã¨ attivo basso, se "1" il driver Ã¨ attivo con Enable = "1"


// ---> Registri

/*! \def WORD_TIPO_DISPOSITIVO
\brief	( R ) Serve ad identificare differenti configurazioni della scheda RoboController
    Ritorna il dato letto nella define \ref TIPO_DISPOSITIVO come tipo di dispositivo
*/
#define WORD_TIPO_DISPOSITIVO           0

/*! \def WORD_VERSIONE_FIRMWARE
\brief	( R ) Numero progressivo che indica le versioni firmware.
    Ritorna il dato letto nella define \ref VERSIONE_FIRMWARE
    Parte da 1 per ciascun differente \ref WORD_TIPO_DISPOSITIVO
*/
#define WORD_VERSIONE_FIRMWARE          1

/*! \def WORD_ADDRESS_SLAVE
\brief	( R/W EEPROM ) Indirizzo modbus del dispositivo.
        Varia tra 1 e 254 ( default=1 )
*/
#define WORD_ADDRESS_SLAVE              2

/*! \def WORD_RITARDO_SERIALE
\brief	( R ) Tempo espresso in mSec che il dispositivo attende prima di inviare
    la risposta Ã© particolarmente importante in caso di collegamenti Wireless
    o dova la commutazione TX/RX del canale di comunicazione non sia istantanea.
    Ritorna il dato letto nella define \ref RITARDO_RISPOSTA_SERIALE
*/
#define WORD_RITARDO_SERIALE            3

/*! \def WORD_STATUSBIT1
\brief 	( R/W ) Word utilizzata a BIT per abilitare/disabilitare le varie funzioni della scheda.

    *  - WORD_STATUSBIT1.0 : Flag \ref FLG_JOYMODE, modalitÃ  RoboController/JoyStick
    *      Se a 1 RoboController in modalitÃ  joystick manuale, ai motori vengono inviati direttamente
    *      i valori delle WORD \ref WORD_PWM_CH1 e \ref WORD_PWM_CH2, sono bypassati tutti i controlli e le
    *      protezioni, utile per test manuali sulla meccanica e per portare robot pesanti in posizione.
    *  - WORD_STATUSBIT1.1 : Flag \ref FLG_COMWATCHDOG, attiva WatchDog su comunicazione
    *      Se a "1" viene attivato un controllo sulla  comunicazione e, dopo assenza di comunicazione
    *      per VarModbus[\ref INDICE_COMWATCHDOG_TIME] mSec il programma ferma i motori portando a Zero la
    *      loro velocitÃ . I motori non sono disattivati e basta riscrivere la velocitÃ  per ripartire,
    *      questo permette di recuperare il robot.
    *  - WORD_STATUSBIT1.2 : Salvataggio dati in EEPROM, se messo a 1 i dati che prevedono di essere me
        *      memorizzati in EEPROM si salvano automaticamente. Se messo a "0" vengono cambiati ma non salvati e
        *      alla riaccensione vengono ricaricati gli ultimi dati letti.
    *  - WORD_STATUSBIT1.3 :
    *  - WORD_STATUSBIT1.4 :
*/
#define WORD_STATUSBIT1                 4

/*! \def WORD_STATUSBIT2
\brief	( R/W )
*/
#define WORD_STATUSBIT2                 5

/*! \def WORD_PWM_CH1
\brief	( R/W )
*/
#define WORD_PWM_CH1                    6

/*! \def WORD_PWM_CH2
\brief	( R/W ))
*/
#define WORD_PWM_CH2                    7

/*! \def WORD_TENSIONE_ALIM
\brief	( R )
*/
#define WORD_TENSIONE_ALIM              8

/*! \def WORD_AN1
\brief	( R )
*/
#define WORD_AN1                        9

/*! \def WORD_AN2
\brief	( R )
*/
#define WORD_AN2                        10

/*! \def WORD_AN3
\brief	( R )
*/
#define WORD_AN3                        11

/*! \def WORD_AN4
\brief	( R )
*/
#define WORD_AN4                        12

/*! \def WORD_COMWATCHDOG_TIME
\brief	( R/W EEPROM ) Tempo (mSec) massimo tra due comunicazioni prima di bloccare i motori.
*/
#define WORD_COMWATCHDOG_TIME           13

/*! \def  WORD_FLAG_TARATURA
\brief	( W ) Word gestita a bit per avviare la taratura degli ingressi
*/
#define WORD_FLAG_TARATURA              14

/*! \def  WORD_VAL_TAR_FS
\brief	( W ) Valore da associare al fondoscala dell'ingresso analogico in taratura
*/
#define WORD_VAL_TAR_FS                 15

/*! \def WORD_ENC1_TICK
\brief 	( R )
*/
#define WORD_ENC1_TICK                  16

/*! \def WORD_ENC1_PERIOD
\brief	( R )
*/
#define WORD_ENC1_PERIOD                17

/*! \def WORD_ENC2_TICK
\brief	( R )
*/
#define WORD_ENC2_TICK                  18

/*! \def WORD_ENC2_PERIOD
\brief	( R )
*/
#define WORD_ENC2_PERIOD                19

/*! \def WORD_ENC1_SPEED
\brief	( R )
*/
#define WORD_ENC1_SPEED                 20

/*! \def WORD_ENC2_SPEED
\brief	( R )
*/
#define WORD_ENC2_SPEED                 21

/*! \def WORD_RD_PWM_CH1 : Mi permette in ogni istante di leggere il dato effettivo di PWM, utile per il tuning
\brief	( R )
*/
#define WORD_RD_PWM_CH1                 22

/*! \def WORD_RD_PWM_CH2 : Mi permette in ogni istante di leggere il dato effettivo di PWM, utile per il tuning
\brief	( R )
*/
#define WORD_RD_PWM_CH2                 23

/* *****************************************************************************
 WORD MODBUS USATE PER LA TELEMETRIAL ROBOT, MAPPATE DALL'INDIRIZZO 50
  ******************************************************************************/

// VelocitÃ  angolare della ruota in rad/Sec
#define WORD_ROBOT_WHEEL_SPEED_LEFT             50
#define WORD_ROBOT_WHEEL_SPEED_RIGHT            51

// VelocitÃ  lineare del robot in mm/Sec
// non abbiamo i decimali in modbus, per i m/Sec basta dividere per 100
#define WORD_ROBOT_SPEED                        52

// ...non so come esprimerla...
#define WORD_ROBOT_DIRECTION                    53




///*! \def WORD_PID1_DEBUG
//\brief	( R )
//*/
//#define WORD_PID1_DEBUG                 22
//
///*! \def WORD_PID2_DEBUG
//\brief	( R )
//*/
//#define WORD_PID2_DEBUG                 23

/* *****************************************************************************
 WORD MODBUS USATE PER LA CONFIGURAZIONE DEL ROBOT, MAPPATE DALL'INDIRIZZO 200
  ******************************************************************************/
#define WORD_ROBOT_DIMENSION_WEIGHT             200
#define WORD_ROBOT_DIMENSION_WIDTH              201
#define WORD_ROBOT_DIMENSION_HEIGHT             202
#define WORD_ROBOT_DIMENSION_LENGHT             203
#define WORD_ROBOT_DIMENSION_WHEELBASE          204
#define WORD_ROBOT_WHEEL_RADIUS_LEFT            205 // Raggio in Centesimi di mm della ruota ( 3,6Cm = 3600 Cent/mm)
#define WORD_ROBOT_WHEEL_RADIUS_RIGHT           206 // Raggio in Centesimi di mm della ruota ( 3,6Cm = 3600 Cent/mm)
#define WORD_ROBOT_ENCODER_CPR_LEFT             207 // Numero di impulsi encoder per rivoluzione della ruota ( giÃ  moltiplicato per 4 )
#define WORD_ROBOT_ENCODER_CPR_RIGHT            208 // Numero di impulsi encoder per rivoluzione della ruota ( giÃ  moltiplicato per 4 )
//#define FLG2_EEPROM_ENCODER_POSITION     0x0001  //!< Se "0" Encoder calettato al motore, se "1" encoder calettato alla ruota
#define WORD_ROBOT_MOTOR_RPMMAX_LEFT            209
#define WORD_ROBOT_MOTOR_RPMMAX_RIGHT           210
#define WORD_ROBOT_MOTOR_IMAX_LEFT              211
#define WORD_ROBOT_MOTOR_IMAX_RIGHT             212
#define WORD_ROBOT_MOTOR_TORQUEMAX_LEFT         213
#define WORD_ROBOT_MOTOR_TORQUEMAX_RIGHT        214
#define WORD_ROBOT_GEARBOX_RATIO_AXE_LEFT       215
#define WORD_ROBOT_GEARBOX_RATIO_AXE_RIGHT      216
#define WORD_ROBOT_GEARBOX_RATIO_MOTOR_LEFT     217
#define WORD_ROBOT_GEARBOX_RATIO_MOTOR_RIGHT    218
#define WORD_ROBOT_CHARGED_BATT                 219
#define WORD_ROBOT_DISCHARGED_BATT              220

/* *****************************************************************************
 WORD MODBUS USATE PER LA CONFIGURAZIONE DEL PID, MAPPATE DALL'INDIRIZZO 250
  ******************************************************************************/
#define WORD_PID_P_LEFT                         250
#define WORD_PID_I_LEFT                         251
#define WORD_PID_D_LEFT                         252
#define WORD_PID_P_RIGHT                        253
#define WORD_PID_I_RIGHT                        254
#define WORD_PID_D_RIGHT                        255
#define WORD_PID_RAMP_LEFT                      256
#define WORD_PID_RAMP_RIGHT                     257
#define WORD_PID_ERROR_LEFT                     258
#define WORD_PID_ERROR_RIGHT                    259

//#define WORD_PID_P_LEFT                         250
//#define WORD_PID_I_LEFT                         252
//#define WORD_PID_D_LEFT                         254
//#define WORD_PID_P_RIGHT                        256
//#define WORD_PID_I_RIGHT                        258
//#define WORD_PID_D_RIGHT                        260
//
//#define WORD_PID_RAMP_LEFT                      262
//#define WORD_PID_RAMP_RIGHT                     264
//
//#define WORD_PID_ERROR_LEFT                     266
//#define WORD_PID_ERROR_RIGHT                    268

/* *****************************************************************************
 WORD MODBUS USATE PER IL DEBUG, MAPPATE DALL'INDIRIZZO 60000
  ******************************************************************************/

/*! \def WORD_DEBUG_1
\brief	( R/W ) Le ultime word le tengo per debug, possono servirmi a monitorare dati od eseguire comandi di test.
*/
#define WORD_DEBUG_00                    60000
#define WORD_DEBUG_01                    60001
#define WORD_DEBUG_02                    60002
#define WORD_DEBUG_03                    60003
#define WORD_DEBUG_04                    60004
#define WORD_DEBUG_05                    60005
#define WORD_DEBUG_06                    60006
#define WORD_DEBUG_07                    60007
#define WORD_DEBUG_08                    60008
#define WORD_DEBUG_09                    60009
#define WORD_DEBUG_10                    60010
#define WORD_DEBUG_11                    60011
#define WORD_DEBUG_12                    60012
#define WORD_DEBUG_13                    60013
#define WORD_DEBUG_14                    60014
#define WORD_DEBUG_15                    60015
#define WORD_DEBUG_16                    60016
#define WORD_DEBUG_17                    60017
#define WORD_DEBUG_18                    60018
#define WORD_DEBUG_19                    60019


// <--- Registri
