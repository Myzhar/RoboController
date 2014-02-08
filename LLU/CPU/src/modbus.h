// Dimensioni buffer
#define MODBUS_N_BYTE_RX                128
#define MODBUS_N_BYTE_TX                128

#define BIT_PER_WORD			16
#define MAX_WORD_LETTURA_MULTIPLA       28 //14


// Codici MODBUS
#define CODE_BITS_READING		0x01
#define CODE_BITS_READING_BIS		0x02
#define CODE_WORDS_READING		0x03
#define CODE_WORDS_READING_BIS		0x04
#define CODE_SINGLE_BIT_WRITING		0x05
#define CODE_SINGLE_WORD_WRITING	0x06
#define CODE_MULTIPLE_BITS_WRITING	0x0F
#define CODE_MULTIPLE_WORDS_WRITING	0x10

// Codici risposta modbus
#define ILLEGAL_FUNCTION_CODE           1

// StatoSeriale
#define WAIT_MESSAGE                    1
#define WAIT_TX				2
#define INIT_COM			3

// define relative al modbus
#define	VERSIONE_FIRMWARE               1	// versione firmware V1.00
#define TIPO_DISPOSITIVO		1	/*! Costante che definirà il tipo di dispositivo:
                                                        - 1 = RoboController V2 Default
                                                        - 2 =
                                                        - 3 =
                                                        - 4 =
                                                */
//#define INDIRIZZO_DISPOSITIVO		1
#define TIME_OUT_MODBUS                 2	// x1 mSec
#define RITARDO_RISPOSTA_SERIALE	10
