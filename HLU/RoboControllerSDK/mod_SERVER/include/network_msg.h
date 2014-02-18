#ifndef NETWORK_MSG_H
#define NETWORK_MSG_H

// ---> TCP Commands and Messages
#define     MESSAGES            100
#define     MSG_CONNECTED       (MESSAGES + 1) //< Returns the id of the connected board
#define     MSG_OK              (MESSAGES + 2)
#define     MSG_FAILED          (MESSAGES + 3)
#define     MSG_READ_REPLY      (MESSAGES + 4)
#define     MSG_RC_NOT_FOUND    (MESSAGES + 5) // RoboController Board not found
#define     MSG_WRITE_OK        (MESSAGES + 6)
#define     MSG_SERVER_PING_REQ (MESSAGES + 7) // Only over TCP
#define     MSG_SERVER_PING_OK  (MESSAGES + 8) // Only over TCP

#define     COMMANDS            200
#define     CMD_CONNECT         (COMMANDS + 1)
#define     CMD_DISCONNECT      (COMMANDS + 2)
#define     CMD_RD_MULTI_REG    (COMMANDS + 3) // Returns the values read from the board
#define     CMD_WR_MULTI_REG    (COMMANDS + 4)
// <--- TCP Commands and Messages

#endif // NETWORK_MSG_H
