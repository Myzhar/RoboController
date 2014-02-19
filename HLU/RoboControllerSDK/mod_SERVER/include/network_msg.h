#ifndef NETWORK_MSG_H
#define NETWORK_MSG_H

// ---> TCP Commands and Messages
#define     MESSAGES            100
#define     MSG_CONNECTED       (MESSAGES + 1) ///< Returns the id of the connected board
#define     MSG_OK              (MESSAGES + 2) ///< Confirms the execution of a command
#define     MSG_FAILED          (MESSAGES + 3) ///< Tells that a command failed (followed by the command that generated it if available and a data field)
#define     MSG_READ_REPLY      (MESSAGES + 4) ///< Message sent after a "Read" request
#define     MSG_RC_NOT_FOUND    (MESSAGES + 5) ///< RoboController Board not found by the server
#define     MSG_WRITE_OK        (MESSAGES + 6) ///< Tells that a Write command was successful (followed by the first regster written and the total of registers)
#define     MSG_SERVER_PING_OK  (MESSAGES + 7)

#define     COMMANDS            200
#define     CMD_GET_ROBOT_CTRL  (COMMANDS + 1) ///< Get exclusive control of the robot to send movements commands
#define     CMD_REL_ROBOT_CTRL  (COMMANDS + 2) ///< Release the control of the robot
#define     CMD_RD_MULTI_REG    (COMMANDS + 3) ///< Asks the values of "n" consequtive registers
#define     CMD_WR_MULTI_REG    (COMMANDS + 4) ///< Sends the values of "n" consequtive registers
#define     CMD_SERVER_PING_REQ (MESSAGES + 5)
// <--- TCP Commands and Messages

#endif // NETWORK_MSG_H
