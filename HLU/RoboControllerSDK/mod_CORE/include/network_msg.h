#ifndef NETWORK_MSG_H
#define NETWORK_MSG_H

#define TCP_START_VAL 0x55AA /*!< Start word for TCP data block */
#define UDP_START_VAL 0xAA55 /*!< Start word for UDP data block */

// ---> TCP Commands and Messages
#define     MESSAGES                100
#define     MSG_CONNECTED           (MESSAGES + 1)  ///< Returns the id of the connected board
#define     MSG_                    (MESSAGES + 2)  ///< Not used
#define     MSG_FAILED              (MESSAGES + 3)  ///< Tells that a command failed (followed by the command that generated it if available and a data field)
#define     MSG_READ_REPLY          (MESSAGES + 4)  ///< Message sent after a "Read" request
#define     MSG_RC_NOT_FOUND        (MESSAGES + 5)  ///< RoboController Board not found by the server
#define     MSG_WRITE_OK            (MESSAGES + 6)  ///< Tells that a Write command was successful (followed by the first regster written and the total of registers)
#define     MSG_SERVER_PING_OK      (MESSAGES + 7)  ///< Is received in reply to PING_REQ if everything is fine
#define     MSG_ROBOT_CTRL_OK       (MESSAGES + 8)  ///< Received if the client takes the Motion Control of the robot successfully with @ref CMD_GET_ROBOT_CTRL message
#define     MSG_ROBOT_CTRL_KO       (MESSAGES + 9)  ///< Received if the client tries to take the Motion Control of the Robot, but there is another one controlling it
#define     MSG_ROBOT_CTRL_RELEASED (MESSAGES + 19) ///< Received if the client released the Motion Control of the robot successfully

#define     COMMANDS                200
#define     CMD_GET_ROBOT_CTRL      (COMMANDS + 1) ///< Get exclusive control of the motion of the robot
#define     CMD_REL_ROBOT_CTRL      (COMMANDS + 2) ///< Release the motion control of the robot
#define     CMD_RD_MULTI_REG        (COMMANDS + 3) ///< Asks the values of "n" consequtive modbus registers on the RoboController board
#define     CMD_WR_MULTI_REG        (COMMANDS + 4) ///< Sends the values of "n" consequtive modbus registers on the RoboController board
#define     CMD_SERVER_PING_REQ     (COMMANDS + 5) ///< Client sends this message to verify that server is running
// <--- TCP Commands and Messages

#endif // NETWORK_MSG_H
