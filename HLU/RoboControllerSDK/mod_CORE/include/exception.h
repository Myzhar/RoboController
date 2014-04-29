#ifndef EXCEPTION_H_
#define EXCEPTION_H_

#include "RoboControllerSDK_global.h"

#include <cstdarg>
#include <string.h>
#include <stdio.h>

namespace roboctrl
{

typedef enum
{
    excNotDefined = 0, /**< Generic Exception */
    excTcpNotConnected, /**< TCP Server not connected */
    extTcpConnectionRefused, /** TCP Server refused Connection */
    excUdpNotConnected, /**< TCP Server not connected */
    excProtocolBadConfirmation,  /**< Received a connection confirmation when not requested */
    excCommunicationLost, /**< The server does not reply to requests */
    excRoboControllerNotFound, /**< The RoboController board does not communicate */
    excNoWebcamFound /**< The Webcam server failed to connect to webcam */
} RcExceptionType;

/// A generic error exception that can be thrown by the application
class ROBOCONTROLLERSDKSHARED_EXPORT RcException
{
protected:
    char message[1024];
    RcExceptionType mExcType;

    RcException()
    {
        message[0] = 0;
        mExcType = excNotDefined;
    }

public:
    /// Construct an exception
    /// \param message It is a printf-like format string, arguments follows ;)
    RcException( RcExceptionType type, const char* message, ...)
    {
        va_list va;
        va_start(va, message);
        vsnprintf(this->message, sizeof(this->message) - 1, message, va);
        va_end(va);

        mExcType = type;
    }

    /// Construct an exception
    /// \param message It is a printf-like format string, arguments follows ;)
    RcException( RcExceptionType type, const char* message, RcException* e, ...)
    {
        va_list va;
        va_start(va, e);
        vsnprintf(this->message, sizeof(this->message) - 1, message, va);
        strcat((char*) this->message, " ");
        strcat((char*) this->message, (const char*) e->getExcMessage());
        va_end(va);

        mExcType = type;
    }

    /// @brief The exception error message
    /// @returns The error message associated to the exception
    const char* getExcMessage() const
    {
        return message;
    }

    /// @brief The exception error message
    /// @returns The error message associated to the exception
    operator const char*() const
    {
        return (const char*) message;
    }

    /// @brief The exception error type
    /// @returns The error type associated to the exception
    RcExceptionType GetType()
    {
        return mExcType;
    }

};

}

#endif /* EXCEPTION_H_ */
