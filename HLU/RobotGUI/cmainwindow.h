#ifndef CMAINWINDOW_H
#define CMAINWINDOW_H

#include <QMainWindow>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>
#include <QLabel>
#include <QProgressBar>

#include <robocontrollersdk.h>
#include <qwebcamclient.h>

/*#ifndef android
#include "qglopencvwidget.h"
#else
//TODO add OpenCV Widget without OpenGL
#endif*/

// >>>>> INI names
#ifdef ANDROID
#define INI_FOLDER "RobotGUI"
#endif
#define ROB_IP "robot_ip"
#define ROB_TCP_PORT "robot_tcp_port"
#define ROB_UDP_CTRL_PORT "robot_udp_control_port"
#define ROB_UDP_STAT_PORT_SEND "robot_udp_status_port_send"
#define ROB_UDP_STAT_PORT_LISTEN "robot_udp_status_port_listen"
#define ROB_UDP_TELEM_PORT_LISTEN "robot_udp_telem_port_listen"
#define PID_ENABLED "pid_enabled"
// <<<<< INI names


using namespace roboctrl;

namespace Ui {
class CMainWindow;
}

class CMainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit CMainWindow(QWidget *parent = 0);
    ~CMainWindow();

public slots:
    void onNewImage();
    
private slots:
    void on_actionPidEnabled_triggered();
    void onConnectButtonClicked();
    void onFindServerButtonClicked();
    void onNewJoypadValues(float x, float y);
    //void onNewMotorSpeed( quint16 mot, double speed );
    //void onNewMotorSpeeds(double,double);
    void onNewRobotConfiguration( RobotConfiguration& robConf );
    //void onNewBatteryValue( double battVal);
    void onNewBoardStatus(BoardStatus& status);
    void onNewTelemetryAvailable();

    void onRobotControlTaken();
    void onRobotControlReleased();
    void onRobotControlNotTaken();

    void on_actionRobot_Configuration_triggered();

    void on_actionBattery_Calibration_triggered();

protected:
    virtual void resizeEvent(QResizeEvent * ev) Q_DECL_OVERRIDE;
    virtual bool eventFilter(QObject *obj, QEvent *ev) Q_DECL_OVERRIDE;

    virtual void timerEvent( QTimerEvent* event ) Q_DECL_OVERRIDE;

    void stopTimers();
    void startTimers();

    void updateBatteryInfo();
    void updateSpeedInfo();

private:
    Ui::CMainWindow *ui;

    // >>>> GUI Objects
    QSettings* mIniSettings;
    QString mRobIpAddress;
    QLineEdit* mRobIpLineEdit;
    int mRobTcpPort;
    int mRobUdpControlPort;
    int mRobUdpStatusPortSend;
    int mRobUdpStatusPortListen;
    int mRobUdpTelemetryPortListen;
    QPushButton* mPushButtonConnect;
    QPushButton* mPushButtonFindServer;

    QLabel* mStatusLabel;
    QLabel* mBatteryLabel;

    QProgressBar* mStatusBattLevelProgr;
    // <<<< GUI Objects

    bool mPidEnabled;
    bool mOpenRobotConfig; /*!< Indicates that Robot Configuration dlg must be opened when receiving a new configuration message */
    bool mRobotConfigValid; /*!< Indicates that a new valid Robot Configuration has been received */

    bool mSpeedRequested; /*! Indicates that the speeds of the motors have been requested and we are waiting for reply */
    bool mMotorSpeedLeftValid;
    bool mMotorSpeedRightValid;
    double mMotorSpeedRight;
    double mMotorSpeedLeft;

    bool mHasRobotCtrl; ///< Indicates if the GUI has control of the robot

    RoboControllerSDK* mRoboCtrl; /*!< Pointer to RoboControllerSDK object */
    RobotConfiguration mRoboConf; /*!< Robot Configuration */

    QWebcamClient* mWebcamClient; /*!< Webcam client */

    float mMaxMotorSpeed; /*!< Max linear speed for each motor */

    int mSpeedSendTimer; /*!< Timer to read Joypad position and send related speed to Robot */
    int mStatusReqTimer; /*!< Timer to request status of the robot to server */
    int mFrameReqTimer; /*!< Timer to request frame to Webcam server */

    float mJoyMot[2]; /*!< Values of the joypad */
    float mLastJoyMot[2]; /*!< Last used values of the joypad */

    bool mNewImageAvailable; /*!< Used to retrieve image from WebcamServer */
    bool mNewTelemetryAvailable; /*! Used to retrieve image from RobotClient */

    cv::Mat mDefaultBgImg; /*!< Default background image */
    bool mFirstResize; /*!< Used to understand if @ref ResizeEvent is called for the first time */

    RobotTelemetry mTelemetry; /*! Contains the last telemetry value */


/*#ifndef android
    QGlOpenCVWidget* mOpenCVWidget;
#else
    // TODO QOpenCVWidget* mOpenCVWidget;
#endif*/
};

#endif // CMAINWINDOW_H
