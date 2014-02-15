#ifndef CMAINWINDOW_H
#define CMAINWINDOW_H

#include <QMainWindow>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>
#include <QLabel>
#include <robocontrollersdk.h>

// >>>>> INI names
#ifdef ANDROID
#define INI_FOLDER "RobotGUI"
#endif
#define ROB_IP "robot_ip"
#define ROB_TCP_PORT "robot_tcp_port"
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
    
private slots:
    void on_actionPidEnabled_triggered();
    void onConnectButtonClicked();
    void onNewJoypadValues(float x, float y);
    void onNewMotorSpeed( int mot, double speed );

    void on_actionRobot_Configuration_triggered();

protected:
    virtual void resizeEvent(QResizeEvent * ev) Q_DECL_OVERRIDE;
    virtual bool eventFilter(QObject *obj, QEvent *ev) Q_DECL_OVERRIDE;

    virtual void timerEvent( QTimerEvent* event ) Q_DECL_OVERRIDE;

private:
    Ui::CMainWindow *ui;

    // >>>> GUI Objects
    QSettings* mIniSettings;
    QString mRobIpAddress;
    QLineEdit* mRobIpLineEdit;
    int mRobTcpPort;
    QLineEdit* mRobTcpPortLineEdit;
    QPushButton* mConnectButton;

    QLabel* mStatusLabel;
    // <<<< GUI Objects

    bool mPidEnabled;

    RoboControllerSDK* mRoboCtrl; /*!< Pointer to RoboControllerSDK object */

    float mMaxMotorSpeed; /*!< Max linear speed for each motor */

    int mSpeedReqTimer;
    int mStatusReqTimer;
};

#endif // CMAINWINDOW_H
