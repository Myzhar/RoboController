#ifndef CMAINWINDOW_H
#define CMAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QSettings>
#include <robocontrollersdk.h>



namespace Ui {
class CMainWindow;
}

using namespace roboctrl;

class CMainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit CMainWindow(QWidget *parent = 0);
    ~CMainWindow();

private:
    void disconnectServer();
    void SendOptionsToBoard();
    
private slots:
    void on_verticalSlider_SetPoint_valueChanged(int value);
    void on_verticalSlider_Kp_valueChanged(int value);
    void on_verticalSlider_Ki_valueChanged(int value);
    void on_verticalSlider_Kd_valueChanged(int value);

    void on_pushButton_SetPoint_clicked();
    void on_pushButton_SetGains_clicked();
    void updateGains();

    void on_combo_motor_idx_currentIndexChanged(int index);
    void on_pushButton_Reset_clicked();
    void on_pushButton_StartStop_clicked();

    void onSdkServerConnected();
    void onSdkServerDisconnected();
    void onNewMotorSpeedValue( quint16 motorIdx, double value );
    void onNewMotorPIDGains( quint16 motorIdx, quint16 Kp, quint16 Ki, quint16 Kd );
    void onNewRobotConfiguration( RobotConfiguration& robConf );
    void onNewBoardStatus(BoardStatus& status);

    void onDataTimerTimeout();

    void updatePlot();

    void requestCurrentMotorVal();

    void on_action_Disconnect_triggered();
    void on_action_Connect_TCP_Server_triggered();
    void on_actionEEPROM_to_Config_File_triggered();
    void on_actionConfig_File_to_EEPROM_triggered();

    void on_actionRead_Config_File_triggered();

    void on_checkBox_WD_enable_clicked();
    void on_checkBox_PID_enable_clicked();
    void on_checkBox_save_eeprom_enable_clicked();
    void on_checkBox_ramps_enable_clicked();

private:
    Ui::CMainWindow *ui;

    int mCurrMotorIdx; /**< Current selected motor */

    double mSetPoint[2]; /**< Last SetPoint of each motor */
    int mKp[2]; /**< Proportional gain */
    int mKi[2]; /**< Integral gain */
    int mKd[2]; /**< Derivative gain */

    double  mGraphRange; /**< Time range of the graph in msec */
    int     mUpdateTimeMsec; /**< Data update time */
    quint64 mStartTime; /**< Acquiring start time */
    QTimer  mDataTimer; /**< Data request timer */

    QVector<double> mTimeVec; /**< Vector of the times (keys) */
    QVector<double> mSetPointVec; /**< Vector of the SetPoints */
    QVector<double> mCurrMotorValVec; /**< Vector of the motor speeds */
    QVector<double> mErrorVec; /**< Vector of the errors */

    unsigned int    mTcpServerPort; /**< Port of the TCP server */
    QString         mTcpServerAddr; /**< Address of the TCP server */

    QSettings       mSettings; /**< Ini file settings */

    RoboControllerSDK* mRcComm; /**< Pointer to RoboController SDK object */

    bool mConfToIni; /**< Indicates if the robot configuration is to be saved on ini file */
    bool mConfToEeprom; /**< Indicates if the robot configuration is to be saved on EEPROM from ini file */
};

#endif // CMAINWINDOW_H
