#include <QString>
#include <QtTest>
#include <QCoreApplication>

#include "robocontrollersdk.h"

using namespace roboctrl;

class RoboTestUnitTest : public QObject
{
    Q_OBJECT

public:
    RoboTestUnitTest();

private Q_SLOTS:
    void initTestCase();
    void cleanupTestCase();
    void testBoardStatus_RW();
    void testMotorSpeedRobotStopped_R();
    void testMotorPWMRobotStopped_R();
    void testWatchdog();
    void testMotorPWM_RW();

public slots:
    void onNewBoardStatus(BoardStatus& status);
    void onNewSpeedValue(quint16 mot, double speed);
    void onNewSpeedValues( double speed0, double speed1 );
    void onNewMotorPwmValue(quint16 mot, quint16 val);
    void onNewWatchdogTime(quint64 wdTime);

private:
    RoboControllerSDK* mRoboCtrl;

    BoardStatus mBoardStatus;

    quint16 mMotorIdx;
    double mMotorSpeeds[2];
    quint16 mMotorPWMs[2];

    bool mReplyReceived;

    quint64 mWdTime;
};


RoboTestUnitTest::RoboTestUnitTest()
{
    mRoboCtrl=NULL;
    mReplyReceived = false;
}

void RoboTestUnitTest::initTestCase()
{
    QString serverAdd = RoboControllerSDK::findServer();

    QTest::qSleep( 250 );

    QVERIFY2(!serverAdd.isEmpty(), "Server not found. Test failed on starting" );

    mRoboCtrl = new RoboControllerSDK( serverAdd );

    QTest::qSleep( 250 );

    QTRY_VERIFY(mRoboCtrl->isRunning() );

    connect( mRoboCtrl, SIGNAL(newBoardStatus(BoardStatus&)),
             this, SLOT(onNewBoardStatus(BoardStatus&)) );

    connect( mRoboCtrl, SIGNAL(newMotorSpeedValue(quint16,double)),
             this, SLOT(onNewSpeedValue(quint16,double)) );
    connect( mRoboCtrl, SIGNAL(newMotorSpeedValues(double,double)),
             this, SLOT(onNewSpeedValues(double,double)) );

    connect( mRoboCtrl, SIGNAL(newMotorPwmValue(quint16,quint16)),
             this, SLOT(onNewMotorPwmValue(quint16,quint16)) );

    connect( mRoboCtrl, SIGNAL(newWatchdogTime(quint64)),
             this, SLOT(onNewWatchdogTime(quint64)) );
}

void RoboTestUnitTest::cleanupTestCase()
{
    delete mRoboCtrl;
    mRoboCtrl = NULL;
}

void RoboTestUnitTest::testBoardStatus_RW()
{
    mReplyReceived = false;
    mRoboCtrl->getBoardStatus();
    QTRY_VERIFY(mReplyReceived);

    mBoardStatus.accelRampEnable = !mBoardStatus.accelRampEnable;
    mBoardStatus.pidEnable = !mBoardStatus.pidEnable;
    mBoardStatus.saveToEeprom = !mBoardStatus.saveToEeprom;
    mBoardStatus.wdEnable = !mBoardStatus.wdEnable;

    BoardStatus stat = mBoardStatus;

    bool ok = false;
    try
    {
        ok = mRoboCtrl->setBoardStatus(mBoardStatus);
    }
    catch( RcException &e )
    {
        QFAIL( "setBoardStatus: TCP Message did not reach destination");
    }
    QVERIFY( ok==true );

    QTest::qSleep( 250 );

    mReplyReceived = false;
    mRoboCtrl->getBoardStatus();
    QTRY_VERIFY(mReplyReceived);

    QVERIFY( mBoardStatus.accelRampEnable == stat.accelRampEnable );
    QVERIFY( mBoardStatus.pidEnable == stat.pidEnable );
    QVERIFY( mBoardStatus.saveToEeprom == stat.saveToEeprom );
    QVERIFY( mBoardStatus.wdEnable == stat.wdEnable );

    QTest::qSleep( 250 );

    mBoardStatus.accelRampEnable=false;
    mBoardStatus.pidEnable=false;
    mBoardStatus.saveToEeprom=false;
    mBoardStatus.wdEnable=false;

    ok = false;
    try
    {
        ok = mRoboCtrl->setBoardStatus(mBoardStatus);
    }
    catch( RcException &e )
    {
        QFAIL( "setBoardStatus: TCP Message did not reach destination");
    }
    QVERIFY( ok==true );

    QTest::qSleep( 250 );

    mReplyReceived = false;
    mRoboCtrl->getBoardStatus();
    QTRY_VERIFY(mReplyReceived);

    QVERIFY( mBoardStatus.accelRampEnable == false );
    QVERIFY( mBoardStatus.pidEnable == false );
    QVERIFY( mBoardStatus.saveToEeprom == false );
    QVERIFY( mBoardStatus.wdEnable == false );
}

void RoboTestUnitTest::testMotorSpeedRobotStopped_R()
{
    mReplyReceived = false;
    mRoboCtrl->getMotorSpeed( 0 );
    QTRY_VERIFY(mReplyReceived);
    QVERIFY2( mMotorSpeeds[0]==0.0, "Motor 0 speed is not null" );

    QTest::qSleep(250);

    mReplyReceived = false;
    mRoboCtrl->getMotorSpeed( 1 );
    QTRY_VERIFY(mReplyReceived);
    QVERIFY2( mMotorSpeeds[1]==0.0, "Motor 1 speed is not null" );

    QTest::qSleep(250);

    mReplyReceived = false;
    mRoboCtrl->getMotorSpeeds( );
    QTRY_VERIFY(mReplyReceived);
    QVERIFY2( mMotorSpeeds[0]==0.0, "Motor 0 speed is not null" );
    QVERIFY2( mMotorSpeeds[1]==0.0, "Motor 1 speed is not null" );

    QTest::qSleep(250);
}

void RoboTestUnitTest::testMotorPWMRobotStopped_R()
{
    for( int i=0; i<2; i++ )
    {
        mReplyReceived = false;
        mRoboCtrl->getMotorPWM( i );
        QTRY_VERIFY(mReplyReceived);
        QVERIFY2( mMotorPWMs[i]==0, tr("Motor 0 PWM is not null - Val:  %1").arg(mMotorPWMs[i]).toStdString().c_str() );

        QTest::qSleep(250);
    }
}

void RoboTestUnitTest::testWatchdog()
{
    // >>>>> Disable All
    mBoardStatus.accelRampEnable=false;
    mBoardStatus.pidEnable=false;
    mBoardStatus.saveToEeprom=false;
    mBoardStatus.wdEnable=false;

    bool ok = false;
    try
    {
        ok = mRoboCtrl->setBoardStatus(mBoardStatus);
    }
    catch( RcException &e )
    {
        QFAIL( "setBoardStatus: TCP Message did not reach destination");
    }
    QVERIFY( ok==true );
    // <<<<< Disable All

    QTest::qSleep(250);

    // >>>>> Test Board Status
    mReplyReceived = false;
    mRoboCtrl->getBoardStatus();
    QTRY_VERIFY(mReplyReceived);

    QVERIFY( mBoardStatus.accelRampEnable == false );
    QVERIFY( mBoardStatus.pidEnable == false );
    QVERIFY( mBoardStatus.saveToEeprom == false );
    QVERIFY( mBoardStatus.wdEnable == false );
    // <<<<< Test Board Status

    QTest::qSleep(250);

    mReplyReceived = false;
    mRoboCtrl->getWatchdogTime();
    QTRY_VERIFY(mReplyReceived);

    quint64 newWd = mWdTime-100;

    mRoboCtrl->enableWatchdog( newWd );
    QTest::qSleep(250);

    mReplyReceived = false;
    mRoboCtrl->getBoardStatus();
    QTRY_VERIFY(mReplyReceived);

    QVERIFY( mBoardStatus.accelRampEnable == false );
    QVERIFY( mBoardStatus.pidEnable == false );
    QVERIFY( mBoardStatus.saveToEeprom == false );
    QVERIFY( mBoardStatus.wdEnable == true );

    mReplyReceived = false;
    mRoboCtrl->getWatchdogTime();
    QTRY_VERIFY(mReplyReceived);

    QVERIFY2( mWdTime == newWd, "Watchdog Time setting failed" );
    QTest::qSleep(250);

    mRoboCtrl->enableWatchdog( 500 );
    QTest::qSleep(250);

    /*mRoboCtrl->setMotorPWM( 0, 1000 );
    mRoboCtrl->setMotorPWM( 1, 1000 );
    QTest::qSleep(500);

    QTest::qSleep( 750 ); // The motors should be stoppedby Watchdog

    mReplyReceived = false;
    mRoboCtrl->getMotorSpeeds( );
    QTRY_VERIFY(mReplyReceived);
    QVERIFY2( mMotorSpeeds[0]==0.0, "Motor 0 speed is not null" );
    QVERIFY2( mMotorSpeeds[1]==0.0, "Motor 1 speed is not null" );*/

    mRoboCtrl->disableWatchdog();
    QTest::qSleep(250);

    // >>>>> Disable All
    mBoardStatus.accelRampEnable=false;
    mBoardStatus.pidEnable=false;
    mBoardStatus.saveToEeprom=false;
    mBoardStatus.wdEnable=false;

    ok = false;
    try
    {
        ok = mRoboCtrl->setBoardStatus(mBoardStatus);
    }
    catch( RcException &e )
    {
        QFAIL( "setBoardStatus: TCP Message did not reach destination");
    }
    QVERIFY( ok==true );
    // <<<<< Disable All
}

void RoboTestUnitTest::testMotorPWM_RW()
{
    mRoboCtrl->getRobotControl();
    QTest::qSleep(250);

    // >>>>> Disable PID
    mBoardStatus.accelRampEnable=false;
    mBoardStatus.pidEnable=false;
    mBoardStatus.saveToEeprom=false;
    mBoardStatus.wdEnable=false;

    bool ok = false;
    try
    {
        ok = mRoboCtrl->setBoardStatus(mBoardStatus);
    }
    catch( RcException &e )
    {
        QFAIL( "setBoardStatus: TCP Message did not reach destination");
    }
    QVERIFY( ok==true );
    // <<<<< Disable PID



    for( int i=0; i<2; i++ )
    {
        int speed=0;
        for( int j=0; j<9; j++ )
        {
            speed -= 250;
            mRoboCtrl->setMotorPWM( i, speed );
            QTest::qSleep(500);
        }

        mReplyReceived = false;
        mRoboCtrl->getMotorSpeed( i );
        QTRY_VERIFY(mReplyReceived);
        QVERIFY2( mMotorSpeeds[i]>=0.0, tr("Motor %1 is not going backward").arg(i).toStdString().c_str() );
        QTest::qSleep(500);

        for( int j=0; j<9; j++ )
        {
            speed += 250;
            mRoboCtrl->setMotorPWM( i, speed );
            QTest::qSleep(500);
        }

        QTest::qSleep(500);

        mReplyReceived = false;
        mRoboCtrl->getMotorSpeed( i );
        QTRY_VERIFY(mReplyReceived);
        QVERIFY2( qAbs(mMotorSpeeds[i])<0.02, tr("Motor %1 has not been stopped - speed: %2").arg(i).arg(mMotorSpeeds[i]).toStdString().c_str() );
        QTest::qSleep(500);

        for( int j=0; j<9; j++ )
        {
            speed += 250;
            mRoboCtrl->setMotorPWM( i, speed );
            QTest::qSleep(500);
        }

        mReplyReceived = false;
        mRoboCtrl->getMotorSpeed( i );
        QTRY_VERIFY(mReplyReceived);
        QVERIFY2( mMotorSpeeds[i]<=0.0, tr("Motor %1 is not going forward").arg(i).toStdString().c_str() );
        QTest::qSleep(500);

        for( int j=0; j<9; j++ )
        {
            speed -= 250;
            mRoboCtrl->setMotorPWM( i, speed );
            QTest::qSleep(500);
        }

        QTest::qSleep(500);

        mReplyReceived = false;
        mRoboCtrl->getMotorSpeed( i );
        QTRY_VERIFY(mReplyReceived);
        QVERIFY2( qAbs(mMotorSpeeds[i])<0.02, tr("Motor %1 has not been stopped - speed: %2").arg(i).arg(mMotorSpeeds[i]).toStdString().c_str() );
        QTest::qSleep(500);
    }

    mRoboCtrl->releaseRobotControl();
    QTest::qSleep(250);
}

void RoboTestUnitTest::onNewBoardStatus(BoardStatus& status)
{
    mReplyReceived=true;
    mBoardStatus = status;
}

void RoboTestUnitTest::onNewSpeedValue( quint16 mot, double speed )
{
    mReplyReceived=true;
    mMotorIdx = mot;
    mMotorSpeeds[mot] = speed;
}

void RoboTestUnitTest::onNewSpeedValues( double speed0, double speed1 )
{
    mReplyReceived=true;
    mMotorSpeeds[0] = speed0;
    mMotorSpeeds[1] = speed1;
}

void RoboTestUnitTest::onNewMotorPwmValue(quint16 mot, quint16 val)
{
    mReplyReceived=true;
    mMotorIdx = mot;
    mMotorPWMs[mot] = val;
}

void RoboTestUnitTest::onNewWatchdogTime( quint64 wdTime )
{
    mReplyReceived=true;
    mWdTime = wdTime;
}

QTEST_MAIN(RoboTestUnitTest)

#include "tst_robotestunittest.moc"
