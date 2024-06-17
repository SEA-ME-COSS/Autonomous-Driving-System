#ifndef HEADUNITQTCLASS_HPP
#define HEADUNITQTCLASS_HPP

#include <QObject>
#include <QCanBus>
#include <QCanBusDevice>
#include <QCanBusFrame>
#include <QDebug>

class HeadUnitQtClass : public QObject
{
    Q_OBJECT
    Q_PROPERTY(qreal steering READ steering WRITE setSteering NOTIFY steeringChanged)
    Q_PROPERTY(qreal throttle READ throttle WRITE setThrottle NOTIFY throttleChanged)
    Q_PROPERTY(qreal xposition READ xposition WRITE setXposition NOTIFY xpositionChanged)
    Q_PROPERTY(qreal yposition READ yposition WRITE setYposition NOTIFY ypositionChanged)
    Q_PROPERTY(qreal orientation READ orientation WRITE setOrientation NOTIFY orientationChanged)

private:
    qreal Qsteering;
    qreal Qthrottle;
    qreal Qxposition;
    qreal Qyposition;
    qreal Qorientation;

    quint32 steering_id = QString("0x00").toUInt(nullptr, 16);
    quint32 throttle_id = QString("0x01").toUInt(nullptr, 16);
    quint32 xposition_id = QString("0x02").toUInt(nullptr, 16);
    quint32 yposition_id = QString("0x03").toUInt(nullptr, 16);
    quint32 orientation_id = QString("0x04").toUInt(nullptr, 16);
    quint32 adsmode_id = QString("0x05").toUInt(nullptr, 16);

    QCanBusDevice *canDevice = nullptr;
    QString errorString;
    static const int PAYLOAD_SIZE = 4;
    quint8 data[PAYLOAD_SIZE];
    qreal decryption;

private slots:
    void processReceivedFrames();

public:
    explicit HeadUnitQtClass(QObject *parent = nullptr);
    ~HeadUnitQtClass();

    qreal steering() const;
    qreal throttle() const;
    qreal xposition() const;
    qreal yposition() const;
    qreal orientation() const;

    void setSteering(qreal _steering);
    void setThrottle(qreal _throttle);
    void setXposition(qreal _xposition);
    void setYposition(qreal _yposition);
    void setOrientation(qreal _orientation);

    Q_INVOKABLE void sendAdsMessage(bool onoff);

signals:
    void steeringChanged();
    void throttleChanged();
    void xpositionChanged();
    void ypositionChanged();
    void orientationChanged();
};

#endif
