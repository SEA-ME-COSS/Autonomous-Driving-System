#include "HeadUnitQtClass.hpp"

HeadUnitQtClass::HeadUnitQtClass(QObject *parent) : QObject(parent) {
    Qsteering = 0.0;
    Qthrottle = 0.0;
    Qxposition = 0.0;
    Qyposition = 0.0;
    Qorientation = 0.0;

    canDevice = QCanBus::instance()->createDevice("socketcan", "can0", &errorString);

    if (!canDevice) {
        qDebug() << "Failed to create CAN device:" << errorString;
        return;
    }

    connect(canDevice, &QCanBusDevice::framesReceived, this, &HeadUnitQtClass::processReceivedFrames);

    if (!canDevice->connectDevice()) {
        qDebug() << "Failed to connect to CAN device:" << canDevice->errorString();
        delete canDevice;
        canDevice = nullptr;
    }
}

HeadUnitQtClass::~HeadUnitQtClass() {
    if (canDevice) {
        canDevice->disconnectDevice();
        delete canDevice;
    }
}

qreal HeadUnitQtClass::steering() const {
    return Qsteering;
}

qreal HeadUnitQtClass::throttle() const {
    return Qthrottle;
}

qreal HeadUnitQtClass::xposition() const {
    return Qxposition;
}

qreal HeadUnitQtClass::yposition() const {
    return Qyposition;
}

qreal HeadUnitQtClass::orientation() const {
    return Qorientation;
}

void HeadUnitQtClass::setSteering(qreal _steering) {
    Qsteering = _steering;
    emit steeringChanged();
}

void HeadUnitQtClass::setThrottle(qreal _throttle) {
    Qthrottle = _throttle;
    emit throttleChanged();
}

void HeadUnitQtClass::setXposition(qreal _xposition) {
    Qxposition = _xposition;
    emit xpositionChanged();
}

void HeadUnitQtClass::setYposition(qreal _yposition) {
    Qyposition = _yposition;
    emit ypositionChanged();
}

void HeadUnitQtClass::setOrientation(qreal _orientation) {
    Qorientation = _orientation;
    emit orientationChanged();
}

void HeadUnitQtClass::processReceivedFrames() {
    while (canDevice->framesAvailable()) {
        QCanBusFrame frame = canDevice->readFrame();
        QByteArray payload = frame.payload();
        for (int i = 0; i < PAYLOAD_SIZE; i++) {
            data[i] = static_cast<quint8>(payload[i]);
        }

        if (frame.frameId() == steering_id) {  // steering
            decryption = data[1] + data[2] * 0.01;
            if (data[0] == 1) {
                decryption *= -1;
            }

            setSteering(decryption);
            continue;
        }

        if (frame.frameId() == throttle_id) {  // throttle
            decryption = data[1] + data[2] * 0.01;
            if (data[0] == 1) {
                decryption *= -1;
            }

            setThrottle(decryption);
            continue;
        }

        if (frame.frameId() == xposition_id) {  // xposition
            decryption = data[1] + data[2] * 0.01;
            if (data[0] == 1) {
                decryption *= -1;
            }

            setXposition(decryption);
            continue;
        }

        if (frame.frameId() == yposition_id) {  // yposition
            decryption = data[1] + data[2] * 0.01;
            if (data[0] == 1) {
                decryption *= -1;
            }

            setYposition(decryption);
            continue;
        }

        if (frame.frameId() == orientation_id) {  // orientation
            decryption = data[1];
            if (data[0] == 1) {
                decryption *= -1;
            }

            setOrientation(decryption);
            continue;
        }
    }
}

Q_INVOKABLE void HeadUnitQtClass::sendAdsMessage(bool onoff) {
    QCanBusFrame frame;
    QByteArray payload;

    if (onoff) {
        payload = QByteArray::fromHex("01000000");
    } else {
        payload = QByteArray::fromHex("00000000");
    }

    frame.setFrameId(adsmode_id);
    frame.setPayload(payload);

    canDevice->writeFrame(frame);
}
