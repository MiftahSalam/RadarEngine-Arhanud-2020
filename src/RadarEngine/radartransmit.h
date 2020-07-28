#ifndef RADARTRANSMIT_H
#define RADARTRANSMIT_H

#include <QObject>
#include <QUdpSocket>

#include "radarengine_global.h"

namespace RadarEngineARND {

class RadarEngine;

class RADAR_ENGINE_ARND_EXPORT RadarTransmit : public QObject
{
    Q_OBJECT
public:
    explicit RadarTransmit(QObject *parent = 0, RadarEngine *re = nullptr);

    void setControlValue(ControlType controlType, int value);
    void setMulticastData(QString addr,uint port);
    void setRange(int meters);

    QUdpSocket socket;

signals:

public slots:
    void RadarTx();
    void RadarStby();
    void RadarStayAlive();

private:
    QString _data;
    uint _data_port;
    RadarEngine *m_re;
};

}

#endif // RADARTRANSMIT_H
