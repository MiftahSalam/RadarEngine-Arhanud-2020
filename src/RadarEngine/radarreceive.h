#ifndef RADARRECEIVE_H
#define RADARRECEIVE_H

#include <QObject>
#include <QThread>
#include <QMutex>

#include "radarengine_global.h"
#include "radarengine.h"

namespace RadarEngineARND {

class RadarEngine;

class RADAR_ENGINE_ARND_EXPORT RadarReceive : public QThread
{
    Q_OBJECT
public:
    explicit RadarReceive(QObject *parent = 0,RadarEngine *engine = nullptr);

    ~RadarReceive();

    void exitReq();
    void setMulticastData(QString addr,uint port);
    void setMulticastReport(QString addr,uint port);

signals:
    void ProcessRadarSpoke(int angle_raw, QByteArray data,int dataSize);
    void updateReport(quint8 report_type,quint8 report_field,quint32 value);
    void signal_changeAntena(QString sig);

protected:
    void run();

private:
    void processFrame(QByteArray data, int len);
    void processReport(QByteArray data, int len);

    bool exit_req;
    int radarIdRef;
    QString _data;
    uint _data_port;
    QString _report;
    uint _report_port;
    QMutex mutex;
    RadarEngine *m_engine;
};

}

#endif // RADARRECEIVE_H
