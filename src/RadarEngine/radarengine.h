#ifndef RADARENGINE_H
#define RADARENGINE_H

#include "radarengine_global.h"
#include "radarreceive.h"
#include "radartransmit.h"
#include "radardraw.h"
#include "radararpa.h"

#include <QTimer>

#define SECONDS_TO_REVOLUTIONS(x) ((x)*2 / 5)
#define TRAIL_MAX_REVOLUTIONS SECONDS_TO_REVOLUTIONS(600) + 1 //241
typedef quint8 TrailRevolutionsAge;

namespace RadarEngineARND {

class RADAR_ENGINE_ARND_EXPORT RadarEngine : public QObject
{
    Q_OBJECT
public:
    RadarEngine(QObject *parent=nullptr);
    ~RadarEngine();

    struct line_history
    {
      quint8 line[RETURNS_PER_LINE];
      quint64 time;
      double lat;
      double lon;
    };

    line_history m_history[LINES_PER_ROTATION];
    BlobColour m_colour_map[UINT8_MAX + 1];
    QColor m_colour_map_rgb[BLOB_COLOURS];

    RadarDraw *radarDraw;
    RadarArpa *radarArpa;

signals:
    void signal_updateReport();
    void signal_plotRadarSpoke(int angle, u_int8_t* data, size_t len);
    void signal_range_change(int range);
    void signal_stay_alive();
    void signal_sendTx();
    void signal_sendStby();
    void signal_state_change();
    void signal_forceExit();

private slots:
    void receiveThread_Report(quint8 report_type, quint8 report_field, quint32 value);
    void radarReceive_ProcessRadarSpoke(int, QByteArray, int, int);
    void trigger_ReqRadarSetting();
    void timerTimeout();
    void trigger_clearTrail();
    void trigger_ReqRangeChange(int range);
    void trigger_ReqControlChange(int ct,int val);

private:
    struct TrailBuffer
    {
        TrailRevolutionsAge relative_trails[LINES_PER_ROTATION][RETURNS_PER_LINE];
        TrailRevolutionsAge copy_of_relative_trails[LINES_PER_ROTATION][RETURNS_PER_LINE];

        double lat;
        double lon;
    };
    TrailBuffer m_trails;
    BlobColour m_trail_colour[TRAIL_MAX_REVOLUTIONS + 1];

    RadarReceive *radarReceive;
    RadarTransmit *radarTransmit;

    QTimer *timer;

    quint64 radar_timeout;
    quint64 data_timeout;
    quint64 stay_alive_timeout;

    int m_range_meters;
    int m_old_range;

    RadarState cur_radar_state;
    bool old_draw_trails;
    int old_trail;

    void ComputeColourMap();
    void ResetSpokes();
    void ZoomTrails(float zoom_factor);
    void ClearTrails();
    void ComputeTargetTrails();
};

}

#endif // RADARENGINE_H
