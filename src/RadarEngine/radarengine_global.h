#ifndef RADARENGINE_GLOBAL_H
#define RADARENGINE_GLOBAL_H

#include <QtCore/qglobal.h>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QDateTime>
#include <QtCore/QDebug>
#include <QColor>

#ifdef RADAR_ENGINE_ARND_STATIC
#   define RADAR_ENGINE_ARND_EXPORT
#else
#  if defined(RADAR_ENGINE_LIBRARY)
#    define RADAR_ENGINE_ARND_EXPORT Q_DECL_EXPORT
#  else
#    define RADAR_ENGINE_ARND_EXPORT Q_DECL_IMPORT
#  endif
#endif

/*
   VERSION is (major << 16) + (minor << 8) + patch.
*/
#define RADAR_ENGINE_ARND_VERSION RADAR_ENGINE_ARND_VERSION_CHECK(RADAR_ENGINE_ARND_VERSION_MAJOR, RADAR_ENGINE_ARND_VERSION_MINOR, RADAR_ENGINE_ARND_VERSION_PATCH)

/*
   can be used like #if (RADAR_ENGINE_ARND_VERSION >= RADAR_ENGINE_ARND_VERSION_CHECK(1, 3, 0))
*/
#define RADAR_ENGINE_ARND_VERSION_CHECK(major, minor, patch) ((major<<16)|(minor<<8)|(patch))

#define RETURNS_PER_LINE (512)
#define SPOKES (4096)
#define LINES_PER_ROTATION (2048)
#define DEGREES_PER_ROTATION (360)

#define WATCHDOG_TIMEOUT (10000)
#define DATA_TIMEOUT (10000)
#define STAYALIVE_TIMEOUT (5000)

#define TIMED_OUT(t, timeout) (t >= timeout)

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

#define SCALE_RAW_TO_DEGREES(raw) ((raw) * (double)DEGREES_PER_ROTATION / SPOKES)
#define SCALE_RAW_TO_DEGREES2048(raw) ((raw) * (double)DEGREES_PER_ROTATION / LINES_PER_ROTATION)
#define SCALE_DEGREES_TO_RAW(angle) ((int)((angle) * (double)SPOKES / DEGREES_PER_ROTATION))
#define SCALE_DEGREES_TO_RAW2048(angle) ((int)((angle) * (double)LINES_PER_ROTATION / DEGREES_PER_ROTATION))
#define MOD_DEGREES(angle) (fmod(angle + 2 * DEGREES_PER_ROTATION, DEGREES_PER_ROTATION))
#define MOD_ROTATION(raw) (((raw) + 2 * SPOKES) % SPOKES)
#define MOD_ROTATION2048(raw) (((raw) + 2 * LINES_PER_ROTATION) % LINES_PER_ROTATION)

#ifndef deg2rad
#define deg2rad(x) ((x)*2 * M_PI / 360.0)
#endif
#ifndef rad2deg
#define rad2deg(x) ((x)*360.0 / (2 * M_PI))
#endif

enum BlobColour {
    BLOB_NONE,
    BLOB_HISTORY_0,
    BLOB_HISTORY_1,
    BLOB_HISTORY_2,
    BLOB_HISTORY_3,
    BLOB_HISTORY_4,
    BLOB_HISTORY_5,
    BLOB_HISTORY_6,
    BLOB_HISTORY_7,
    BLOB_HISTORY_8,
    BLOB_HISTORY_9,
    BLOB_HISTORY_10,
    BLOB_HISTORY_11,
    BLOB_HISTORY_12,
    BLOB_HISTORY_13,
    BLOB_HISTORY_14,
    BLOB_HISTORY_15,
    BLOB_HISTORY_16,
    BLOB_HISTORY_17,
    BLOB_HISTORY_18,
    BLOB_HISTORY_19,
    BLOB_HISTORY_20,
    BLOB_HISTORY_21,
    BLOB_HISTORY_22,
    BLOB_HISTORY_23,
    BLOB_HISTORY_24,
    BLOB_HISTORY_25,
    BLOB_HISTORY_26,
    BLOB_HISTORY_27,
    BLOB_HISTORY_28,
    BLOB_HISTORY_29,
    BLOB_HISTORY_30,
    BLOB_HISTORY_31,
    BLOB_WEAK, //33
    BLOB_INTERMEDIATE, //34
    BLOB_STRONG //35
};
#define BLOB_HISTORY_MAX BLOB_HISTORY_31 //32
#define BLOB_COLOURS (BLOB_STRONG + 1) //36
#define BLOB_HISTORY_COLOURS (BLOB_HISTORY_MAX - BLOB_NONE) //32

typedef enum ControlType
{
    CT_GAIN,
    CT_SEA,
    CT_RAIN,
    CT_INTERFERENCE_REJECTION,
    CT_TARGET_SEPARATION,
    CT_NOISE_REJECTION,
    CT_TARGET_BOOST,
    CT_TARGET_EXPANSION,
    CT_REFRESHRATE,
    CT_SCAN_SPEED,
    CT_BEARING_ALIGNMENT,
    CT_SIDE_LOBE_SUPPRESSION,
    CT_ANTENNA_HEIGHT,
    CT_LOCAL_INTERFERENCE_REJECTION,
    CT_MAX  // Keep this last, see below
} ControlType;


enum RadarReportType
{
    RADAR_STATE,
    RADAR_FILTER,
    RADAR_TYPE,
    RADAR_ALIGN,
    RADAR_SCAN_AND_SIGNAL
};

enum RadarState
{
    RADAR_OFF,
    RADAR_WAKING_UP,
    RADAR_STANDBY,
    RADAR_TRANSMIT
};

enum RadarFilter
{
    RADAR_GAIN,
    RADAR_RAIN,
    RADAR_SEA,
    RADAR_TARGET_BOOST,
    RADAR_LOCAL_INTERFERENCE_REJECTION,
    RADAR_TARGET_EXPANSION,
    RADAR_RANGE
};

enum RadarAlign
{
    RADAR_BEARING,
    RADAR_ANTENA
};
enum RadarScanSignal
{
    RADAR_SCAN_SPEED,
    RADAR_NOISE_REJECT,
    RADAR_TARGET_SEPARATION,
    RADAR_LOBE_SUPRESION,
    RADAR_INTERFERENT
};

struct ReportFilter
{
    quint8 gain;
    quint8 rain;
    quint8 sea;
    quint8 targetBoost;
    quint8 LInterference;
    quint8 targetExpan;
    quint32 range;
};
struct ReportAlign
{
    int bearing;
    quint16 antena_height;
};
struct ReportScanSignal
{
    quint16 scan_speed;
    quint8 noise_reject;
    quint8 target_sep;
    quint8 side_lobe_suppression;
    quint8 local_interference_rejection;
};
struct RadarSettings
{
    bool show_rings;
    bool headingUp;
    bool show_compass;
    bool show_heading_marker;
    int last_scale;
    QString ip_data;
    uint port_data;
    QString ip_report;
    uint port_report;
    QString ip_command;
    uint port_command;
};
struct MapSettings
{
    bool show;
    bool loading;
    quint8 mode;
};
struct ProxySetting{
    QString host;
    QString username;
    QString password;
    int port;
    bool enable;
};

struct IFFSettings
{
    bool show_track;
    QString ip;
    uint port;
};
struct MqttSettings
{
    QString id;
    QString ip;
    uint port;
};
struct ADSBSettings
{
    bool show_track;
    QString config;
    int type;
};
struct ARPASettings
{
    int min_contour_length;
    int search_radius1;
    int search_radius2;
    int max_target_size;
    bool create_arpa_by_click;
    bool show;
    QString ip;
    uint port;
};
struct RadarRange {
  int meters; //command to radar and display
  int actual_meters; //based on range feedback
  const char *name;
};

static const RadarRange g_ranges_metric[] =
{
    /* */
    {200, 407, "200 m"},
    {1852/4, 813, "1/4 NM"},
    {1852/2, 1627, "1/2 NM"},
    {1852*3/4, 2441, "3/4 NM"},
    {1852*3/2, 4883, "1.5 NM"},
    {1852*3, 5000192, "3 NM"}, //5000192
    {1852*6, 10000384, "6 NM"}, //10000384
    {1852*12, 20001280, "12 NM"}, //20001280
    {1852*24, 40002560, "24 NM"}, //40002560
    {1852*36, 60004352, "36 NM"}, //60004352
    {1852*48, 80005120, "48 NM"}, //80005120
    {1852*64, 106674176, "64 NM"}, //106674176
    {1852*72, 120008704, "72 NM"}, //120008704
};
struct TrailSettings
{
    bool enable;
    int trail;
};
struct MTISettings
{
    bool enable;
    quint8 threshold;
};

static const QList<int> distanceList = QList<int>()<<5000000 //0
                                                  <<2000000
                                                 <<1000000
                                                <<1000000
                                               <<1000000
                                              <<100000
                                             <<100000
                                            <<50000
                                           <<50000
                                          <<10000
                                         <<10000
                                        <<10000
                                       <<1000
                                      <<1000
                                     <<500
                                    <<200
                                   <<100
                                  <<50; //17


extern RadarState state_radar;
extern ReportFilter filter;
extern ReportAlign align;
extern ReportScanSignal scanSignal;
extern RadarSettings radar_settings;
extern ARPASettings arpa_settings;
extern TrailSettings trail_settings;
extern MTISettings mti_settings;
extern QDateTime cur_elapsed_time;
extern double currentOwnShipLat;
extern double currentOwnShipLon;
extern double currentHeading;

#endif // RADARENGINE_GLOBAL_H
