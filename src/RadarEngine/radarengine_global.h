#ifndef RADARENGINE_GLOBAL_H
#define RADARENGINE_GLOBAL_H

#include <QtCore/qglobal.h>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QDateTime>
#include <QtCore/QDebug>
#include <QColor>

#include <math.h>

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

#define ANTENE_COUNT (3) //3 antena

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
    RADAR_STATE = 0,
    RADAR_FILTER,
    RADAR_TYPE,
    RADAR_ALIGN,
    RADAR_SCAN_AND_SIGNAL
};

enum RadarState
{
    RADAR_OFF,
    RADAR_STANDBY,
    RADAR_TRANSMIT,
    RADAR_WAKING_UP,
    RADAR_NO_SPOKE
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
    quint8 wakingup_time;
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
    bool power_on;
    bool op_mode;
    bool show_rings;
    bool headingUp;
    bool show_compass;
    bool show_sweep;
    bool show_heading_marker;
    int last_scale;
    bool enable;
    QString ip_data;
    uint port_data;
    QString ip_report;
    uint port_report;
    QString ip_command;
    uint port_command;
    bool enable1;
    QString ip_data1;
    uint port_data1;
    QString ip_report1;
    uint port_report1;
    QString ip_command1;
    uint port_command1;
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
    QString ip1;
    uint port1;
    QString ip2;
    uint port2;
};
struct MqttSettings
{
    QString id;
    QString ip;
    uint port;
};
struct AnteneSwitchSettings
{
    QString ip;
    QString ip2;
    uint port;
    uint port2;
};
struct ADSBSettings
{
    bool show_track;
    bool show_attr;
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
    bool show_track;
    bool show_attr;
    QString ip;
    uint port;
};

struct RadarRange {
  int meters; //command to radar and display
  uint actual_meters; //based on range feedback
  const char *name;
};
/*
static const RadarRange g_ranges_metric[] =
{
    {500, 5000, "500 m"},
    {1000, 10000, "1 Km"},
    {2000, 20000, "2 Km"},
    {4000, 40000, "4 Km"},
    {6000, 60000, "6 Km"},
    {8000, 80000, "8 Km"},
    {12000, 120000, "12 Km"},
    {24000, 240000, "24 Km"},
    {36000, 360000, "36 Km"},
    {48000, 480000, "48 Km"},
    {64000, 640000, "64 Km"},
    {72000, 720000, "72 Km"},
};
*/
/*
static const RadarRange g_ranges_metric[] =
{
    {200, 2320, "200 m"},
    {1852/4, 4630, "1/4 NM"},
    {1852/2, 9260, "1/2 NM"},
    {1852*3/4, 13890, "3/4 NM"},
    {1852*3/2, 27780, "1.5 NM"},
    {1852*3, 55560, "3 NM"},
    {1852*6, 111120, "6 NM"},
    {1852*12, 222240, "12 NM"},
    {1852*24, 444480, "24 NM"},
    {1852*36, 666720, "36 NM"},
    {1852*48, 888960, "48 NM"},
    {1852*64, 1185280, "64 NM"},
    {1852*72, 1333440, "72 NM"},
};
*/
/*
static const RadarRange g_ranges_metric[] =
{
    {200, 406, "200 m"}, //407 atw 406
    {1852/4, 813, "1/4 NM"},
    {1852/2, 1627, "1/2 NM"},
    {1852*3/4, 2441, "3/4 NM"},
    {1852*3/2, 4883, "1.5 NM"},
    {1852*3, 9766, "3 NM"},
    {1852*6, 19532, "6 NM"},
    {1852*12, 39065, "12 NM"},
    {1852*24, 78130, "24 NM"},
    {1852*36, 117196, "36 NM"},
    {1852*48, 156260, "48 NM"},
    {1852*64, 208348, "64 NM"},
    {1852*72, 234392, "72 NM"},
};
     */

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
struct GPSStatus
{
    bool gps_valid;
    bool gps_online;
    bool hdt_valid;
    bool hdt_online;
};
/*
*/
static const QList<int> distanceList = QList<int>()<<5000000 //0
                                                  <<2000000 //1
                                                 <<1000000 //2
                                                <<1000000 //3
                                               <<1000000 //4
                                              <<100000 //5
                                             <<100000 //6
                                            <<50000 //7
                                           <<50000 //8
                                          <<10000 //9
                                         <<10000 //10
                                        <<10000 //11
                                       <<1000 //12
                                      <<1000 //13
                                     <<500 //14
                                    <<200 //15
                                   <<100 //16
                                  <<50; //17
/*
static const QList<int> distanceList = QList<int>()<<100000 //0
                                            <<75000 //1
                                           <<50000 //2
                                          <<30000 //3
                                         <<20000 //4
                                        <<10000 //5
                                       <<5000 //6
                                      <<2000 //7
                                     <<1000 //8
                                    <<500 //9
                                    ;
*/


extern RadarState state_radar;
extern RadarState state_radar1;
extern ReportFilter filter;
extern ReportAlign align;
extern ReportScanSignal scanSignal;
extern RadarSettings radar_settings;
extern ARPASettings arpa_settings[2];
extern IFFSettings iff_settings;
extern ADSBSettings adsb_settings;
extern MqttSettings mqtt_settings;
extern AnteneSwitchSettings antene_switch_settings;
extern MapSettings map_settings;
extern ProxySetting proxy_settings;
extern TrailSettings trail_settings;
extern MTISettings mti_settings;
extern QDateTime cur_elapsed_time;
extern QSet<QString> friendListCode;
extern QSet<QString> hostileListCode;
extern GPSStatus gps_status;
extern double currentOwnShipLat;
extern double currentOwnShipLon;
extern double currentHeading;
extern bool gps_auto;
extern bool hdg_auto;
extern bool first_sweep;
extern int antena_switch;
extern int cur_zoom_lvl;
extern int track_counter;

extern RadarState decideRadarState(const RadarState state1, const RadarState state2);

#endif // RADARENGINE_GLOBAL_H
