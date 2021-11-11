#include "radarengine.h"
#include <Crypto/crypto.h>

#include <QtOpenGL/qgl.h>
#include <unistd.h>

using namespace RadarEngineARND;

RadarState state_radar;
RadarState state_radar1;
ReportFilter filter;
ReportAlign align;
ReportScanSignal scanSignal;
RadarSettings radar_settings;
ARPASettings arpa_settings[2];
IFFSettings iff_settings;
ADSBSettings adsb_settings;
MqttSettings mqtt_settings;
AnteneSwitchSettings antene_switch_settings;
MapSettings map_settings;
ProxySetting proxy_settings;
TrailSettings trail_settings;
MTISettings mti_settings;
QDateTime cur_elapsed_time;
QSet<QString> friendListCode;
QSet<QString> hostileListCode;
GPSStatus gps_status;
double currentOwnShipLat;
double currentOwnShipLon;
double currentHeading;
bool gps_auto;
bool hdg_auto;
bool first_sweep;
int antena_switch;
int cur_zoom_lvl;
int track_counter;

GLubyte old_strength_info[2048][512];
GLubyte new_strength_info[2048][512];

#define MARGIN (100)
#define TRAILS_SIZE (RETURNS_PER_LINE * 2 + MARGIN * 2)

enum {
    TRAIL_15SEC, //6 rev
    TRAIL_30SEC, //12 rev
    TRAIL_1MIN,  //24 rev
    TRAIL_3MIN,  //72 rev
    TRAIL_5MIN, //120 rev
    TRAIL_10MIN, //240 rev
    TRAIL_CONTINUOUS,
    TRAIL_ARRAY_SIZE
};

RadarState decideRadarState(const RadarState state1, const RadarState state2)
{
    RadarState cur_radar_state;

    if(state1 == RADAR_TRANSMIT || state2 == RADAR_TRANSMIT) cur_radar_state = RADAR_TRANSMIT;
    else if(state1 == RADAR_STANDBY && state2 == RADAR_STANDBY) cur_radar_state = RADAR_STANDBY;
    else if(state1 == RADAR_WAKING_UP || state2 == RADAR_WAKING_UP) cur_radar_state = RADAR_WAKING_UP;
    else if(state1 == RADAR_OFF && state2 == RADAR_OFF) cur_radar_state = RADAR_OFF;

    return cur_radar_state;
}

RadarEngine::RadarEngine(QObject *parent, int id):
    QObject(parent), radarId(id)
{
    qDebug()<<Q_FUNC_INFO;

    if(radarId < 0)
    {
        qDebug()<<Q_FUNC_INFO<<"invalid radar id";
        return;
    }

    radar_timeout = 0;
    m_range_meters = 0;

    cur_radar_state = state_radar;
    old_draw_trails = trail_settings.enable;
    old_trail = trail_settings.trail;

    raw_data_proj = new quint8 [RETURNS_PER_LINE];

//    const float rad_proj[ANTENE_COUNT] = { 1.}; //tes
//    const float rad_proj[ANTENE_COUNT] = { 1., 1., 1.}; //tes
    const float rad_proj[ANTENE_COUNT] = { 1., cos(M_PI/9.0), cos(M_PI/4.5)};
    for(int antena=0; antena<ANTENE_COUNT; antena++)
    {
        for (int radius = 0; radius < RETURNS_PER_LINE; radius++)
        {
            rad_proj_cur[radius][antena] = static_cast<int>(radius*rad_proj[antena]);
        }
    }

    //tes
    for (int brn = 0; brn < LINES_PER_ROTATION; brn++) {
        for (int rad = 0; rad < RETURNS_PER_LINE; rad++) {
            for (int rgb_i = 0; rgb_i < 3; ++rgb_i) {
                raw_data_vec.append(0);
            }
        }
    }
    raw_data_vec[0] = 0;
    raw_data_vec[1] = 0;
    raw_data_vec[2] = 255;
    /*
    for (int var = 0; var < raw_data_vec.size()-3; var++) {
        if(var%3)
        {
            raw_data_vec[var] = 0;
            raw_data_vec[var+1] = 0;
            raw_data_vec[var+2] = 0;
        }
        else
        {
            raw_data_vec[var] = 255;
            raw_data_vec[var+1] = 255;
            raw_data_vec[var+2] = 255;
        }
    }
    */
    /*
    for (int brn = 0; brn < LINES_PER_ROTATION; brn++) {
        for (int rad = 0; rad < RETURNS_PER_LINE; rad++) {
            int cur_raw_data_vec_ptr_x = 2*(511+GetP2CLookupTable()->intx[brn][rad]);
            int cur_raw_data_vec_ptr_y = 1024*(512-GetP2CLookupTable()->inty[brn][rad]);
            quint8 raw = 255;

            if(rad%64 && brn%64)
                raw = 0;

            for (int rgb_i = 0; rgb_i < 3; ++rgb_i) {
                raw_data_vec[cur_raw_data_vec_ptr_x+cur_raw_data_vec_ptr_y+rgb_i] = raw;
            }

        }
    }
    */
    /*
    for (int var = 0; var < 1024; var++) {
        raw_data_vec.replace(var,255);
    }
    for (int var = 524288; var < 524288+300; var++) {
        raw_data_vec.replace(var,255);
    }
    for (int var = 525312; var < 525312+100; var++) {
        raw_data_vec.replace(var,255);
    }
    for (int var = 523776; var < 523776+100; var++) {
        raw_data_vec.replace(var,255);
    }
    for (int var = 786432; var < 786432+200; var++) {
        raw_data_vec.replace(var,255);
    }
    for (int var = 0; var < 20; var++) {
        raw_data_vec.replace(raw_data_vec.size()-2-var,255);
    }
    */

    ComputeColourMap();
    ComputeTargetTrails();

    timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(timerTimeout()));

    cur_elapsed_time = Crypto::initProtect();
    if(!Crypto::checkProtect(cur_elapsed_time))
    {
        qDebug()<<"not valid";
        exit(0);
    }

    radarReceive = new RadarReceive(this,this);
    radarTransmit = new RadarTransmit(this,this);
    radarDraw = RadarDraw::make_Draw(this,0);
    for (int i = 0; i < ANTENE_COUNT; ++i) {
        radarArpa[i] = new RadarArpa(this,this,i);
    }

    connect(radarReceive,&RadarReceive::updateReport,
            this,&RadarEngine::receiveThread_Report);
    connect(radarReceive,&RadarReceive::ProcessRadarSpoke,
            this,&RadarEngine::radarReceive_ProcessRadarSpoke);
    connect(radarReceive,&RadarReceive::signal_changeAntena,this,&RadarEngine::signal_changeAntena);

    connect(this,SIGNAL(signal_sendStby()),radarTransmit,SLOT(RadarStby()));
    connect(this,SIGNAL(signal_sendTx()),this,SLOT(trigger_ReqTx()));
    connect(this,SIGNAL(signal_stay_alive()),radarTransmit,SLOT(RadarStayAlive()));

    trigger_ReqRadarSetting();
    timer->start(1000);
}

void RadarEngine::trigger_ReqTx()
{
    bool tx = false;
    if(radar_settings.enable && radarId == 0)
        tx = true;
    if(radar_settings.enable1 && radarId == 1)
        tx = true;

    if(tx) radarTransmit->RadarTx();
}
RadarEngine::~RadarEngine()
{
    Crypto::setProtect(cur_elapsed_time);
    radarReceive->exitReq();
}

void RadarEngine::timerTimeout()
{
    quint64 now = QDateTime::currentMSecsSinceEpoch();

    //    qDebug()<<Q_FUNC_INFO<<cur_elapsed_time<<TIME_EXPIRED;
    /*
    */
    cur_elapsed_time = cur_elapsed_time.addSecs(1);
    if(!Crypto::checkProtect(cur_elapsed_time))
    {
        qDebug()<<"expired";
        Crypto::setProtect(cur_elapsed_time);
        emit signal_sendStby();
        emit signal_forceExit();
    }

//    RadarState *cur_radar_state = radarId ? &state_radar1 : &state_radar;
    qDebug()<<Q_FUNC_INFO<<"state_radar"<<(int)state_radar<<"state_radar1"<<(int)state_radar1;
    qDebug()<<Q_FUNC_INFO<<"state_radar addrs"<<&state_radar<<"state_radar1 addrs"<<&state_radar1;


    /*
    if(state_radar == RADAR_STANDBY && state_radar1 == RADAR_STANDBY)
        ResetSpokes();

    if(*cur_radar_state == RADAR_TRANSMIT && TIMED_OUT(now,data_timeout))
    {
        *cur_radar_state = RADAR_STANDBY;
    }
    if(*cur_radar_state == RADAR_TRANSMIT && TIMED_OUT(now,stay_alive_timeout))
    {
        emit signal_stay_alive();
        stay_alive_timeout = now + STAYALIVE_TIMEOUT;
    }
    if(*cur_radar_state == RADAR_STANDBY && TIMED_OUT(now,radar_timeout))
    {
        *cur_radar_state = RADAR_OFF;
        ResetSpokes();
    }
    */


//    RadarState cur_radar_state = state_radar;
    if(state_radar == RADAR_TRANSMIT && TIMED_OUT(now,stay_alive_timeout))
    {
        emit signal_stay_alive();
        stay_alive_timeout = now + STAYALIVE_TIMEOUT;
    }
    if(((state_radar == RADAR_STANDBY) | (state_radar == RADAR_TRANSMIT)) && TIMED_OUT(now,radar_timeout))
    {
        state_radar = RADAR_OFF;
        ResetSpokes();
    }

//    state_radar = RADAR_STANDBY; //temporary
//    state_radar = RADAR_TRANSMIT; //temporary

    if(cur_radar_state != state_radar)
    {
        cur_radar_state = state_radar;
        emit signal_state_change();
    }

    if(old_draw_trails != trail_settings.enable)
    {
        if(!trail_settings.enable)
            ClearTrails();

        ComputeColourMap();
        ComputeTargetTrails();
        old_draw_trails = trail_settings.enable;
    }
    if(old_trail != trail_settings.trail)
    {
        ClearTrails();
        ComputeColourMap();
        ComputeTargetTrails();
        old_trail = trail_settings.trail;
    }
}

void RadarEngine::trigger_ReqControlChange(int ct, int val)
{
    radarTransmit->setControlValue((ControlType)ct,val);
}

void RadarEngine::radarReceive_ProcessRadarSpoke(int angle_raw, QByteArray data, int dataSize)

{
//    qDebug()<<Q_FUNC_INFO<<"radarId"<<radarId<<m_range_meters<<m_old_range<<angle_raw;
    quint64 now = QDateTime::currentMSecsSinceEpoch();
    radar_timeout = now + WATCHDOG_TIMEOUT;
    data_timeout = now + DATA_TIMEOUT;
//    state_radar = RADAR_TRANSMIT; //need for offline mode


//    RadarState *cur_radar_state = &state_radar;
    RadarState *cur_radar_state = radarId ? &state_radar1 : &state_radar;
    *cur_radar_state = RADAR_TRANSMIT;


    short int hdt_raw = radar_settings.headingUp ? 0 : SCALE_DEGREES_TO_RAW(currentHeading);
    int bearing_raw = angle_raw + hdt_raw;

    int angle = MOD_ROTATION2048(angle_raw / 2);
    int bearing = MOD_ROTATION2048(bearing_raw / 2);

    quint8 *raw_data = (quint8*)data.data();
    memset(raw_data_proj,0,dataSize);

//    raw_data[RETURNS_PER_LINE - 1] = 200;  //  range ring, for testing

    const int CUR_ANTENA = antena_switch;


    //tes create spoke matrix
    int cur_raw_data_vec_ptr_x,cur_raw_data_vec_ptr_y;


    quint8 weakest_normal_blob = 50; //next load from configuration file
    quint8 *hist_data = m_history[bearing][CUR_ANTENA].line;

    m_history[bearing][CUR_ANTENA].time = now;
    m_history[bearing][CUR_ANTENA].lat = currentOwnShipLat;
    m_history[bearing][CUR_ANTENA].lon = currentOwnShipLon;

    for (int radius = 0; radius < data.size(); radius++)
    {
        raw_data_proj[rad_proj_cur[radius][CUR_ANTENA]] = raw_data[radius];
//        qDebug()<<Q_FUNC_INFO<<"rad_proj_cur"<<rad_proj_cur<<radius;
        if(mti_settings.enable)
        {
//            new_strength_info[bearing][radius] = raw_data[radius];
            new_strength_info[bearing][radius] = raw_data_proj[rad_proj_cur[radius][CUR_ANTENA]];
            if(abs((int)(old_strength_info[bearing][radius] - new_strength_info[bearing][radius])) < mti_settings.threshold)
//                raw_data[radius]=0;
            raw_data_proj[rad_proj_cur[radius][CUR_ANTENA]]=0;
        }

        hist_data[rad_proj_cur[radius][CUR_ANTENA]] = hist_data[rad_proj_cur[radius][CUR_ANTENA]] << 1;  // shift left history byte 1 bit
//        hist_data[radius] = hist_data[radius] << 1;  // shift left history byte 1 bit
        // clear leftmost 2 bits to 00 for ARPA
        hist_data[rad_proj_cur[radius][CUR_ANTENA]] = hist_data[rad_proj_cur[radius][CUR_ANTENA]] & 63;
//        hist_data[radius] = hist_data[radius] & 63;
//        if (raw_data[radius] >= weakest_normal_blob)
            if (raw_data_proj[rad_proj_cur[radius][CUR_ANTENA]] >= weakest_normal_blob)
        {
            // and add 1 if above threshold and set the left 2 bits, used for ARPA
                hist_data[rad_proj_cur[radius][CUR_ANTENA]] = hist_data[rad_proj_cur[radius][CUR_ANTENA]] | 192;
//                hist_data[radius] = hist_data[radius] | 192;
        }

        /*
        */
        if(mti_settings.enable)
//            old_strength_info[bearing][radius] = new_strength_info[bearing][radius];
        old_strength_info[bearing][rad_proj_cur[radius][CUR_ANTENA]] = new_strength_info[bearing][rad_proj_cur[radius][CUR_ANTENA]];

        //tes
//        cur_raw_data_vec_ptr_x = 2*(511+GetP2CLookupTable()->intx[bearing][radius]);
//        cur_raw_data_vec_ptr_y = 1024*(512-GetP2CLookupTable()->inty[bearing][radius]);
//        raw_data_vec[cur_raw_data_vec_ptr_x+cur_raw_data_vec_ptr_y] = raw_data_proj[rad_proj_cur[radius][CUR_ANTENA]];
//        qDebug()<<Q_FUNC_INFO<<"cur_raw_data_vec_ptr_x"<<cur_raw_data_vec_ptr_x<<"cur_raw_data_vec_ptr_y"<<cur_raw_data_vec_ptr_y<<"total"<<523776+cur_raw_data_vec_ptr_x-cur_raw_data_vec_ptr_y<<"bearing"<<bearing<<"radius"<<radius;
//        qDebug()<<Q_FUNC_INFO<<"raw_data_vec"<<raw_data_vec[523776+cur_raw_data_vec_ptr_x-cur_raw_data_vec_ptr_y]<<"raw_data"<<raw_data_proj[rad_proj_cur[radius][CUR_ANTENA]];
    }

    /*check Guardzone*/
    //    if(gz_settings.show && gz_settings.enable_alarm)
    //        m_gz->ProcessSpokePoly(angle, raw_data, rng_gz);
    //    m_gz->ProcessSpoke(angle, raw_data, m_history[bearing].line, rng_gz);


    /*Trail handler*/
    if(trail_settings.enable)
    {
        if (m_old_range != m_range_meters && m_old_range != 0 && m_range_meters != 0)
        {
            // zoom trails
            float zoom_factor = (float)m_old_range / (float)m_range_meters;
            ZoomTrails(zoom_factor);
        }
        if (m_old_range == 0 || m_range_meters == 0)
        {
            ClearTrails();
        }
        m_old_range = m_range_meters;

        // Relative trails
        quint8 *trail = m_trails.relative_trails[angle];
        for (int radius = 0; radius < dataSize - 1; radius++)
        {
//            if (raw_data[radius] >= weakest_normal_blob)
                if (raw_data_proj[radius] >= weakest_normal_blob)
                *trail = 1;

            else
            {
                if (*trail > 0 && *trail < TRAIL_MAX_REVOLUTIONS)
                    (*trail)++;

//                raw_data[radius] = m_trail_colour[*trail];
                raw_data_proj[radius] = m_trail_colour[*trail];
            }
            trail++;
        }
    }

//    emit signal_plotRadarSpoke(bearing,raw_data,dataSize);
    emit signal_plotRadarSpoke(bearing,raw_data_proj,dataSize);
}

void RadarEngine::ResetSpokes()
{
    quint8 zap[RETURNS_PER_LINE];

    qDebug()<<Q_FUNC_INFO<<"reset spokes, history and trails";

    memset(zap, 0, sizeof(zap));
    memset(m_history, 0, sizeof(m_history));

    for (size_t r = 0; r < LINES_PER_ROTATION; r++)
        emit signal_plotRadarSpoke(r,zap,sizeof(zap));

}

void RadarEngine::ZoomTrails(float zoom_factor)
{
    // zoom relative trails
    memset(&m_trails.copy_of_relative_trails, 0, sizeof(m_trails.copy_of_relative_trails));
    for (int i = 0; i < LINES_PER_ROTATION; i++)
    {
        for (int j = 0; j < RETURNS_PER_LINE; j++)
        {
            int index_j = (int((float)j * zoom_factor));
            if (index_j >= RETURNS_PER_LINE) break;
            if (m_trails.relative_trails[i][j] != 0)
            {
                m_trails.copy_of_relative_trails[i][index_j] = m_trails.relative_trails[i][j];
            }
        }
    }
    memcpy(&m_trails.relative_trails[0][0], &m_trails.copy_of_relative_trails[0][0], sizeof(m_trails.copy_of_relative_trails));
}
void RadarEngine::ClearTrails()
{
    memset(&m_trails, 0, sizeof(m_trails));
}
void RadarEngine::ComputeColourMap()
{
    for (int i = 0; i <= UINT8_MAX; i++)
    {
        m_colour_map[i] = (i >= 200 /*red strong threshold*/) ? BLOB_STRONG
                                                              : (i >= 100 /*green strong threshold*/)
                                                                ? BLOB_INTERMEDIATE
                                                                : (i >= 50 /*blue strong threshold*/) ? BLOB_WEAK : BLOB_NONE; //next implementation load from conf file
        //        qDebug()<<Q_FUNC_INFO<<"color map "<<i<<m_colour_map[i];
    }

    for (int i = 0; i < BLOB_COLOURS; i++)
        m_colour_map_rgb[i] = QColor(0, 0, 0);

    QColor color = radarId ? QColor(0, 150, 0) : QColor(0, 255, 0);

    m_colour_map_rgb[BLOB_STRONG] = color;
    m_colour_map_rgb[BLOB_INTERMEDIATE] = color;
    m_colour_map_rgb[BLOB_WEAK] = color;

    if (trail_settings.enable)
    {
        /*
        int a1 = 255;
        int a2 = 0;
        float delta_a = (float)((a2 - a1) / BLOB_HISTORY_COLOURS);

        for (BlobColour history = BLOB_HISTORY_0;
             history <= BLOB_HISTORY_MAX;
             history = (BlobColour)(history + 1))
        {
            m_colour_map[history] = history;
            m_colour_map_rgb[history] = QColor(0, 255, 0,a1);
            a1 += (int)delta_a;
        }
        */
        int r1 = 255.0;
        int g1 = 255.0;
        int b1 = 255.0;
        int r2 = 0.0;
        int g2 = 0.0;
        int b2 = 0.0;
        float delta_r = (float)((r2 - r1) / BLOB_HISTORY_COLOURS);
        float delta_g = (float)((g2 - g1) / BLOB_HISTORY_COLOURS);
        float delta_b = (float)((b2 - b1) / BLOB_HISTORY_COLOURS);

        for (BlobColour history = BLOB_HISTORY_0;
             history <= BLOB_HISTORY_MAX;
             history = (BlobColour)(history + 1))
        {
            m_colour_map[history] = history;
            m_colour_map_rgb[history] = QColor(r1, g1, b1);
            r1 += (int)delta_r;
            g1 += (int)delta_g;
            b1 += (int)delta_b;
        }
    }
}

void RadarEngine::ComputeTargetTrails()
{
    static TrailRevolutionsAge maxRevs[TRAIL_ARRAY_SIZE] =
    {
        SECONDS_TO_REVOLUTIONS(15),
        SECONDS_TO_REVOLUTIONS(30),
        SECONDS_TO_REVOLUTIONS(60),
        SECONDS_TO_REVOLUTIONS(180),
        SECONDS_TO_REVOLUTIONS(300),
        SECONDS_TO_REVOLUTIONS(600),
        TRAIL_MAX_REVOLUTIONS + 1
    };

    TrailRevolutionsAge maxRev = maxRevs[trail_settings.trail];
    if (!trail_settings.enable)
        maxRev = 0;


    TrailRevolutionsAge revolution;
    double coloursPerRevolution = 0.;
    double colour = 0.;

    // Like plotter, continuous trails are all very white (non transparent)
    if (trail_settings.enable && (trail_settings.trail < TRAIL_CONTINUOUS))
        coloursPerRevolution = BLOB_HISTORY_COLOURS / (double)maxRev;

    qDebug()<<Q_FUNC_INFO<<"Target trail value "<<maxRev<<"revolutions";

    // Disperse the BLOB_HISTORY values over 0..maxrev
    for (revolution = 0; revolution <= TRAIL_MAX_REVOLUTIONS; revolution++)
    {
        if (revolution >= 1 && revolution < maxRev)
        {
            m_trail_colour[revolution] = (BlobColour)(BLOB_HISTORY_0 + (int)colour);
            colour += coloursPerRevolution;
        }
        else
            m_trail_colour[revolution] = BLOB_NONE;

    }
}

void RadarEngine::trigger_ReqRangeChange(int rId, int range)
{
    if(rId == radarId) radarTransmit->setRange(range);
}

void RadarEngine::trigger_clearTrail()
{
    ClearTrails();
}

void RadarEngine::trigger_ReqRadarSetting()
{
    ResetSpokes();
    radarReceive->exitReq();
    sleep(1);

    switch (radarId) {
    case 0:
        radarReceive->setMulticastData(radar_settings.ip_data,radar_settings.port_data);
        radarReceive->setMulticastReport(radar_settings.ip_report,radar_settings.port_report);
        radarTransmit->setMulticastData(radar_settings.ip_command,radar_settings.port_command);
        break;
    case 1:
        radarReceive->setMulticastData(radar_settings.ip_data1,radar_settings.port_data1);
        radarReceive->setMulticastReport(radar_settings.ip_report1,radar_settings.port_report1);
        radarTransmit->setMulticastData(radar_settings.ip_command1,radar_settings.port_command1);
        break;
    default:
        break;
    }

    bool stby = false;
    if(!radar_settings.enable && radarId == 0)
        stby = true;
    if(!radar_settings.enable1 && radarId == 1)
        stby = true;
    if(stby) emit signal_sendStby();

    radarReceive->start();

}

void RadarEngine::checkRange(int new_range)
{
    if ((m_range_meters != (uint)new_range))
    {
        m_range_meters = new_range;
        emit signal_range_change(radarId,new_range/10);
        ResetSpokes();
        qDebug()<<Q_FUNC_INFO<<"detected spoke range change from "<<m_range_meters<<" to "<<new_range;
    }
}

void RadarEngine::receiveThread_Report(quint8 report_type, quint8 report_field, quint32 value)
{
    quint64 now = QDateTime::currentMSecsSinceEpoch();
    radar_timeout = now + WATCHDOG_TIMEOUT;
//    RadarState *cur_radar_state = &state_radar;
    RadarState *cur_radar_state = radarId ? &state_radar1 : &state_radar;

//    qDebug()<<Q_FUNC_INFO;
    switch (report_type)
    {
    case RADAR_STATE:
//        if(radarId == 0)
        *cur_radar_state = (RadarState)report_field;
        if(*cur_radar_state == RADAR_TRANSMIT && TIMED_OUT(now,data_timeout))
        {
//            emit signal_state_change();
            *cur_radar_state = RADAR_NO_SPOKE;
            ResetSpokes();
        }

        if(!radarId) filter.wakingup_time = static_cast<quint8>(value);

        qDebug()<<Q_FUNC_INFO<<"report status radar"<<radarId<<(RadarState)report_field;
        break;
    case RADAR_FILTER:
        switch (report_field)
        {
        case RADAR_GAIN:
            filter.gain = value;
//            qDebug()<<Q_FUNC_INFO<<"report gain"<<filter.gain;
            emit signal_updateReport();
            break;
        case RADAR_RAIN:
            filter.rain = value;
//            qDebug()<<Q_FUNC_INFO<<"report rain"<<filter.rain;
            emit signal_updateReport();
            break;
        case RADAR_SEA:
            filter.sea = value;
//            qDebug()<<Q_FUNC_INFO<<"report sea"<<filter.sea;
            emit signal_updateReport();
            break;
        case RADAR_TARGET_BOOST:
            filter.targetBoost = value;
//            qDebug()<<Q_FUNC_INFO<<"report TargetBoost"<<filter.targetBoost;
            break;
        case RADAR_LOCAL_INTERFERENCE_REJECTION:
            filter.LInterference = value;
//            qDebug()<<Q_FUNC_INFO<<"report local interference"<<filter.LInterference;
            break;
        case RADAR_TARGET_EXPANSION:
            filter.targetExpan = value;
//            qDebug()<<Q_FUNC_INFO<<"report argetExpan"<<filter.targetExpan;
            break;
        case RADAR_RANGE:
            filter.range = value;
            checkRange(value);
            qDebug()<<Q_FUNC_INFO<<"report range"<<filter.range;
            break;
        default:
            break;
        }
        break;
    case RADAR_ALIGN:
        switch (report_field)
        {
        case RADAR_BEARING:
            // bearing alignment
            align.bearing = (int)value / 10;
            if (align.bearing > 180)
                align.bearing = align.bearing - 360;
            qDebug()<<Q_FUNC_INFO<<"report radar bearing alignment"<<align.bearing;
            break;
        case RADAR_ANTENA:
            // antenna height
            align.antena_height = value;
            qDebug()<<Q_FUNC_INFO<<"report radar antenna_height"<<align.antena_height/1000;
            break;
        default:
            break;
        }
        break;
    case RADAR_SCAN_AND_SIGNAL:
        switch (report_field)
        {
        case RADAR_SCAN_SPEED:
            scanSignal.scan_speed = value;
            qDebug()<<Q_FUNC_INFO<<"report radar scan_speed"<<scanSignal.scan_speed ;
            break;
        case RADAR_NOISE_REJECT:
            scanSignal.noise_reject = value;
            qDebug()<<Q_FUNC_INFO<<"report radar noise_reject"<<scanSignal.noise_reject ;
            break;
        case RADAR_TARGET_SEPARATION:
            scanSignal.target_sep = value;
            qDebug()<<Q_FUNC_INFO<<"report radar target_sep"<<scanSignal.target_sep ;
            break;
        case RADAR_LOBE_SUPRESION:
            scanSignal.side_lobe_suppression = value; //0->auto
            qDebug()<<Q_FUNC_INFO<<"report radar side_lobe_suppression"<<scanSignal.side_lobe_suppression ;
            break;
        case RADAR_INTERFERENT:
            scanSignal.local_interference_rejection = value;
            qDebug()<<Q_FUNC_INFO<<"report radar local_interference_rejection"<<scanSignal.local_interference_rejection ;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

}
