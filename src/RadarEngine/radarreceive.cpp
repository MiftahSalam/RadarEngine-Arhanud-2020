#include "radarreceive.h"

#include <QUdpSocket>

using namespace RadarEngineARMD;

struct RadarReport_01C4_18
{  // 01 C4 with length 18
    quint8 what;                 // 0   0x01
    quint8 command;              // 1   0xC4
    quint8 radar_status;         // 2
    quint8 field3;               // 3
    quint8 field4;               // 4
    quint8 field5;               // 5
    quint16 field6;              // 6-7
    quint16 field8;              // 8-9
    quint16 field10;             // 10-11
};
struct RadarReport_02C4_99
{     // length 99
    quint8 what;                    // 0   0x02
    quint8 command;                 // 1 0xC4
    quint32 range;                  //  2-3   0x06 0x09
    quint16 field4;                 // 6-7    0
    quint32 field8;                 // 8-11   1
    quint8 gain;                    // 12
    quint8 field13;                 // 13  ==1 for sea auto
    quint8 field14;                 // 14
    quint16 field15;                // 15-16
    quint32 sea;                    // 17-20   sea state (17)
    quint8 field21;                 // 21
    quint8 rain;                    // 22   rain clutter
    quint8 field23;                 // 23
    quint32 field24;                // 24-27
    quint32 field28;                // 28-31
    quint8 field32;                 // 32
    quint8 field33;                 // 33
    quint8 interference_rejection;  // 34
    quint8 field35;                 // 35
    quint8 field36;                 // 36
    quint8 field37;                 // 37
    quint8 target_expansion;        // 38
    quint8 field39;                 // 39
    quint8 field40;                 // 40
    quint8 field41;                 // 41
    quint8 target_boost;            // 42
};

struct RadarReport_03C4_129
{
    quint8 what;
    quint8 command;
    quint8 radar_type;  //01 = 4G, 08 = 3G, 0F = BR24
    quint8 u00[55];     // Lots of unknown
    quint16 firmware_date[16];
    quint16 firmware_time[16];
    quint8 u01[7];
};

struct RadarReport_04C4_66
{  // 04 C4 with length 66
    quint8 what;                 // 0   0x04
    quint8 command;              // 1   0xC4
    quint32 field2;              // 2-5
    quint16 bearing_alignment;   // 6-7
    quint16 field8;              // 8-9
    quint16 antenna_height;      // 10-11
};

struct RadarReport_08C4_18
{           // 08 c4  length 18
    quint8 what;                          // 0  0x08
    quint8 command;                       // 1  0xC4
    quint8 field2;                        // 2
    quint8 local_interference_rejection;  // 3
    quint8 scan_speed;                    // 4
    quint8 sls_auto;                      // 5
    quint8 field6;                        // 6
    quint8 field7;                        // 7
    quint8 field8;                        // 8
    quint8 side_lobe_suppression;         // 9
    quint16 field10;                      // 10-11
    quint8 noise_rejection;               // 12    noise rejection
    quint8 target_sep;                    // 13
};

struct common_header
{
    quint8 headerLen;       // 1 bytes
    quint8 status;          // 1 bytes
    quint8 scan_number[2];  // 2 bytes, 0-4095
    quint8 u00[4];          // 4 bytes
    quint8 angle[2];        // 2 bytes
    quint8 heading[2];      // 2 bytes heading with RI-10/11. See bitmask explanation above.
};

struct br4g_header
{
    quint8 headerLen;       // 1 bytes
    quint8 status;          // 1 bytes
    quint8 scan_number[2];  // 2 bytes, 0-4095
    quint8 u00[2];          // Always 0x4400 (integer)
    quint8 largerange[2];   // 2 bytes or -1
    quint8 angle[2];        // 2 bytes
    quint8 heading[2];      // 2 bytes heading with RI-10/11 or -1. See bitmask explanation above.
    quint8 smallrange[2];   // 2 bytes or -1
    quint8 rotation[2];     // 2 bytes, rotation/angle
    quint8 u02[4];          // 4 bytes signed integer, always -1
    quint8 u03[4];          // 4 bytes signed integer, mostly -1 (0x80 in last byte) or 0xa0 in last byte
};                       /* total size = 24 */
struct radar_line
{
    union
    {
        common_header common;
        br4g_header br4g;
    };
    quint8 data[RETURNS_PER_LINE];
};
struct radar_frame_pkt
{
    quint8 frame_hdr[8];
    radar_line line[120];  //  scan lines, or spokes
};

RadarReceive::RadarReceive(QObject *parent, RadarEngine *engine) :
    QThread(parent),m_engine(engine)
{
    if(!m_engine)
    {
        qDebug()<<Q_FUNC_INFO<<"cannot create without RadarEngine objek";
    }
    exit_req = false;
}

RadarReceive::~RadarReceive()
{
}
void RadarReceive::exitReq()
{
    mutex.lock();
    exit_req = true;
    mutex.unlock();
}
void RadarReceive::setMulticastData(QString addr, uint port)
{
    _data = addr;
    _data_port = port;
}
void RadarReceive::setMulticastReport(QString addr, uint port)
{
    _report = addr;
    _report_port = port;
}

void RadarReceive::run()
{
    //    qDebug()<<Q_FUNC_INFO;
    QUdpSocket socketDataReceive;
    QUdpSocket socketReportReceive;
    QString data_thread = _data;
    QString report_thread = _report;
    uint data_port_thread = _data_port;
    uint reportport_thread = _report_port;
    exit_req = false;

    QHostAddress groupAddress = QHostAddress(data_thread);
    if(socketDataReceive.bind(QHostAddress::AnyIPv4, data_port_thread, QUdpSocket::ShareAddress))
    {
        socketDataReceive.joinMulticastGroup(groupAddress);
        qDebug()<<Q_FUNC_INFO<<"bind data multicast access succesed"<<data_thread<<data_port_thread;
    }
    groupAddress = QHostAddress(report_thread);
    if(socketReportReceive.bind(QHostAddress::AnyIPv4,reportport_thread, QUdpSocket::ShareAddress))
    {
        socketReportReceive.joinMulticastGroup(groupAddress);
        qDebug()<<Q_FUNC_INFO<<"bind report multicast access succesed"<<report_thread<<reportport_thread;
    }

    while(!exit_req)
    {
        if(socketDataReceive.state()==QAbstractSocket::BoundState)
        {
//            qDebug()<<Q_FUNC_INFO<<"data report multicast access ";
            while (socketDataReceive.hasPendingDatagrams())
            {
                QByteArray datagram;
                datagram.resize(socketDataReceive.pendingDatagramSize());
                socketDataReceive.readDatagram(datagram.data(), datagram.size());

                processFrame(datagram,datagram.size());
//                qDebug()<<Q_FUNC_INFO<<"Receive datagram with size "<<datagram.size();
            }
        }
        else
        {
            qDebug()<<Q_FUNC_INFO<<"try bind data multicast access ";
            groupAddress = QHostAddress(data_thread);
            if(socketDataReceive.bind(QHostAddress::AnyIPv4,data_port_thread, QUdpSocket::ShareAddress))
            {
                socketDataReceive.joinMulticastGroup(groupAddress);
                qDebug()<<Q_FUNC_INFO<<"bind data multicast access succesed";
            }
            else
            {
                qDebug()<<Q_FUNC_INFO<<"bind data access failed "<<socketDataReceive.errorString();
            }

        }

        if(socketReportReceive.state()==QAbstractSocket::BoundState)
        {
//            qDebug()<<Q_FUNC_INFO<<"bind report multicast access ";
            while (socketReportReceive.hasPendingDatagrams())
            {
                QByteArray datagram;
                datagram.resize(socketReportReceive.pendingDatagramSize());
                socketReportReceive.readDatagram(datagram.data(), datagram.size());

                processReport(datagram,datagram.size());
                qDebug()<<Q_FUNC_INFO<<"Receive datagram report with size "<<datagram.size();
            }
        }
        else
        {
            qDebug()<<Q_FUNC_INFO<<"try bind report multicast access ";
            groupAddress = QHostAddress(report_thread);
            if(socketReportReceive.bind(QHostAddress::AnyIPv4,reportport_thread, QUdpSocket::ShareAddress))
            {
                socketReportReceive.joinMulticastGroup(groupAddress);
                qDebug()<<Q_FUNC_INFO<<"bind report multicast access succesed";
            }
            else
            {
                qDebug()<<Q_FUNC_INFO<<"bind report access failed "<<socketReportReceive.errorString();
            }

        }

    }
    qDebug()<<Q_FUNC_INFO<<"radar receive terminated";

}
void RadarReceive::processReport(QByteArray data, int len)
{
    qDebug()<<Q_FUNC_INFO;

    const quint8 *report =  (const quint8*)data.constData();
    if (report[1] == 0xC4)
    {
        switch ((len << 8) + report[0])
        {
        case (18 << 8) + 0x01:
        {  //  length 18, 01 C4
            RadarReport_01C4_18 *s = (RadarReport_01C4_18 *)report;
            // Radar status in byte 2
            if (s->radar_status != 0)
            {
                switch (report[2])
                {
                case 0x01:
                    emit updateReport(0,1,0);
                    break;
                case 0x02:
                    emit updateReport(0,2,0);
                    break;
                case 0x05:
                    emit updateReport(0,3,0);
                    break;
                default:
                    break;
                }
            }
            break;
        }

        case (99 << 8) + 0x02:
        {  // length 99, 02 C4
            RadarReport_02C4_99 *s = (RadarReport_02C4_99 *)report;
            if (s->field8 == 1) //auto
                emit updateReport(1,0,0);
            else
            {
                s->gain = s->gain !=0 ? s->gain : 1;
                emit updateReport(1,0,s->gain);
            }

            emit updateReport(1,1,s->rain);
            if (s->field13 == 0x01) //auto
                emit updateReport(1,2,0);
            else
            {
                s->sea = s->sea !=0 ? s->sea : 1;
                emit updateReport(1,2,s->sea);
            }
            emit updateReport(1,3,s->target_boost);
            emit updateReport(1,4,s->interference_rejection);
            emit updateReport(1,5,s->target_expansion);
            emit updateReport(1,6,s->range/10);
            break;
        }

        case (66 << 8) + 0x04:
        {  // 66 bytes starting with 04 C4
            RadarReport_04C4_66 *data = (RadarReport_04C4_66 *)report;

            emit updateReport(3,0,data->bearing_alignment);
            emit updateReport(3,1,data->antenna_height);
            break;
        }
        case (18 << 8) + 0x08:
        {  // length 18, 08 C4
            // contains scan speed, noise rejection and target_separation and sidelobe suppression
            RadarReport_08C4_18 *s08 = (RadarReport_08C4_18 *)report;

            emit updateReport(4,0,s08->scan_speed);
            emit updateReport(4,1,s08->noise_rejection);
            emit updateReport(4,2,s08->target_sep);
            emit updateReport(4,4,s08->local_interference_rejection);

            if (s08->sls_auto == 1)
            {
                emit updateReport(4,3,0);
            }
            else
            {
                s08->side_lobe_suppression = s08->side_lobe_suppression != 0 ?
                            s08->side_lobe_suppression : 1;
                emit updateReport(4,3,s08->side_lobe_suppression);
            }
            break;
        }
        default:
        {
            qDebug()<<Q_FUNC_INFO<<"receive unknown report. size"<<len;
            break;
        }
        }
        return ;
    }
}

void RadarReceive::processFrame(QByteArray data, int len)
{
    radar_frame_pkt *packet = (radar_frame_pkt *)data.data();

    if (len < (int)sizeof(packet->frame_hdr)) {
        return;
    }

    int scanlines_in_packet = (len - sizeof(packet->frame_hdr)) / sizeof(radar_line);
    if (scanlines_in_packet != 32)
    {
        qDebug()<<Q_FUNC_INFO<<"broken packet";
    }

    for (int scanline = 0; scanline < scanlines_in_packet; scanline++)
    {
        radar_line *line = &packet->line[scanline];

        // Validate the spoke
        if (line->common.headerLen != 0x18)
        {
            qDebug()<<Q_FUNC_INFO<<"strange header length "<<line->common.headerLen;
            continue;
        }
        if (line->common.status != 0x02 && line->common.status != 0x12)
        {
            qDebug()<<Q_FUNC_INFO<<"strange header status "<<line->common.status;
        }

        int range_raw = 0;
        int angle_raw = 0;
        int range_meters = 0;

//        short int heading_raw = 0;
//        heading_raw = (line->common.heading[1] << 8) | line->common.heading[0];

        short int large_range = (line->br4g.largerange[1] << 8) | line->br4g.largerange[0];
        short int small_range = (line->br4g.smallrange[1] << 8) | line->br4g.smallrange[0];
        angle_raw = (line->br4g.angle[1] << 8) | line->br4g.angle[0];

        /* tapping result
             * tx : 100 -> rec : 176/2c0
             * tx : 250 -> rec : 439/6dc
             * tx : 500 -> rec : 879/dbc
             * tx : 750 -> rec : 2029/1498
             * tx : 1 km -> rec : 2841
             * tx : 1.5 km -> rec :  4057
             * tx : 2 km -> rec :  3514
             * tx : 3 km -> rec :  8285
             * tx : 4 km -> rec :  9940
             * tx : 6 km -> rec :  16538
             * tx : 8 km -> rec :  21263 large range?
             * tx : 12 km-> rec :  31950 large range?
             * tx : 16 km-> rec :  -19411 large range?
             * tx : 24 km-> rec :  -1636 large range?
             * tx : 36     -> rec :  -2255 large range?
             * tx : 48 km -> rec :  -23349 large range?
             * tx : 64 km-> rec :  -9286 large range?
             * tx : 72 km-> rec :  -2255 large range?
             * tx : 96 km-> rec :  -23349 large range?

      from lib plugin
      if (large_range == 0x80) {
        if (small_range == -1) {
          range_raw = 0;  // Invalid range received
        } else {
          range_raw = small_range;
        }
      } else {
        range_raw = large_range * 256;
      }
      range_meters = range_raw / 4;

*/
        if (large_range == 0x80)
        {
            if (small_range == -1)
            {
                range_raw = 0;  // Invalid range received
            }
            else
            {
                range_raw = small_range/4;
            }
        }
        else
        {
            range_raw = large_range;
        }
        range_meters = range_raw;
//        qDebug()<<Q_FUNC_INFO<<range_raw;

        const char *data_p = (const char *)line->data;
        QByteArray raw_data = QByteArray(data_p,512);
        //      qDebug()<<Q_FUNC_INFO<<"sizeof data "<<sizeof(line->data)<<"size data array"<<raw_data.size();
        emit ProcessRadarSpoke(angle_raw,
                               raw_data,
                               RETURNS_PER_LINE,
                               range_meters
                               );
    }
}
