#include "radartransmit.h"

using namespace RadarEngineARND;

RadarTransmit::RadarTransmit(QObject *parent, RadarEngine *re):
    QObject(parent),m_re(re)
{
    socket.setSocketOption(QAbstractSocket::MulticastTtlOption, 1);
}
void RadarTransmit::setMulticastData(QString addr, uint port)
{
    qDebug()<<Q_FUNC_INFO<<addr<<port;
    _data = addr;
    _data_port = port;
}

void RadarTransmit::setRange(int meters)
{
    qDebug()<<Q_FUNC_INFO<<"transmit: range "<<meters;
    if (meters >= 50 && meters <= 120008704)
    {
        unsigned int decimeters = (unsigned int)meters * 10;
        const uchar pck[6] = {0x03,
                              0xc1,
                              (const char)((decimeters >> 0) & 0XFFL),
                              (const char)((decimeters >> 8) & 0XFFL),
                              (const char)((decimeters >> 16) & 0XFFL),
                              (const char)((decimeters >> 24) & 0XFFL)};
        //        qDebug()<<Q_FUNC_INFO<<"transmit: range "<<meters<<"raw "<<decimeters;

        socket.writeDatagram((const char*)&pck,6,QHostAddress(_data),_data_port);

    }
}
void RadarTransmit::RadarStby()
{
    qDebug()<<Q_FUNC_INFO;

    const uchar standby1[3]={0x00,0xC1,0x01};
    const uchar standby2[3]={0x01,0xC1,0x00};

    socket.writeDatagram((const char*)&standby1,3,QHostAddress(_data),_data_port);
    socket.waitForBytesWritten();
    socket.writeDatagram((const char*)&standby2,3,QHostAddress(_data),_data_port);

}
void RadarTransmit::RadarTx()
{
    qDebug()<<Q_FUNC_INFO;
    const uchar transmit1[3]={0x00,0xC1,0x01};
    const uchar transmit2[3]={0x01,0xC1,0x01};

    socket.writeDatagram((const char*)&transmit1,3,QHostAddress(_data),_data_port);
    socket.waitForBytesWritten();
    socket.writeDatagram((const char*)&transmit2,3,QHostAddress(_data),_data_port);

}
void RadarTransmit::RadarStayAlive()
{
//    qDebug()<<Q_FUNC_INFO;

    const uchar transmit1[2]={0xA0,0xC1};
    const uchar transmit2[2]={0x03,0xC2};
    const uchar transmit3[2]={0x04,0xC2};
    const uchar transmit4[2]={0x05,0xC2};

    socket.writeDatagram((const char*)&transmit1,2,QHostAddress(_data),_data_port);
    socket.writeDatagram((const char*)&transmit2,2,QHostAddress(_data),_data_port);
    socket.writeDatagram((const char*)&transmit3,2,QHostAddress(_data),_data_port);
    socket.writeDatagram((const char*)&transmit4,2,QHostAddress(_data),_data_port);

}
void RadarTransmit::setControlValue(ControlType controlType, int value) {  // sends the command to the radar
    //  bool r = false;

    switch (controlType)
    {
    case CT_REFRESHRATE:
    case CT_BEARING_ALIGNMENT:
    {
        // to be consistent with the local bearing alignment of the pi
        // this bearing alignment works opposite to the one an a Lowrance display
        if (value < 0)
        {
            value += 360;
        }
        int v = value * 10;
        int v1 = v / 256;
        int v2 = v & 255;
        quint8 cmd[4] = {0x05, 0xc1, (quint8)v2, (quint8)v1};
        qDebug()<<Q_FUNC_INFO<<"Bearing alignment:"<<v;
        socket.writeDatagram((const char *)cmd,4,QHostAddress(_data),_data_port);
        break;
    }
    case CT_GAIN:
    {
        if (value < 0)
        {  // AUTO gain
            quint8 cmd[] =
            {
                0x06, 0xc1, 0, 0, 0, 0, 0x01, 0, 0, 0, 0xad  // changed from a1 to ad
            };
            qDebug()<<Q_FUNC_INFO<<"AUTO gain";
            socket.writeDatagram((const char *)cmd,11,QHostAddress(_data),_data_port);
        }
        else
        {  // Manual Gain
            int v = (value + 1) * 255 / 100;
            if (v > 255)
            {
                v = 255;
            }
            quint8 cmd[] = {0x06, 0xc1, 0, 0, 0, 0, 0, 0, 0, 0, (quint8)v};
            qDebug()<<Q_FUNC_INFO<<"manual gain"<<value;
            socket.writeDatagram((const char *)cmd,11,QHostAddress(_data),_data_port);
        }
        break;
    }

    case CT_SEA:
    {
        if (value < 0)
        {  // Sea Clutter - Auto
            quint8 cmd[11] = {0x06, 0xc1, 0x02, 0, 0, 0, 0x01, 0, 0, 0, 0xd3};
            qDebug()<<Q_FUNC_INFO<<"Auto Sea";
            socket.writeDatagram((const char *)cmd,11,QHostAddress(_data),_data_port);
        }
        else
        {  // Sea Clutter
            int v = (value + 1) * 255 / 100;
            if (v > 255)
            {
                v = 255;
            }
            quint8 cmd[] = {0x06, 0xc1, 0x02, 0, 0, 0, 0, 0, 0, 0, (quint8)v};
            qDebug()<<Q_FUNC_INFO<<"manual Sea"<<value;
            socket.writeDatagram((const char *)cmd,11,QHostAddress(_data),_data_port);
        }
        break;
    }

    case CT_RAIN:
    {  // Rain Clutter - Manual. Range is 0x01 to 0x50
        int v = (value + 1) * 255 / 100;
        if (v > 255)
        {
            v = 255;
        }
        quint8 cmd[] = {0x06, 0xc1, 0x04, 0, 0, 0, 0, 0, 0, 0, (quint8)v};
        qDebug()<<Q_FUNC_INFO<<"Manual rain"<<value;
        socket.writeDatagram((const char *)cmd,11,QHostAddress(_data),_data_port);
        break;
    }

    case CT_SIDE_LOBE_SUPPRESSION:
    {
        if (value < 0)
        {
            quint8 cmd[] = {// SIDE_LOBE_SUPPRESSION auto
                           0x06, 0xc1, 0x05, 0, 0, 0, 0x01, 0, 0, 0, 0xc0};
            qDebug()<<Q_FUNC_INFO<<"auto SIDE_LOBE_SUPPRESSION";
            socket.writeDatagram((const char *)cmd,11,QHostAddress(_data),_data_port);
        }
        else
        {
            int v = (value + 1) * 255 / 100;
            if (v > 255)
            {
                v = 255;
            }
            quint8 cmd[] = {0x6, 0xc1, 0x05, 0, 0, 0, 0, 0, 0, 0, (quint8)v};
            qDebug()<<Q_FUNC_INFO<<"manual SIDE_LOBE_SUPPRESSION"<<value;
            socket.writeDatagram((const char *)cmd,11,QHostAddress(_data),_data_port);
        }
        break;
    }
    case CT_INTERFERENCE_REJECTION:
    {
        quint8 cmd[] = {0x08, 0xc1, (quint8)value};
        qDebug()<<Q_FUNC_INFO<<"CT_INTERFERENCE_REJECTION"<<value;
        socket.writeDatagram((const char *)cmd,3,QHostAddress(_data),_data_port);
        break;
    }

    case CT_TARGET_EXPANSION:
    {
        quint8 cmd[] = {0x09, 0xc1, (quint8)value};
        qDebug()<<Q_FUNC_INFO<<"CT_TARGET_EXPANSION"<<value;
        socket.writeDatagram((const char *)cmd,3,QHostAddress(_data),_data_port);
        break;
    }

    case CT_TARGET_BOOST:
    {
        quint8 cmd[] = {0x0a, 0xc1, (quint8)value};
        qDebug()<<Q_FUNC_INFO<<"CT_TARGET_BOOST"<<value;
        socket.writeDatagram((const char *)cmd,3,QHostAddress(_data),_data_port);
        break;
    }
    case CT_LOCAL_INTERFERENCE_REJECTION:
    {
        if (value < 0) value = 0;
        if (value > 3) value = 3;
        quint8 cmd[] = {0x0e, 0xc1, (quint8)value};
        qDebug()<<Q_FUNC_INFO<<"CT_LOCAL_INTERFERENCE_REJECTION"<<value;
        socket.writeDatagram((const char *)cmd,3,QHostAddress(_data),_data_port);
        break;
    }

    case CT_SCAN_SPEED:
    {
        quint8 cmd[] = {0x0f, 0xc1, (quint8)value};
        qDebug()<<Q_FUNC_INFO<<"CT_SCAN_SPEED"<<value;
        socket.writeDatagram((const char *)cmd,3,QHostAddress(_data),_data_port);
        break;
    }
    case CT_NOISE_REJECTION:
    {
        quint8 cmd[] = {0x21, 0xc1, (quint8)value};
        qDebug()<<Q_FUNC_INFO<<"CT_NOISE_REJECTION"<<value;
        socket.writeDatagram((const char *)cmd,3,QHostAddress(_data),_data_port);
        break;
    }

    case CT_TARGET_SEPARATION:
    {
        quint8 cmd[] = {0x22, 0xc1, (quint8)value};
        qDebug()<<Q_FUNC_INFO<<"CT_TARGET_SEPARATION"<<value;
        socket.writeDatagram((const char *)cmd,3,QHostAddress(_data),_data_port);
        break;
    }
    case CT_ANTENNA_HEIGHT:
    {
        int v = value * 1000;  // radar wants millimeters, not meters :-)
        int v1 = v / 256;
        int v2 = v & 255;
        quint8 cmd[10] = {0x30, 0xc1, 0x01, 0, 0, 0, (quint8)v2, (quint8)v1, 0, 0};
        qDebug()<<Q_FUNC_INFO<<"CT_ANTENNA_HEIGHT"<<value;
        socket.writeDatagram((const char *)cmd,10,QHostAddress(_data),_data_port);
        break;
    }
    }
}
