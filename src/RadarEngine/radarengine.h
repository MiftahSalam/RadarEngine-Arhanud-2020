#ifndef RADARENGINE_H
#define RADARENGINE_H

#include "radarengine_global.h"
#include "radarreceive.h"

namespace RadarEngineARMD {

class RADAR_ENGINE_EXPORT RadarEngine : public QObject
{
    Q_OBJECT
public:
    RadarEngine();

private:
    QList<RadarReceive*> radarConList;
};

}

#endif // RADARENGINE_H
