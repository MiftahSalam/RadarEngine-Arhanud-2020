#ifndef RADARARPA_H
#define RADARARPA_H

#include <QObject>

#include "radarengine_global.h"
#include "arpatarget.h"

namespace RadarEngineARND {

class RadarEngine;

class RadarArpa : public QObject
{
    Q_OBJECT
public:
    explicit RadarArpa(QObject *parent = 0,RadarEngine *ri=0);

    int m_number_of_targets,range_meters;
    ARPATarget *m_target[MAX_NUMBER_OF_TARGETS];

    bool MultiPix(int ang, int rad);
    void AcquireNewMARPATarget(Position p);
    int AcquireNewARPATarget(Polar pol, int status);
    void RefreshArpaTargets();
    void DeleteTarget(Position p);
    void DeleteAllTargets();
    void RadarLost()
    {
        DeleteAllTargets();
    }

private:
    bool Pix(int ang, int rad);
    void AcquireOrDeleteMarpaTarget(Position target_pos, int status);
    RadarEngine *m_ri;

};

}
#endif // RADARARPA_H
