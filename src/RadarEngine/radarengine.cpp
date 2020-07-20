#include "radarengine.h"

using namespace RadarEngineARMD;

RadarEngine::RadarEngine()
{
    radarConList.append(new RadarReceive(this,this));
}
