#ifndef RADARENGINE_GLOBAL_H
#define RADARENGINE_GLOBAL_H

#include <QtCore/qglobal.h>

#ifdef RADAR_ENGINE_ARMED_STATIC
#   define RADAR_ENGINE_ARMED_EXPORT
#else
#  if defined(RADAR_ENGINE_LIBRARY)
#    define RADAR_ENGINE_EXPORT Q_DECL_EXPORT
#  else
#    define RADAR_ENGINE_EXPORT Q_DECL_IMPORT
#  endif
#endif

/*
   VERSION is (major << 16) + (minor << 8) + patch.
*/
#define RADAR_ENGINE_ARMED_VERSION RADAR_ENGINE_ARMED_VERSION_CHECK(RADAR_ENGINE_ARMED_VERSION_MAJOR, RADAR_ENGINE_ARMED_VERSION_MINOR, RADAR_ENGINE_ARMED_VERSION_PATCH)

/*
   can be used like #if (RADAR_ENGINE_ARMED_VERSION >= RADAR_ENGINE_ARMED_VERSION_CHECK(1, 3, 0))
*/
#define RADAR_ENGINE_ARMED_VERSION_CHECK(major, minor, patch) ((major<<16)|(minor<<8)|(patch))

#define RETURNS_PER_LINE (512)      //  radars generate 512 separate values per range, at 8 bits each

#endif // RADARENGINE_GLOBAL_H
