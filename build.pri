# uncomment if you want to build a static RADAR_ENGINE_ARND library
DEFINES += RADAR_ENGINE_ARND_STATIC

RADAR_ENGINE_ARND_VERSION_MAJOR = 1
RADAR_ENGINE_ARND_VERSION_MINOR = 0
RADAR_ENGINE_ARND_VERSION_PATCH = 0

DEFINES += RADAR_ENGINE_ARND_VERSION_MAJOR=$${RADAR_ENGINE_ARND_VERSION_MAJOR}
DEFINES += RADAR_ENGINE_ARND_VERSION_MINOR=$${RADAR_ENGINE_ARND_VERSION_MINOR}
DEFINES += RADAR_ENGINE_ARND_VERSION_PATCH=$${RADAR_ENGINE_ARND_VERSION_PATCH}
DEFINES += RADAR_ENGINE_ARND_VERSION_STR='\\"$${RADAR_ENGINE_ARND_VERSION_MAJOR}.$${RADAR_ENGINE_ARND_VERSION_MINOR}.$${RADAR_ENGINE_ARND_VERSION_PATCH}\\"'
