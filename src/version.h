#ifndef VERSION_FILE
#define VERSION_FILE

#define MAJOR 0
#define MINOR 0
#define BUILD 2

#define VSTRING(x) #x
#define TO_STRING(x) VSTRING(x)

#define VERSION_HEX (MAJOR<<16)|(MINOR<<8)|BUILD
#define VERSION_STR TO_STRING(MAJOR) "." TO_STRING(MINOR) "." TO_STRING(BUILD)
#ifdef ESP32
#define FIRMWARE "esp32_version_" VERSION_STR
#else
#define FIRMWARE "esp8266_version_" VERSION_STR
#endif
#endif