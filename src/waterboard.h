#ifndef UWATERCOUNTER_FILE
#define UWATERCOUNTER_FILE

#include"config.h"
#include<WiFiManager.h>
#include<TinyMqtt.h>

#define WATER_DEVICE_IDENTY (('W'<<8)|'B')

#define HOT_WATER 0
#define COLD_WATER 1

#define MAX_WATER_DEVICE 2

#define BATTERY_PIN 4
#define CONNECT_PIN 12
#define COLD_PIN 13
#define HOT_PIN COLD_PIN+1

#define TIME_SEC_TO_MS(ms) ms*1000
#define TIME_MIN_TO_SEC(m) m*60
#define TIME_HOUR_TO_MIN(h) h*60
#define TIME_DAY_TO_HOUR(d) d*24

#define TIME_BETWEEN_SLEEP TIME_SEC_TO_MS(TIME_MIN_TO_SEC(2))
#define TIME_BETWEEN_UPDATE TIME_MIN_TO_SEC(60)
#define TIME_BETWEEN_PUBLISH 60000

struct WaterBoardDevice
{
	WaterBoardDevice()
	{
		state=0;
		value=0;
	};
	unsigned char type:1;
	unsigned char state:1;
	unsigned char pin:5;
	
	unsigned long value;
	char serial[32];
};
struct MQTTServerData
{
	char server[40];
	short port;
};

class UWaterCounter
{
public:
	UWaterCounter();
	UWaterCounter(int hot_pin,int cold_pin);

	void setup();
	void loop();

	void StartWiFiManager() { isWiFiManager = true; };

	void save();

	void light_sleep();
private:
	void InitialiseWiFiManager();

	bool isWiFiManager,is_initialise;
	WiFiManager wifiManager;
	
	WiFiClient client;

	String mqttData;

	MqttClient mqtt_client;
	WaterBoardDevice water_devices[MAX_WATER_DEVICE];
	MQTTServerData serverData;
	unsigned long lastUpdaterTime;
	bool need_save;

	void initPinAndInterrupt();
	void connect();
	bool makeData();

	bool readEEPROMConfig();
	void saveDevicesToEEPROM();
};
#endif