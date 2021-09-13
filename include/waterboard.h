#ifndef UWATERCOUNTER_FILE
#define UWATERCOUNTER_FILE

#include<WiFiManager.h>

#define WATER_DEVICE_IDENTY (('W'<<8)|'B')

#define HOT_WATER 0
#define COLD_WATER 1

#define MAX_WATER_DEVICE 2

#define BATTERY_PIN 4
#define CONNECT_PIN 12
#define COLD_PIN 13
#define HOT_PIN COLD_PIN+1

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

class UWaterCounter
{
public:
	UWaterCounter();

	void setup();
	void loop();

	void save();

	void light_sleep();
private:
	bool is_initialise;
	WiFiManager wifiManager;
	
	WiFiClient client;

	String mqttData;

	WaterBoardDevice water_devices[MAX_WATER_DEVICE];
	bool need_save;

	bool readEEPROMConfig();
	void initPinAndInterrupt();
	void connect();
	bool makeData();

	void resetConfig();

	void updateResource();
	void updateMessage();

	void saveDevicesToEEPROM();

	void LinkDevice();
	void UnLinkDevice();
};
#endif