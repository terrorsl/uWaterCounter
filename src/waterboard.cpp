#include"../include/version.h"
#include"../include/waterboard.h"
#include"../include/updater.h"

#include <WiFiUdp.h>
#include <NTPClient.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

#include<Chronos.h>
//#include"getTime.h"

#include<Ticker.h>

#include<EEPROM.h>
#include <ArduinoJson.h>

//#define MQTT_DEBUG_ON 1
#include "MQTT.h"

#define ThisDeviceName(out,id) sprintf(out,"UWaterCounter_%d",id)

//ADC_MODE(ADC_VCC);

WiFiClient net;
MQTTClient mqtt;//("waterboard","192.168.212.103",1883);
WiFiManager wifiManager;

namespace EventTimeType
{
	enum
	{
		Save,
		Updater,
		Max
	};
};

//NTPService ntpService;

DefineCalendarType(Calendar,EventTimeType::Max);
Calendar calendar;
volatile bool send_data_to_server=false;

uint64_t before,after;

Ticker led_blink_tiker;
int led_state = 1023;
int sign = 1;
char step = 19;
void led_ticker_callback()
{
	if (led_state>1023)
		sign = -1;
	if (led_state<255)
		sign = 1;

	int cstep = step*sign;
	led_state = led_state + cstep;
	analogWrite(LED_BUILTIN, led_state);
};
void saveConfig()
{
	char ssid[32];
	ThisDeviceName(ssid, ESP.getChipId());
	mqtt.connect(ssid);
	mqtt.publish("new", ssid);
};

void IRAM_ATTR WaterDeviceCallback(void *arg)
{
	cli();
	WaterBoardDevice *device=(WaterBoardDevice*)arg;
	device->value++;
	Serial.print("WaterDeviceCallback:");
	Serial.println(device->value);
	send_data_to_server=true;
	sei();
};
void IRAM_ATTR BatteryDeviceCallback(void *arg)
{
	UWaterCounter *water = (UWaterCounter*)arg;
	cli();
	water->save();
	digitalWrite(LED_BUILTIN, LOW);
	send_data_to_server = true;
	sei();
};
void IRAM_ATTR WIFIConnectManagerCallback(void *arg)
{
	cli();
	UWaterCounter *data = (UWaterCounter*)arg;
	data->StartWiFiManager();
	sei();
	/*led_blink_tiker.attach_ms(100, led_ticker_callback);
	char ssid[32];
	ThisDeviceName(ssid,ESP.getChipId());

	WiFiManagerParameter custom_mqtt_server("server", "mqtt server", "uhome.local", 40);
	WiFiManagerParameter custom_mqtt_port("port", "mqtt port", "1883", 6);
	wifiManager.addParameter(&custom_mqtt_server);
	wifiManager.addParameter(&custom_mqtt_port);

	wifiManager.setSaveConfigCallback(saveConfig);
	wifiManager.resetSettings();

	wifiManager.autoConnect(ssid);;
	cli();
	MQTTServerData *data = (MQTTServerData*)arg;
	//water->SetMqttServerData(custom_mqtt_server.getValue(), String(custom_mqtt_port.getValue()).toInt());
	sei();
	led_blink_tiker.detach();*/
};

void myConnectedCb()
{
	Serial.println("connected to MQTT server");
		//mqtt.disconnect();
}
void myDataCb(String& topic, String& data)
{

  Serial.print(topic);
  Serial.print(": ");
  Serial.println(data);
}

uint64_t getRTCTime()
{
	uint64_t cali = system_rtc_clock_cali_proc();
	uint64_t cycles = system_get_rtc_time();
	cali = (cali * 1000) >> 12;

	return (cali * cycles) / 1000;
}

bool UWaterCounter::readEEPROMConfig()
{
	int max_need_rom=2+sizeof(WaterBoardDevice)*MAX_WATER_DEVICE;
	unsigned long address=0;
	EEPROM.begin(max_need_rom);
	unsigned short identy;
	EEPROM.get(address,identy);
	address+=2;
	if(identy==WATER_DEVICE_IDENTY)
	{
		for(int i=0;i<MAX_WATER_DEVICE;i++)
		{
			EEPROM.get(address,water_devices[i]);
			address+=sizeof(WaterBoardDevice);
		}
		return false;
	}
	return true;
};
void UWaterCounter::initPinAndInterrupt()
{
	pinMode(CONNECT_PIN,INPUT_PULLUP);
	pinMode(BATTERY_PIN, INPUT_PULLUP);
	pinMode(LED_BUILTIN,OUTPUT);

	attachInterruptArg(digitalPinToInterrupt(CONNECT_PIN), WIFIConnectManagerCallback, this, FALLING);
	attachInterruptArg(digitalPinToInterrupt(BATTERY_PIN), BatteryDeviceCallback, this, FALLING);

	for(int i=0;i<MAX_WATER_DEVICE;i++)
	{
		pinMode(water_devices[i].pin,INPUT_PULLUP);
		attachInterruptArg(digitalPinToInterrupt(water_devices[i].pin),WaterDeviceCallback,&water_devices[i],FALLING);
	}

	digitalWrite(LED_BUILTIN, HIGH);
};
void UWaterCounter::connect()
{
	WiFi.begin();
	WiFi.waitForConnectResult();

	mqtt_client.connect(serverData.server, serverData.port);
};

void UWaterCounter::setup()
{	
	Serial.begin(115200);
	//ESP_LOGD("setup begin");

	bool firstTime=readEEPROMConfig();
	initPinAndInterrupt();

	Serial.println(VERSION_STR);

	//if(firstTime || digitalRead(CONNECT_PIN)==LOW)
	//	WIFIConnectManager(this);

	/*Updater updater;
	updater.initialise();
	updater.run();*/
#if 0
	//attachInterruptArg(digitalPinToInterrupt(CONNECT_PIN),WaterDeviceCallback,&water_devices[0],FALLING);
	//attachInterrupt(digitalPinToInterrupt(LED_PIN),LedDeviceCallback,FALLING);

	//wifiManager.setConfigPortalTimeout(60);
	//wifiManager.setDebugOutput(false);
	is_initialise=false;

	calendar.add(Chronos::Event(EventTimeType::Save,Chronos::Mark::Hourly(60),Chronos::Span::Seconds(10)));
	calendar.add(Chronos::Event(EventTimeType::Updater,Chronos::Mark::Weekly(Chronos::Weekday::Wednesday,23,0,0),Chronos::Span::Seconds(10)));

	int max_need_rom=2+sizeof(WaterBoardDevice)*MAX_WATER_DEVICE;
	unsigned long address=0;
	EEPROM.begin(max_need_rom);
	unsigned short identy;
	EEPROM.get(address,identy);
	address+=2;
	if(identy==WATER_DEVICE_IDENTY)
	{
		for(int i=0;i<MAX_WATER_DEVICE;i++)
		{
			EEPROM.get(address,water_devices[i]);
			address+=sizeof(WaterBoardDevice);

			pinMode(water_devices[i].pin,INPUT_PULLUP);
		}
	}
	else
	{
		for(int i=0;i<MAX_WATER_DEVICE;i++)
		{
			water_devices[i].pin=i+COLD_PIN;
			pinMode(water_devices[i].pin,INPUT_PULLUP);
		}
		firstTime=true;
	}
	EEPROM.end();

	need_save=false;

	if(firstTime || digitalRead(CONNECT_PIN)==LOW)
	{
		led_blink_tiker.attach_ms(100,led_ticker_callback);
		String ssid=DEVICE_NAME+'_'+ESP.getChipId();
		wifiManager.setSaveConfigCallback(saveConfig);
		wifiManager.autoConnect(ssid.c_str()); 
	}
	/*else
	{
	}*/

	/*if(digitalRead(CONNECT_PIN)==LOW)
	{
		String ssid=DEVICE_NAME+'_'+ESP.getChipId();
		wifiManager.autoConnect(ssid.c_str());
	}
	else
		WiFi.waitForConnectResult();
		*/
#if 0
	WiFi.persistent(false);
	WiFi.disconnect(true);
	WiFi.mode(WIFI_STA);
	//WiFi.setSleepMode(WIFI_LIGHT_SLEEP,0xFFFFFFF);

	WiFi.begin("terror","terror23011985");
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		//send(".");
	}

	//mqtt.onConnected(myConnectedCb);
	//mqtt.onData(myDataCb);

	/*mqtt.connect();
	String data("456");
	mqtt.publish("waterboard/value",data,0,1);
	//mqtt.subscribe("broker/counter");
	mqtt.subscribe("new");*/
	Serial.println(ESP.getChipId());
	Serial.println("setup done");
#endif
#endif
	before=getRTCTime();
	after=before;
	//WiFi.forceSleepBegin();
};
static bool state=LOW;
static bool send_over=false;

void UWaterCounter::light_sleep()
{
	Serial.println("goto sleep");
	delay(200);
	wifi_station_disconnect();
	delay(1000);
	wifi_set_opmode(NULL_MODE);    // set WiFi mode to null mode.
	os_delay_us(5000);
	wifi_fpm_set_sleep_type(LIGHT_SLEEP_T); // light sleep
	/*PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U,3);*/
	/*gpio_pin_wakeup_enable(14, GPIO_PIN_INTR_LOLEVEL);*/
	for(int i=0;i<MAX_WATER_DEVICE;i++)
		gpio_pin_wakeup_enable(GPIO_ID_PIN(water_devices[i].pin),GPIO_PIN_INTR_LOLEVEL);
	gpio_pin_wakeup_enable(GPIO_ID_PIN(CONNECT_PIN), GPIO_PIN_INTR_LOLEVEL);
	gpio_pin_wakeup_enable(GPIO_ID_PIN(BATTERY_PIN), GPIO_PIN_INTR_LOLEVEL);
	wifi_fpm_open();
	/*wifi_fpm_set_wakeup_cb(fpm_wakup_cb_func1); // Set wakeup callback*/
	wifi_fpm_do_sleep(0xFFFFFFF);
	delay(200);
	Serial.println("wakeup");
};

bool UWaterCounter::makeData()
{
	bool ret=false;
	cli();
	if(send_data_to_server)
	{
		DynamicJsonDocument doc(1024);
		doc["firmware"] = FIRMWARE;
		doc["serial"][0]=water_devices[0].serial;
		doc["serial"][1]=water_devices[1].serial;
		doc["data"][0]=water_devices[0].value;
		doc["data"][1]=water_devices[1].value;
		serializeJson(doc,mqttData);
		send_data_to_server=false;
		ret=true;
	}
	sei();
	return ret;
};
void UWaterCounter::InitialiseWiFiManager()
{
	isWiFiManager = false;
	char ssid[32];
	ThisDeviceName(ssid, ESP.getChipId());

	WiFiManagerParameter custom_mqtt_server("server", "mqtt server", "uhome.local", 40);
	WiFiManagerParameter custom_mqtt_port("port", "mqtt port", "1883", 6);
	wifiManager.addParameter(&custom_mqtt_server);
	wifiManager.addParameter(&custom_mqtt_port);

	//wifiManager.setSaveConfigCallback(saveConfig);
	wifiManager.resetSettings();

	wifiManager.autoConnect(ssid);

	strcpy(serverData.server,custom_mqtt_server.getValue());
	serverData.port=String(custom_mqtt_port.getValue()).toInt();
};
void UWaterCounter::loop()
{
	if (isWiFiManager)
		InitialiseWiFiManager();
	if (WiFi.isConnected() == false)
		connect();
	if (timeClient.isTimeSet())
	{
		if (lastUpdaterTime == 0)
			lastUpdaterTime = timeClient.getEpochTime();
		if (timeClient.getEpochTime() - lastUpdaterTime > TIME_BETWEEN_UPDATE)
		{
			lastUpdaterTime = timeClient.getEpochTime();
			Serial.print(TIME_BETWEEN_UPDATE);
			Serial.print("\n");
			Updater *updater=new Updater();
			if (updater->initialise())
				updater->run();
			delete updater;
		}
	}
	if (makeData())
	{
		char ssid[32];
		ThisDeviceName(ssid, ESP.getChipId());
		mqtt_client.publish(ssid, mqttData);
	}

	mqtt_client.loop();
	timeClient.update();

	//ms
	float delta = (after - before) / 1000.f;
	if (delta > TIME_BETWEEN_SLEEP)
	{
		if(mqtt_client.connected())
			mqtt_client.close();
		timeClient.end();
		before = after;
		delay(200);
		light_sleep();
	}
	if(delta>TIME_SEC_TO_MS(1))
		Serial.println(delta);
	after = getRTCTime();

#if 0
	//ms
	float delta=(after-before)/1000.f;
	if(delta>=TIME_BETWEEN_PUBLISH)
	{
		if(makeData())
		{
			Serial.println("makeData()");
			connect();
			timeClient.begin();
			//todo update if no mqtt
			if(WiFi.isConnected())
			{
				if (mqtt_client.connected())
				{
					char ssid[32];
					ThisDeviceName(ssid, ESP.getChipId());
					mqtt_client.publish(ssid, mqttData);
				}
			}
			else
			{
				light_sleep();
			}
		}
		else
		{
			light_sleep();
		}
		before=after;
	}
	after=getRTCTime();
#endif

#if 0
	//delay(200);
	float delta=(after-before)/1000.f;
	if(delta>3000)
	{
		Serial.println("go light sleep");
		delay(200);
		//light_sleep();
		delay(200);
		Serial.println("wake up");
		before=after;
	}
	after=getRTCTime();
	Serial.println(delta);
	delay(200);
	return;
	/*mqtt.connect();
	String data(water_devices[0].value);
	mqtt.publish("waterboard/value",data,0,1);
	mqtt.disconnect();*/
	//if(WiFi.status()!=WL_CONNECTED)
	if(WiFi.isConnected()==false)
	{
		Serial.println("connect ot WiFi");
		connect();
		/*WiFi.begin();
		delay(1000);*/
	}
	else
	{
		cli();
		if(send_data_to_server)
		{
			DynamicJsonDocument doc(1024);
			doc["serial"][0]=water_devices[0].serial;
			doc["serial"][1]=water_devices[1].serial;
			doc["data"][0]=water_devices[0].value;
			doc["data"][1]=water_devices[1].value;
			serializeJson(doc, mqttData);
			send_data_to_server=false;

			/*if(mqtt.isConnected()==false)
			{
				mqtt.connect();
				while(mqtt.isConnected()==false);
			}
			//else
			{
				Serial.println("send data to mqtt server");
				String data;//(water_devices[0].value);

				DynamicJsonDocument doc(1024);
				doc["serial"][0]=water_devices[0].serial;
				doc["serial"][1]=water_devices[1].serial;
				doc["data"][0]=water_devices[0].value;
				doc["data"][1]=water_devices[1].value;
				serializeJson(doc, data);

				String topic("waterboard/");
				topic=topic+ESP.getChipId()+"/data";
				mqtt.publish(topic,data,0,1);
				//mqtt.disconnect();
				delay(5000);
				send_data_to_server=false;
				send_over=true;
			}*/
		}
		sei();

		if(mqttData.length()==0)
		{
			delay(2000);
			Serial.println("sleep");
			WiFi.forceSleepBegin();
			delay(20000);
			Serial.println("wake up");
			WiFi.forceSleepWake();
			WiFi.waitForConnectResult();
		}
		else
		{
			//if(mqtt.isConnected()==false)
			{
				mqtt.connect();
				delay(5000);
				String topic("waterboard/");
				topic=topic+ESP.getChipId()+"/data";
				mqtt.publish(topic,mqttData,0,1);
				mqttData.clear();
				delay(5000);
				//mqtt.disconnect();
				//delay(5000);
			}
		}
	}
#endif
	/*float v=ESP.getVcc()/1024.0;
	Serial.println(v);*/
	
	/*
	WiFi.forceSleepBegin();
	Serial.println("WiFi is down");
	delay(20000);
	state=!state;
	digitalWrite(BUILTIN_LED,state);
	WiFi.forceSleepWake();
	Serial.println("WiFi is up");
	delay(2000);
	*/

	/*if(WiFi.isConnected()==false)
	{
		String ssid=DEVICE_NAME+'_'+ESP.getChipId();
		if(wifiManager.autoConnect(ssid))
		{
			Serial.println("connect");
			//Serial.println(client.connect("192.168.4.1",1500));

			Serial.println(WiFi.gatewayIP().toString().c_str());
			Serial.println(WiFi.localIP());
			Serial.println(WiFi.dnsIP().toString().c_str());

			ntpService.initialise("ru.pool.ntp.org");
			ntpService.SetCheckInterval(SECS_PER_HOUR);
			//WiFi.hostByName("ru.pool.ntp.org", timeServerIP);
			//setSyncProvider(getNtpTime);
			Updater updater;
			updater.initialise();
			updater.run();

			connect.connect();
		}
	}
	else
		updateMessage();

	updateResource();

	Chronos::Event::Occurrence occurrenceList[4];
	int numOngoing = calendar.listOngoing(4,occurrenceList, Chronos::DateTime::now());
	if(numOngoing)
	{
		// At least one event is happening at "nowTime"...
		for(int i = 0; i < numOngoing; i++)
		{
			switch(occurrenceList[i].id)
			{
			case EventTimeType::Save:
				if(need_save)
					saveDevicesToEEPROM();
				break;
			case EventTimeType::Updater:
				{
					Updater updater;
					if(updater.initialise())
					{
						saveDevicesToEEPROM();
						updater.run();
					}
				}
				break;
			}
		}
	}*/
};
void UWaterCounter::updateResource()
{
	for(int i=0;i<MAX_WATER_DEVICE;i++)
	{
		if(digitalRead(water_devices[i].pin)==LOW)
		{
			if(water_devices[i].state==0)
			{
				water_devices[i].value+=10;
				water_devices[i].state=1;
				need_save=true;
			}
		}
		else
			water_devices[i].state=0;
	}
};
void UWaterCounter::save()
{
	saveDevicesToEEPROM();
};
void UWaterCounter::saveDevicesToEEPROM()
{
	need_save=false;

	int max_need_rom=2+sizeof(WaterBoardDevice)*MAX_WATER_DEVICE+sizeof(MQTTServerData);
	unsigned long address=0;
	EEPROM.begin(max_need_rom);
	unsigned short identy;
	EEPROM.put(address,identy);
	address+=2;
	for(int i=0;i<MAX_WATER_DEVICE;i++)
	{
		EEPROM.put(address,water_devices[i]);
		address+=sizeof(WaterBoardDevice);
	}
	EEPROM.put(address, serverData);
	EEPROM.end();
};
UWaterCounter::UWaterCounter():isWiFiManager(false)
{
	water_devices[0].pin=HOT_PIN;
	water_devices[0].type=HOT_WATER;

	water_devices[1].pin=COLD_PIN;
	water_devices[1].type=COLD_WATER;

	strcpy(serverData.server, "uhome.local");
	serverData.port = 1883;
	lastUpdaterTime = 0;
};