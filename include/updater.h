#ifndef UPDATER_FILE
#define UPDATER_FILE

#include<ESP_OTA_GitHub.h>
#include<CertStoreBearSSL.h>
#include<LittleFS.h>

#define GHOTA_USER "terrorsl"
#define GHOTA_REPO "uWaterCounter"
#define GHOTA_CURRENT_TAG "0.0.0"
#define GHOTA_BIN_FILE "uwatercounter.ino.esp8266.bin"
#define GHOTA_ACCEPT_PRERELEASE 0

class Updater
{
public:
	Updater():ota(0)
	{
	};
	~Updater()
	{
		if(ota)
			delete ota;
	}
	bool initialise()
	{
		// Retrieve certificates.
		LittleFS.begin();
		int numCerts = certStore.initCertStore(LittleFS, PSTR("/certs.idx"), PSTR("/certs.ar"));
		//SPIFFS.begin();
		//int numCerts = certStore.initCertStore(SPIFFS, PSTR("/certs.idx"), PSTR("/certs.ar"));
		if (numCerts == 0) {
			Serial.println(F("No certs found. Did you run certs-from-mozill.py and upload the SPIFFS directory before running?"));
			return false; // Can't connect to anything w/o certs!
		}
		ota=new ESPOTAGitHub(&certStore,GHOTA_USER,GHOTA_REPO,GHOTA_CURRENT_TAG,GHOTA_BIN_FILE,GHOTA_ACCEPT_PRERELEASE);
		return true;
	};
	void run()
	{
		if(ota==0)
			return;
		if(ota->checkUpgrade())
		{
			Serial.println(ota->getUpgradeURL());
			if(ota->doUpgrade()==false)
				Serial.println(ota->getLastError());
		}
		else
			Serial.println(ota->getLastError());
	};
private:
	BearSSL::CertStore certStore;
	ESPOTAGitHub *ota;
};
#endif