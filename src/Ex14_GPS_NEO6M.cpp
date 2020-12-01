
#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <memory>
#include <string>
#include <NMEAGPS.h>
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.
#include "modules/ScreenDebugger.h"

#define EARTH_RADIUS_METERS 6371000

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C _u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);

#define PIN_LED 32
//#define DUMP_AT_COMMANDS
#define CHECK_GPRS_EVERY_MS 20000

// objeto de comunicação serial do SIM800L
HardwareSerial SerialGPS(2);
TwoWire I2Cscreen = TwoWire(0);
ScreenDebugger _TheDebug(&_u8g2, 5, 10);

// // objeto da bibliteca com as funções GSM
// TinyGsm modemGSM(SerialGSM);
// velocidade da serial tanto do SIM800L quanto do monitor serial
//const int BAUD_RATE = 28800;//9600;
// pinos aonde os reles serão ligados e RX / TX aonde o SIM800L será ligado
//const int RX_PIN = 4, TX_PIN = 2;

int _timeout=0;

//------------------------------------------------------------
// Check that the config files are set up properly

#if !defined( NMEAGPS_PARSE_RMC )
#error You must uncomment NMEAGPS_PARSE_RMC in NMEAGPS_cfg.h!
#endif

#if !defined( GPS_FIX_TIME )
#error You must uncomment GPS_FIX_TIME in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_LOCATION )
#error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_SPEED )
#error You must uncomment GPS_FIX_SPEED in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_SATELLITES )
#error You must uncomment GPS_FIX_SATELLITES in GPSfix_cfg.h!
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
#error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

//------------------------------------------------------------

static NMEAGPS  _TheNeoGps; // This parses the GPS characters
static gps_fix  _TheFix;

//Envia comando AT e aguarda até que uma resposta seja obtida
String sendAT(String command)
{
	String response = "";
	SerialGPS.println(command);
	// aguardamos até que haja resposta do SIM800L
	while (!SerialGPS.available());

	response = SerialGPS.readString();

	return response;
}

void setup()
{
	Serial.begin(115200);
	// wait for serial monitor to open
	while (!Serial);

	log_d("Setup GPS...");
	SerialGPS.begin(9600);

	pinMode(PIN_LED, OUTPUT);

	I2Cscreen.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000); //0=default 100000 100khz

	log_d("Begin Display...");
	_u8g2.setBusClock(100000);
	_u8g2.begin();
	_u8g2.setFont(u8g2_font_5x8_mf);
	_TheDebug.SetFont(ScreenDebugger::SIZE1);
	_TheDebug.NewLine("Setup Complete!");
}

template<typename ... Args>
static std::string string_format(const std::string& format, Args ... args)
{
	size_t size = snprintf(nullptr, 0, format.c_str(), args ...) + (size_t)1; // Extra space for '\0'
	std::unique_ptr<char[]> buf(new char[size]);
	snprintf(buf.get(), size, format.c_str(), args ...);
	return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

void loop()
{
	// String msg, number;
	// bool anyData=false;
	// std::list<std::string> sats;

	// while(SerialGPS.available()) {
	// 	std::string text=SerialGPS.readStringUntil('\n').c_str();
	// 	//Serial.write(SerialGPS.read());
	// 	Serial.println(text.c_str());
	// 	//$GPGSV,4,2,15,08,,,25,10,,,24,11,,,24,13,,,24*71
	// 	if(text.find("GPGSV")!=-1) {
	// 		sats.push_back(text.substr(7));
	// 	}
	// 	// if(text.find("GPGSA") != -1) {
	// 	// 	sats.clear();
	// 	// }
	// 	anyData=true;
	// }
	// _TheDebug.NewLines(sats);

	// // if(sats.size()) {
	// // 	DrawInfo(sats);
	// // }

	// while(Serial.available())	{
	// 	SerialGPS.write(Serial.read());
	// }
	// if(!anyData) {
	// 	delay(100);
	// }
	char buff[80];

	while(_TheNeoGps.available(SerialGPS)) {
		_TheFix = _TheNeoGps.read();
		if(_TheFix.valid.location) {
			snprintf(buff, sizeof(buff), "Fix! (%3.3f,%3.3f) ", _TheFix.latitude(), _TheFix.longitude());
			_TheDebug.NewLine(buff);
			snprintf(buff, sizeof(buff), "Alt=%d Sp=%3.2f", _TheFix.altitude_cm(), _TheFix.speed_kph());
			_TheDebug.NewLine(buff);
		}
		else {
			_TheDebug.NewLine("Not Fixed :(");
		}
   }
}

float DistanceBetween2Points(float Lat1, float Lon1, float Lat2, float Lon2, float unit_conversion)
{
	float dLat = radians(Lat2 - Lat1);
	float dLon = radians(Lon2 - Lon1);

	float a = sin(dLat / 2.0f) * sin(dLat / 2.0f) +
		cos(radians(Lat1)) * cos(radians(Lat2)) *
		sin(dLon / 2.0f) * sin(dLon / 2.0f);

	float d = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));

	return d * EARTH_RADIUS_METERS * unit_conversion;
}
