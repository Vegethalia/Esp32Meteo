
#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <U8g2lib.h>
#include <string>
#include "modules/MyBME280.h"
#include "mykeys.h"

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_LED_MIO 32
#define PIN_BUTTON 4
#define I2C_ADDRESS_BME280 0x76
#define I2C_BUS_SPEED 100000 //100000, 400000

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// ADAFRUIT Feeds / Related stuff
#define ADAFRUIT_ADDR      "io.adafruit.com"
#define ADAFRUIT_PORT      1883

#define FEED_TEMPERATURE   "/feeds/temperatura"
#define FEED_HUMIDITY      "/feeds/humitat"
#define FEED_PRESSURE      "/feeds/pressio"
#define FEED_TURN_ON_PWM   "/feeds/turnonled"
#define FEED_INTENSITY_PWN "/feeds/ledintensity"
#define FEED_LUX           "/feeds/lux"

#define PRESSURE_OFFSET 14 //looks like my sensor always returns the real pressure minus this offset
#define DEBOUNCE_TIME 250 // Filtre anti-rebond (debouncer)

//GLOBAL OBJECTS
WiFiClient _TheWifi;
PubSubClient _ThePubSub;
TwoWire _TheIc2Wire(0);
MyBME280 _TheBME;

//-----FORWARD DECLARATIONS
bool CheckWifi(); //Returns true if the Wifi Network is ready
bool CheckMQTT(); //Returns true if MQTT connections are working
bool UpdateValues(); //returns true if values were successfully read from BME280 sensor
void PrintValuesOnScreen(int millis);
void PubSubCallback(char *pTopic, uint8_t *pData, unsigned int dalaLength);
//-----FORWARD DECLARATIONS

//SetUp SH1106 / SSD1306
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA); //R0 = no rotate
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);

//VARIABLES
unsigned long _delayTimeUpt = 30000;
unsigned long _delayTimeLoop = 50;
unsigned long _lastProcessMillis = 0;

byte _lastClockChar = 0;
uint16_t _totalUpdateTime = 0;	 //used to accomulate the screen update time between sensor readings
uint16_t _numUpdates = 0;				 //number of updates accomulated on _totalUpdateTime
uint16_t _lastAvgUpdateTime = 0; //_totalUpdateTime/_numUpdates of the last period

//state vars
bool _BlueLedON = false;
bool _UpdateRequired = false;
byte _BlueLedIntensity = 50; //default intensity

volatile uint32_t _DebounceTimer = 0;

void IRAM_ATTR ButtonPressed()
{
	if (millis() - _DebounceTimer >= DEBOUNCE_TIME)
	{
		_DebounceTimer = millis();
		Serial.println("INTERRUPTED BY BUTTON!!");

		int pinValue = digitalRead(PIN_BUTTON);
		log_d("[%d] Pin value=%d", (int)millis(), pinValue);

		_BlueLedON = !_BlueLedON;
		_UpdateRequired = true;
	}
}

void setup()
{
	// put your setup code here, to run once:
	Serial.begin(115200);
	pinMode(PIN_LED_MIO, OUTPUT);
	pinMode(PIN_BUTTON, INPUT);

	// wait for serial monitor to open
	while (!Serial);

	//Initialize I2C bus
	log_d("Initializing i2c sda=%d scl=%d speed=%dkhz", PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_SPEED / 1000);
	if (!_TheIc2Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_SPEED)) { //busspeed=0 => default 100000
		log_d("i2c initialization failed...");
		delay(2000);
		ESP.restart();
	}
	//Initialize BME280
	if(_TheBME.Init(I2C_ADDRESS_BME280, _TheIc2Wire, 20000)!=MyBME280::ERROR::OK) {
		log_d("Could not find a valid BME280 sensor, check wiring!");
		delay(2000);
		ESP.restart();
	}
	_TheBME.SetPressureOffset(PRESSURE_OFFSET);

	log_d("Begin Display...");
	u8g2.setBusClock(I2C_BUS_SPEED);
	//	u8g2.beginSimple();// does not clear the display and does not wake up the display  user is responsible for calling clearDisplay() and setPowerSave(0)
	u8g2.begin();
	u8g2.enableUTF8Print();							 // enable UTF8 support for the Arduino print()
	u8g2.setContrast(_BlueLedIntensity); //set default contrast
																			 //	PrintValuesOnScreen(_delayTimeUpt); //Print 1st values

	//Initialize Wifi
	wl_status_t statuswf = WiFi.begin(WIFI_SSID, WIFI_PASS);
	if (statuswf != WL_CONNECTED)	{
		log_d("Couldn't get a wifi connection!");
	}
	else {
		log_d("Connected to wifi [%s]", WIFI_SSID);
	}
	
	//configure interrupts for buttons
	attachInterrupt(PIN_BUTTON, ButtonPressed, RISING);

	//Initial Read
	_TheBME.ReadSensor();
}

void loop()
{
	auto now = millis();

	if ((now - _lastProcessMillis) >= _delayTimeUpt) {
		_lastProcessMillis=now;
		digitalWrite(PIN_LED_MIO, HIGH); // turn on the "updating LED"
		delay(200);
		digitalWrite(PIN_LED_MIO, LOW); // turn off the "updating LED"

		if (_TheBME.ReadSensor() == MyBME280::ERROR::OK && CheckWifi() && CheckMQTT()) {
			_ThePubSub.publish((std::string(ADAIO_USER).append(FEED_TEMPERATURE)).c_str(), String(_TheBME.GetLatestTemperature()).c_str());
			_ThePubSub.publish((std::string(ADAIO_USER).append(FEED_HUMIDITY)).c_str(), String(_TheBME.GetLatestHumidity()).c_str());
			_ThePubSub.publish((std::string(ADAIO_USER).append(FEED_PRESSURE)).c_str(), String(_TheBME.GetLatestPressure()).c_str());
		}
	}
	if (_UpdateRequired) {
		if (_BlueLedON)
		{ //turn on screen
			u8g2.setPowerSave(0);
			u8g2.setContrast(_BlueLedIntensity);
		}
		else
		{ //turn off screen
			u8g2.setPowerSave(1);
		}
	}
	if (_BlueLedON)	{
		PrintValuesOnScreen(_delayTimeUpt - (now - _lastProcessMillis));
	}

	if (_ThePubSub.connected())	{
		_ThePubSub.loop(); //allow the pubsubclient to process incoming messages
	}

	delay(_delayTimeLoop);
}

bool CheckWifi()
{
	if (!WiFi.isConnected()) {
		log_d("Reconnecting to Wifi...");
		WiFi.reconnect();
	}
	if (WiFi.status() != WL_CONNECTED) {
		log_d("Couldn't get a wifi connection!");
		return false;
	}
	return true;
}

bool CheckMQTT()
{
	if (!_ThePubSub.connected()) {
		_ThePubSub.setClient(_TheWifi);
		_ThePubSub.setServer(ADAFRUIT_ADDR, ADAFRUIT_PORT);
		_ThePubSub.setCallback(PubSubCallback);
		if (!_ThePubSub.connect("PChanMQTT", ADAIO_USER, ADAIO_KEY)) {
			log_d("ERROR!! PubSubClient was not able to connect to AdafruitIO!!");
			return false;
		}
		else	{ //Subscribe to the on/off button and the slider
			log_d("PubSubClient connected to AdafruitIO!!");
			if (!_ThePubSub.subscribe((std::string(ADAIO_USER).append(FEED_TURN_ON_PWM)).c_str())) {
				log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", FEED_TURN_ON_PWM);
			}
			if (!_ThePubSub.subscribe((std::string(ADAIO_USER).append(FEED_INTENSITY_PWN)).c_str())) {
				log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", FEED_INTENSITY_PWN);
			}
		}
	}
	return true;
}

void PubSubCallback(char *pTopic, uint8_t *pData, unsigned int dataLenght)
{
	std::string theTopic(pTopic);
	std::string theMsg;

	for (uint16_t i = 0; i < dataLenght; i++)	{
		theMsg.push_back((char)pData[i]);
	}
	log_d("Received message from [%s]: [%s]", theTopic.c_str(), theMsg.c_str());

	if (theTopic.find(FEED_TURN_ON_PWM) != std::string::npos)	{
		if (theMsg == "ON")	{
			log_d("Turning on the screen!, intensity=%d", _BlueLedIntensity);
			_BlueLedON = true;
		}
		else {
			log_d("Turning off the screen...");
			_BlueLedON = false;
		}
	}
	else if (theTopic.find(FEED_INTENSITY_PWN) != std::string::npos) {
		auto intensity = std::atoi(theMsg.c_str());
		log_d("Changing screen intensity=%d", intensity);
		_BlueLedIntensity = intensity;
	}
}

void PrintValuesOnScreen(int millisPending)
{
	char buffer[128];
	char c = '-';
	int8_t fh = 10, h;
	int printTime = millis();

	switch (_lastClockChar)
	{
	case 0:
		c = '\\';
		break;
	case 1:
		c = '|';
		break;
	case 2:
		c = '/';
		break;
	case 3:
		c = '-';
		break;
	}
	_lastClockChar = (_lastClockChar + 1) % 4;

	if (_lastAvgUpdateTime == 0 && _numUpdates)
	{
		_lastAvgUpdateTime = _totalUpdateTime / _numUpdates;
	}

	u8g2.firstPage();
	do
	{
		//u8g2.clearBuffer();
		u8g2.setFont(u8g2_font_sirclivethebold_tr); //7 pixels
		snprintf(buffer, sizeof(buffer), "Valors Actuals");
		//		auto strwidth=u8g2.getStrWidth(buffer);
		//u8g2.setCursor((SCREEN_WIDTH-strwidth)/2, h);
		u8g2.setCursor(0, fh);
		u8g2.print(buffer);

		u8g2.setFont(u8g2_font_profont11_mf); 
		h = fh * 2.4;
		u8g2.setCursor(5, h);
		snprintf(buffer, sizeof(buffer), "%-11s:[%2.1fºC]", "Temperatura", _TheBME.GetLatestTemperature());
		u8g2.print(buffer);
		h += fh * 1.4;
		snprintf(buffer, sizeof(buffer), "%-11s:[%d%%]", "Humitat", (int)_TheBME.GetLatestHumidity());
		u8g2.setCursor(5, h);
		u8g2.print(buffer);
		h += fh * 1.4;
		snprintf(buffer, sizeof(buffer), "%-12s:[%dhPa]", "Pressió", (int)_TheBME.GetLatestPressure());
		u8g2.setCursor(5, h);
		u8g2.print(buffer);

		u8g2.setFont(u8g2_font_profont10_mf); //6 pixels monospace
		snprintf(buffer, sizeof(buffer), "%c %2ds %c     [%3dms]", c, millisPending / 1000, c, _lastAvgUpdateTime);
		u8g2.drawStr(15, 64, buffer);
	} while (u8g2.nextPage());
	//u8g2.sendBuffer();

	//log_d("Updated Screen in [%dms]...",  millis()-printTime);
	_numUpdates++;
	_totalUpdateTime += (millis() - printTime);
}
