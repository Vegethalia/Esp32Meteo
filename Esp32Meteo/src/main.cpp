
#include <Arduino.h>
#include <AdafruitIO_WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8g2lib.h>
#include <esp_wifi.h>
#include "mykeys.h"

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_LED_MIO 32
#define PIN_LED_PWM 25
#define PIN_BUTTON 4
#define I2C_ADDRESS_BME280 0x76
#define I2C_BUS_SPEED 100000 //100000

#define PWMCHANNEL 0
#define PWMRESOLUTION 8

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define FEED_TURN_ON_PWN "TurnOnLed"
#define FEED_INTENSITY_PWN "LedIntensity"

#define SEALEVELPRESSURE_HPA (1013.25)
#define PRESSURE_OFFSET 14 //looks like my sensor always returns the real pressure minus this offset

#define DEBOUNCE_TIME 250 // Filtre anti-rebond (debouncer)

//-----FORWARD DECLARATIONS
bool UpdateValues(); //returns true if values were successfully read from BME280 sensor
void onMessageOnOff(AdafruitIO_Data *data);
void FirstTimeProcess();
void PrintValuesOnScreen(int millis);
//void PrintRemainingTime(int millis);
//-----FORWARD DECLARATIONS

AdafruitIO_WiFi iowifi(ADAIO_USER, ADAIO_KEY, WIFI_SSID, WIFI_PASS);

// SetUp ADAFRUIT Feeds
AdafruitIO_Feed *adafruit_temp = iowifi.feed("Temperatura");
AdafruitIO_Feed *adafruit_hum = iowifi.feed("Humitat");
AdafruitIO_Feed *adafruit_press = iowifi.feed("Pressio");
//AdafruitIO_Feed *adafruit_alt = iowifi.feed("Altitut");
AdafruitIO_Feed *adafruitread_blueledon = iowifi.feed(FEED_TURN_ON_PWN);
AdafruitIO_Feed *adafruitread_blueledpower = iowifi.feed(FEED_INTENSITY_PWN);

//SetUp SH1106
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA); //R0 = no rotate
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);

//Our own TwoWire instance so we can configure wich pins use as I2C
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme; // BME280 I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

//VARIABLES
unsigned long _delayTimeUpt = 30000;
unsigned long _delayTimeLoop = 50;
unsigned long _lastProcessMillis = 0;

float _lastTempValue = 0.0f;
float _lastHumValue = 0.0f;
float _lastPressValue = 1000.0f;
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
	//  pinMode(PIN_MOTOR_PWM, OUTPUT);

	// wait for serial monitor to open
	while (!Serial)
		;

	//Configure PWM pin
	ledcSetup(PWMCHANNEL, 20000, PWMRESOLUTION); //channel0, freq=¿1000?, resolution=8bits
	ledcAttachPin(PIN_LED_PWM, PWMCHANNEL);
	//ledcAttachPin(PIN_MOTOR_PWM, PWMCHANNEL);

	//Detect BME
	Serial.println("Detecting BME280...");
	I2CBME.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_SPEED); //0=default 100000 100khz

	// default settings (you can also pass in a Wire library object like &Wire2)
	bool status = bme.begin(I2C_ADDRESS_BME280, &Wire); // &I2CBME);
	if (!status)
	{
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		delay(100);
		while (1)
			;
	}
	Serial.println("BME280 sensor found!!");
	Serial.println();

	log_d("Begin Display...");
	u8g2.setBusClock(I2C_BUS_SPEED);
	//	u8g2.beginSimple();// does not clear the display and does not wake up the display  user is responsible for calling clearDisplay() and setPowerSave(0)
	u8g2.begin();
	u8g2.enableUTF8Print();							 // enable UTF8 support for the Arduino print()
	u8g2.setContrast(_BlueLedIntensity); //set default contrast
																			 //	PrintValuesOnScreen(_delayTimeUpt); //Print 1st values

	// connect to io.adafruit.com
	Serial.print("Connecting to Adafruit IO");
	iowifi.connect();
	// wait for a connection
	while (iowifi.status() < AIO_CONNECTED)
	{
		Serial.print("(");
		Serial.print(iowifi.status());
		Serial.print(")");
		delay(500);
	}
	// we are connected
	Serial.println();
	Serial.print("iowifi Status=");
	Serial.println(iowifi.statusText());

	// set up a message handler for the 'sharedFeed' feed.
	// the handleMessage function (defined below)
	// will be called whenever a message is
	// received from adafruit io.
	if (adafruitread_blueledon)
	{
		adafruitread_blueledon->onMessage(onMessageOnOff);
	}
	if (adafruitread_blueledpower)
	{
		adafruitread_blueledpower->onMessage(onMessageOnOff);
	}

	attachInterrupt(PIN_BUTTON, ButtonPressed, RISING);
}

void loop()
{
	//iowifi.run(); is required for all sketches.
	//it should always be present at the top of your loop function.
	//it keeps the client connected to io.adafruit.com, and processes any incoming data.
	iowifi.run();

	auto now = millis();

	if ((now - _lastProcessMillis) >= _delayTimeUpt)
	{
		// log_d("Starting wifi: %d", esp_wifi_start());
		// Serial.print("Connecting to Adafruit IO");
		// iowifi.connect();
		// // wait for a connection
		// while(iowifi.status() < AIO_CONNECTED) {
		// 	Serial.print("("); Serial.print(iowifi.status()); Serial.print(")");
		// 	delay(500);
		// }
		// iowifi.run();
		_lastProcessMillis = now;

		digitalWrite(PIN_LED_MIO, HIGH); // turn on the LED
		delay(200);
		digitalWrite(PIN_LED_MIO, LOW); // turn off the LED

		UpdateValues();
		if (_numUpdates)
		{
			_lastAvgUpdateTime = _totalUpdateTime / _numUpdates;
			log_d("_totalUpdateTime=%d _numUpdates=%d _lastAvgUpdateTime=%d", _totalUpdateTime, _numUpdates, _lastAvgUpdateTime);
			_totalUpdateTime = _numUpdates = 0;
		}
		// log_d("Stopping wifi: %d", esp_wifi_stop());
	}
	if (_UpdateRequired)
	{
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
	if (_BlueLedON)
	{
		PrintValuesOnScreen(_delayTimeUpt - (now - _lastProcessMillis));
	}

	delay(_delayTimeLoop);
}

void onMessageOnOff(AdafruitIO_Data *data)
{
	if (!data)
	{
		Serial.println("NULL DATA!!");
		return;
	}
	char buff[1000];

	snprintf(buff, sizeof(buff), "Message received from feed [%s]:[%s]", data->feedName(), data->value());
	Serial.println(buff);

	if (strcmp(data->feedName(), FEED_TURN_ON_PWN) == 0)
	{
		if (strcmp(data->toChar(), "ON") == 0)
		{
			Serial.print("Encenent el blue led, intensitat=");
			Serial.println(_BlueLedIntensity);
			_BlueLedON = true;
			ledcWrite(PWMCHANNEL, _BlueLedIntensity);

			//turn on screen
			u8g2.setPowerSave(0);
			u8g2.setContrast(_BlueLedIntensity);
		}
		else
		{
			Serial.println("Apagant el blue led...");
			_BlueLedON = false;
			ledcWrite(PWMCHANNEL, 0);

			//turn off screen
			u8g2.setPowerSave(1);
		}
	}
	else if (strcmp(data->feedName(), FEED_INTENSITY_PWN) == 0)
	{
		snprintf(buff, sizeof(buff), "Rebuda intensitat de led=%d", data->toInt());
		Serial.println(buff);
		if (data->toInt() < 0)
		{
			_BlueLedIntensity = 0;
		}
		else if (data->toInt() > 255)
		{
			_BlueLedIntensity = 255;
		}
		else
		{
			_BlueLedIntensity = data->toInt();
		}
		u8g2.setContrast(_BlueLedIntensity);
		if (_BlueLedON)
		{
			ledcWrite(PWMCHANNEL, _BlueLedIntensity);
		}
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

		u8g2.setFont(u8g2_font_profont11_mf); //7 pixels monospace
		h = fh * 2.4;
		u8g2.setCursor(5, h);
		snprintf(buffer, sizeof(buffer), "%-11s:[%2.1fºC]", "Temperatura", _lastTempValue);
		u8g2.print(buffer);
		h += fh * 1.4;
		snprintf(buffer, sizeof(buffer), "%-11s:[%d%%]", "Humitat", (int)_lastHumValue);
		u8g2.setCursor(5, h);
		u8g2.print(buffer);
		h += fh * 1.4;
		snprintf(buffer, sizeof(buffer), "%-12s:[%dhPa]", "Pressió", (int)_lastPressValue);
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

bool UpdateValues()
{
	char buffer[100];
	float temp = bme.readTemperature();
	float hum = bme.readHumidity();
	float press = bme.readPressure() / 100.0;
	bool err = false;
	//	float alt=bme.readAltitude(SEALEVELPRESSURE_HPA);

	if (!isnan(temp) && temp > (-20) && temp < 100)
	{
		_lastTempValue = temp;
		if (!adafruit_temp->save(_lastTempValue))
		{
			Serial.println("Error updating temperature :(");
		}
		sprintf(buffer, "Temperature = %2.1fºC", _lastTempValue);
		Serial.println(buffer);
	}
	else
	{
		Serial.println("Error reading temperature :(");
		err = true;
	}

	if (!isnan(hum) && hum >= 0 && hum <= 100)
	{
		_lastHumValue = hum;
		if (!adafruit_hum->save(_lastHumValue))
		{
			Serial.println("Error updating humnidity :(");
		}
		sprintf(buffer, "Humidity = %2.1f%%", _lastHumValue);
		Serial.println(buffer);
	}
	else
	{
		Serial.println("Error reading humidity :(");
		err = true;
	}

	if (!isnan(press) && press > 900 && press < 1100)
	{
		_lastPressValue = press + PRESSURE_OFFSET;
		if (!adafruit_press->save(_lastPressValue))
		{
			Serial.println("Error updating pressure :(");
		}
		sprintf(buffer, "Pressure = %3.1fhpa", _lastPressValue);
		Serial.println(buffer);
	}
	else
	{
		Serial.println("Error reading pressure :(");
		err = true;
	}

	Serial.println();

	return !err;
}
