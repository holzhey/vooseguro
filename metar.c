/*

 METAR (VOO SEGURO)
 by Alexandre Lehmann Holzhey
 
 */

#include <SoftwareSerial.h>                         // Software serial library
#include <BMP085.h>                                 // Pressure and temperature sensor library
#include <DHT.h>                                    // Humidity sensor library
#include <Wire.h>                                   // I2C communication library
#include <RTClib.h>                                 // Real Time Clock library
#include <LiquidCrystal_I2C.h>                      // Liquid Crystal (LCD) I2C library
#include <Metro.h>                                  // Metro timing library
#include <EEPROM.h>                                 // EEPROM I/O library
#include <Math.h>                                   // Math functions library
#include "Timer.h"                                  // Timer function library

#define DHTPIN 7                                    // Humidity sensor digital in
#define DHTTYPE DHT22                               // Humidity sensor type
#define BH1750_Device 0x23                          // I2C address for pressure and temperature sensor
#define BUTTON_MODE_PIN 5                           // Mode button digital in
#define BUTTON_ACTION_PIN 4                         // Action button digital in
#define ANEMOMETER_ACTION_LED 13                    // Anemometer interrupt action led digital out       
#define WIND_DIRECTION_ANALOG_PIN 0                 // Analog input port for wind direction reading
#define GPRS_MODULE_POWER_PIN 10                    // GPRS module power on digital out
#define GPRS_MODULE_RX_PIN 9                        // GPRS module RX digital out
#define GPRS_MODULE_TX_PIN 8                        // GPRS module TX digital out
#define I2C_LCD_ADDRESS 0x27                        // I2C LCD address
#define TIMER_SENSORS 10000                         // Sensors read interval
#define TIMER_LUX 1000                              // Luminosity read interval
#define LUX_FEW 5                                   // Covering seconds for FEW cloud status
#define LUX_SCT 10                                  // Covering seconds for SCT cloud status
#define LUX_BKN 20                                  // Covering seconds for BKN cloud status
#define LUX_OVC 40                                  // Covering seconds for OVC cloud status
#define TIMER_COVER 60000                           // Clouds cover calculation interval
#define TIMER_METAR 3000                            // Metar rendering interval
#define TIMER_PUBLISH 30000                         // Server publish interval
#define TIMER_ANEMOMETER 6000                       // Instant wind calculation interval
#define TIMER_WINDS 60000                           // Wind gust and average reset interval
#define DELAY_ANEMOMETER_LOW 50                     // Anemometer interrupt on low delay time
#define DELAY_ANEMOMETER_HIGH 20                    // Anemometer interrupt on high delay time
#define STS_OK 0                                    // Status OK for readings
#define STS_ERR_SENSOR_HUMIDITY 1                   // Status ERROR for humidity reading
#define MEMORY_ICAO 20                              // EEPROM address for ICAO storage
#define MEMORY_LAT 24                               // EEPROM address for latitude storage
#define MEMORY_LNG 26                               // EEPROM address for longitude storage
#define STR_METAR_SIZE 500                          // Size for METAR/TEMP char array
#define MAX_STRING 256                              // Maximum string from PROGMEM size

char stringBuffer[MAX_STRING];                      // Buffer to get string from flash
char stringTmp1[12];                                // Alternative buffer for comparators
char stringTmp2[12];                                // Alternative buffer for comparators

// Get strings from flash instead of SRAM
char* getString(const char* str) {
  strcpy_P(stringBuffer, (char*)str);
  return stringBuffer;
}

// Strings constants

const char strAT[] PROGMEM = "AT";
const char strOK[] PROGMEM = "OK";
const char strVersion[] PROGMEM = "Voo Seguro 1.0";
const char strInit[] PROGMEM = "Inicializando...";
const char strAtReg1[] PROGMEM = "AT+CREG?";
const char strAtReg2[] PROGMEM = "+CREG: 0,1";
const char strAtReg3[] PROGMEM = "+CREG: 0,5";
const char strAtCsq[] PROGMEM = "AT+CSQ";
const char strAtCgatt[] PROGMEM = "AT+CGATT=1";
const char strAtGprsDetach[] PROGMEM = "AT+CGATT=0";
const char strAtSapbrGprs[] PROGMEM = "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"";
const char strAtSapbrApn[] PROGMEM = "AT+SAPBR=3,1,\"APN\",\"VIVO\"";
const char strAtSapbrOpen[] PROGMEM = "AT+SAPBR=1,1";
const char strAtSapbrClose[] PROGMEM = "AT+SAPBR=0,1";
const char strAtHttpInit[] PROGMEM = "AT+HTTPINIT";
const char strAtHttpAction[] PROGMEM = "AT+HTTPACTION=0";
const char strAtHttpRead[] PROGMEM = "AT+HTTPREAD";
const char strAtHttpTerminate[] PROGMEM = "AT+HTTPTERM";
const char strUrlBase[] PROGMEM = "AT+HTTPPARA=\"URL\",\"http://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?action=updateraw&ID=IRIOGRAN45&PASSWORD=metar12345";
const char strUrlBool[] PROGMEM = "AT+HTTPPARA=\"URL\",\"http://bool.eti.br/metar.php?action=debug";
const char strOffline[] PROGMEM = " (Offline)";
const char strSending[] PROGMEM = "(Enviando)";
const char strFailure[] PROGMEM = "( Falhou )";
const char strTypeCustom[] PROGMEM = "&softwaretype=custom";
const char strTypeDateUtc[] PROGMEM = "&dateutc=";
const char strTypeHumidity[] PROGMEM = "&humidity=";
const char strTypeDewpoint[] PROGMEM = "&dewptf=";
const char strTypeTemperature[] PROGMEM = "&tempf=";
const char strTypePressure[] PROGMEM = "&baromin=";
const char strTypeWindSpeed[] PROGMEM = "&windspeedmph=";
const char strTypeWindGust[] PROGMEM = "&windgustmph=";
const char strTypeClouds[] PROGMEM = "&clouds=";
const char strTypeWindDir[] PROGMEM = "&winddir=";
const char strTypeRainIn[] PROGMEM = "&rainin=";
const char strTypeDailyRainIn[] PROGMEM = "&dailyrainin=";
const char strTypeWindSpdMphAvg2m[] PROGMEM = "&windspdmph_avg2m=";
const char strTypeWindDirAvg2m[] PROGMEM = "&winddir_avg2m=";
const char strTypeWindGustMph10m[] PROGMEM = "&windgustmph_10m=";
const char strMsgConfig1[] PROGMEM = "Config 1/7 : Dia";
const char strMsgConfig2[] PROGMEM = "Config 2/7 : Mes";
const char strMsgConfig3[] PROGMEM = "Config 3/7 : Ano";
const char strMsgConfig4[] PROGMEM = "Config 4/7 : Hora";
const char strMsgConfig5[] PROGMEM = "Config 5/7 : Minuto";
const char strMsgConfig6[] PROGMEM = "Config 6/7 : ICAO";
const char strMsgConfig7[] PROGMEM = "Config 7/7 : GPS";
const char strMETAR[] PROGMEM = "METAR ";
const char strAUTO[] PROGMEM = " AUTO ";
const char strWINDCALM[] PROGMEM = "00000KT ";
const char strREMARK[] PROGMEM = " RMK A01";
const char strLat[] PROGMEM = "Lat ";
const char strLng[] PROGMEM = "Lng ";
const char strDoubleArrows[] PROGMEM = "vv";
const char strQuadrupleArrows[] PROGMEM = "vvvv";
const char strGPSArrows[] PROGMEM = "    vvv";
const char strFEW[] PROGMEM = "FEW";
const char strSCT[] PROGMEM = "SCT";
const char strBKN[] PROGMEM = "BKN";
const char strOVC[] PROGMEM = "OVC";
const char strSKC[] PROGMEM = "SKC";

boolean buttonAction = false;                       // Flag used for action button control
boolean buttonMode = false;                         // Flag used for mode button control

RTC_DS1307 rtc;                                     // Initialize RTC clock library
DHT dht(DHTPIN, DHTTYPE);                           // Initialize humidity sensor library
BMP085 dps = BMP085();                              // Initialize pressure and temperature sensor library
LiquidCrystal_I2C lcd(I2C_LCD_ADDRESS, 20, 4);      // Initialize I2C LCD library
SoftwareSerial gsmSerial(GPRS_MODULE_RX_PIN, GPRS_MODULE_TX_PIN); // Initialize GPRS software gsmSerial

// Metro handlers used for tasks timing

Metro sensorsMetro = Metro(TIMER_SENSORS);          // Read sensors task
Metro metarMetro = Metro(TIMER_METAR);              // Render METAR task
Metro publishMetro = Metro(TIMER_PUBLISH);          // Publish data task
Metro anemometerMetro = Metro(TIMER_ANEMOMETER);    // Calculate instant wind task
Metro windsMetro = Metro(TIMER_WINDS);              // Calculate winds (speed & gust) task
Metro luxMetro = Metro(TIMER_LUX);                  // Read luminosity and handle history task
Metro coverMetro = Metro(TIMER_COVER);              // Estimate cloud cover task

// Timer handlers used for wind direction reading
Timer winddirTimer;                                 // Read wind direction reading timer interrupt handler

// Global vars

char metar[STR_METAR_SIZE] = "";                    // String buffer, used with METAR and URL
boolean gsmOk;                                      // Flag for GSM modem status (power up)
float humidity = 0;                                 // Humidity in percent
long baro = 0;                                      // Barometric pressure in hPa
long temp = 0;                                      // Temperature in Celsius
unsigned int windDirection = 0;                     // Wind direction in degrees
long dewpoint = 0;                                  // Dewpoint in Celsius
unsigned int windSpeed = 0;                         // Wind speed in knots
unsigned int windAvg = 0;                           // Wind speed average in knots
boolean windAvgFirst = true;                        // Wind speed first average flag
unsigned int windGust = 0;                          // Wind speed gust in knots
unsigned int anemometerRevolutions= 0;              // Anemometer revolutions
unsigned long anemometerLastRead = 0;               // Last anemometer read millis
unsigned long anemometerInterruptLast = 0;          // Last anemometer interrupt millis
int anemometerState = LOW;                          // Anemometer pulse state
unsigned int lux = 0;                               // Luminosity in Lux
unsigned int estimatedLux = 0;                      // Estimated luminosity in Lux
unsigned long lastCoverDetect = 0;                  // Lastest cloudy event detected
boolean coverStatus = false;                        // Cloud event status
byte cloudState = 0;                                // Cloud cover state

unsigned int lastLux;
unsigned long cloudIniTime;
unsigned int cloudIniLux;
byte cover = 0;                                     // Cloud cover in percent 
int status = STS_OK;                                // Last reading status
char icao[5] = "SBXX";                              // ICAO code
float latitude = 0;                                 // Latitude in decimals
float longitude = 0;                                // Longitude in decimals
int gps_deg = 0;                                    // Decimal to degree temp vars
int gps_min = 0;                                    
int gps_sec = 0;

void setup() {

  // Serial init
  // (debug purposes, to be removed - more memory without Software Serial!!)
  Serial.begin(19200);

  // LCD I2C init
  lcd.begin(20, 4);
  lcd.init();
  lcd.clear();
  lcd.print(getString(strVersion));
  lcd.setCursor(0, 1);
  lcd.print(getString(strInit));
  delay(3000);

  // GSM module init
  pinMode(GPRS_MODULE_POWER_PIN, OUTPUT);
  gsmSerial.begin(19200);
  power_on();

  // Input/Ouput setup
  pinMode(BUTTON_MODE_PIN, INPUT);
  pinMode(BUTTON_ACTION_PIN, INPUT);
  pinMode(ANEMOMETER_ACTION_LED, OUTPUT);

  // I2C init
  Wire.begin();

  // Lux sensor init
  Configure_BH1750();

  // RTC init and first setup
  rtc.begin();
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(__DATE__, __TIME__));
  } 

  // Read ICAO from EEPROM
  for (int c = 0; c < 4; c++) {
    char ic = EEPROM.read(MEMORY_ICAO + c);
    if (ic >= 65 and ic <= 90) {
      icao[c] = ic;
    }
  }

  // Read GPS position from EEPROM
  for (int type = 0; type < 2; type++) {
    int address = MEMORY_LAT;
    float limit = 90;
    if (type == 1) {
      address = MEMORY_LNG;
      limit = 180;
    }
    float coord = doTransformDegree2Decimal(EEPROM.read(address++), EEPROM.read(address++), EEPROM.read(address));
    if ((coord < -limit) || (coord > limit)) {
      coord = 0;
    }
    if (type == 0) {
      latitude = coord;
    } 
    else {
      longitude = coord;
    }                
  }

  // Humidity sensor init
  dht.begin();

  // Pressure and temperature sensor init
  dps.init();

  // Anemometer setup
  attachInterrupt(0, anemometerInterrupt, CHANGE);
  anemometerLastRead = millis();
  anemometerInterruptLast = millis();
  anemometerRevolutions = 0;  
  windSpeed = 0;
  windAvg = 0;
  windAvgFirst = true;
  windGust = 0;
  
  // Wind direction setup
  pinMode(WIND_DIRECTION_ANALOG_PIN, INPUT);
  windDirection = 0;
  //winddirTimer.every(250, windDirRead);

  // LCD test pattern
  lcd.clear();
  for (byte x = 0; x < 80; x++) {
    lcd.print('#');
  }
  delay(2000);
  lcd.clear();
}

void windDirRead() {
  unsigned int ref = analogRead(WIND_DIRECTION_ANALOG_PIN);
  if (ref > 20) {
    if (ref < 100) {
      windDirection = 0;
    } else {
      if (ref < 300) {
        windDirection = 90;
      } else {
        if (ref < 600) {
          windDirection = 180;
        } else {
          if (ref < 900) {
            windDirection = 270;
          }
        }
      }
    }
  }
}

unsigned int getSpeedFromRevolutions(unsigned int revolutions, unsigned int elapsedms) {
  float rpm = (revolutions * 60 ) / (elapsedms /1000);
  return (unsigned int) (rpm * 0.03);
}

int8_t sendATcommand(char* ATcommand, char* expected_answer1, unsigned int timeout) {
  uint8_t x=0,  answer=0;
  char response[100];
  unsigned long previous;
  memset(response, '\0', 100);
  delay(100);
  while( gsmSerial.available() > 0)  gsmSerial.read();
  gsmSerial.println(ATcommand);
  x = 0;
  previous = millis();
  do{
    if( gsmSerial.available() != 0){    
      response[x] =  gsmSerial.read();
      x++;
      if (strstr(response, expected_answer1) != NULL) {
        answer = 1;
      }
    }
  } 
  while((answer == 0) && ((millis() - previous) < timeout));  
  return answer;
}

int8_t sendATcommand2(char* ATcommand, char* expected_answer1, char* expected_answer2, unsigned int timeout) {
  uint8_t x=0,  answer=0;
  char response[100];
  unsigned long previous;
  memset(response, '\0', 100);
  delay(100);
  while(  gsmSerial.available() > 0)  gsmSerial.read();
  gsmSerial.println(ATcommand);
  x = 0;
  previous = millis();
  do{        
    if( gsmSerial.available() != 0){    
      response[x] =  gsmSerial.read();
      x++;
      if (strstr(response, expected_answer1) != NULL) {
        answer = 1;
      }
      if (strstr(response, expected_answer2) != NULL) {
        answer = 2;
      }
    }
  } 
  while((answer == 0) && ((millis() - previous) < timeout));    
  return answer;
}

void power_on(){
  uint8_t answer=0;
  byte attemps = 0;
  strcpy(stringTmp1, getString(strOK));
  answer = sendATcommand(getString(strAT), stringTmp1, 2000);
  while (answer == 0)
  {
    digitalWrite(GPRS_MODULE_POWER_PIN,HIGH);
    delay(3000);
    digitalWrite(GPRS_MODULE_POWER_PIN,LOW);
    byte ct = 0;
    while(answer == 0){  
      answer = sendATcommand(getString(strAT), stringTmp1, 2000);
      ct++;
      if (ct > 5) break;
    }
    if (answer == 0) {
      attemps++;
      if (attemps > 3) {
        break;
      }
    }
  }
  if (answer == 0) {
    gsmOk = false;
  } else {
    gsmOk = true;
  }
}

unsigned int getHumidity() {
  float humidity = dht.readHumidity();
  if (isnan(humidity)) {
    //status = STS_ERR_SENSOR_HUMIDITY;
    humidity = 0;
  }
  return humidity;
}

unsigned int getBaro() {
  long baro = 0;
  dps.getPressure(&baro);
  unsigned int baro2 = baro / 100;
  return baro2;
}

unsigned int BH1750_Read() {
  unsigned int i=0;
  Wire.beginTransmission(BH1750_Device);
  Wire.requestFrom(BH1750_Device, 2);
  while(Wire.available())
  {
    i <<=8;
    i|= Wire.read();
  }
  Wire.endTransmission();
  return i/1.2;
}

void Configure_BH1750() {
  Wire.beginTransmission(BH1750_Device);
  Wire.write(0x10); // Set resolution to 1 Lux
  Wire.endTransmission();
}

unsigned int getTemp() {
  long temp = 0;
  dps.getTemperature(&temp);
  unsigned int temp2 = temp / 10;
  return temp2;
}

unsigned int getDewPoint(long temperature, float humidity) {
  return (temperature - ((100 - humidity) / 5));
}

char * floatToString(char * outstr, double val, byte precision, byte widthp){
  char temp[16]; //increase this if you need more digits than 15
  byte i;

  temp[0]='\0';
  outstr[0]='\0';

  if(val < 0.0){
    strcpy(outstr,"-\0");  //print "-" sign
    val *= -1;
  }

  if( precision == 0) {
    strcat(outstr, ltoa(round(val),temp,10));  //prints the int part
  }
  else {
    unsigned long frac, mult = 1;
    byte padding = precision-1;
    
    while (precision--)
      mult *= 10;

    val += 0.5/(float)mult;      // compute rounding factor
    
    strcat(outstr, ltoa(floor(val),temp,10));  //prints the integer part without rounding
    strcat(outstr, ".\0"); // print the decimal point

    frac = (val - floor(val)) * mult;

    unsigned long frac1 = frac;

    while(frac1 /= 10) 
      padding--;

    while(padding--) 
      strcat(outstr,"0\0");    // print padding zeros

    strcat(outstr,ltoa(frac,temp,10));  // print fraction part
  }

  // generate width space padding 
  if ((widthp != 0)&&(widthp >= strlen(outstr))){
    byte J=0;
    J = widthp - strlen(outstr);

    for (i=0; i< J; i++) {
      temp[i] = ' ';
    }

    temp[i++] = '\0';
    strcat(temp,outstr);
    strcpy(outstr,temp);
  }

  return outstr;
}

float c2f(long tempCelsius) {
  return (1.8 * tempCelsius) + 32;
}

float kt2mph(long speedKt) {
  return ((speedKt * 6076.0) * (1.0 / 5280.0));
}

void renderMetarRemote(char* url) {
  // Build URL to call

    memset(metar, '\0', STR_METAR_SIZE);
    strcat(metar, url);

    // Append date UTC

    DateTime now = rtc.now();
    char buf[11];

    strcat(metar, getString(strTypeDateUtc));
    strcat(metar, zeroPad(utoa(now.year(), buf, 10), 4));
    strcat(metar, "-");
    strcat(metar, zeroPad(utoa(now.month(), buf, 10), 2));
    strcat(metar, "-");
    strcat(metar, zeroPad(utoa(now.day(), buf, 10), 2));
    strcat(metar, "%20");
    strcat(metar, zeroPad(utoa(((now.hour() + 3) % 24), buf, 10), 2));
    strcat(metar, ":");
    strcat(metar, zeroPad(utoa(now.minute(), buf, 10), 2));
    strcat(metar, ":");
    strcat(metar, zeroPad(utoa(now.second(), buf, 10), 2));

    // Append humidity

    unsigned int tmp = humidity;
    strcat(metar, getString(strTypeHumidity));
    strcat(metar, utoa(tmp, buf, 10));

    // Append dewpoint

    strcat(metar, getString(strTypeDewpoint));
    strcat(metar, floatToString(buf, c2f(dewpoint), 4, 0));

    // Append temperature (in fahrenheit)

    strcat(metar, getString(strTypeTemperature));
    strcat(metar, floatToString(buf, c2f(temp), 4, 0));

    // Append baro pressure (in inHg)
    strcat(metar, getString(strTypePressure));
    strcat(metar, floatToString(buf, (baro * 0.02953600), 8, 0)); // 0.02953500

    // Append software type

    strcat(metar, getString(strTypeCustom));
    
    // Append empty rain data
    
    strcat(metar, getString(strTypeRainIn));
    strcat(metar, utoa(0, buf, 10));
    strcat(metar, getString(strTypeDailyRainIn));
    strcat(metar, utoa(0, buf, 10));

    // Append wind direction (360 degree angle)

    strcat(metar, getString(strTypeWindDir));
    strcat(metar, utoa(windDirection, buf, 10));
    strcat(metar, getString(strTypeWindDirAvg2m));
    strcat(metar, utoa(windDirection, buf, 10));

    // Append wind speed (in mph)

    strcat(metar, getString(strTypeWindSpeed));
    strcat(metar, floatToString(buf, kt2mph(windAvg), 6, 0));
    strcat(metar, getString(strTypeWindSpdMphAvg2m));
    strcat(metar, floatToString(buf, kt2mph(windAvg), 6, 0));

    // Append wind direction average (TODO)

    // Append wind gust (in mph)

    strcat(metar, getString(strTypeWindGust));
    strcat(metar, floatToString(buf, kt2mph(windGust), 6, 0));
    strcat(metar, getString(strTypeWindGustMph10m));
    strcat(metar, floatToString(buf, kt2mph(windGust), 6, 0));
    
    // Append cloud data
    
    strcat(metar, getString(strTypeClouds));
    switch (cloudState) {
      case 0:
        strcat(metar, getString(strSKC));
        break;
      case 1:
        strcat(metar, getString(strFEW));
        break;
      case 2:
        strcat(metar, getString(strSCT));
        break;
      case 3:
        strcat(metar, getString(strBKN));
        break;
      case 4:
        strcat(metar, getString(strOVC));
        break;
    }   

    // Finish URL and set it

    strcat(metar, "\"");
}

void gsmHttp() {
  
  // Read network registration status
  // CREG returns <N>,<STAT>
  // <N> is code presentation, always 0
  // <STAT> is registration status (1 is registered no roaming and 5 is registered with roaming)

  strcpy(stringTmp1, getString(strAtReg2));
  strcpy(stringTmp2, getString(strAtReg3));

  if (sendATcommand2(getString(strAtReg1), stringTmp1, stringTmp2, 2000) == 0 || (!gsmOk)) {
    lcd.setCursor(10, 3);
    lcd.print(getString(strOffline));
  } 
  else {
    byte gsmStatus = 0;
    lcd.setCursor(10, 3);
    lcd.print(getString(strSending));
    
    strcpy(stringTmp1, getString(strOK));

    // Get signal quality report
    // CSQ returns ERROR when no signal from GSM

    sendATcommand(getString(strAtCsq), stringTmp1, 5000);

    // Attach from GPRS device
    // Will return ERROR if GPRS module is fault

    sendATcommand(getString(strAtCgatt), stringTmp1, 5000);

    // Set bearer settings for applications based on IP
    // 3,1 is "set bearer parameters" for profile 1
    // The parameter is "CONTYPE" and the value is "GPRS"
    // Sim 900 supports "CSD" and "GPRS" for "CONTYPE"

    sendATcommand(getString(strAtSapbrGprs), stringTmp1, 5000);

    // Set another bearer setting
    // "APN" is "VIVO"

    sendATcommand(getString(strAtSapbrApn), stringTmp1, 5000);

    // Open bearer for profile 1 (1 = open, 1 = profile)

    sendATcommand(getString(strAtSapbrOpen), stringTmp1, 5000);

    // Initialize HTTP service

    sendATcommand(getString(strAtHttpInit), stringTmp1, 5000);

//    // Build METAR DATA BOOL
//
//    renderMetarRemote(getString(strUrlBool));
//    sendATcommand(metar, stringTmp1, 5000);
//
//    // Do HTTP action (0 = GET, 1 = POST and 2 = HEAD)
//
//    gsmStatus = sendATcommand(getString(strAtHttpAction), stringTmp1, 15000);
    
    // Build METAR DATA WU

    renderMetarRemote(getString(strUrlBase));
    sendATcommand(metar, stringTmp1, 5000);

    // Do HTTP action (0 = GET, 1 = POST and 2 = HEAD)

    gsmStatus = sendATcommand(getString(strAtHttpAction), stringTmp1, 15000);

    // Read response data
//
//    gsmSerial.println(getString(strAtHttpRead));
//    delay(2000);
//    while(gsmSerial.available() > 0) Serial.write(gsmSerial.read());
//
    // Terminate HTTP service

    sendATcommand(getString(strAtHttpTerminate), stringTmp1, 5000);

    // Close bearer for profile 1 (0 = close, 1 = profile)

    sendATcommand(getString(strAtSapbrClose), stringTmp1, 5000);

    // Detach GPRS device

    sendATcommand(getString(strAtGprsDetach), stringTmp1, 5000);

    if (gsmStatus == 0) {
      lcd.setCursor(10, 3);
      lcd.print(getString(strFailure)); 
    } else {
      lcd.setCursor(10, 3);
      for (byte x = 0; x < 10; x++) lcd.print(' ');
    }
  }
}

void anemometerInterrupt() {
  if (anemometerInterruptLast < millis()) {
    if (anemometerState != digitalRead(2)) {
      anemometerState = digitalRead(2);
      digitalWrite(ANEMOMETER_ACTION_LED, anemometerState);
      if (anemometerState == HIGH) {
        anemometerRevolutions++;
        anemometerInterruptLast = millis() + DELAY_ANEMOMETER_HIGH;
      } 
      else {
        anemometerInterruptLast = millis() + DELAY_ANEMOMETER_LOW;
      }
    }
  }
}

void imprimeMetar() {
  for (int pos = 0; pos <= 40; pos = pos + 20) {
    if (pos == 0) {
      lcd.setCursor(0, 0);
    } 
    if (pos == 20) {
      lcd.setCursor(0, 1);
    }
    if (pos == 40) {
      lcd.setCursor(0, 2);
    }
    if (pos == 60) {
      lcd.setCursor(0, 3);
    }
    if (pos < strlen(metar)) {
      for (int x = pos; x < (pos + 20); x++) {
        if (metar[x] == 0) {
          break;
        }
        lcd.print(metar[x]);
      }
      if ((pos + 20) > strlen(metar)) {
        byte diff = (pos + 20) - strlen(metar);
        for (int x = 0; x < diff; x++) {
          lcd.print(' ');
        }
      } 
    } 
    else {
      for (byte x = 0; x < 20; x++) {
        lcd.print(' ');
      }
    }
  }
}

char* zeroPad(char* subject, byte sz) {
  char tmp[8] = "";
  for (byte x = 0; x < (sz - strlen(subject)); x++) {
    strcat(tmp, "0");
  }
  strcat(tmp, subject);
  strcpy(subject, tmp);
  return subject;
}

void renderMetar() {
  memset(metar, '\0', STR_METAR_SIZE);
  strcat(metar, getString(strMETAR));
  strcat(metar, icao);
  strcat(metar, getString(strAUTO));
  DateTime now = rtc.now();
  char buf[8];
  strcat(metar, zeroPad(utoa(now.day(), buf, 10), 2));
  strcat(metar, zeroPad(utoa(now.hour(), buf, 10), 2));
  strcat(metar, zeroPad(utoa(now.minute(), buf, 10), 2));
  strcat(metar, " ");
  if (windAvg < 1) {
    strcat(metar, getString(strWINDCALM));
  } 
  else {
    strcat(metar, zeroPad(utoa(windDirection, buf, 10), 3));
    if (windAvg < 100) {
      strcat(metar, zeroPad(utoa(windAvg, buf, 10), 2));
    } 
    else {
      strcat(metar, zeroPad(utoa(windAvg, buf, 10), 3));
    }
    if ((windGust - windAvg) > 2) {
      strcat(metar, "G");
      if (windGust < 100) {
        strcat(metar, zeroPad(utoa(windGust, buf, 10), 2));
      } 
      else {
        strcat(metar, zeroPad(utoa(windGust, buf, 10), 3));
      }
    }
    strcat(metar, "KT ");
  }
  strcat(metar, utoa(temp, buf, 10));
  strcat(metar, "/");
  strcat(metar, utoa(dewpoint, buf, 10));
  switch (cloudState) {
    case 0:
      break;
    case 1:
      strcat(metar, " ");
      strcat(metar, getString(strFEW));
      break;
    case 2:
      strcat(metar, " ");
      strcat(metar, getString(strSCT));
      break;
    case 3:
      strcat(metar, " ");
      strcat(metar, getString(strBKN));
      break;
    case 4:
      strcat(metar, " ");
      strcat(metar, getString(strOVC));
      break;
  }   
  strcat(metar, " Q");
  strcat(metar, utoa(baro, buf, 10));
  strcat(metar, getString(strREMARK));
}

boolean buttonPressed(boolean &state, int pin) {
  if (!state) {
    if (digitalRead(pin) == HIGH) {
      state = true;
      return true;
    }
  } 
  else {
    if (digitalRead(pin) == LOW) {
      state = false;
    }
  }
  return false;
}

void doDayIncrement() {
  DateTime now = rtc.now();
  int tmp = now.day() + 1;
  if (tmp > 31) tmp = 1;
  rtc.adjust(DateTime(now.year(), now.month(), tmp, now.hour(), now.minute(), now.second()));
}

void doMonthIncrement() {
  DateTime now = rtc.now();
  int tmp = now.month() + 1;
  if (tmp > 12) tmp = 1;
  rtc.adjust(DateTime(now.year(), tmp, now.day(), now.hour(), now.minute(), now.second()));
}

void doYearIncrement() {
  DateTime now = rtc.now();
  int tmp = now.year() + 1;
  if (tmp > 2030) tmp = 2010;
  rtc.adjust(DateTime(tmp, now.month(), now.day(), now.hour(), now.minute(), now.second()));
}

void doHourIncrement() {
  DateTime now = rtc.now();
  rtc.adjust(DateTime(now.year(), now.month(), now.day(), ((now.hour() + 1) % 24), now.minute(), now.second()));
}

void doMinuteIncrement() {
  DateTime now = rtc.now();
  rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), ((now.minute() + 1) % 60), now.second()));
}

void doPrintTimestamp() {
  DateTime now = rtc.now();
  char buf[8];
  lcd.setCursor(2,2);
  lcd.print(zeroPad(utoa(now.day(), buf, 10), 2));
  lcd.print('/');
  lcd.print(zeroPad(utoa(now.month(), buf, 10), 2));
  lcd.print('/');
  lcd.print(zeroPad(utoa(now.year(), buf, 10), 4));
  lcd.print(' ');
  lcd.print(zeroPad(utoa(now.hour(), buf, 10), 2));
  lcd.print(':');
  lcd.print(zeroPad(utoa(now.minute(), buf, 10), 2));
}

char* doTransformDecimal2Degree(char* result, float coord, int &c_deg, int &c_min, int &c_sec) {
  c_deg = round(trunc(coord));
  coord = abs((coord - trunc(coord))) * 60;
  c_min = round(trunc(coord));
  coord = (coord - trunc(coord));
  c_sec = round(trunc(coord * 60));
  memset(result, '\0', 15);
  if (c_deg < 0) {
    strcat(result, "-");
  } 
  else {
    strcat(result, "+"); 
  }
  char buf[10];
  strcat(result, zeroPad(itoa(abs(c_deg), buf, 10), 3));
  strcat(result, " ");
  strcat(result, zeroPad(itoa(c_min, buf, 10), 2));
  strcat(result, "'");
  //  strcat(result, zeroPad(itoa(c_sec, buf, 10), 2));
  strcat(result, "00");
  strcat(result, "\"");
  return result;
}

float doTransformDegree2Decimal(int c_deg, int c_min, int c_sec) {
  int signal = 1;
  if (c_deg < 0) {
    signal = -1; 
  }
  float result = (1.0 * c_deg + ((signal * c_min) / 60.0) + ((signal * c_sec) / 3600.0));
  return result;
}

void loop() {

  // Sensors read task
  if (sensorsMetro.check()) {
    humidity = getHumidity();
    baro = getBaro();
    temp = getTemp();
    dewpoint = getDewPoint(temp, humidity);
    sensorsMetro.reset();
  }

  // Luminosity read and samples handling task
  if (luxMetro.check()) {
    lux = BH1750_Read();
    
    // New cover algo
    if (estimatedLux == 0) {
      estimatedLux = lux - 10;
    }
    if (!coverStatus) {
      if (lux < estimatedLux) {
        lastCoverDetect = millis();
        coverStatus = true;
      }
    } else {
      unsigned long coverTime = (millis() - lastCoverDetect) / 1000;
      if (coverTime < LUX_FEW) {
        cloudState = 0;
      } else {
        if (coverTime < LUX_SCT) {
          cloudState = 1;
        } else {
          if (coverTime < LUX_BKN) {
            cloudState = 2;
          } else {
            if (coverTime < LUX_OVC) {
              cloudState = 3;
            } else {
              cloudState = 4;
            }
          }
        }
      }
      if (lux > estimatedLux) {
        coverStatus = false;
      }
    }
  }

  // Clouds cover delay reset
  if (coverMetro.check()) {
    if (lux > estimatedLux) {
        cloudState = 0;
    }
    coverMetro.reset();
  }

  // Winds calculation task
  if (anemometerMetro.check()) {
    noInterrupts();
    windSpeed = getSpeedFromRevolutions(anemometerRevolutions, millis() - anemometerLastRead);
    anemometerRevolutions = 0;
    anemometerLastRead = millis();
    interrupts();
    if (windAvgFirst) {
      windAvg = windSpeed;
      windAvgFirst = false;
    } 
    else {
      windAvg = (unsigned int) ((windAvg + windSpeed) / 2);
    }
    if (windSpeed > windGust) {
      windGust = windSpeed;
    }
    anemometerMetro.reset();
  }

  // Winds reset cycle task
  if (windsMetro.check()) {
    windAvg = 0;
    windGust = 0;
    windAvgFirst = true;
    windsMetro.reset();
  }

  // Setup procedures
  if (buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
    lcd.clear();
    lcd.print(getString(strMsgConfig1));
    lcd.setCursor(2, 1);
    lcd.print(getString(strDoubleArrows));
    while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
      doPrintTimestamp();
      if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
        doDayIncrement();
      }
    }
    lcd.clear();
    lcd.print(getString(strMsgConfig2));
    lcd.setCursor(5, 1);
    lcd.print(getString(strDoubleArrows));
    while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
      doPrintTimestamp();
      if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
        doMonthIncrement();
      }
    }
    lcd.clear();
    lcd.print(getString(strMsgConfig3));
    lcd.setCursor(8, 1);
    lcd.print(getString(strQuadrupleArrows));
    while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
      doPrintTimestamp();
      if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
        doYearIncrement();
      }
    }
    lcd.clear();
    lcd.print(getString(strMsgConfig4));
    lcd.setCursor(13, 1);
    lcd.print(getString(strDoubleArrows));
    while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
      doPrintTimestamp();
      if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
        doHourIncrement();
      }
    }
    lcd.clear();
    lcd.print(getString(strMsgConfig5));
    lcd.setCursor(16, 1);
    lcd.print(getString(strDoubleArrows));
    while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
      doPrintTimestamp();
      if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
        doMinuteIncrement();
      }
    }
    lcd.clear();
    lcd.print(getString(strMsgConfig6));
    for (int c = 0; c < 4; c++) {
      lcd.setCursor(4 + c, 1);
      lcd.print(" v");
      while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
        lcd.setCursor(5, 2);
        lcd.print(icao);
        if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
          if (++icao[c] > 90) {
            icao[c] = 65;
          }
        }
      }
      EEPROM.write(MEMORY_ICAO + c, icao[c]);
    }
    lcd.clear();
    lcd.print(getString(strMsgConfig7));
    int typelimit = 90;
    float coord = 0;
    for (int type = 0; type < 2; type++) {
      lcd.setCursor(1, 2);
      if (type == 0) {
        lcd.print(getString(strLat));
      } 
      else {
        lcd.print(getString(strLng));
        typelimit = 180;
      }
      int subtypeoffset[3] = {
        2, 6, 9      };
      char buf[15] = "";
      for (int subtype = 0; subtype < 2; subtype++) {
        while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
          lcd.setCursor(subtypeoffset[subtype], 1);
          lcd.print(getString(strGPSArrows));
          for (int f = 1; f < (10 - subtypeoffset[subtype]); f++) {
            lcd.print(' ');  
          }
          lcd.setCursor(5, 2);
          coord = longitude;
          if (type == 0) {
            coord = latitude;
          }
          lcd.print(doTransformDecimal2Degree(buf, coord, gps_deg, gps_min, gps_sec));
          if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
            switch (subtype) {
            case 0:
              gps_deg++;
              if (gps_deg > typelimit) {
                gps_deg = -typelimit;
              }
              break;
            case 1:
              gps_min++;
              gps_min %= 60;
              break;
            case 2:
              gps_sec++;
              gps_sec %= 60;
              break;
            }
            coord = doTransformDegree2Decimal(gps_deg, gps_min, gps_sec);
            if (type == 0) {
              latitude = coord;
            } 
            else {
              longitude = coord;
            }                
          }
        }
      }
      int address = MEMORY_LAT;
      int value = 0;
      if (type == 1) {
        address = MEMORY_LNG;
      }
      for (int offset = 0; offset < 3; offset++) {
        switch (offset) {
        case 0:
          value = gps_deg;
          break;
        case 1:
          value = gps_min;
          break;
        case 2:
          value = gps_sec;
          break;
        }
        EEPROM.write(address + offset, value);
      }
    }
    lcd.clear();
  }

  // METAR render task
  if ((metarMetro.check()) && (status == STS_OK)) {
    renderMetar();
    lcd.home();
    imprimeMetar();
  }

  // Publish task
  if (publishMetro.check()) {
    gsmHttp();
    publishMetro.reset();
  }
  
  // Wind direction reading (TODO: Interrupts)
  windDirRead();
 
}
