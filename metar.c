/*

  METAR (VOO SEGURO) - Version 1.0
  Developed by Alexandre Lehmann Holzhey in 2014
  
*/

#include <SoftwareSerial.h>
#include <Time.h>
#include <BMP085.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <Metro.h>
#include <EEPROM.h>
#include <Math.h>

#define DHTPIN 7                                    // Humidity sensor digital in
#define DHTTYPE DHT22                               // Humidity sensor type
#define BH1750_Device 0x23                          // I2C address for pressure and temperature sensor
#define BUTTON_MODE_PIN 5                           // Mode button digital in
#define BUTTON_ACTION_PIN 4                         // Action button digital in
#define GPRS_MODULE_POWER_PIN 10                    // GPRS module power on digital out
#define GPRS_MODULE_RX_PIN 9                        // GPRS module RX digital out
#define GPRS_MODULE_TX_PIN 8                        // GPRS module TX digital out
#define I2C_LCD_ADDRESS 0x27                        // I2C LCD address
#define TIMER_HUMIDITY 20000                        // Humidity read interval
#define TIMER_BARO 21010                            // Pressure read interval
#define TIMER_TEMP 22030                            // Temperature read interval
#define TIMER_DEWPOINT 23070                        // Dewpoint calculation interval
#define TIMER_LUX 2000                              // Luminosity read interval
#define TIMER_COVER 60000                           // Clouds cover calculation interval
#define LUX_SAMPLES (TIMER_COVER / TIMER_LUX)       // Maximum samples for luminosity history
#define TIMER_METAR 3000                            // Metar rendering interval
#define TIMER_PUBLISH 2400000                       // Server publish interval
#define TIMER_ANEMOMETER 5000                       // Instant wind calculation interval
#define TIMER_WINDS 20000                           // Wind average and constant calculation interval
#define TIMER_BUTTON_MODE 100                       // Mode button reading interval
#define DELAY_ANEMOMETER_LOW 50                     // Anemometer interrupt on low delay time
#define DELAY_ANEMOMETER_HIGH 20                    // Anemometer interrupt on high delay time
#define STS_OK 0                                    // Status OK for readings
#define STS_ERR_SENSOR_HUMIDITY 1                   // Status ERROR for humidity reading
#define MEMORY_ICAO 20                              // EEPROM address for ICAO storage
#define MEMORY_LAT 24                               // EEPROM address for latitude storage
#define MEMORY_LNG 26                               // EEPROM address for longitude storage

boolean buttonAction = false;                       // Flag used for action button control
boolean buttonMode = false;                         // Flag used for mode button control
Metro buttonModeMetro = Metro(TIMER_BUTTON_MODE);   // Initialize mode button Metro

RTC_DS1307 rtc;                                     // Initialize RTC clock library
DHT dht(DHTPIN, DHTTYPE);                           // Initialize humidity sensor library
BMP085 dps = BMP085();                              // Initialize pressure and temperature sensor library
LiquidCrystal_I2C lcd(I2C_LCD_ADDRESS, 20, 4);      // Initialize I2C LCD library

SoftwareSerial gsmSerial(GPRS_MODULE_RX_PIN, GPRS_MODULE_TX_PIN); // Initialize GPRS software serial

Metro metarMetro = Metro(TIMER_METAR);
char metar[61] = "";

float humidity = 0;
Metro humidityMetro = Metro(TIMER_HUMIDITY);

long baro = 0;
Metro baroMetro = Metro(TIMER_BARO);

long temp = 0;
Metro tempMetro = Metro(TIMER_TEMP);

Metro publishMetro = Metro(TIMER_PUBLISH);

long dewpoint = 0;
Metro dewpointMetro = Metro(TIMER_DEWPOINT);

unsigned int windSpeed = 0;
unsigned int windAvg = 0;
boolean windAvgFirst = true;
unsigned int windGust = 0;
Metro anemometerMetro = Metro(TIMER_ANEMOMETER);
Metro windsMetro = Metro(TIMER_WINDS);

unsigned int anemometerRevolutions= 0;
unsigned long anemometerLastRead = 0;
unsigned long anemometerInterruptLast = 0;
int anemometerState = LOW;

unsigned int lux = 0;
byte cover = 0;
Metro luxMetro = Metro(TIMER_LUX);
Metro coverMetro = Metro(TIMER_COVER);
struct samples {
  unsigned int maxLux;
  unsigned int minLux;
  unsigned int avgLux;
  unsigned int history[LUX_SAMPLES];
  unsigned int samples;
};
typedef struct samples LuxSamples;
LuxSamples luxSamples;

int status = STS_OK;
char icao[5] = "SBXX";

float latitude = 0;
float longitude = 0;
int gps_deg = 0;
int gps_min = 0;
int gps_sec = 0;

unsigned int getSpeedFromRevolutions(unsigned int revolutions, unsigned int elapsedms) {
  //return (((revolutions * anemometerUnit) / 100 ) / (elapsedms / 1000)) * 1.943844;
  float rpm = (revolutions * 60 )/ (elapsedms /1000);
  return (rpm * 0.03);
}


int8_t sendATcommand(char* ATcommand, char* expected_answer1, unsigned int timeout){

    uint8_t x=0,  answer=0;
    char response[100];
    unsigned long previous;

    memset(response, '\0', 100);    // Initialize the string

    delay(100);

    while( gsmSerial.available() > 0) gsmSerial.read();    // Clean the input buffer

    gsmSerial.println(ATcommand);    // Send the AT command 


        x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        if(gsmSerial.available() != 0){    
            response[x] = gsmSerial.read();
            x++;
            // check if the desired answer is in the response of the module
            if (strstr(response, expected_answer1) != NULL)    
            {
                answer = 1;
            }
        }
        // Waits for the asnwer with time out
    }
    while((answer == 0) && ((millis() - previous) < timeout));    

    return answer;
}



int8_t sendATcommand2(char* ATcommand, char* expected_answer1, 
char* expected_answer2, unsigned int timeout){

    uint8_t x=0,  answer=0;
    char response[100];
    unsigned long previous;

    memset(response, '\0', 100);    // Initialize the string

    delay(100);

    while( gsmSerial.available() > 0) gsmSerial.read();    // Clean the input buffer

    gsmSerial.println(ATcommand);    // Send the AT command 


        x = 0;
    previous = millis();

    // this loop waits for the answer
    do{        
        if(gsmSerial.available() != 0){    
            response[x] = gsmSerial.read();
            x++;
            // check if the desired answer 1 is in the response of the module
            if (strstr(response, expected_answer1) != NULL)    
            {
                answer = 1;
            }
            // check if the desired answer 2 is in the response of the module
            if (strstr(response, expected_answer2) != NULL)    
            {
                answer = 2;
            }
        }
        // Waits for the asnwer with time out
    }while((answer == 0) && ((millis() - previous) < timeout));    

    return answer;
}

void power_on(){

    uint8_t answer=0;
    answer = sendATcommand("AT", "OK", 2000);
    while (answer == 0)
    {
        digitalWrite(GPRS_MODULE_POWER_PIN,HIGH);
        delay(3000);
        digitalWrite(GPRS_MODULE_POWER_PIN,LOW);
        byte ct = 0;
        while(answer == 0){  
            answer = sendATcommand("AT", "OK", 2000);
            ct++;
            if (ct > 5) break;
        }
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

unsigned int BH1750_Read()
{
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
  
 void Configure_BH1750()
 {
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

int numPad(int num, int qtd) {
  return num;
}

void setup() {
  lcd.begin(20, 4);
  lcd.init();
  lcd.clear();
  lcd.print("Voo Seguro 0.5");
  lcd.setCursor(0, 1);
  lcd.print("Inicializando...");
  delay(3000);
  Serial.begin(19200);
  pinMode(GPRS_MODULE_POWER_PIN, OUTPUT);
  gsmSerial.begin(19200);
  power_on();
  pinMode(BUTTON_MODE_PIN, INPUT);
  pinMode(BUTTON_ACTION_PIN, INPUT);
  pinMode(13, OUTPUT);
  Wire.begin();
  Configure_BH1750();
  rtc.begin();
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(__DATE__, __TIME__));
  } 
  for (int c = 0; c < 4; c++) {
    char ic = EEPROM.read(MEMORY_ICAO + c);
    if (ic >= 65 and ic <= 90) {
      icao[c] = ic;
    }
  }
  
  // Restaura gps
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
    } else {
      longitude = coord;
    }                
  }
  
  dht.begin();
  dps.init();
  attachInterrupt(0, anemometerInterrupt, CHANGE);
  anemometerLastRead = millis();
  anemometerRevolutions = 0;  
  windSpeed = 0;
  windAvg = 0;
  windAvgFirst = true;
  windGust = 0;
  luxSamples.maxLux = 0;
  luxSamples.minLux = 99999;
  luxSamples.avgLux = 0;
  luxSamples.samples = 0;
  lcd.clear();
  for (byte x = 0; x < 80; x++) {
    lcd.print('#');
  }
  delay(2000);
  //setTime(00, 00, 00, 01, 01, 2012);
  lcd.clear();
}

void gsmHttp() {
  if (sendATcommand2("AT+CREG?", "+CREG: 0,1", "+CREG: 0,5", 2000) == 0) {
    lcd.setCursor(0, 3);
    lcd.print("(Offline)");
  } else {
    lcd.setCursor(0, 3);
    lcd.print("(Enviando)");
    sendATcommand("AT+CSQ", "OK", 5000);
    sendATcommand("AT+CGATT?", "OK", 5000);
    sendATcommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 5000);
    sendATcommand("AT+SAPBR=3,1,\"APN\",\"VIVO\"", "OK", 5000);
    sendATcommand("AT+SAPBR=1,1", "OK", 5000);
    sendATcommand("AT+HTTPINIT", "OK", 5000);
//    sendATcommand("AT+HTTPPARA=\"URL\",\"http://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?action=updateraw&ID=IRIOGRAN45&password=sal24816&dateutc=2014-05-10+19%3A00%3A00&humidity=33&tempf=60\"", "OK", 5000);
    char url[128] = "AT+HTTPPARA=\"URL\",\"http://bool.eti.br/metar.php?metar=";
    char encMetar[128] = "";
    byte y = 0;
    for (byte x = 0; x < strlen(metar); x++) {
      switch (metar[x]) {
        case 32: // %20
          encMetar[y++] = 37;
          encMetar[y++] = 50;
          encMetar[y++] = 48;
          break;
        case 47: // %2F
          encMetar[y++] = 37;
          encMetar[y++] = 50;
          encMetar[y++] = 70;
          break;
        default:
          encMetar[y++] = metar[x];
          break;
      }
    }
    encMetar[y] = 0;
    strcat(url, encMetar);
    strcat(url, "\"");
    sendATcommand(url, "OK", 5000);
    sendATcommand("AT+HTTPACTION=0", "OK", 15000);
    //gsmSerial.println("AT+HTTPREAD");
    //delay(1000);
    //ShowSerialData();
    lcd.setCursor(0, 3);
    for (byte x = 0; x < 20; x++) {
      lcd.print(' ');
    }
  }
}

void anemometerInterrupt() {
  if (anemometerInterruptLast < millis()) {
    if (anemometerState != digitalRead(2)) {
      anemometerState = digitalRead(2);
      digitalWrite(13, anemometerState);
      if (anemometerState == HIGH) {
        anemometerRevolutions++;
        anemometerInterruptLast = millis() + DELAY_ANEMOMETER_HIGH;
      } else {
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
    } else {
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

void renderMetar(long temp, long dewpoint, float humidity, long baro, unsigned int lux) {
    if (windAvg != -1) {
      memset(metar, '\0', 60);
      strcat(metar, "METAR ");
      strcat(metar, icao);
      strcat(metar, " ");
      DateTime now = rtc.now();
      char buf[8];
      strcat(metar, zeroPad(utoa(now.day(), buf, 10), 2));
      strcat(metar, zeroPad(utoa(now.hour(), buf, 10), 2));
      strcat(metar, zeroPad(utoa(now.minute(), buf, 10), 2));
      strcat(metar, "   ");
      strcat(metar, utoa(windAvg, buf, 10));
      if ((windGust - windAvg) > 2) {
        strcat(metar, "G");
        strcat(metar, utoa(windGust, buf, 10));
      }
      strcat(metar, "KT ");
      strcat(metar, utoa(temp, buf, 10));
      strcat(metar, "/");
      strcat(metar, utoa(dewpoint, buf, 10));
      strcat(metar, " Q");
      strcat(metar, utoa(baro, buf, 10));
    }
}

boolean buttonPressed(boolean &state, int pin) {
  if (!state) {
      if (digitalRead(pin) == HIGH) {
        state = true;
        return true;
      }
    } else {
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
  //setTime(hour(), minute(), second(), ((day() + 1) % 30), month(), year());
}

void doMonthIncrement() {
  DateTime now = rtc.now();
  int tmp = now.month() + 1;
  if (tmp > 12) tmp = 1;
  rtc.adjust(DateTime(now.year(), tmp, now.day(), now.hour(), now.minute(), now.second()));
  //setTime(hour(), minute(), second(), day(), ((month() + 1) % 12), year());
}

void doYearIncrement() {
  DateTime now = rtc.now();
  int tmp = now.year() + 1;
  if (tmp > 2030) tmp = 2010;
  rtc.adjust(DateTime(tmp, now.month(), now.day(), now.hour(), now.minute(), now.second()));
  //setTime(hour(), minute(), second(), day(), month(), year() + 1);
}

void doHourIncrement() {
  DateTime now = rtc.now();
  rtc.adjust(DateTime(now.year(), now.month(), now.day(), ((now.hour() + 1) % 24), now.minute(), now.second()));
  //setTime(((hour() + 1) % 24), minute(), second(), day(), month(), year());
}

void doMinuteIncrement() {
  DateTime now = rtc.now();
  rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), ((now.minute() + 1) % 60), now.second()));
  //setTime(hour(), (minute() + 1) % 60, second(), day(), month(), year());
}

void doPrintTimestamp() {
  DateTime now = rtc.now();
  lcd.setCursor(2,2);
  lcd.print(now.day());
  lcd.print('/');
  lcd.print(now.month());
  lcd.print('/');
  lcd.print(now.year());
  lcd.print(' ');
  lcd.print(now.hour());
  lcd.print(':');
  lcd.print(now.minute());
}

short getDayNumber() {
  DateTime now = rtc.now();
  unsigned int y = now.year();
  unsigned int m = now.month();
  unsigned int d = now.day();
  short DN;
  int days[]={0,31,59,90,120,151,181,212,243,273,304,334};    // Number of days at the beginning of the month in a not leap year.
//Start to calculate the number of day
  if (m==1 || m==2){
    DN = days[(m-1)]+d;                     //for any type of year, it calculate the number of days for January or february
  }                        // Now, try to calculate for the other months
  else if ((y % 4 == 0 && y % 100 != 0) ||  y % 400 == 0){  //those are the conditions to have a leap year
    DN = days[(m-1)]+d+1;     // if leap year, calculate in the same way but increasing one day
  }
  else {                                //if not a leap year, calculate in the normal way, such as January or February
    DN = days[(m-1)]+d;
  }
  return DN;
}

double getDaylightTime(float lat) {
  int yearday = getDayNumber();
  Serial.println("-----");
  Serial.println(yearday);
  Serial.println(lat, 6);
  Serial.println("-----");
  double d = 23.45 * sin(0.98630137 * (284.0 + yearday));
  Serial.println(d, 6);
  double td = 0.133333 * acos(-tan(lat) * tan(d));
  Serial.println(td, 6);
  Serial.println("-----");
  return td;
}

String doTransformDecimal2Degree(float coord, int &c_deg, int &c_min, int &c_sec) {
  c_deg = round(trunc(coord));
  coord = abs((coord - trunc(coord))) * 60;
  c_min = round(trunc(coord));
  coord = (coord - trunc(coord));
  c_sec = round(trunc(coord * 60));
  String result = "";
  if (c_deg < 0) {
    result = "-";
  } else {
    result = "+"; 
  }
  result += numPad(abs(c_deg), 3);
  result += " ";
  result += numPad(c_min, 2);
  result += "'";
  result += numPad(c_sec, 2);
  result += "\"";
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
  
  //if (gsmSerial.available()) {
  //  Serial.write(gsmSerial.read());
  //}
  //if (Serial.available()) {
  //  gsmSerial.write(Serial.read()); 
  //}
  
  if (humidityMetro.check()) {
    humidity = getHumidity();
  }
  
  if (baroMetro.check()) {
    baro = getBaro();
  }
  
  if (tempMetro.check()) {
    temp = getTemp();
  }
  
  if (dewpointMetro.check()) {
    dewpoint = getDewPoint(temp, humidity);
  }
  
  if (luxMetro.check()) {
    lux = BH1750_Read();
    if (luxSamples.samples == 0) {
      luxSamples.avgLux = lux;
    } else {
      luxSamples.avgLux = (luxSamples.avgLux + lux) / 2;
    }
    if (lux > luxSamples.maxLux) {
      luxSamples.maxLux = lux;
    }
    if (lux < luxSamples.minLux) {
      luxSamples.minLux = lux;
    }
    luxSamples.history[luxSamples.samples] = lux;
    luxSamples.samples++;
  }
  
  if (coverMetro.check()) {
    unsigned int luxDelta = luxSamples.history[0];
    Serial.println("Cover calculation...");
    for (unsigned int x = 1; x < luxSamples.samples; x++) {
      luxDelta += (luxSamples.history[x] - luxDelta);
      Serial.print(luxSamples.history[x]);
      Serial.print(" ");
    }
    luxDelta = luxDelta - luxSamples.history[0];
    Serial.println(" ");
    Serial.println(luxDelta);
    unsigned int luxTotalDelta = (luxSamples.maxLux - luxSamples.minLux);
    float luxFactor = luxDelta / luxTotalDelta;
    Serial.println(luxTotalDelta);
    luxSamples.maxLux = 0;
    luxSamples.minLux = 99999;
    luxSamples.avgLux = 0;
    luxSamples.samples = 0;
  }
  
  if (anemometerMetro.check()) {
    noInterrupts();
    windSpeed = getSpeedFromRevolutions(anemometerRevolutions, millis() - anemometerLastRead);
    anemometerRevolutions = 0;
    anemometerLastRead = millis();
    interrupts();
    if (windAvgFirst) {
      windAvg = windSpeed;
      windAvgFirst = false;
    } else {
      windAvg = (windAvg + windSpeed) / 2;
    }
    if (windSpeed > windGust) {
      windGust = windSpeed;
    }
  }
  
  if (windsMetro.check()) {
    windAvg = 0;
    windGust = 0;
    windAvgFirst = true;
  }
  
  if (buttonModeMetro.check()) {
    if (buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
      lcd.clear();
      lcd.print("Config 1/7 : Dia");
      lcd.setCursor(2, 3);
      lcd.print("vv");
      while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
        doPrintTimestamp();
        if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
          doDayIncrement();
        }
      }
      lcd.clear();
      lcd.print("Config 2/7 : Mes");
      lcd.setCursor(5, 3);
      lcd.print("vv");
      while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
        doPrintTimestamp();
        if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
          doMonthIncrement();
        }
      }
      lcd.clear();
      lcd.print("Config 3/7 : Ano");
      lcd.setCursor(8, 3);
      lcd.print("vvvv");
      while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
        doPrintTimestamp();
        if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
          doYearIncrement();
        }
      }
      lcd.clear();
      lcd.print("Config 4/7 : Hora");
      lcd.setCursor(13, 3);
      lcd.print("vv");
      while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
        doPrintTimestamp();
        if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
          doHourIncrement();
        }
      }
      lcd.clear();
      lcd.print("Config 5/7 : Minuto");
      lcd.setCursor(16, 3);
      lcd.print("vv");
      while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
        doPrintTimestamp();
        if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
          doMinuteIncrement();
        }
      }
      lcd.clear();
      lcd.print("Config 6/7 : ICAO");
      for (int c = 0; c < 4; c++) {
        lcd.setCursor(4 + c, 3);
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
      lcd.print("Config 7/7 : GPS");
      int typelimit = 90;
      float coord = 0;
      for (int type = 0; type < 2; type++) {
        lcd.setCursor(1, 2);
        if (type == 0) {
          lcd.print("Lat ");
        } else {
          lcd.print("Lng ");
          typelimit = 180;
        }
        int subtypeoffset[3] = {2, 6, 9};
        for (int subtype = 0; subtype < 3; subtype++) {
          while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
            lcd.setCursor(subtypeoffset[subtype], 3);
            lcd.print("    vvv");
            for (int f = 1; f < (10 - subtypeoffset[subtype]); f++) {
              lcd.print(" ");  
            }
            lcd.setCursor(5, 2);
            coord = longitude;
            if (type == 0) {
              coord = latitude;
            }
            lcd.print(doTransformDecimal2Degree(coord, gps_deg, gps_min, gps_sec));
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
              } else {
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
      lcd.print("Status");
      lcd.setCursor(2, 2);
      lcd.print("Daylight = ");
      lcd.print(getDaylightTime(latitude), 4);
      lcd.setCursor(2, 3);
      lcd.print("Dias = ");
      lcd.print(getDayNumber());
      while (!buttonPressed(buttonMode, BUTTON_MODE_PIN)) {
        lcd.setCursor(2, 4);
        lcd.print("AneRPM:");
        lcd.print(anemometerRevolutions);
        if (buttonPressed(buttonAction, BUTTON_ACTION_PIN)) {
        }
      }
      lcd.clear();
    }
  }
  
  if ((metarMetro.check()) && (status == STS_OK)) {
    renderMetar(temp, dewpoint, humidity, baro, lux);
    lcd.home();
    imprimeMetar();
  }
  
  if (publishMetro.check()) {
    gsmHttp();
  }

}
