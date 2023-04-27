/* 
 Weather Shield Example
 By: Nathan Seidle
 SparkFun Electronics
 Date: November 16th, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 Much of this is based on Mike Grusin's USB Weather Board code: https://www.sparkfun.com/products/10586
 
 This code reads all the various sensors (wind speed, direction, rain gauge, humidty, pressure, light, batt_lvl)
 and reports it over the serial comm port. This can be easily routed to an datalogger (such as OpenLog) or
 a wireless transmitter (such as Electric Imp).
 
 Measurements are reported once a second but windspeed and rain gauge are tied to interrupts that are
 calcualted at each report.
 
 This example code assumes the GP-635T GPS module is attached.
 
 */

#include <Wire.h> //I2C needed for sensors
#include "MPL3115A2.h" //Pressure sensor
#include "HTU21D.h" //Humidity sensor
#include <SoftwareSerial.h> //Needed for GPS
#include <TinyGPS++.h> //GPS parsing
//#include <LiquidCrystal_I2C.h>//16x2 LCD library
//It requires using fm's LiquidCrystal library replacement:
//https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home

//This is for the 16x2 LCD
//Most of them are using this
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
//Some use this PCF8574
//LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

////this is for RTC
//int clockAddress = 0x68;  // This is the I2C address
//int command = 0;  // This is the command char, in ascii form, sent from the serial port     
//long previousMillis = 0;  // will store last time Temp was updated
//byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
//
//byte decToBcd(byte val)
//{
//  return ( (val/10*16) + (val%10) );
//}
//
//// Convert binary coded decimal to normal decimal numbers
//byte bcdToDec(byte val)
//{
//  return ( (val/16*10) + (val%16) );
//}
//
//// Gets the date and time from the ds1307 and prints result
//char* getDateDs1307(int flag) {
//  //if flag == 0 : date output
//  //if flag == 1 : time output
//  // Reset the register pointer
//  Wire.beginTransmission(clockAddress);
//  Wire.write(byte(0x00));
//  Wire.endTransmission();
//
//  Wire.requestFrom(clockAddress, 7);
//
//  // A few of these need masks because certain bits are control bits
//  second     = bcdToDec(Wire.read() & 0x7f);
//  minute     = bcdToDec(Wire.read());
//
//  // Need to change this if 12 hour am/pm
//  hour       = bcdToDec(Wire.read() & 0x3f);  
//  dayOfWeek  = bcdToDec(Wire.read());
//  dayOfMonth = bcdToDec(Wire.read());
//  month      = bcdToDec(Wire.read());
//  year       = bcdToDec(Wire.read());
//
//  char sza[32];
//  if (flag==0)
//    sprintf(sza, "%02d-%02d-%02d",year,month,dayOfMonth);
//  if (flag==1)
//    sprintf(sza, "%02d:%02d:%02d",hour,minute,second);
//  return(sza);
//}
////end of RTC


TinyGPSPlus gps;

//For Arduino Uno
//static const int RXPin = 5, TXPin = 4; //GPS is attached to pin 4(TX from GPS) and pin 5(RX into GPS)
//For Arduino Mega
static const int RXPin = 50, TXPin = 51; //GPS is attached to pin 7(TX from GPS) and pin 6(RX into GPS)
SoftwareSerial ss(RXPin, TXPin); 

MPL3115A2 myPressure; //Create an instance of the pressure sensor
HTU21D myHumidity; //Create an instance of the humidity sensor

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte RAIN = 2;
const byte WSPEED = 3;
const byte GPS_PWRCTL = 6; //Pulling this pin low puts GPS to sleep but maintains RTC and RAM
//For Arduino Uno Only
//const byte STAT1 = 7;
//to be removed if using a GSM board
//const byte STAT2 = 8;

// analog I/O pins
const byte WDIR = A0;
const byte LIGHT = A1;
const byte BATT = A2;
const byte REFERENCE_3V3 = A3;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSecond; //The millis counter to see when a second rolls by
long loopMSecond; //time it took to loop once in millis
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_5m; //Keeps track of where we are in rain5min over last 5 minutes array of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;

//We need to keep track of the following variables:
//Wind speed/dir each update (no storage)
//Wind gust/dir over the day (no storage)
//Wind speed/dir, avg over 2 minutes (store 1 per second)
//Wind gust/dir over last 10 minutes (store 1 per minute)
//Rain over the past hour (store 1 per minute)
//Total rain over date (store one per day)

byte windspdavg[120]; //120 bytes to keep track of 2 minute average
int winddiravg[120]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain
volatile float rain5min[5]; //5 floating numbers to keep track of 5 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedms = 0; // [mph instantaneous wind speed]
float windgustms = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
float windspdms_avg2m = 0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
float windgustms_10m = 0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
float humidity = 0; // [%]
float tempf = 0; // [temperature F]
float rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
volatile float dailyrainin = 0.0; // [rain inches so far today in local time]
volatile float rainin5min=0.0; // [rain inches so far last 5 minutes in local time]
//float baromin = 30.03;// [barom in] - It's hard to calculate baromin locally, do this in the agent
float pressure = 0;
//float dewptf; // [dewpoint F] - It's hard to calculate dewpoint locally, do this in the agent

float batt_lvl = 11.8; //[analog value from 0 to 1023]
float light_lvl = 455; //[analog value from 0 to 1023]
//Rain time stamp
int Rainindi=0;
//Variables used for GPS
//unsigned long age;
//int year;
//byte month, day, hour, minute, second, hundredths;

//Calibrate rain bucket here
//Rectangle raingauge from Sparkfun.com weather sensors
//float rain_bucket_mm = 0.011*25.4;//Each dump is 0.011" of water
//DAVISNET Rain Collector 2
float rain_bucket_mm = 0.01*25.4;//Each dump is 0.01" of water

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain, Rainindtime, Rainindlast;

//for loop
int i;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

    if (raininterval > 100) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += rain_bucket_mm; 
    rainHour[minutes] += rain_bucket_mm; //Increase this minute's amount of rain
    rain5min[minutes_5m] += rain_bucket_mm; // increase this 5 mnts amout of rain
    rainlast = raintime; // set up for next event
  }

  //Rain or not (1 or 0)
  if(rainin >0)
  {
    Rainindi=1;
    Rainindtime = millis();
  }
  if(rainin ==0)
  {
    Rainindi=0; 
    Rainindlast = millis();
    Rainindlast = Rainindtime;
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }

}


void setup()
{
  //Switch on the GPS
  pinMode(GPS_PWRCTL, OUTPUT);
  digitalWrite(GPS_PWRCTL, HIGH); //Pulling this pin low puts GPS to sleep but maintains RTC and RAM

  //Start Serial port reporting
  Serial.begin(9600);

  // set up the LCD's number of columns and rows:
//  lcd.begin(16, 2);
//  lcd.backlight();
//  lcd.print("Start MWS");

  //Start software serial for GPS
  //Begin listening to GPS over software serial at 9600. This should be the default baud of the module.
  ss.begin(9600); 
  Serial.print(F("lon,lat,altitude,sats,date,GMTtime,winddir"));
  Serial.print(F(",windspeedms,windgustms,windgustdir,windspdms_avg2m,winddir_avg2m,windgustms_10m,windgustdir_10m"));
  Serial.print(F(",humidity,tempc,rainhourmm,raindailymm,rainindicate,rain5min,pressure,batt_lvl,light_lvl"));

  //For Arduino UNO Only
  //pinMode(STAT1, OUTPUT); //Status LED Blue
  //pinMode(STAT2, OUTPUT); //Status LED Green

  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

  //Configure the pressure sensor
  myPressure.begin(); // Get sensor online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

    //Configure the humidity sensor
  myHumidity.begin();

  seconds = 0;
  lastSecond = millis();

  // attach external interrupt pins to IRQ functions
  attachInterrupt(0, rainIRQ, FALLING);
  attachInterrupt(1, wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();
  //digitalWrite(STAT2, HIGH); //Blink stat LED 1 second
  //delay(1000);
  //digitalWrite(STAT2, LOW); //Blink stat LED
  smartdelay(6000); //Wait 60 seconds, and gather GPS data
  minutes = gps.time.minute();
  minutes_5m = gps.time.minute();
  minutes_10m = gps.time.minute();
  seconds = gps.time.second();
  lastSecond = millis();
  //digitalWrite(STAT2, HIGH); //Blink stat LED 1 second
  //delay(1000);
  //digitalWrite(STAT2, LOW); //Blink stat LED
 // lcd.print("Weather Station online!");
}

void loop()
{
  //Keep track of which minute it is
  loopMSecond = millis() - lastSecond;
  //digitalWrite(STAT2, HIGH); //Blink stat LED
  lastSecond = millis();

  //Take a speed and direction reading every second for 2 minute average
  seconds_2m += loopMSecond/1000;
  if(++seconds_2m > 119) seconds_2m = 0;

  //Calc the wind speed and direction every second for 120 second to get 2 minute average
  float currentSpeed = get_wind_speed();
  int currentDirection = get_wind_direction();
  windspdavg[seconds_2m] = (int)currentSpeed;
  winddiravg[seconds_2m] = currentDirection;

  //Check to see if this is a gust for the minute
  if(currentSpeed > windgust_10m[minutes_10m])
  {
    windgust_10m[minutes_10m] = currentSpeed;
    windgustdirection_10m[minutes_10m] = currentDirection;
  }

  //Check to see if this is a gust for the day
  if(currentSpeed > windgustms)
  {
    windgustms = currentSpeed;
    windgustdir = currentDirection;
  }
  seconds += loopMSecond/1000;
  if(++seconds > 59) seconds = 0;
  minutes += seconds/60;
  if(++minutes > 59) minutes = 0;
  minutes_5m += seconds/60;
  if(++minutes_5m > 4){
    //Report all readings every 5 minutes
    //printWeather();
    minutes_5m = 0;
    for (i=0;i<5;i++) rain5min[i] = 0;
  }
  minutes_10m += seconds/60;
  if(++minutes_10m > 9){
    minutes_10m = 0;
    for (i=0;i<10;i++) windgust_10m[i] = 0;
  }

  rainHour[minutes] = 0; //Zero out this minute's rainfall amount
  rain5min[minutes_5m] = 0; //Zero out this minute's rain
  windgust_10m[minutes_10m] = 0; //Zero out this minute's gust

  //Report all readings
  printWeather();
  //Turn off stat LED
  //digitalWrite(STAT2, LOW); 
  //Wait 1 second, and gather GPS data
  smartdelay(800); 
}

//While we delay for a given amount of time, gather GPS data
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } 
  while (millis() - start < ms);
}


//Calculates each of the variables that wunderground is expecting
void calcWeather()
{
  //Calc winddir
  winddir = get_wind_direction();

  //Calc windspeed
  windspeedms = get_wind_speed();

  //Calc windgustms
  //Calc windgustdir
  //Report the largest windgust today
  windgustms = 0;
  windgustdir = 0;

  //Calc windspdms_avg2m
  float temp = 0;
  for(int i = 0 ; i < 120 ; i++)
    temp += windspdavg[i];
  temp /= 120.0;
  windspdms_avg2m = temp;

  //Calc winddir_avg2m
  temp = 0; //Can't use winddir_avg2m because it's an int
  for(int i = 0 ; i < 120 ; i++)
    temp += winddiravg[i];
  temp /= 120;
  winddir_avg2m = temp;

  //Calc windgustms_10m
  //Calc windgustdir_10m
  //Find the largest windgust in the last 10 minutes
  windgustms_10m = 0;
  windgustdir_10m = 0;
  //Step through the 10 minutes  
  for(int i = 0; i < 10 ; i++)
  {
    if(windgust_10m[i] > windgustms_10m)
    {
      windgustms_10m = windgust_10m[i];
      windgustdir_10m = windgustdirection_10m[i];
    }
  }

  //Calc humidity
  humidity = myHumidity.readHumidity();
  if(myHumidity.readTemperature())
  {
    tempf = myHumidity.readTemperature();
  } 
  else
  {
    //Calc tempf from pressure sensor
    tempf = myPressure.readTemp();
  }

  //Total rainfall for the day is calculated within the interrupt
  //Calculate amount of rainfall for the last 5 minutes
  rainin5min = 0;  
  for(int i = 0 ; i < 5 ; i++)
    rainin5min += rain5min[i];
  //Calculate amount of rainfall for the last 60 minutes
  rainin = 0;  
  for(int i = 0 ; i < 60 ; i++)
    rainin += rainHour[i];

  //Calc pressure
  pressure = myPressure.readPressure();

  //Calc light level
  light_lvl = get_light_level();

  //Calc battery level
  batt_lvl = get_battery_level();

}

//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
float get_light_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);

  float lightSensor = analogRead(LIGHT);

  operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V

  lightSensor = operatingVoltage * lightSensor;

  return(lightSensor);
}

//Returns the voltage of the raw pin based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
//Battery level is connected to the RAW pin on Arduino and is fed through two 5% resistors:
//3.9K on the high side (R1), and 1K on the low side (R2)
float get_battery_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);

  float rawVoltage = analogRead(BATT);

  operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V

  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin

  rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage

  return(rawVoltage);
}

//Returns the instataneous wind speed
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck; //750ms

  deltaTime /= 1000.0; //Convert to seconds

  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();

  //windSpeed *= 1.492; //4 * 1.492 = 5.968MPH
  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH
  windSpeed *= 1.609344; //5.968MPH * 1.609344 = 9.604564992KMPH
  windSpeed /= 3.6; //9.604564992KMPH / 3.6 = 2.66793472MPS

  return(windSpeed);
}

//Read the wind direction sensor, return heading in degrees
int get_wind_direction() 
{
  unsigned int adc;

  adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 380) return (113);//NOT WORKING
  else if (adc < 393) return (68);//NOT WORKING
  else if (adc < 414) return (90);//NOT WORKING
  else if (adc < 456) return (90);//(158);//E is 90 degrees CW from North = 0
  else if (adc < 508) return (135);//SE is 135 degrees CW from North = 0
  else if (adc < 551) return (180);//(203);//S is 180 degrees CW from North = 0
  else if (adc < 615) return (45);//(180);//NE is 45 degrees CW from North = 0
  else if (adc < 680) return (23);//NOT WORKING
  else if (adc < 746) return (225);//(45);//SW is 225 degrees CW from North = 0
  else if (adc < 801) return (248);//NOT WORKING
  else if (adc < 833) return (0);//(225);//N is 0 degrees CW from North = 0
  else if (adc < 878) return (338);//NOT WORKING
  else if (adc < 913) return (325);//(0);//NW is 325 degrees CW from North = 0
  else if (adc < 940) return (293);//NOT WORKING
  else if (adc < 967) return (270);//(315);//W is 270 degrees CW from North = 0
  else if (adc < 990) return (270);//NOT WORKING
  else return (-1); // error, disconnected?
}


//Prints the various variables directly to the port
//I don't like the way this function is written but Arduino doesn't support floats under sprintf
void printWeather()
{
  calcWeather(); //Go calc all the various sensors

  Serial.println();
  Serial.print(gps.location.lng(), 6);//[0]
  Serial.print(",");
  Serial.print(gps.location.lat(), 6);//[1]
  Serial.print(",");
  Serial.print(gps.altitude.meters());//[2]
  Serial.print(",");
  Serial.print(gps.satellites.value());//[3]

  char sz[32];
  Serial.print(",");
  sprintf(sz, "%02d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());//[4]
  Serial.print(sz);

  Serial.print(",");
  sprintf(sz, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());//[5]
  Serial.print(sz); 

  Serial.print(",");
  Serial.print(get_wind_direction());//[6]
  Serial.print(",");
  Serial.print(get_wind_speed(), 1);//[7]
  Serial.print(",");
  Serial.print(windgustms, 1);//[8]
  Serial.print(",");
  Serial.print(windgustdir);//[9]
  Serial.print(",");
  Serial.print(windspdms_avg2m, 1);//[10]
  Serial.print(",");
  Serial.print(winddir_avg2m);//[11]
  Serial.print(",");
  Serial.print(windgustms_10m, 1);//[12]
  Serial.print(",");
  Serial.print(windgustdir_10m);//[13]
  Serial.print(",");
  Serial.print(humidity, 1);//[14]
  Serial.print(",");
  Serial.print(tempf, 1);//[15] Celsius
  Serial.print(",");
  Serial.print(rainin, 3);//[16] hourly
  Serial.print(",");
  Serial.print(dailyrainin, 3);//[17]
  Serial.print(",");
  Serial.print(Rainindi,1);//[18] 5min indicatior (flag 0/1)
  Serial.print(",");
  Serial.print(rainin5min,3);//[19] 5min rain (mm/5min)
  Serial.print(",");
  Serial.print(pressure, 2);//[20]
  Serial.print(",");
  Serial.print(batt_lvl, 2);//[21]
  Serial.print(",");
  Serial.print(light_lvl, 2);//[22]
  Serial.print(",");

  //--------------------------
  //PRINT TO LCD
  //--------------------------

//  lcd.clear();
 // lcd.print(F("Lon:"));
 // lcd.print(gps.location.lng(), 6);
//lcd.setCursor(0, 2);
 // lcd.print(F("Lat:"));
//  lcd.print(gps.location.lat(), 6);
 // delay(5000);

 // lcd.clear();
 // sprintf(sz, "%02d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
//  lcd.print(sz);
//  lcd.setCursor(0, 2);
 // sprintf(sz, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
 // lcd.print(sz);
 // lcd.print(F(" GMT"));
  delay(5000);

//  lcd.clear();
//  lcd.print(F("WD: "));
//  lcd.print(get_wind_direction());
//  lcd.print(F(" N=0 CW"));
//  lcd.setCursor(0, 2);
//  lcd.print(F("WS: "));
//  lcd.print(get_wind_speed(), 1);
//  lcd.print(F(" m/s"));
//  delay(5000);
//
//  lcd.clear();
//  lcd.print(F("H: "));
//  lcd.print(myHumidity.readHumidity(),2);
//  lcd.print(F(" %"));
//  lcd.setCursor(0, 2);
//  lcd.print(F("T: "));
//  lcd.print(myPressure.readTemp(),2);
//  lcd.print(F(" C"));
//  delay(5000);
//
//  lcd.clear();
//  lcd.print("R:");
//  lcd.print(rainin5min,2);
//  lcd.print(" mm/5min");
//  lcd.setCursor(0, 2);
//  lcd.print(F("R:"));
//  lcd.print(rainin,2);
//  lcd.print(F(" mm/h"));
//  delay(5000);
//
//  lcd.clear();
//  lcd.print("R:");
//  lcd.print(dailyrainin,2);
//  lcd.print("(mm/d)");
//  lcd.setCursor(0, 2);
//  lcd.print(F("P:"));
//  lcd.print(myPressure.readPressure()/100.0,2);
//  lcd.print(F(" hPa"));
//  delay(5000);
//
//  lcd.clear();
//  lcd.print(F("Battery Level"));
//  lcd.print(get_battery_level());
//  lcd.clear();
//  lcd.print(F("Sunlight"));
//  lcd.setCursor(0, 2);
//  lcd.print(get_light_level());
//  delay(2000);


}




