  // Author:      Tom Simpson
  // Date:        05/05/2020
  // File:        DHT_LCD_X10-2.INO
  //
  /* Description
     Uses a DHT22 sensor to measure temperature and humidity. A 16x2 LCD display
     switches between temperature/humidity and day/date/time info every 10 seconds. 
     Also monitors a LDR and Paralax PIR sensor and controls a lamp using x10 controller 
     and lamp module to turn a light on at a designated time each day. All other times,
     turn on the lamp for a short period when motion is detected and it's dark. 
     The scheduled lamp activation is delayed until it is dark. The time and date is 
     kept via a DS1307 RTC with battery backup.
  */
  /* Revision History
     TAS 05/05/2020 Initial file submission 
     TAS 05/21/2020 fix time comparisons
  */
  // Includes:
  #include <DHT.h>           // DHT adafruit sensor library
  #include <LiquidCrystal.h> // LCD display library
  #include <RTClib.h>        // RTC library works well with DS1307
  #include <x10.h>           // X10 library
  #include <x10constants.h>  // required X10 constants

  /* Connection Instructions:
     Connect pin 1 (on the left) of the sensor to +5V
     NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
     to 3.3V instead of 5V!
     Connect pin 2 of the sensor to whatever your Arduino data pin is for the DHT
     Connect pin 4 (on the right) of the sensor to GROUND
     Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
     pin 1 = data, pin 2 = +5, pin 3 = gnd for my sensor (yours may be different!)
     X10 connections:
     Connection to power is through a PL513 interface. I cut the end off a standard 
     RJ-11 phone cable and wired them to my breadboard. 
                      (looking at PL513)
       Yellow = data          (1) - connect to 10K pullup resistor -> +5v
       Green  = ground        (2)
       Red    = ground        (3)
       Black  = zero crossing (4)
     X10 Software: https://github.com/DougC/arduino-x10
  
     Parallax PIR sensor: 
       Pin 1 = ground
       Pin 2 = Vcc (+5v)
       Pin 3 = Vout (0 or 1)
  */
  
  #define zcPin 8         // Arduino Uno pin 8 => x10 zero crossing pin
  #define dataPin 9       // Arduino Uno pin 9 => x10 data pin
  #define LED1Pin 6       // LED is connected to pin 6
  #define pirPin 10       // PIR input
  #define ldrPin A0       // analog input for LDR
  #define repeatTimes 3   // How many times each X10 message should repeat.
                          // In an electrically noisy environment, you  
                          // can set this higher.

  //Define UNO pins needed for LCD
  #define rs 12
  #define en 11
  #define d4 5
  #define d5 4
  #define d6 3
  #define d7 2
  
  #define DHTPIN 7        // Digital pin connected to the DHT sensor

  // Uncomment whatever sensor type you're using!
  #define DHTTYPE DHT11   // DHT 11
  //#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321
  //#define DHTTYPE DHT21 // DHT 21 (AM2301)

  const float referenceVolts=5.0;  // for converting count to volts
  const byte DARK=LOW;
  const byte LIGHT=HIGH;
  char daysOfTheWeek[7][12] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

  // ---------- Change these to suit your situation ----------
  const long interval = 30000;          // interval to check x10 event time (30 sec)
  const long delayinterval = 120000;    // interval to keep lamp on (2 minutes)
  const long interval2 = 10000;         // interval to switch between displays (10 seconds)
  const String  off_time = "2300";      // time of day to turn lamp off
  const String  on_time = "1930";       // time of day to turn lamp on
  const float darkVolts = 1.0;          // dark level to turn lights on
  const byte x10_unit = UNIT_1;         // x10 unit number to control
  const byte x10_HC = HOUSE_B;          // x10 house code to control
  
  unsigned long previousMillis = 0;        // last time for x10 check
  unsigned long delayMillis = 0;           // last time for light-on check
  unsigned long previousMillis2 = 0;       // last time for display toggle

  byte x10_state=LOW;   // init x10 unit state
  byte x10_event_state=false; // true=in timed event
  byte pir_state=LOW;
  byte timed_delay=false;
  byte ldr_state=DARK;
  byte disp_state=false;   // used to switch time display, true=time, false=temperature

  // ---------------
  bool debug=false;        // add extra output for debug if true
  // ---------------
  
  x10 myHouse;             // set up a new x10 instance
  
  // init the LCD library with the interface pins #
  LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

  DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor.

  RTC_DS1307 rtc;          // set up a new RTC instance

void setup() 
{
  Serial.begin(9600);
  if (debug == true) Serial.println(F(" -------- DHT11,LCD, X10 test!"));
  
  pinMode(LED1Pin,OUTPUT);
  digitalWrite(LED1Pin,HIGH); // LED off    

  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif
  
  myHouse.init(zcPin, dataPin);       // get the power line timings for x10 ops
  Serial.println(myHouse.version());  // display them

  dht.begin();
 
  if (debug == true) Serial.println(F("Program Startup"));

  lcd.begin(16, 2);              // set up the LCD's number of col and rows
  lcd.clear();
  delay(1000);
  lcd.setCursor(0,0);
  lcd.print(F("Program Startup"));  // Print a message to the LCD.
  lcd.setCursor(0,1);
  lcd.print(F("T. Simpson v1"));
  delay(5000);

  if (! rtc.begin()) 
  {
    Serial.println("*** Could not find RTC. Unable to continue.");
    lcd.setCursor(0,1);
    lcd.print(F("** RTC NOT FOUND"));
    while (1);       // hang
  }

  if (! rtc.isrunning()) 
  {
    Serial.println(F("RTC is NOT running. Set the time!"));
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  //                  yyyy mm  dd hh  mm ss
  // rtc.adjust(DateTime(2020, 5, 20, 10, 59, 0));   // manually set time/date
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // use compiled time/date

  display_timedate();           // display all day-date-time info  
  String hhmm=get_str_time("");    
  ldr_state=check_LDR(ldrPin);  // check for dark or light

  if (debug == true)
  {
    Serial.print(F("x10 ON time = "));
    Serial.print(on_time);
    Serial.print(F("  x10 OFF time = "));
    Serial.println(off_time);
    Serial.print(F("hhmm: "));
    Serial.println(hhmm);
    Serial.print(F("LDR State = "));
    Serial.println(ldr_state);
  }
  
  if ((hhmm >= on_time) && (hhmm <= off_time))
  {
    x10_event_state=true;
    x10_state=x10_event(ON);   // set the initial x10 state if it's time and dark
  }
  else
  {
    x10_event_state=false;
    x10_state=x10_event(OFF);  // make sure we know what the init state is
  }
  
  lcd.setCursor(0,1);
  lcd.print(F("waiting for PIR"));
  Serial.println(F("waiting 35 sec for PIR..."));
  delay(35000);                 // give the PIR sensor time to stabilize
}

void loop() 
{    
  unsigned long currentMillis = millis();
  
  if (debug==true)
  {
    Serial.println();
    Serial.println(F("-- TOP LOOP --"));
  }

  String hhmm=get_str_time("");        // hhmm integer time
  pir_state=pir_read(pirPin);    // check motion detector
  ldr_state=check_LDR(ldrPin);   // check for dark or light

  if (debug==true) 
  {
    Serial.print(F("x10 ON time = "));
    Serial.print(on_time);
    Serial.print(F("  x10 OFF time = "));
    Serial.println(off_time);
    Serial.print(F("hhmm: "));
    Serial.println(hhmm);
    Serial.print(F("timed_delay = "));
    Serial.println(timed_delay);
    Serial.print(F("x10 state = "));
    Serial.println(x10_state);   
    Serial.print(F("PIR state = "));
    Serial.println(pir_state);   
    Serial.print(F("x10 event state = "));
    Serial.println(x10_event_state);   
  }

  // Turn on the x10 lamp module for a couple of minutes 
  // if there is motion, and it's dark, and the lamp is off.
  // This is checked often so the action is immediate.
  
  if (pir_state==HIGH)
  {
    if (debug==true) Serial.println(F("PIR=HIGH"));
    if (x10_state==LOW)
    {
      if (debug==true) Serial.println(F("x10=LOW"));
      if (ldr_state==DARK)
      {
        if (debug==true) Serial.println(F("LDR=LOW"));
        int LDRin=analogRead(ldrPin);                 // Read LDR value
        float volts_in=(LDRin/1023.0)*referenceVolts; // testing
        Serial.print(F("LDR Volts: "));               // testing
        Serial.println(volts_in);                     // testing
        Serial.print(F("--Begin time delay at: "));
        Serial.println(get_str_time(":"));
        x10_state=x10_event(ON);
        delayMillis = currentMillis;   // start time delay
        timed_delay=true;
        delay(1000);
      }
    }
  }

  if (timed_delay==true)
  {
    if(x10_state==HIGH)
    {
      if (currentMillis - delayMillis >= delayinterval)
      {   // turn off the x10 module after the timed delay 
          Serial.print(F("  Delay interval = "));
          Serial.println((currentMillis - delayinterval)/1000);
          delayMillis = currentMillis;      // save the last time you checked
          if (debug==true)
          {
            Serial.print(F("Free Memory: "));
            Serial.println(memoryFree());
          }
          Serial.print(F("--End time delay at: "));
          Serial.println(get_str_time(":"));
          x10_state=x10_event(OFF);
          delay(5000);
          timed_delay=false;
      }
    }
  }
  
  // this code gets executed every 10 seconds
  if (currentMillis - previousMillis2 >= interval2)
  { // toggle between time/humidity and date/time displays on LCD
    if (debug == true)
    {
      Serial.print(F("Free Memory: "));
      Serial.println(memoryFree());
    }
    
    previousMillis2 = currentMillis;  // save the last time you checked
    blink_led(LED1Pin, 250);          // blink LED to show activity
    if (disp_state==true)
    {
      display_time_lcd();
      disp_state=false;               // display time on LCD
    }
    else
    {
      disp_temperature_lcd();         // display temperature & humidity on LCD
      disp_state=true;
    }
  }

  // This code gets executed every 30 seconds
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;  // save the last time you checked x10
    if (debug==true) Serial.println("x10 event time checked at: "+get_str_time(":"));
    // Check the current time and the lamp-on time.  Turn the x10 module on
    // if it's between the on-time and off-time period and it's dark.
    if (x10_state==LOW)
    {
      if (hhmm >= on_time)
      {
        if (hhmm <= off_time)
        {
          if (ldr_state==DARK)
          {  
            x10_state=x10_event(ON);
            x10_event_state=true;
          }
        }
      }
    }
    // Turn off the x10 lamp module if the
    // current time = lamp-off time.
    if ((hhmm >= off_time) && (x10_event_state=true))
    {
      x10_state=x10_event(OFF); 
      x10_event_state=false;
    }
  }                   // end if currentMillis

  if (debug==true)
  { 
    Serial.println(F("--- END LOOP ---"));
    Serial.println();
  }

  if (debug==true) 
     delay(3000);
  else
     delay(1000);
}    	       		 // end main loop

// *************** Begin Subroutines **************

extern int __bss_end;
extern void *__brkval;

// funtion to return the amount of free RAM
int memoryFree()
{
  int freeValue;
  if ((int)__brkval==0)
    freeValue = ((int)&freeValue)-((int)&__bss_end);
  else
    freeValue = ((int)&freeValue)-((int)__brkval);

  return freeValue; 
}

void display_time_lcd()
{    
  DateTime now = rtc.now();         // read the RTC

  String lcd_line;
  String str_time=get_str_time(":");              // hh:mm.ss
  String today=daysOfTheWeek[now.dayOfTheWeek()]; // SAT-SUN
  lcd.clear();
  lcd.setCursor(0,0);
  lcd_line = today+"  "+get_str_date();
  lcd.print(lcd_line);      // Print day-date message to the LCD.
  if (debug==true) Serial.println(lcd_line);  // display to console
  lcd.setCursor(0,1);       // Print time message to the LCD.
  lcd_line = "Time: "+str_time;
  lcd.print(lcd_line);
  if (debug==true) Serial.println(lcd_line);  // display to console
  return;
}

String get_str_time(String fmc)
{
  DateTime now = rtc.now();         // read the RTC

  String str_time;                             // formatted time string
  String str_hr = String(now.hour());          // 0-24
  String str_minute = String(now.minute());    // 0-59
  String str_second = String(now.second());    // 0-59
  String str_month = String(now.month());      // 1-12
  String str_day = String(now.day());          // 1-31
  String str_year = String(now.year());        // yyyy
  str_hr = pad_l(str_hr, 2, "0");              // make hour digits 0-9 -> 00-09
  str_minute = pad_l(str_minute, 2, "0");      // make minute digits 0-9 -> 00-09
  str_month  = pad_l(str_month, 2, "0");       // make hour digits 0-9 -> 00-09
  str_second = pad_l(str_second, 2, "0");      // make seconds digits 0-9 -> 00-09
  if (fmc == "")
    str_time=str_hr+str_minute;                    // hhmm time string for comparisons
  else
    str_time=str_hr+fmc+str_minute+"."+str_second; // hh:mm.ss formatted time string for display
    
  return str_time;
}

String get_str_date()
{
  DateTime now = rtc.now();         // read the RTC

  String str_month = String(now.month());  // 1-12
  String str_day = String(now.day());      // 1-31
  String str_year = String(now.year());    // yyyy
  str_day = pad_l(str_day, 2, "0");        // make day no digits 0-9 -> 00-09
  str_month  = pad_l(str_month, 2, "0");   // make hour digits 0-9 -> 00-09
  return str_month+"/"+str_day+"/"+str_year;
}

byte pir_read(int pin)
{
  int val = digitalRead(pin);
  if (val == HIGH)
  {
    Serial.println(" *** Motion Detected at: "+get_str_time(":"));
    delay(1000);
  }
  return val;
}

byte x10_event(byte cmd)
{
	if (cmd==ON)
  { 
    Serial.print(F(" ---- x10 Lamp ON at: "));
    Serial.println(get_str_time(":"));
    x10_state=HIGH;
 	}
  if (cmd==OFF) 
  {
    Serial.print(F(" ---- x10 Lamp OFF at: "));
    Serial.println(get_str_time(":"));
    x10_state=LOW;
  }
    
	// send a command 3 times:
  myHouse.write(x10_HC, x10_unit, 3);
	myHouse.write(x10_HC, cmd, 3);
  blink_led(LED1Pin,60);         // blink LED 3x as visual indicator
  blink_led(LED1Pin,60);
  blink_led(LED1Pin,60);

  return x10_state;
}

byte check_LDR(int pin)
{
  int ldr_state;
  String state_str;
  float volts_in;
  int LDRin=analogRead(pin);   //Read LDR value
  volts_in=(LDRin/1023.0)*referenceVolts;
  if (debug==true) 
  {
    Serial.print(F("Count: "));
    Serial.print(LDRin);
    Serial.print(F(" LDR voltage: "));
    Serial.println(volts_in);
  }
  if (volts_in < darkVolts)
  {
    ldr_state = DARK;
    state_str = "DARK";
  }
  else
  {
    ldr_state = LIGHT;
    state_str = "LIGHT";
  }
    
  if (debug==true)
  {
    Serial.print(F("LDR State: "));
    Serial.println(state_str);
  }

  return ldr_state;   // return LDR state
}

void blink_led(int pin,int d)
{
   digitalWrite(pin,LOW); // LED on    
   delay(d);
   digitalWrite(pin,HIGH); // LED off    
   return;
}

void disp_temperature_lcd()
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) 
  {
    if (debug == true) Serial.println(F("Failed to read from DHT sensor!"));
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F(" DHT read fail! "));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  if (debug == true) 
  {
    Serial.print(t);
    Serial.print(F("°C  "));
    Serial.print(f);
    Serial.print(F("°F "));
    Serial.print(F(" Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Heat index: "));
    Serial.print(hic);
    Serial.print(F("°C  "));
    Serial.print(hif);
    Serial.println(F("°F"));
  }
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print(F(" Temp : "));
  lcd.print(f);
  lcd.print(F(" F"));
  lcd.setCursor(0,1);
  lcd.print(F(" Humid: "));
  lcd.print(h);
  lcd.print(F(" %"));
  return;
}

String pad_l(String in_str, int pad_len, String p_chr)
{ // add pad characters to left of input string
  in_str.trim();
  if (in_str.length() < pad_len)
  {
    for (int i=1; i <= (pad_len - in_str.length()); i++)
    {
      in_str = p_chr + in_str;
    }
  }
  return in_str;
}

String pad_r(String in_str, int pad_len, String p_chr)
{ // add pad characters to right of input string
  in_str.trim();
  if (in_str.length() < pad_len)
  {
    for (int i=1; i <= (pad_len - in_str.length()); i++)
    {
      in_str = in_str + p_chr;
    }
  }
  return in_str;
}

void display_timedate()
{
    DateTime now = rtc.now();

    Serial.println(F("+-----------------------+"));
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(F("  "));
    Serial.print(now.month(), DEC);
    Serial.print(F("/"));
    Serial.print(now.day(), DEC);
    Serial.print(F("/"));
    Serial.print(now.year(), DEC);
    Serial.print(F("   "));
    Serial.print(now.hour(), DEC);
    Serial.print(F(":"));
    Serial.print(now.minute(), DEC);
    Serial.print(F("."));
    Serial.println(now.second(), DEC);
    Serial.println(F("+-----------------------+"));
    Serial.println();
}
