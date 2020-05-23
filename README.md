# DHT22-x10-PIR-RTC
  Author:      Tom Simpson
  Date:        05/05/2020
  File:        DHT_LCD_X10-2.INO
  
  Description
     Uses a DHT22 sensor to measure temperature and humidity. A 16x2 LCD display
     switches between temperature/humidity and day/date/time info every 10 seconds. 
     Also monitors a LDR and Paralax PIR sensor and controls a lamp using x10 controller 
     and lamp module to turn a light on at a designated time each day. All other times,
     turn on the lamp for a short period when motion is detected and it's dark. 
     The scheduled lamp activation is delayed until it is dark. The time and date is 
     kept via a DS1307 RTC with battery backup.
  
  Revision History
     TAS 05/05/2020 Initial file submission 
     TAS 05/21/2020 fix time comparisons
