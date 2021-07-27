#include "Adafruit_VL53L0X.h"
#include <LiquidCrystal_I2C.h>

#define MAX 1700  //  LONG-RANGE  для двух метров
//#define MAX 1100  //  для полутора метров
//#define MAX 1500  //  LONG-RANGE  для полутора метров

//#define DEBUG
//#define WIFI_DEBUG
//#define LCD_POSITIVE
#define WIFI
#define BATTERY_MODE
//#define DONT_SEND

#include <ESP8266WiFi.h>
#ifdef WIFI
#include <WiFiClient.h>
#endif

//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, NEGATIVE);  //  POSITIVE;
LiquidCrystal_I2C *lcd;
//bool flip = true;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool outOfRange = true;
unsigned counter = 0;

//char ssid[] = "DIR-615";         // your network SSID (name)
//char pass[] = "76543210";        // your network password
char ssid[] = "staff dream";         // your network SSID (name)
char pass[] = "89196902777";        // your network password
char wrongSsid[] = "DIR-6156";         // your network SSID (name)
char server[] = "parfum.nailmail.h1n.ru";
#ifdef WIFI
int status = WL_IDLE_STATUS;     // the Wifi radio's status
WiFiClient client;
#endif
unsigned long lastSentMillis = 0;
unsigned counterHttp = 0;
String httpAnswer;
bool wifiError = false;
bool wifiConnectionError = false;
bool lcdOn = true;
unsigned long lastLcdBlinkMillis = 0;
unsigned long wifiConnectionMillis = 0;
unsigned wifiErrorBlinkInterval = 1000 * 2;
unsigned wifiConnectionErrorBlinkInterval = 1000 / 3;
unsigned blinkInterval;
unsigned long wifiSendInterval = 60000 * 2 - 5000;             //  в режиме передачи это 2 минуты перед первым сеансом, а между неудачными сеансами установим в другом месте 10 минут
//unsigned long wifiRegularSendInterval = 595000;   //  в режиме передачи это десять минут между успешными сеансами
unsigned long wifiRegularSendInterval = 60000 * 30 - 5000;      //  в режиме передачи это 30 минут между успешными сеансами
unsigned long wifiConnectMillis;
bool firstSend = true;

void setup() {
  delay(5000); //  на всякий случай для предотвращения watchdog

  ESP.wdtDisable();
  
  pinMode(LED_BUILTIN, INPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(D3, INPUT);
  digitalWrite(D3, LOW);
  pinMode(D4, INPUT);
  digitalWrite(D4, LOW);
  pinMode(D5, INPUT);
  digitalWrite(D5, LOW);
  pinMode(D6, INPUT);
  digitalWrite(D6, LOW);
  pinMode(D7, INPUT);
  digitalWrite(D7, LOW);
  pinMode(D8, INPUT);
  digitalWrite(D8, LOW);
  Serial.begin(9600);
  while (!Serial);

//  VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&lox, 12000);
//  lox.setMeasurementTimingBudget(200000);
  if (!lox.begin(0x29)) {
    #ifdef DEBUG
    Serial.println(F("Failed to boot VL53L0X"));
    #endif
  }

  lcd = new LiquidCrystal_I2C(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  //  SDA = D2-GPIO4, SCL = D1-GPIO5
//  Serial.println((int)(*lcd), DEC);
  lcd->begin(16, 2);
  if (lcdOn)
  #ifdef LCD_POSITIVE
  lcd->backlight();
  #else
  lcd->noBacklight();
  #endif
  lcd->clear();
  lcd->print(F("Connecting"));

  #ifdef WIFI
  connectWifi(1);
  delay(1);
  #endif

//  clock_prescale_set(clock_div_256);
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  lastLcdBlinkMillis++;
  do
    lox.rangingTest(&measure, false);
  while (measure.RangeMilliMeter == 0);
//  lox.setTimeout(350);

  #ifdef DEBUG
  Serial.print(F("measure.RangeStatus = "));  //  восстановить
  Serial.println(measure.RangeStatus);        //  восстановить
  #endif // DEBUG
  if (measure.RangeStatus != 4 && measure.RangeMilliMeter < MAX) {
//  if (measure.RangeStatus != 4) {
//  if (measure.RangeStatus == 0 && measure.RangeMilliMeter < MAX) {
    #ifdef DEBUG
    Serial.print(F("Distance (mm): "));       //  восстановить
    Serial.println(measure.RangeMilliMeter);  //  восстановить
    #endif // DEBUG
    if (outOfRange) {
      //  добавил на месте 201902261611 нач
      do
        lox.rangingTest(&measure, false);
      while (measure.RangeMilliMeter == 0);
      //  добавил на месте 201902261611 кон
      //  добавил и условие 201902261611 нач
      if (measure.RangeStatus != 4 && measure.RangeMilliMeter < MAX)
      //  добавил и условие 201902261611 кон
        outOfRange = false;
    }
  }
  else {
    if (!outOfRange) {
      outOfRange = true;
      lcd->clear();
      #ifdef DEBUG
      Serial.println(F("lcd->clear()"));
      #endif  //  DEBUG
//      lcd->print(F("D="));
//      lcd->print(F("MAX+"));

      counter++;
      counterHttp++;
      #ifdef DEBUG
      Serial.println("counter = " + String(counter));
      Serial.println("counterHttp = " + String(counterHttp));
      #endif DEBUG

      lcd->setCursor(0, 1);
      lcd->print(F("C="));
      lcd->print(counter);
      if (wifiConnectionError) {
        lcd->setCursor(0, 0);
        lcd->print("WIFI ER1");
      }
      else if (wifiError) {
        lcd->setCursor(0, 0);
        lcd->print("WIFI ER2");
      }
      else {
        lcd->setCursor(0, 0);
        lcd->print("WIFI OK ");
      }
    }
  }
//  -------------------------------------------------
//  Вместо этого delay пробую организовать light-sleep примерно на то же время
//  delay(50);
  // For some reason, moving timer_list pointer to the end of the list lets us achieve light sleep
  extern os_timer_t *timer_list;
  while(timer_list != 0) {
    // doing the actual disarm doesn't seem to be necessary, and causes stuff to not work
    //  os_timer_disarm(timer_list);
    timer_list = timer_list->timer_next;
  }
  Serial.print("L_S");
  Serial.flush();

  wifi_set_opmode_current(NULL_MODE);
  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_set_wakeup_cb(wakeup_cb);
  //wifi_fpm_do_sleep(0xFFFFFF);  // works but requires interupt to wake up
  wifi_fpm_do_sleep(350 * 1000);
  delay (351);
//  -------------------------------------------------

  #ifdef WIFI
//  Serial.print("millis() - lastSentMillis = ");
//  Serial.println(millis() - lastSentMillis);

  #ifndef DONT_SEND
  if (millis() - lastSentMillis > wifiSendInterval) {
    //-------------------------------------------------
    //  сюда добавил, из колбэка убрал
    wifi_set_opmode(STATION_MODE);
    wifi_station_connect();
    //-------------------------------------------------
    connectWifi(1); //  !!!!!!! проба
    httpRequest();
    lastSentMillis = millis();
    if (!wifiError && !wifiConnectionError && httpAnswer.indexOf("200 OK") > -1) {
      counterHttp = 0;
      wifiSendInterval = wifiRegularSendInterval; //  30 минут
    }
    #ifdef WIFI_DEBUG
    Serial.print(httpAnswer.indexOf("200 OK"));
    #endif  //  WIFI_DEBUG
  }
  #endif  //  DONT_SEND

  #ifndef BATTERY_MODE
  if (wifiError || wifiConnectionError) {
    if (wifiConnectionError)
      blinkInterval = wifiConnectionErrorBlinkInterval;
    else
      blinkInterval = wifiErrorBlinkInterval;
    if (millis() - lastLcdBlinkMillis > blinkInterval) {
      if (lcdOn) {
        #ifdef LCD_POSITIVE
        lcd->noBacklight();
        #else
        lcd->backlight();
        #endif  //  LCD_POSITIVE
        lcdOn = false;
      }
      else {
        #ifdef LCD_POSITIVE
        lcd->backlight();
        #else
        lcd->noBacklight();
        #endif  //  LCD_POSITIVE
        lcdOn = true;
      }
      lastLcdBlinkMillis = millis();
    }
  }
  else if (!lcdOn) {
    #ifdef LCD_POSITIVE
    lcd->backlight();
    #else
    lcd->noBacklight();
    #endif  //  LCD_POSITIVE
    lcdOn = true;
  }
  #else //  #ifndef BATTERY_MODE
  if (wifiError || wifiConnectionError) {
    if (wifiConnectionError) {
      lcd->setCursor(0, 0);
      lcd->print("WIFI ER1");
    }
    else if (wifiError) {
      lcd->setCursor(0, 0);
      lcd->print("WIFI ER2");
    }
  }
  #endif  //  #ifndef BATTERY_MODE
  #endif  //  WIFI
}

void printWifiStatus()
{
  #ifdef WIFI
  #ifdef WIFI_DEBUG
  Serial.print(F("SSID: "));                 // print the SSID of the network you're attached to
  Serial.println(WiFi.SSID());
  #endif  //  WIFI_DEBUG

  IPAddress ip = WiFi.localIP();          // print your WiFi shield's IP address
  #ifdef WIFI_DEBUG
  Serial.print(F("IP Address: "));
  Serial.println(ip);
  #endif  //  WIFI_DEBUG

  long rssi = WiFi.RSSI();                // print the received signal strength
  #ifdef WIFI_DEBUG
  Serial.print(F("Signal strength (RSSI):"));
  Serial.print(rssi);
  Serial.println(F(" dBm"));
  #endif  //  WIFI_DEBUG
  #endif  //  WIFI
}

void httpRequest()                        // this method makes a HTTP connection to the server
{
  #ifdef WIFI
  #ifdef WIFI_DEBUG
  Serial.println();
  #endif  //  WIFI_DEBUG

  if (wifiConnectionError)
    connectWifi(1);

  if (!wifiConnectionError) {
    client.stop();                          // close any connection before send a new request // this will free the socket on the WiFi shield
  
    if (client.connect(server, 80)) {       // if there's a successful connection
      #ifdef WIFI_DEBUG
      Serial.println(F("Connecting..."));
      #endif  //  WIFI_DEBUG
      
      // send the HTTP PUT request
      client.println("GET /api.php?id=51&dt=201902261444&placeid=7&ins=" + String(counterHttp, DEC) + "&outs=" +String(counter, DEC) + " HTTP/1.1");
      client.println(F("Host: parfum.nailmail.h1n.ru"));
      client.println(F("Connection: close"));
      client.println();
      httpAnswer = client.readStringUntil('\r');
      #ifdef WIFI_DEBUG
      Serial.println(httpAnswer);
      #endif  //  WIFI_DEBUG
      wifiError = false;
      wifiSendInterval = wifiRegularSendInterval; //  30 минут
      firstSend = false;
    }
    else {
      #ifdef WIFI_DEBUG
      Serial.println(F("Connection failed"));  // if you couldn't make a connection
      #endif  //  WIFI_DEBUG
      wifiError = true;
      wifiSendInterval = 60000 * 10 - 5000; //  10 минут для повторной попытки
      connectWifi(1); //  восстановить
    }
  }
  else
    wifiSendInterval = 60000 * 10 - 5000; //  10 минут для повторной попытки
  #endif  //  WIFI
}

void connectWifi(int whichSsid) {
  #ifdef WIFI
  byte tryings = 0;
  lcd->clear();
  lcd->print(F("Connecting"));
//  wifi_set_opmode(WIFI_STA);
  WiFi.mode(WIFI_STA);
  if (whichSsid == 1)
    status = WiFi.begin(ssid, pass);
  else
    status = WiFi.begin(wrongSsid, pass);
  #ifdef WIFI_DEBUG
  Serial.println("");
  #endif  //  WIFI_DEBUG
  wifiConnectionMillis = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    tryings++;
    if (tryings == 7)
      lcd->setCursor(0, 1);
    lcd->print(F("."));
    Serial.print(".");
    if (millis() - wifiConnectionMillis > 7000) {
      break;
    }
  }
  lcd->clear();
  lcd->setCursor(0, 1);
  lcd->print(F("C="));
  lcd->print(counter);
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnectionError = false;
    lcd->setCursor(0, 0);
    lcd->print("WIFI OK ");
  }
  else {
    wifiConnectionError = true;
    lcd->setCursor(0, 0);
    lcd->print("WIFI ER1");
  }
  #ifdef WIFI_DEBUG
  Serial.println("");
  Serial.println("IP address: " + WiFi.localIP());
  #endif  //  WIFI_DEBUG
  printWifiStatus();
  #endif  //  WIFI
  delay(1);
}

void wakeup_cb() {
  wifi_fpm_close();
  //-------------------------------------------------
  //  Этот код я добавил, поскольку в оригинале WiFi отключен
//  wifi_set_opmode(STATION_MODE);
//  wifi_station_connect();
  //-------------------------------------------------
  //  А этот я закомментировал, т.к. Wi-Fi нам нужен
//  WiFi.forceSleepBegin();  // if you dont need wifi, ~22ma instead of ~75ma
  //-------------------------------------------------
  
  // required here, otherwise the delay after sleep will not be interrupted, thanks vlast3k
  // see: #6 from https://github.com/esp8266/Arduino/issues/1381#issuecomment-279117473
  Serial.printf("wakeup_cb\n");
  Serial.flush();
}

