#define USING_THINGSPEAK  1
#define USING_WIFI        1
#define USING_LCD         0
#define SENSE_TEMPERATURE 0
#define SENSE_LIGHT       1
#define SENSE_BAROMETER   1
#define SENSE_BATTERY     1

#include <My_Password.h>    // containing personal wifi/thingspeak/etc password settings
#include <Time.h>
#include <LBattery.h>
#include <Wire.h>
#include <Arduino.h>

#if SENSE_TEMPERATURE==1
  #include <DHT_linkit.h>
#endif

#if SENSE_LIGHT==1
  #include <Digital_Light_TSL2561.h>
#endif

#if SENSE_BAROMETER==1
  #include <HP20x_dev.h>
  #include <KalmanFilter.h>
#endif

#if USING_LCD==1
  #include <rgb_lcd.h>
#endif

#if USING_WIFI==1
  #include <LWiFi.h>
  #include <LWiFiClient.h>  
  #ifndef _WIFI_
    #define WIFI_AP "default_AP"            // replace your WiFi AP SSID
    #define WIFI_PASSWORD "default_password"  // replace your WiFi AP password
    #define WIFI_AUTH LWIFI_OPEN           // choose from LWIFI_OPEN, LWIFI_WPA, or LWIFI_WEP according to your AP
  #endif
  LWiFiClient client;
  LWifiStatus wifistatus;
  // Variable Setup
  long lastConnectionTime = 0; 
  boolean lastConnected = false;
  int failedCounter = 0;
#endif

#if USING_THINGSPEAK==1
  #ifndef _THINGSPEAK_API_KEY_
    #define THINGSPEAK_API_KEY "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
  #endif
  const int updateThingSpeakInterval = 3 * 1000;      // Time interval in milliseconds to update ThingSpeak (number of seconds * 1000 = interval)
#endif

#if SENSE_TEMPERATURE==1
  // Grove - Temperature and Humidity Sensor Pro
  // Interface: Analog (or Digital)
  // http://www.seeedstudio.com/wiki/Grove_-_Temperature_and_Humidity_Sensor_Pro
  // for LinkIt One, use the library: https://github.com/Seeed-Studio/Grove_Starter_Kit_For_LinkIt/tree/master/libraries/Humidity_Temperature_Sensor
  #define DHTPIN 2     // what pin we're connected to
  #define DHTTYPE DHT22   // DHT 22  (AM2302)
  DHT_linkit dht(DHTPIN, DHTTYPE);
#endif

#if SENSE_BAROMETER==1
  // Grove - Barometer (High-Accuracy)
  // Interface: I2C
  // http://www.seeedstudio.com/wiki/Grove_-_Barometer_(High-Accuracy)
  unsigned char ret = 0;

  KalmanFilter t_filter;    //temperature filter
  KalmanFilter p_filter;    //pressure filter
  KalmanFilter h_filter;    //altitude filter
#endif

#if USING_LCD==1
  //====================
  // Grove - LCD RGB Backlight
  // Interface: I2C
  // http://www.seeedstudio.com/wiki/Grove_-_LCD_RGB_Backlight
  rgb_lcd lcd;
#endif


void setup()
{
    Serial.begin(9600);
#if USING_LCD==1
    lcd.begin(16, 2);
    lcd.setRGB(0,0,0);
#endif

#if SENSE_TEMPERATURE==1
    // Grove - Temperature and Humidity Sensor Pro
    dht.begin();
#endif


#if SENSE_LIGHT==1
    // Grove - Digital Light Sensor
    TSL2561.init();
#endif

#if SENSE_BAROMETER==1
    // Grove - Barometer (High-Accuracy)
    // Power up,delay 150ms,until voltage is stable
    delay(150);
    HP20x.begin();  // Reset HP20x_dev
    delay(100);
  
    // Determine HP20x_dev is available or not 
    ret = HP20x.isAvailable();
    if(OK_HP20X_DEV == ret){
      Serial.println("HP20x_dev is available.");    
    } else {
      Serial.println("HP20x_dev isn't available.");
    }
#endif
    
#if USING_WIFI==1
    LWiFi.begin();
#endif
}

void loop()
{
int v_int=0;
long v_long=0;
long light=0;
float ba=0, bp=0;
float h=0,t=0;

#if USING_LCD==1
    lcd.setRGB(0,0,0);
#endif

#if SENSE_LIGHT==1
    // Grove - Digital Light Sensor
    light = TSL2561.readVisibleLux();
    #if USING_LCD==1
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("light: " + String(light));

      if(light<750){
        lcd.setRGB(random(128)+random(128), random(128)+random(128), random(128)+random(128));
      } else {
        lcd.setRGB(0,0,0);
      }
    #endif
    delay(2000);
#endif

#if SENSE_TEMPERATURE==1
    // Grove - Temperature and Humidity Sensor Pro
    // use the library: https://github.com/Seeed-Studio/Grove_Starter_Kit_For_LinkIt/tree/master/libraries/Humidity_Temperature_Sensor    
    dht.readHT(&t, &h);
    while (isnan(t) || isnan(h) || t<0 || t>80 || h<0 || h > 100){
      Serial.println("Something wrong with DHT => retry it!");
      delay(100);
      dht.readHT(&t, &h);    
    }
    t = t_filter.Filter(t);
    h = h_filter.Filter(h);
 
    #if USING_LCD==1
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("temp: " + String(t));
    
      lcd.setCursor(0,1);
      lcd.print("humid: " + String(h));
    #endif
    // Wait a bit between measurements.
    delay(2000);
#endif

#if SENSE_BAROMETER==1
    // Grove - Barometer (High-Accuracy)
    if(OK_HP20X_DEV == ret){ 
      //v_long = HP20x.ReadTemperature();
      //float bt = v_long/100.0;
      while(1){
        v_long = HP20x.ReadPressure();
        bp = v_long/100.0;
        v_long = HP20x.ReadAltitude();
        ba = v_long/100.0;
        if (bp>10) break;
        Serial.println("Something wrong with barometer => retry it!");
      }
      bp = p_filter.Filter(bp);

      #if USING_LCD==1
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("pressure: " + String(bp));
        lcd.setCursor(0, 1);
        lcd.print("altitude: " + String(ba));
      #endif
      delay(2000);
    }
#endif

#if SENSE_BATTERY==1
    // Show battery status
    v_int = LBattery.level();
    #if USING_LCD==1
      if (v_int < 50) { lcd.setRGB(255,0,0); }
      lcd.clear();
      if (LBattery.isCharging()){
        lcd.setCursor(0, 0);
        lcd.print(".. Charging ..");
      } else {
        lcd.setCursor(0, 0);
        lcd.print("On Battery");
      }
      lcd.setCursor(0, 1);
      lcd.print("Battery: " + String(v_int, DEC) + "%");
    #endif
    delay(2000);
#endif

#if USING_THINGSPEAK==1
    // upload data to thingspeak
    // reference: https://github.com/iobridge/ThingSpeak-Arduino-Examples/blob/master/Ethernet/Arduino_to_ThingSpeak.ino
    while (client.available()){
      char c = client.read();
      Serial.print(c);
    }

    // Disconnect from ThingSpeak
    if (!client.connected() && lastConnected){
      Serial.println("...disconnected");
      Serial.println();
      client.stop();
    }
  
    // Update ThingSpeak
    if((millis() - lastConnectionTime > updateThingSpeakInterval)){
      Serial.println("about to update ThingSpeak...");
        updateThingSpeak_wifi("field1="+String(t)+"&field2="+String(h)+"&field3="+String(bp)+"&field4="+String(light)+"&field5="+String(v_int));
    }
  
    // Check if LWiFi needs to be restarted
    if (failedCounter > 3 ) { 
      #if USING_WIFI==1
        start_WiFi(); 
      #endif
    }
  
    lastConnected = client.connected();
#endif

}

//=======================================
// reset WiFi connection
//=======================================
void start_WiFi(){
    if (client.connected()) { 
      client.stop(); 
    }

    wifistatus = LWiFi.status();
    if (wifistatus!=LWIFI_STATUS_DISABLED && failedCounter>3){
        Serial.println("before LWiFi.end()...");
        LWiFi.end();
        LWiFi.begin();
        failedCounter = 0;
        Serial.println("after LWiFi.start()..."); 
    } else if (wifistatus==LWIFI_STATUS_DISABLED){
        LWiFi.begin();
        Serial.println("after LWiFi.start()..."); 
    }
    
    Serial.println("Connecting to WiFi AP: " + String(WIFI_AP));
    Serial.println(String(hour(),DEC) + ":" + String(minute(),DEC) + ":" + String(second(),DEC) + "  start start_WiFi()");



    if (LWiFi.connect(WIFI_AP, LWiFiLoginInfo(WIFI_AUTH, WIFI_PASSWORD))<=0){
      Serial.println("WiFi connection failed.....");
    }
    
    wifistatus = LWiFi.status();
    if (wifistatus == LWIFI_STATUS_DISABLED){
      failedCounter++;
      Serial.println("WiFi status: LWIFI_STATUS_DISABLED");
    } else if (wifistatus == LWIFI_STATUS_DISCONNECTED) {
      failedCounter++;
      Serial.println("WiFi status: LWIFI_STATUS_DISCONNECTED");
    } else {
      Serial.println("WiFi status: LWIFI_STATUS_CONNECTED");
      Serial.print("My IP is: ");
      Serial.println(LWiFi.localIP());
      Serial.print("RSSI is: ");
      Serial.println(LWiFi.RSSI());
 //     failedCounter = 0;
    }
}

//=======================================
//  update ThingSpeak
//  Reference: https://github.com/iobridge/ThingSpeak-Arduino-Examples/blob/master/Ethernet/Arduino_to_ThingSpeak.ino
//=======================================
void updateThingSpeak_wifi(String tsData){

  wifistatus = LWiFi.status();
  if (wifistatus == LWIFI_STATUS_DISABLED){
    Serial.println("=> WiFi status: LWIFI_STATUS_DISABLED");
    start_WiFi();
  } else if (wifistatus == LWIFI_STATUS_DISCONNECTED) {
    Serial.println("=> WiFi status: LWIFI_STATUS_DISCONNECTED");
    start_WiFi();
  } else {
    Serial.println("=> WiFi status: LWIFI_STATUS_CONNECTED");
    Serial.print("=> My IP is: ");
    Serial.println(LWiFi.localIP());
    Serial.print("RSSI is: ");
    Serial.println(LWiFi.RSSI());
  }
    
  if (wifistatus != LWIFI_STATUS_CONNECTED){
    failedCounter++;
    Serial.println("Did not try updateThingSpeak because it's not connected....("+String(failedCounter, DEC)+")");
    Serial.println();
    return;
  }
  
  if (client.connect("api.thingspeak.com", 80)){         
    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: "+THINGSPEAK_API_KEY+"\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(tsData.length());
    client.print("\n\n");
    client.print(tsData);
    
    lastConnectionTime = millis();
    
    if (client.connected()){
      Serial.println("Connecting to ThingSpeak succeeded!");      
      failedCounter = 0;
    } else {
      failedCounter++;
      Serial.println("Connection to ThingSpeak failed ("+String(failedCounter, DEC)+")");   
    }
  } else {
    failedCounter++;  
    Serial.println("Connection to ThingSpeak Failed (("+String(failedCounter, DEC)+"))");   
    lastConnectionTime = millis(); 
  }
}
