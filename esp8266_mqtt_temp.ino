/*
 ESP8266 MQTT client for reporting DHT11 temperature

 Requires ESP8266 board, Adafruit Universal and DHT Sensor libraries

 (c) blaz@kristan-sp.si / 2019-09-10
*/

#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#define DEBUG 1

// analog input (A0)
#define ANALOG 1  // enable analog input

// digital input  // general purpose input 
#define DINPUT D0
int DInputState = 0;

/*
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONEWIRE D2          // use pin D2
#define TEMPERATURE_PRECISION 9
OneWire oneWire(ONEWIRE);   // on pin D2 (a 4.7K pull-up resistor is necessary)
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses (up to 16)
DeviceAddress *thermometers[16];
int numSensors = 0;
*/

// PIR sensor or button/switch pin & state
#define PIR     D3      // pin used for PIR (default D3)
int PIRState = 0;       // initialize PIR state

///*
// DHT type temperature/humidity sensors
#include "DHT.h"
#define DHTPIN  D4      // what pin we're connected to (default D4)
#define DHTTYPE DHT11   // DHT11, DHT22
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor
float tempAdjust = 0.0; // temperature adjustment for wacky DHT sensors (retrieved from EEPROM)
//*/

// relay pins
#define RELAYS 4  //number of relays used (max 4)
const int relays[4] = {D5, D6, D7, D8}; // relay pins
int relayState[4] = {0, 0, 0, 0};       // relay states (retrieved from EEPROM in setup()

// Update these with values suitable for your network.
char mqtt_server[40] = "192.168.70.11";
char mqtt_port[7]    = "1883";
char username[33]    = "";
char password[33]    = "";
char MQTTBASE[16]    = "shellies"; // use shellies for Shelly MQTT Domoticz plugin integration

// flag for saving data from WiFiManager
bool shouldSaveConfig = false;

long lastMsg = 0;
char msg[50];
char mac_address[16];
char inTopic[64];   // add MAC address in WiFi setup code
char outTopic[64];  // add MAC address in WiFi setup code
char clientId[20];  // MQTT client ID

WiFiClient espClient;
PubSubClient client(espClient);

// private functions
void reconnect();
char *ftoa(float,char*,int d=2);
void saveConfigCallback ();


//---------------------------------------------------
// MQTT callback function
void callback(char* topic, byte* payload, unsigned int length) {
#if DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
#endif

  // Switch on the LED if an 1 was received as first character
  if ( strstr(topic,"/led/command") ) {
    if ( strncmp((char*)payload,"on",length)==0 ) {
      digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
      // but actually the LED is on; this is because
      // it is active low on the ESP-01)
    } else if ( strncmp((char*)payload,"off",length)==0 ) {
      digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
    }
#if RELAYS
  } else if ( strstr(topic,"/relay/") && strstr(topic,"/command") ) {
    // topic contains relay command

    int relayId = (int)(*(strstr(topic,"/command")-1) - '0');  // get the relay id
    if ( relayId < RELAYS ) {
      if ( strncmp((char*)payload,"on",length)==0 ) {
        // message is on
        digitalWrite(relays[relayId], HIGH);  // Turn the relay on
        relayState[relayId] = 1;
      } else if ( strncmp((char*)payload,"off",length)==0 ) {
        // message is off
        digitalWrite(relays[relayId], LOW);  // Turn the relay off
        relayState[relayId] = 0;
      }

      // publish relay state
      sprintf(outTopic, "%s/%s/relay/%i", MQTTBASE, clientId, relayId);
      sprintf(msg, relayState[relayId]?"on":"off");
      client.publish(outTopic, msg);

      // permanently store relay states to nonvolatile memory
      int b=0;
      for ( int i=RELAYS; i>0; i-- ) {
        b = (b<<1) | (relayState[i-1]&1);
      }
      EEPROM.begin(10);
      EEPROM.write(9,b);
      EEPROM.commit();
      EEPROM.end();
#if DEBUG
      Serial.println(b,HEX);
      Serial.print("Message sent [");
      Serial.print(outTopic);
      Serial.print("] ");
      Serial.println(msg);
#endif
    }
#endif
#ifdef DHTTYPE
  } else if ( strstr(topic,"/temperature/command") ) {
    // insert temperature adjustment and store it into non volatile memory
    
    char tmp[8];
    strncpy(tmp,(char*)payload,length>7? 7: length);
    tmp[length>7? 7: length] = '\0'; // terminate string
    float temp = atof(tmp);
    if ( temp < 100.0 && temp > -100.0 ) {
      tempAdjust = temp;
      ftoa(tempAdjust, tmp, 2);
      EEPROM.begin(10);
      EEPROM.put(0, tmp);
      EEPROM.commit();
      EEPROM.end();
#if DEBUG
      Serial.print("Temperature adjust: ");
      Serial.println(tempAdjust, DEC);
#endif
    }
#endif
  }
}

//-----------------------------------------------------------
// main setup
void setup() {
  Serial.begin(115200);

  // Initialize the BUILTIN_LED pin as an output & set initial state LED on
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
  
  String WiFiMAC = WiFi.macAddress();
  WiFiMAC.replace(":","");
  WiFiMAC.toCharArray(mac_address, 16);
  // Create client ID from MAC address
  sprintf(clientId, "D1mini-%s", &mac_address[6]);

#if DEBUG
  Serial.println("");
  Serial.print("Host name: ");
  Serial.println(WiFi.hostname());
  Serial.print("MAC address: ");
  Serial.println(mac_address);
  Serial.println("Starting WiFi manager");
#endif

//----------------------------------------------------------
  //read configuration from FS json
#if DEBUG
  Serial.println("mounting FS...");
#endif

  if ( SPIFFS.begin() ) {
#if DEBUG
    Serial.println("mounted file system");
#endif
    if ( SPIFFS.exists("/config.json") ) {
      //file exists, reading and loading
#if DEBUG
      Serial.println("reading config file");
#endif
      File configFile = SPIFFS.open("/config.json", "r");
      if ( configFile ) {
#if DEBUG
        Serial.println("opened config file");
#endif
        size_t size = configFile.size();
        
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        
        DynamicJsonDocument doc(size);
        DeserializationError error = deserializeJson(doc, buf.get());
        if ( !error ) {
#if DEBUG
          serializeJson(doc, Serial);
          Serial.println("\nparsed json");
#endif
          strcpy(mqtt_server, doc["mqtt_server"]);
          strcpy(mqtt_port, doc["mqtt_port"]);
          strcpy(username, doc["username"]);
          strcpy(password, doc["password"]);
          strcpy(password, doc["base"]);
        } else {
#if DEBUG
          Serial.println("failed to load json config");
#endif
        }
      }
    } else {
#if DEBUG
      Serial.println("formatting FS");
#endif
      //clean FS, for testing
      SPIFFS.format();
    }
  } else {
#if DEBUG
    Serial.println("failed to mount FS");
#endif
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
  WiFiManagerParameter custom_username("username", "username", username, 32);
  WiFiManagerParameter custom_password("password", "password", password, 32);
  WiFiManagerParameter custom_mqtt_base("base", "MQTT topic base", MQTTBASE, 15);

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  // reset settings (for debugging)
  //wifiManager.resetSettings();
  
  // set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_username);
  wifiManager.addParameter(&custom_password);
  wifiManager.addParameter(&custom_mqtt_base);

  // set minimu quality of signal so it ignores AP's under that quality
  // defaults to 8%
  //wifiManager.setMinimumSignalQuality(10);
  
  // sets timeout until configuration portal gets turned off
  // useful to make it all retry or go to sleep
  // in seconds
  //wifiManager.setTimeout(120);

  // fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect(clientId)) {
#if DEBUG
    Serial.println("failed to connect and hit timeout");
#endif
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  //if you get here you have connected to the WiFi

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(username, custom_username.getValue());
  strcpy(password, custom_password.getValue());
  strcpy(MQTTBASE, custom_mqtt_base.getValue());

  //save the custom parameters to FS
  if ( shouldSaveConfig ) {
#if DEBUG
    Serial.println("saving config");
#endif
    DynamicJsonDocument doc(1024);
    doc["mqtt_server"] = mqtt_server;
    doc["mqtt_port"] = mqtt_port;
    doc["username"] = username;
    doc["password"] = password;
    doc["base"] = MQTTBASE;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    } else {
#if DEBUG
      serializeJson(doc, Serial);
#endif
      serializeJson(doc, configFile);
      configFile.close();

      // clear 10 bytes from EEPROM
      char str[10]="0.0\0\0\0\0\0\0\0";
      EEPROM.begin(10);
      EEPROM.put(10, str);
      EEPROM.commit();
      EEPROM.end();
      delay(120);
    }
  }
//----------------------------------------------------------

  // initialize WiFi manager and auto-connect to previous network or create a hotspot
//  WiFiManager wifiManager;
//  wifiManager.autoConnect(clientId);

  // if we get here we are connected to a wifi network
#if DEBUG
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#endif
  
  // if connected set state LED off
  digitalWrite(BUILTIN_LED, HIGH);

  // request 10 bytes from EEPROM
  EEPROM.begin(10);

#if DEBUG
  Serial.println("");
  Serial.print("EEPROM data: ");
  char str[11];
  EEPROM.get(0,str);
  for ( int i=0; i<10; i++ ) {
//    str[i] = EEPROM.read(i);
    Serial.print(str[i], HEX);
    Serial.print(":");
  }
  str[10] = '\0';
  Serial.print(" (");
  Serial.print(str);
  Serial.println(")");
#endif

#ifdef DINPUT
  pinMode(DINPUT, INPUT);
#endif

#if RELAYS
  // initialize relay pins
#if DEBUG
  Serial.print("Relay states: ");
#endif
  // 10th byte contain 8 relay worth of initial states
  int initRelays = EEPROM.read(9);
  // relay states are stored in bits
  for (int i=0; i<RELAYS; i++ ) {
    pinMode(relays[i], OUTPUT);
    relayState[i] = (initRelays >> 1) & 1;
    digitalWrite(relays[i], relayState[i]? HIGH: LOW);
#if DEBUG
    Serial.print(relayState[i],DEC);
#endif
  }
#if DEBUG
  Serial.println("");
#endif
#endif

#ifdef DHTTYPE
  // initialize DHT sensor
  dht.begin();
  char temp[8];
  EEPROM.get(0,temp);
  tempAdjust = atof(temp);  // temperature adjustment (set via MQTT message)
#if DEBUG
  Serial.print("Temperature adjust: ");
  Serial.println(tempAdjust, DEC);
#endif
#endif

#ifdef ONEWIRE
  // OneWire temperature sensors
  byte *tmpAddr, addr[8];

  // Start up the library
  sensors.begin();

  // locate devices on the bus
#if DEBUG
  Serial.println("Locating OneWire devices...");
  Serial.print("Found ");
  Serial.print(numSensors=sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
#else
  numSensors = sensors.getDeviceCount();
#endif
  oneWire.reset_search();
  for ( int i=0; !oneWire.search(addr) && i<16; i++ ) {
#if DEBUG
      Serial.print("Sensor found: ");
      for (uint8_t j = 0; j < 8; j++) {
        // zero pad the address if necessary
        if (addr[j] < 16) Serial.print("0");
        Serial.print(addr[j], HEX);
      }
      Serial.println("");
#endif
    if (OneWire::crc8(addr, 7) != addr[7])
      Serial.println("CRC error.");
      thermometers[i] = null;
    else {
      // store up to 16 sensor addresses
      tmpAddr = (byte*)malloc(8);
      memcpy(tmpAddr,addr,8);
      sensors.setResolution(tmpAddr, TEMPERATURE_PRECISION);
      thermometers[i] = (DeviceAddress *)tmpAddr;
    }
  }
#endif

  // done reading EEPROM
  EEPROM.end();

  // initialize MQTT connection & provide callback function
  client.setServer(mqtt_server, atoi(mqtt_port));
  client.setCallback(callback);
}

//-----------------------------------------------------------
// main loop
void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // publish status every 60s
  long now = millis();
  if (now - lastMsg > 60000) {
    lastMsg = now;
    
#ifdef DHTTYPE
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    float f = dht.readTemperature(true);
    
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println("Failed to read from DHT sensor!");
      sprintf(outTopic, "%s/%s/sensor/temperature", MQTTBASE, clientId);
      sprintf(msg, "err");
    } else {
#if DEBUG
      Serial.print("DHT temperature: ");
      Serial.print(t);
      Serial.print("C; humidity: ");
      Serial.print(h);
      Serial.println("%");
#endif
      // may use Shelly MQTT API (shellies/shellyht-MAC/sensor/temperature)
      sprintf(outTopic, "%s/%s/sensor/temperature", MQTTBASE, clientId);
      sprintf(msg, "%.2f", t + tempAdjust);
      client.publish(outTopic, msg);
      sprintf(outTopic, "%s/%s/sensor/temperature_f", MQTTBASE, clientId);
      sprintf(msg, "%.2f", f + tempAdjust*(float)(9/5));
      client.publish(outTopic, msg);
      sprintf(outTopic, "%s/%s/sensor/humidity", MQTTBASE, clientId);
      sprintf(msg, "%.2f", h);
      client.publish(outTopic, msg);
    }
#endif

#ifdef ONEWIRE
    // may use non-standard Shelly MQTT API (shellies/shellyhtx-MAC/sensor/temperature/i)
    for ( int i=0; i<numThermometers; i++ ) {
      if ( *thermometers[i] ) {
        // not a faulty thermometer
        float tempC = sensors.getTempC(*thermometers[i]);
#if DEBUG
        Serial.print("OneWire ");
        Serial.print(i);
        Serial.print(" (");
        for (uint8_t j = 0; j < 8; j++) {
          // zero pad the address if necessary
          if ((*thermometers[i])[j] < 16) Serial.print("0");
          Serial.print((*thermometers[i])[j], HEX);
        }
        Serial.print(") : temperature=");
        Serial.print(tempC);
        Serial.println("C");
#endif
        sprintf(outTopic, "%s/%s/temperature/%i", MQTTBASE, clientId, i);
        sprintf(msg, "%.2f", tempC);
        client.publish(outTopic, msg);
      }
    }
#endif

#if RELAYS
    // may use Shelly MQTT API (shellies/shelly4pro-MAC/relay/i)
    for ( int i=0; i < RELAYS; i++ ) {
      sprintf(outTopic, "%s/%s/relay/%i", MQTTBASE, clientId, i);
      sprintf(msg, relayState[i]?"on":"off");
      client.publish(outTopic, msg);
    }
#endif

#ifdef DINPUT
    // may use Shelly MQTT API as (shellies/shelly1-MAC/input/i)
    sprintf(outTopic, "%s/%s/input/%i", MQTTBASE, clientId, 0);
    client.publish(outTopic, DInputState == HIGH? "1": "0");
#endif

#ifdef PIR
    // may use Shelly MQTT API as (shellies/shellysense-MAC/sensor/motion)
    sprintf(outTopic, "%s/%s/sensor/motion", MQTTBASE, clientId);
    client.publish(outTopic, PIRState == HIGH? "1": "0");
#endif

#ifdef ANALOG
    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.2V):
    char voltage[10];
    sprintf(outTopic, "%s/%s/analog", MQTTBASE, clientId);
    sprintf(msg, "%1.3f", (float)sensorValue / 1023.0);
    client.publish(outTopic, msg);
    // may use Shelly MQTT API as (shellies/shellyem-MAC/emeter/i/voltage)
    sprintf(outTopic, "%s/%s/emeter/0/voltage", MQTTBASE, clientId);
    sprintf(msg, "%1.3f", (float)sensorValue * (3.2 / 1023.0));
    client.publish(outTopic, msg);
    //client.publish(outTopic, ftoa(sensorValue * (3.2 / 1023.0), voltage, 3));
#if DEBUG
    // print out the value you read:
    Serial.print("Analog voltage: ");
    Serial.println(voltage);
#endif
#endif

  // end 60s reporting
  }

#ifdef DINPUT
  // detect digital input cahnge and publish immediately
  int lDState = digitalRead(DINPUT);
  if (lDState != DInputState ) {
    DInputState = lDState;
    sprintf(outTopic, "%s/%s/input/%i", MQTTBASE, clientId, 0);
    client.publish(outTopic, DInputState == HIGH? "1": "0");
#if DEBUG
    Serial.print("Digital input: ");
    Serial.println(DInputState == HIGH? "1": "0");
#endif
  }
#endif

#ifdef PIR
  // detect motion and publish immediately
  int lPIRState = digitalRead(PIR);
  if (lPIRState != PIRState ) {
    PIRState = lPIRState;
    sprintf(outTopic, "%s/%s/sensor/motion", MQTTBASE, clientId);
    client.publish(outTopic, PIRState == HIGH? "1": "0");
#if DEBUG
    Serial.print("PIR motion: ");
    Serial.println(PIRState == HIGH? "on": "off");
#endif
  }
#endif

  // wait 2s until next update (also DHT limitation)
  delay(2000);
}

//----------------------------------------------------
// MQTT reconnect handling
void reconnect() {
  // Loop until we're reconnected
  while ( !client.connected() ) {
#if DEBUG
    Serial.print("Attempting MQTT connection...");
#endif
    // Attempt to connect
    if ( strlen(username)==0? client.connect(clientId): client.connect(clientId, username, password) ) {
      // Once connected, publish an announcement...
      sprintf(outTopic, "%s/%s/announce", MQTTBASE, clientId);
      sprintf(msg, "Hello there. My IP is %s", WiFi.localIP().toString().c_str());
      client.publish(outTopic, msg);
      // ... and resubscribe
      sprintf(inTopic, "%s/%s/#", MQTTBASE, clientId);
      client.subscribe(inTopic);
#if DEBUG
      Serial.println("connected & subscribed");
      Serial.println(inTopic);
#endif
    } else {
#if DEBUG
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
#endif
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// reverses a string 'str' of length 'len' 
void reverse(char *str, int len) 
{ 
    int i=0, j=len-1, temp; 
    while (i<j) 
    { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; j--; 
    } 
} 

// Converts a given integer x to string str.  d is the number 
// of digits required in output. If d is more than the number 
// of digits in x, then 0s are added at the beginning. 
int intToStr(int x, char *str, int d) 
{ 
    int i = 0, s = x<0;
    while (x) 
    { 
      str[i++] = (abs(x)%10) + '0'; 
      x = x/10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d)
      str[i++] = '0';

    if ( s )
      str[i++] = '-';
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
  
// Converts a floating point number to string. 
char *ftoa(float n, char *res, int afterpoint) 
{ 
  // Extract integer part 
  int ipart = (int)n; 
  
  // Extract floating part 
  float fpart = n - (float)ipart; 
  
  // convert integer part to string 
  int i = intToStr(ipart, res, 0); 
  
  // check for display option after point 
  if (afterpoint != 0) 
  { 
    res[i] = '.';  // add dot 

    // Get the value of fraction part upto given no. 
    // of points after dot. The third parameter is needed 
    // to handle cases like 233.007 
    fpart = fpart * pow(10, afterpoint); 

    intToStr(abs((int)fpart), res + i + 1, afterpoint); 
  }
  return res;
} 

//callback notifying us of the need to save config
void saveConfigCallback () {
#if DEBUG
  Serial.println("Should save config");
#endif
  shouldSaveConfig = true;
}
