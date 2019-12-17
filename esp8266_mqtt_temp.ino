/*
 ESP8266 MQTT client for reporting DHT11 temperature

 Requires ESP8266 board, Adafruit Universal and DHT Sensor libraries

 (c) blaz@kristan-sp.si / 2019-09-10
*/

// NOTE: Enable SPIFFS on the board (64k minimum)
#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>          // multicast DNS
#include <WiFiUdp.h>              // UDP handling
#include <ArduinoOTA.h>           // OTA updates
#include <EEPROM.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#define MQTT_MAX_PACKET_SIZE 1024
#include <PubSubClient.h>

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"

#define DEBUG 1

// general purpose digital inputs (pin D1)
int D1PIN = D1;
int D1InputState = 0;   // pin D1 (0==not in use, 1==in use; 2==reserved)

// PIR sensor or button/switch pin & state
int PIRPIN = D3;        // pin used for PIR (default D3 for Wemos D1 mini shield; D3 must NOT be LOW on boot)
int PIRState = 0;       // initialize PIR state

// NOTE: You cannot use both DHT & Dallas temperature sensors at the same time

// Dallas DS18B20 temperature sensors
#define TEMPERATURE_PRECISION 9
#define ONEWIREPIN D4   // on pin D4 (a 4.7K pull-up resistor is necessary) (default D2 for Wemos D1 mini shield)
OneWire *oneWire = NULL;
DallasTemperature *sensors = NULL;
DeviceAddress thermometers[10];  // array to hold device addresses (up to 10)
int numThermometers = 0;

// DHT type temperature/humidity sensors
#define DHTPIN D4       // what pin we're connected to (default D4 for Wemos D1 mini shield; D4 must NOT be LOW at boot)
float tempAdjust = 0.0; // temperature adjustment for wacky DHT sensors (retrieved from EEPROM)
DHT *dht = NULL;        // DHT11, DHT22, DHT21, none(0)

// relay pins
const int relays[4] = {D5, D6, D7, D0}; // relay pins (may use D8 for fourth if not using analog input; D8 must be LOW at boot!)
int relayState[4] = {0, 0, 0, 0};       // relay states (retrieved from EEPROM in setup()
int numRelays = 0;                      // number of relays used
// uncomment next line for saving relay states to EEPROM
//#define EEPROMSAVE 1

// Update these with values suitable for your network.
char mqtt_server[40] = "192.168.70.11";
char mqtt_port[7]    = "1883";
char username[33]    = "";
char password[33]    = "";
char MQTTBASE[16]    = "test"; // use 'shellies' for Shelly API (ShellyMQTT Domoticz plugin integration)

char c_relays[2]     = "0";
char c_dhttype[8]    = "none";
char c_d1enabled[2]  = "0";
char c_pirsensor[2]  = "0";
char c_analog[2]     = "0";

// flag for saving data from WiFiManager
bool shouldSaveConfig = false;

long lastMsg = 0;
char msg[256];          // MQTT message buffer
char mac_address[16];   // add MAC address in WiFi setup code
char outTopic[64];      // MQTT topic buffer
char clientId[20];      // MQTT client ID ([esp8266|esp01|D1mini|...]_MACaddress)

WiFiClient espClient;
PubSubClient client(espClient);

// private functions
void mqtt_callback(char*, byte*, unsigned int);
void mqtt_reconnect();
char *ftoa(float,char*,int d=2);
void saveConfigCallback();


//-----------------------------------------------------------
// main setup
void setup() {
  char str[11];

  Serial.begin(115200);
  delay(3000);

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
          strcpy(MQTTBASE, doc["base"]);

          strcpy(c_relays, doc["relays"]);
          strcpy(c_dhttype, doc["dhttype"]);
          strcpy(c_pirsensor, doc["pirsensor"]);
          strcpy(c_d1enabled, doc["d1enabled"]);
          strcpy(c_analog, doc["analog"]);
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

  WiFiManagerParameter custom_relays("relays", "Number of relays (0-4; D5-D7,D0)", c_relays, 1);
  WiFiManagerParameter custom_dhttype("dhttype", "Temp. sensor (D4) type (DHT11,DHT22,DS18B20,none)", c_dhttype, 7);
  WiFiManagerParameter custom_pirsensor("pirsensor", "PIR sensor enabled (D pin#)", c_pirsensor, 1);
  WiFiManagerParameter custom_d1enabled("d1enabled", "Digital input enabled (D pin#)", c_d1enabled, 1);
  WiFiManagerParameter custom_analog("analog", "Analog input enabled (0/1)", c_analog, 1);

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

  wifiManager.addParameter(&custom_relays);
  wifiManager.addParameter(&custom_dhttype);
  wifiManager.addParameter(&custom_pirsensor);
  wifiManager.addParameter(&custom_d1enabled);
  wifiManager.addParameter(&custom_analog);

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
  #if DEBUG
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  #endif
  

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(username, custom_username.getValue());
  strcpy(password, custom_password.getValue());
  strcpy(MQTTBASE, custom_mqtt_base.getValue());

  strcpy(c_relays, custom_relays.getValue());
  strcpy(c_dhttype, custom_dhttype.getValue());
  strcpy(c_pirsensor, custom_pirsensor.getValue());
  strcpy(c_d1enabled, custom_d1enabled.getValue());
  strcpy(c_analog, custom_analog.getValue());

  c_analog[0] = '0' + max(min(atoi(c_analog),1),0); // sanity check
  #if DEBUG
  Serial.print("Analog input enabled: ");
  Serial.println(c_analog);
  #endif

  numRelays = max(min(atoi(c_relays),4),0); // sanity check
  c_relays[0] = '0' + numRelays;
  #if DEBUG
  Serial.print("Number of relays: ");
  Serial.println(numRelays, DEC);
  #endif

  if ( strcmp(c_dhttype,"DHT11")==0 ) {
    dht = new DHT(DHTPIN, DHT11);
  } else if ( strcmp(c_dhttype,"DHT21")==0 ) {
    dht = new DHT(DHTPIN, DHT21);
  } else if ( strcmp(c_dhttype,"DHT22")==0 ) {
    dht = new DHT(DHTPIN, DHT22);
  } else if ( strcmp(c_dhttype,"DS18B20")==0 ) {
    oneWire = new OneWire(ONEWIREPIN);          // no need to free this one
    sensors = new DallasTemperature(oneWire);   // no need to free this one, too
  } else {
    strcpy(c_dhttype,"none");
  }

  switch ( c_d1enabled[0] ) {
    case '1' : D1PIN = D1; break;
    case '2' : D1PIN = D2; break;
    case '3' : D1PIN = D3; break;
    case '4' : D1PIN = D4; break;
    case '5' : D1PIN = D5; break;
    case '6' : D1PIN = D6; break;
    case '7' : D1PIN = D7; break;
    default  : c_d1enabled[0]='0'; break;
  }

  switch ( c_pirsensor[0] ) {
    case '3' : PIRPIN = D3; break;
    case '4' : PIRPIN = D4; break;
    case '5' : PIRPIN = D5; break;
    case '6' : PIRPIN = D6; break;
    case '7' : PIRPIN = D7; break;
    default  : c_pirsensor[0]='0'; break;
  }

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

    doc["relays"] = c_relays;
    doc["dhttype"] = c_dhttype;
    doc["pirsensor"] = c_pirsensor;
    doc["d1enabled"] = c_d1enabled;
    doc["analog"] = c_analog;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    } else {
      #if DEBUG
      serializeJson(doc, Serial);
      #endif
      serializeJson(doc, configFile);
      configFile.close();
    }

    // clear 10 bytes from EEPROM
    EEPROM.begin(10);
    EEPROM.put(0, "esp0.0\0\0\0\0");
    EEPROM.commit();
    EEPROM.end();
    delay(120);
    #if DEBUG
    Serial.println("saved EEPROM");
    #endif
  }
//----------------------------------------------------------

  // if connected set state LED off
  digitalWrite(BUILTIN_LED, HIGH);

  // request 10 bytes from EEPROM
  EEPROM.begin(10);

  #if DEBUG
  Serial.println("");
  Serial.print("EEPROM data: ");
  #endif

  for ( int i=0; i<10; i++ ) {
    str[i] = EEPROM.read(i);
    #if DEBUG
    Serial.print(str[i], HEX);
    Serial.print(":");
    #endif
  }
  
  #if DEBUG
  Serial.print(" (");
  Serial.print(str);
  Serial.println(")");
  #endif

  if ( atoi(c_d1enabled) ) {
    pinMode(D1PIN, INPUT);
  }

  if ( numRelays > 0 ) {
    // initialize relay pins
    #if DEBUG
    Serial.print("Relay states: ");
    #endif
    #ifdef EEPROMSAVE
    // 10th byte contain 8 relays (bits) worth of initial states
    int initRelays = EEPROM.read(9);
    #endif
    // relay states are stored in bits
    for ( int i=0; i<numRelays; i++ ) {
      pinMode(relays[i], OUTPUT);
      #ifdef EEPROMSAVE
      relayState[i] = (initRelays >> 1) & 1;
      #endif
      digitalWrite(relays[i], relayState[i]? HIGH: LOW);
      #if DEBUG
      Serial.print(relayState[i],DEC);
      #endif
    }
    #if DEBUG
    Serial.println("");
    #endif
  }

  // set up temperature sensor (either DHT or Dallas)
  if ( strcmp(c_dhttype,"none")!=0 ) {
    
    if ( dht ) {
      // initialize DHT sensor
      dht->begin();
      if ( strncmp(str,"esp",3)==0 ) {
        tempAdjust = atof(&str[3]);  // temperature adjustment (set via MQTT message)
        #if DEBUG
        Serial.print("Temperature adjust: ");
        Serial.println(tempAdjust, DEC);
        #endif
      }
    } // DHT
  
    if ( oneWire ) {
      // Start up the library
      sensors->begin();
    
      // locate devices on the bus
      numThermometers = sensors->getDeviceCount();
      #if DEBUG
      Serial.println("Locating OneWire devices...");
      Serial.print("Found ");
      Serial.print(numThermometers, DEC);
      Serial.println(" devices.");
      // report parasite power requirements
      Serial.print("Parasite power is: ");
      if (sensors->isParasitePowerMode()) Serial.println("ON");
      else Serial.println("OFF");
      #endif
      for ( int i=0; i<numThermometers; i++ ) {
        if ( !sensors->getAddress(thermometers[i], i) ) {
          numThermometers = i;
          break;
        } else {
          sensors->setResolution(thermometers[i], TEMPERATURE_PRECISION);
        }
      }
/*
      oneWire->reset_search();
      for ( int i=0; !oneWire->search(&thermometers[i]) && i<10; i++ ) {
        #if DEBUG
        Serial.print("Sensor found: ");
        for ( uint8_t j = 0; j < 8; j++ ) {
          // zero pad the address if necessary
          if ((byte*)thermometers[i][j] < 16) Serial.print("0");
          Serial.print((byte*)thermometers[i][j], HEX);
        }
        Serial.println("");
        #endif
        if ( OneWire::crc8(thermometers[i], 7) != (byte*)thermometers[i][7] ) {
          #if DEBUG
          Serial.println("CRC error.");
          #endif
        } else {
          // store up to 10 sensor addresses
          tmpAddr = (byte*)malloc(8); // there is no need to free() this memory
          memcpy(tmpAddr,addr,8);
          sensors->setResolution(tmpAddr, TEMPERATURE_PRECISION);
          thermometers[numThermometers++] = (DeviceAddress *)tmpAddr;
        }
      }
*/
    } // end oneWire
  } // end temperature sensors

  // done reading EEPROM
  EEPROM.end();

  if ( atoi(c_analog) ) {
    // pin to power ADC for resistive soil moisture meter
    pinMode(D8, OUTPUT);
  }

  // OTA update setup
  //ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname(clientId);
  //ArduinoOTA.setPassword("ota_password");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    #if DEBUG
    Serial.println("Start updating " + type);
    #endif
  });
  ArduinoOTA.onEnd([]() {
    #if DEBUG
    Serial.println("\nEnd");
    #endif
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    #if DEBUG
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
  });
  ArduinoOTA.onError([](ota_error_t error) {
    #if DEBUG
    Serial.printf("Error[%u]: ", error);
    if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)     Serial.println("End Failed");
    #endif
  });
  ArduinoOTA.begin();

  // initialize MQTT connection & provide callback function
  client.setServer(mqtt_server, atoi(mqtt_port));
  client.setCallback(mqtt_callback);
}

//-----------------------------------------------------------
// main loop
void loop() {
  long now = millis();

  // handle OTA updates
  ArduinoOTA.handle();

  if (!client.connected()) {
    mqtt_reconnect();
  }
  client.loop();

  // publish status every 60s
  if (now - lastMsg > 60000) {
    lastMsg = now;
    
    if ( dht ) {
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      float h = dht->readHumidity();
      // Read temperature as Celsius (the default)
      float t = dht->readTemperature();
      // Read temperature as Fahrenheit (isFahrenheit = true)
      float f = dht->readTemperature(true);
      
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
        sprintf(msg, "%.1f", t + tempAdjust);
        client.publish(outTopic, msg);
        sprintf(outTopic, "%s/%s/sensor/temperature_f", MQTTBASE, clientId);
        sprintf(msg, "%.1f", f + tempAdjust*(float)(9/5));
        client.publish(outTopic, msg);
        sprintf(outTopic, "%s/%s/sensor/humidity", MQTTBASE, clientId);
        sprintf(msg, "%.1f", h);
        client.publish(outTopic, msg);
        if ( round(tempAdjust*10) != 0.0 ) {
          sprintf(outTopic, "%s/%s/temp_adjust", MQTTBASE, clientId);
          sprintf(msg, "%.1f", tempAdjust);
          client.publish(outTopic, msg);
        }
      }
    }

    if ( oneWire ) {
      // may use non-standard Shelly MQTT API (shellies/shellyhtx-MAC/temperature/i)
      for ( int i=0; i<numThermometers; i++ ) {
        float tempC = sensors->getTempC(thermometers[i]);
        #if DEBUG
        Serial.print("OneWire ");
        Serial.print(i);
        Serial.print(" (");
        for (uint8_t j = 0; j < 8; j++) {
          // zero pad the address if necessary
          if ((thermometers[i])[j] < 16) Serial.print("0");
          Serial.print((thermometers[i])[j], HEX);
        }
        Serial.print(") : temperature=");
        Serial.print(tempC);
        Serial.println("C");
        #endif
        if ( numThermometers == 1 )
          sprintf(outTopic, "%s/%s/temperature", MQTTBASE, clientId);
        else
          sprintf(outTopic, "%s/%s/temperature/%i", MQTTBASE, clientId, i);
        sprintf(msg, "%.1f", tempC);
        client.publish(outTopic, msg);
      }
    }

    // may use Shelly MQTT API (shellies/shelly4pro-MAC/relay/i)
    for ( int i=0; i<numRelays; i++ ) {
      sprintf(outTopic, "%s/%s/relay/%i", MQTTBASE, clientId, i);
      sprintf(msg, relayState[i]?"on":"off");
      client.publish(outTopic, msg);
    }

    if ( atoi(c_d1enabled) ) {
      // may use Shelly MQTT API as (shellies/shelly1-MAC/input/i)
      sprintf(outTopic, "%s/%s/input/%i", MQTTBASE, clientId, 0);
      client.publish(outTopic, D1InputState == HIGH? "1": "0");
    }

    if ( atoi(c_pirsensor) ) {
      // may use Shelly MQTT API as (shellies/shellysense-MAC/sensor/motion)
      sprintf(outTopic, "%s/%s/sensor/motion", MQTTBASE, clientId);
      client.publish(outTopic, PIRState == HIGH? "1": "0");
    }

    if ( atoi(c_analog) ) {
      // to overcome electrolysis on resistive soil moisture meter we will use D8 pin to power the AD converter/sensor
      digitalWrite(D8, HIGH);   // power ON (may need to add transistor to the pin)
      delay(500);               // wait a bit to power the ADC circuit
      // read the input on analog pin 0:
      int sensorValue = analogRead(A0);
      digitalWrite(D8, LOW);    // power OFF
      
      // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.2V):
      char voltage[10];
      sprintf(outTopic, "%s/%s/analog", MQTTBASE, clientId);
      sprintf(msg, "%0.3f", (float)sensorValue / 1023.0);
      client.publish(outTopic, msg);
  
      // may use Shelly MQTT API as (shellies/shellyem-MAC/emeter/i/voltage)
      sprintf(outTopic, "%s/%s/emeter/0/voltage", MQTTBASE, clientId);
      sprintf(msg, "%0.3f", (float)sensorValue * (3.2 / 1023.0));
      client.publish(outTopic, msg);
      //client.publish(outTopic, ftoa(sensorValue * (3.2 / 1023.0), voltage, 3));
      #if DEBUG
      // print out the value you read:
      Serial.print("Analog voltage: ");
      Serial.print(msg);
      Serial.println("V");
      #endif
    }

  }  // end 60s reporting

  if ( atoi(c_d1enabled) ) {
    // detect digital input cahnge and publish immediately
    int lDState = digitalRead(D1PIN);
    if (lDState != D1InputState ) {
      D1InputState = lDState;
      sprintf(outTopic, "%s/%s/input/%i", MQTTBASE, clientId, 0);
      client.publish(outTopic, D1InputState == HIGH? "1": "0");
      #if DEBUG
      Serial.print("Digital input D1: ");
      Serial.println(D1InputState == HIGH? "1": "0");
      #endif
    }
  }

  if ( atoi(c_pirsensor) ) {
    // detect motion and publish immediately
    int lPIRState = digitalRead(PIRPIN);
    if ( lPIRState != PIRState ) {
      PIRState = lPIRState;
      sprintf(outTopic, "%s/%s/sensor/motion", MQTTBASE, clientId);
      client.publish(outTopic, PIRState == HIGH? "1": "0");
      #if DEBUG
      Serial.print("PIR motion: ");
      Serial.println(PIRState == HIGH? "on": "off");
      #endif
    }
  }

  // wait 250ms until next update (also DHT limitation)
  delay(250);
}

//---------------------------------------------------
// MQTT callback function
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  char tmp[80];
  boolean lRestart = false;

  payload[length] = '\0'; // "just in case" fix

  // convert to String & int for easier handling
  String newPayload = String((char *)payload);

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

  } else if ( strstr(topic,"/relay/") && strstr(topic,"/command") ) {
    // topic contains relay command

    int relayId = (int)(*(strstr(topic,"/command")-1) - '0');  // get the relay id (0-3)
    if ( relayId < numRelays ) {
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
      #if DEBUG
      Serial.print("Message sent [");
      Serial.print(outTopic);
      Serial.print("] ");
      Serial.println(msg);
      #endif

      #ifdef EEPROMSAVE
      // permanently store relay states to nonvolatile memory
      int b=0;
      for ( int i=numRelays; i>0; i-- ) {
        b = (b<<1) | (relayState[i-1]&1);
      }
      EEPROM.begin(10);
      EEPROM.write(9,b);
      EEPROM.commit();
      EEPROM.end();
      #if DEBUG
      Serial.print("Relay EEPROM save: ");
      Serial.println(b,BIN);
      #endif
      #endif
    }

  } else if ( strstr(topic,"/temperature/command") ) {
    // insert temperature adjustment and store it into non volatile memory (wonky DHT sensors)
    
    char tmp[8];
    strncpy(tmp,(char*)payload,length>7? 7: length);
    tmp[length>7? 7: length] = '\0'; // terminate string
    float temp = atof(tmp);
    if ( temp < 100.0 && temp > -100.0 ) {
      tempAdjust = temp;
      sprintf(tmp, "esp%3.1f", tempAdjust);
      EEPROM.begin(10);
      for ( int i=0; i<9; i++ ) {
        EEPROM.write(i, tmp[i]);
      }
      EEPROM.commit();
      EEPROM.end();
      #if DEBUG
      Serial.print("Temperature adjust: ");
      Serial.println(tempAdjust, DEC);
      #endif
    }

  } else if ( strstr(topic,"/command/restart") ) {

    // restart ESP
    ESP.reset();
    delay(1000);
    
  } else if ( strstr(topic,"/command/reset") ) {

    // erase 10 bytes from EEPROM
    EEPROM.begin(10);
    for ( int i=0; i<10; i++ ) {
      EEPROM.write(i, '\0');
    }
    EEPROM.commit();
    EEPROM.end();
    delay(120);  // wait for write to complete

    // clean FS
    SPIFFS.format();

    // clear WiFi data & disconnect
    WiFi.disconnect();

    // restart ESP
    ESP.reset();
    delay(1000);
    
  }
}

//----------------------------------------------------
// MQTT reconnect handling
void mqtt_reconnect() {
  char inTopic[64];
  
  // Loop until we're reconnected
  while ( !client.connected() ) {
    #if DEBUG
    Serial.println("Attempting MQTT connection...");
    #endif
    // Attempt to connect
    if ( strlen(username)==0? client.connect(clientId): client.connect(clientId, username, password) ) {
      // Once connected, publish an announcement...
      DynamicJsonDocument doc(256);
      doc["mac"] = WiFi.macAddress(); //.toString().c_str();
      doc["ip"] = WiFi.localIP().toString();  //.c_str();
      doc["relays"] = c_relays;
      doc["dhttype"] = c_dhttype;
      doc["analog"] = c_analog;
      doc["pirsensor"] = c_pirsensor;
      doc["d1enabled"] = c_d1enabled;
      doc["tempadjust"] = ftoa(tempAdjust, inTopic, 2);
      
      size_t n = serializeJson(doc, msg);
      #if DEBUG
      Serial.println(msg);
      #endif
      sprintf(outTopic, "%s/%s/announce", MQTTBASE, clientId);
      client.publish(outTopic, msg, n);
      
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
