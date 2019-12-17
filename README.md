# An easy to use/configure ESP8266 multisensor/switch firmware supporting OTA updates
Frimware for full blown ESP8266 (NodeMCU, Wemos D1, ...) that supports multiple temperature sensors, relays and inputs. Uses Shelly (TM) like APIfor easy integration into home automation software.

### Features:
- Configure WiFi on the fly
- Enable/disable sensors, relays, inputs on first use
- Restart and/or reset ESP to initial state via MQTT message
- Set-up MQTT topic used by home automation SW ([compatible with shellies/# topic](https://shelly-api-docs.shelly.cloud/#shelly-family-overview))
- Over the air updates supported

### Instructions:
- Use 64k SPIFFS or larger (WiFi Manager configuration) when uploading
- Required libraries are all available from within Arduino IDE Library manager
- During first use (or if selected WiFi is not available) configure ESP by connecting to ESP-xxxxxx SSID and navigate to http://192.168.4.1
- Update sensors/relays via MQTT messages:
  - [MQTTtopic]/D1mini-xxxxxx/temperature/command 
  - [MQTTtopic]/D1mini-xxxxxx/relay/[i]/command 
- Sensor/relay status is reported every minute to the topic:
  - [MQTTtopic]/D1mini-xxxxxx/sensor/temperature `DHT sensor`
  - [MQTTtopic]/D1mini-xxxxxx/sensor/temperature_f `DHT sensor`
  - [MQTTtopic]/D1mini-xxxxxx/sensor/humidity `DHT sensor`
  - [MQTTtopic]/D1mini-xxxxxx/temperature[/[i]] `Dallas DS18B20 sensors`
  - [MQTTtopic]/D1mini-xxxxxx/relay/[i] `relays 1-4`
  - [MQTTtopic]/D1mini-xxxxxx/sensor/motion  `PIR sensor`
  - [MQTTtopic]/D1mini-xxxxxx/emeter/0/voltage `analog input`
  - [MQTTtopic]/D1mini-xxxxxx/input/0 `digital input`
- Sensor/relay pins are fixed but are chosen so that they don't interfer with ESP boot process (Wemos D1 mini layout)
- If you want more than 4 relays, you can sacrifice sensors not used and update relays & relayStates arrays
