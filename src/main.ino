#ifdef ESP32
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#error Platform not supported
#endif
#include <WiFiClientSecure.h>

#include <PubSubClient.h>
//#define MQTT_MAX_PACKET_SIZE 512 //maybe only on arduino ide modify MQTT_MAX_PACKET_SIZE 256 to 512

#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson (use v6.xx)
#include <time.h>
#define emptyString String()

//Follow instructions from https://github.com/debsahu/ESP-MQTT-AWS-IoT-Core/blob/master/doc/README.md
//Enter values in secrets.h ▼
#include "secrets.h"

#if !(ARDUINOJSON_VERSION_MAJOR == 6 and ARDUINOJSON_VERSION_MINOR >= 7)
#error "Install ArduinoJson v6.7.0-beta or higher"
#endif

const int MQTT_PORT = 8883;
const char MQTT_SUB_TOPIC[] = "$aws/things/"THINGNAME"/shadow/update/accepted";

const char MQTT_SUB_TOPIC_DELTA[] = "$aws/things/"THINGNAME"/shadow/update/delta";

const char MQTT_SUB_TOPIC_GET[] = "$aws/things/"THINGNAME"/shadow/get/accepted";

const char MQTT_PUB_TOPIC[] = "$aws/things/"THINGNAME"/shadow/update";

const char MQTT_PUB_TOPIC_GET[] = "$aws/things/"THINGNAME"/shadow/get";

//$aws/things/espThing/shadow/update

#ifdef USE_SUMMER_TIME_DST
uint8_t DST = 1;
#else
uint8_t DST = 0;
#endif

int led_pin = 2;

BearSSL::WiFiClientSecure net;

#ifdef ESP8266
BearSSL::X509List cert(cacert);
BearSSL::X509List client_crt(client_cert);
BearSSL::PrivateKey key(privkey);
#endif

PubSubClient client(net);

unsigned long lastMillis = 0;
time_t now;
//time_t nowish = 1510592825;
time_t nowish = 8 * 3600 * 2;


void reportState(int lightState)
{
   char buffer[50];
   sprintf(buffer, "{\"state\": {\"reported\": {\"lights\": %d},\"desired\": null}}", lightState);
  if (!client.publish(MQTT_PUB_TOPIC, buffer, false))
    pubSubErr(client.state());
}

void NTPConnect(void)                                                         //npt connect
{
  Serial.print("Setting time using SNTP");
  configTime(TIME_ZONE * 3600, DST * 3600, "1.cn.pool.ntp.org", "2.cn.pool.ntp.org", "3.cn.pool.ntp.org");
  now = time(nullptr);
  while (now < nowish)
  {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("done!");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}

void messageReceived(char *topic, byte *payload, unsigned int length)       //message recieved
{
  String msgIN = "";
  for (int i=0;i<length;i++)
  {
    msgIN += (char)payload[i];
  }
  String msgString = msgIN;
  Serial.println("Recieved [" + String(topic) + "]: "+ msgString);
  
  StaticJsonDocument<200> doc;
  StaticJsonDocument<64> filter;
  
//******************************************************

  if(strcmp(topic, MQTT_SUB_TOPIC_GET) == 0) 
  {
    filter["state"]["reported"]["lights"] = true;
    DeserializationError error = deserializeJson(doc,msgString, DeserializationOption::Filter(filter));
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    
    if(doc["state"]["desired"]["lights"]==false){  
      digitalWrite(led_pin,HIGH);                //Led Turned off on high signal as it is active low
      reportState(doc["state"]["desired"]["lights"]);
    }
    else if(doc["state"]["desired"]["lights"]==true){ 
      digitalWrite(led_pin,LOW);               //Led Turned on by low signal as it is active low
      reportState(doc["state"]["desired"]["lights"]);
    }
    else if(doc["state"]["reported"]["lights"]==false){  
      digitalWrite(led_pin,HIGH);                //Led Turned off on high signal as it is active low
      reportState(doc["state"]["reported"]["lights"]);
    }
    else if(doc["state"]["reported"]["lights"]==true){ 
      digitalWrite(led_pin,LOW);               //Led Turned on by low signal as it is active low
      reportState(doc["state"]["reported"]["lights"]);
    }
    else 
    {
      reportState(0);// send default state
    }
    
    Serial.println("get/accepted");

    
    return;
  }
  else if(strcmp(topic, MQTT_SUB_TOPIC_DELTA) == 0) // update to reported: state, desired: null
  {
    filter["state"]["lights"] = true;
    DeserializationError error = deserializeJson(doc,msgString, DeserializationOption::Filter(filter));
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }

    if(doc["state"]["lights"]==false){  
      digitalWrite(led_pin,HIGH);                //Led Turned off on high signal as it is active low
      reportState(doc["state"]["lights"]);
    }
    else if(doc["state"]["lights"]==true){ 
      digitalWrite(led_pin,LOW);               //Led Turned on by low signal as it is active low
      reportState(doc["state"]["lights"]);
    }
    else 
    {
      reportState(0);// send default state
    }
    
    Serial.println("update/delta");
  }
  else if(strcmp(topic, MQTT_SUB_TOPIC) == 0)
  {
    filter["state"]["desired"]["lights"] = true;
    DeserializationError error = deserializeJson(doc,msgString, DeserializationOption::Filter(filter));
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    Serial.println("update/accepted");
  }
  else
  {
    Serial.println("IDK what to do");
  }


//***********************************************


  Serial.println("end of sub feed back");
  return;
  Serial.println("end of sub feed back check");
 
  
  filter["state"]["reported"]["lights"] = true;
  filter["state"]["reported"]["lights"] = true;
  DeserializationError error = deserializeJson(doc,msgString, DeserializationOption::Filter(filter));
  
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  /*
{
  "state": {
    "reported": {
      "lights": false
    },
    "desired": null
  }
}

{
  "state": {
    "desired": {
      "lights": false
    }
  }
}




  */
  
  if(doc["state"]["desired"]["lights"]==false){  
    digitalWrite(led_pin,HIGH);                //Led Turned off on high signal as it is active low
  }  
  if(doc["state"]["desired"]["lights"]==true){ 
    digitalWrite(led_pin,LOW);               //Led Turned on by low signal as it is active low
  }
  Serial.println();
}

void pubSubErr(int8_t MQTTErr)                                              //pub sub err
{
  if (MQTTErr == MQTT_CONNECTION_TIMEOUT)
    Serial.print("Connection tiemout");
  else if (MQTTErr == MQTT_CONNECTION_LOST)
    Serial.print("Connection lost");
  else if (MQTTErr == MQTT_CONNECT_FAILED)
    Serial.print("Connect failed");
  else if (MQTTErr == MQTT_DISCONNECTED)
    Serial.print("Disconnected");
  else if (MQTTErr == MQTT_CONNECTED)
    Serial.print("Connected");
  else if (MQTTErr == MQTT_CONNECT_BAD_PROTOCOL)
    Serial.print("Connect bad protocol");
  else if (MQTTErr == MQTT_CONNECT_BAD_CLIENT_ID)
    Serial.print("Connect bad Client-ID");
  else if (MQTTErr == MQTT_CONNECT_UNAVAILABLE)
    Serial.print("Connect unavailable");
  else if (MQTTErr == MQTT_CONNECT_BAD_CREDENTIALS)
    Serial.print("Connect bad credentials");
  else if (MQTTErr == MQTT_CONNECT_UNAUTHORIZED)
    Serial.print("Connect unauthorized");
}

void connectToMqtt(bool nonBlocking = false)                                        //connect to mqtt
{
  Serial.print("MQTT connecting ");
  while (!client.connected())
  {
    if (client.connect(THINGNAME))
    {
      Serial.println("connected!");
     // if (!client.subscribe(MQTT_SUB_TOPIC))
        //pubSubErr(client.state());
      if (!client.subscribe(MQTT_SUB_TOPIC_GET))
        pubSubErr(client.state());
      if (!client.subscribe(MQTT_SUB_TOPIC_DELTA))
        pubSubErr(client.state());
    }
    else
    {
      Serial.print("failed, reason -> ");
      pubSubErr(client.state());
      if (!nonBlocking)
      {
        Serial.println(" < try again in 5 seconds");
        delay(5000);
      }
      else
      {
        Serial.println(" <");
      }
    }
    if (nonBlocking)
      break;
  }
}

void connectToWiFi(String init_str)                                            //connect to wifi
{
  if (init_str != emptyString)
    Serial.print(init_str);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  if (init_str != emptyString)
    Serial.println("ok!");
}

void checkWiFiThenMQTT(void)                                                  //check wifi then mqtt
{
  connectToWiFi("Checking WiFi");
  connectToMqtt();
}

unsigned long previousMillis = 0;
const long interval = 5000;

void checkWiFiThenMQTTNonBlocking(void)                                       //check wifi then mqtt nonblocking
{
  connectToWiFi(emptyString);
  if (millis() - previousMillis >= interval && !client.connected()) {
    previousMillis = millis();
    connectToMqtt(true);
  }
}

void checkWiFiThenReboot(void)                                               //checck wifi then reboot
{
  connectToWiFi("Checking WiFi");
  Serial.print("Rebooting");
  ESP.restart();
}

void sendData(void)                                                         //send data
{
  DynamicJsonDocument jsonBuffer(JSON_OBJECT_SIZE(3) + 100);
  JsonObject root = jsonBuffer.to<JsonObject>();
  JsonObject state = root.createNestedObject("state");
  JsonObject state_reported = state.createNestedObject("reported");
  state_reported["lights"] = !digitalRead(led_pin);
  Serial.printf("Sending  [%s]: ", MQTT_PUB_TOPIC);
  serializeJson(root, Serial);
  Serial.println();
  char shadow[measureJson(root) + 1];
  serializeJson(root, shadow, sizeof(shadow));
  if (!client.publish(MQTT_PUB_TOPIC, shadow, false))
    pubSubErr(client.state());
}

void setup()                                                                              //setup
{
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);
  Serial.println();
  Serial.println();
#ifdef ESP32
  WiFi.setHostname(THINGNAME);
#else
  WiFi.hostname(THINGNAME);
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  connectToWiFi(String("Attempting to connect to SSID: ") + String(ssid));

  NTPConnect();

#ifdef ESP32
  net.setCACert(cacert);
  net.setCertificate(client_cert);
  net.setPrivateKey(privkey);
#else
  net.setTrustAnchors(&cert);
  net.setClientRSACert(&client_crt, &key);
#endif

  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(messageReceived);

  connectToMqtt();

  client.publish(MQTT_PUB_TOPIC_GET, "{}", false);
}

void loop()                                                                   //loop
{
  now = time(nullptr);
  if (!client.connected())
  {
    checkWiFiThenMQTT();
    //checkWiFiThenMQTTNonBlocking();
    //checkWiFiThenReboot();
  }
  else
  {
    client.loop();
//    if (millis() - lastMillis > 10000)
//    {
//      lastMillis = millis();
//      sendData();
//    }
  }
}
