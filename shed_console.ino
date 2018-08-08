/***************************************************
  Adafruit MQTT Library Ethernet Example

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Alec Moore
  Derived from the code written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>
#include <Adafruit_Sensor.h>

#include <DHT.h>
#include <DHT_U.h>

/************************* Adafruit.io Setup *********************************/
#define VERSION_MESSAGE F("Shed Console v0.13 29/07/18")

#define AIO_SERVER      "192.168.2.20"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "mosquitto"
#define AIO_KEY         "qq211"

#define DHT_PIN 7
#define DHT_VCC_PIN 2
#define LIGHT_PIN 6
#define LEDLIGHT_PIN 5

#define WILL_FEED AIO_USERNAME "/feeds/nodes.shed"
#define SERVER_LISTEN_PORT 80
#define MQTT_CONNECT_RETRY_MAX 5
#define MQTT_PING_INTERVAL_MS 60000

byte mac[] = {0xBE, 0xBD, 0xEE, 0xEF, 0xFE, 0xFD};
//IPAddress iotIP (192, 168, 0, 103);

// Uncomment the type of sensor in use:
#define DHT_TYPE DHT11     // DHT 22 (AM2302)
DHT_Unified dht(DHT_PIN, DHT_TYPE);

unsigned long sensorDelayMs;
unsigned long lastSensorRead = 0;
unsigned long lastPing = 0; // timestamp
unsigned long connectedSince = 0; // timestamp
unsigned long now = 0; // timestamp
unsigned long nextConnectionAttempt = 0; // timestamp
unsigned long failedConnectionAttempts = 0;

EthernetClient client;
EthernetServer server(SERVER_LISTEN_PORT);

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/shed.temperature");
Adafruit_MQTT_Publish humid = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/shed.humidity");
Adafruit_MQTT_Publish lastwill = Adafruit_MQTT_Publish(&mqtt, WILL_FEED);
Adafruit_MQTT_Subscribe shedled = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.shedled");
Adafruit_MQTT_Subscribe shedlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.shedlight");

/*************************** Sketch Code ************************************/

#define halt(s) { Serial.println(F( s )); while(1);  }

void(* __resetFunc) (void) = 0; //declare reset function @ address 0

void resetFunc(const __FlashStringHelper* msg, unsigned long delayMs) {
  Serial.println(msg);
  Serial.print(F("Resetting in "));
  Serial.print(delayMs / 1000);
  Serial.println(F("s"));
  delay(delayMs);
  __resetFunc();
}


void initSensor() {
  // Initialize device.
  dht.begin();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");

  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.print  ("Min Delay:    "); Serial.print(sensor.min_delay / 1000); Serial.println("ms");

  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  //sensorDelayMs = sensor.min_delay / 1000;
  sensorDelayMs = 30000;
}

void readSensor() {

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F(" *C"));
  }

  temp.publish(event.temperature);

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }

  humid.publish(event.relative_humidity);
}


void setup() {
  pinMode(DHT_VCC_PIN, OUTPUT);
  digitalWrite(DHT_VCC_PIN, HIGH);

  // Disable SD card
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, HIGH);

  pinMode(LEDLIGHT_PIN, OUTPUT);
  digitalWrite(LEDLIGHT_PIN, HIGH);
  
  delay(2000);

  Serial.begin(115200);
  Serial.println(VERSION_MESSAGE);
  Serial.println(F("Joining the network..."));
  Ethernet.begin(mac);
  delay(2000); //give the ethernet a second to initialize
  Serial.println(Ethernet.localIP());

  if (Ethernet.localIP() == IPAddress(0,0,0,0)) {
    resetFunc(F("DHCP resolution failed"), 30000);
  }

    Serial.println(F("MQTT subscribe"));

  //mqtt.subscribe(&lamptoggle);
  mqtt.subscribe(&shedled);
  mqtt.subscribe(&shedlight);
  mqtt.will(WILL_FEED, "0");

  server.begin();

  initSensor();
}


void loop() {
  now = millis();
  Ethernet.maintain();
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    Serial.print(F("Got: "));
    Serial.println((char *)subscription->lastread);
    if (subscription == &shedled) {
      if (strcmp((char *)subscription->lastread, "1") == 0) {
        digitalWrite(LEDLIGHT_PIN, LOW);
      }
      if (strcmp((char *)subscription->lastread, "0") == 0) {
        digitalWrite(LEDLIGHT_PIN, HIGH);
      }
    } else if (subscription == &shedlight) {
      if (strcmp((char *)subscription->lastread, "1") == 0) {
        digitalWrite(LIGHT_PIN, LOW);
      }
      if (strcmp((char *)subscription->lastread, "0") == 0) {
        digitalWrite(LIGHT_PIN, HIGH);
      }
    }
  }

  /*
  // Now we can publish stuff!
  Serial.print(F("\nSending photocell val "));
  Serial.print(x);
  Serial.print("...");
  if (! photocell.publish(x++)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }*/
  
  // Get temperature event and print its value.
  if (lastSensorRead + sensorDelayMs   < now) {
    Serial.println(F("sensorRead"));
    lastSensorRead = now;
    readSensor();
  }

  handleHttpClientRequest();

  MQTT_ping();
  delay(1000);
}

void MQTT_ping() {

  if (!mqtt.connected()) {
    return;
  }

  if (lastPing + MQTT_PING_INTERVAL_MS < now) {
    Serial.println(F("Ping"));
    lastPing = now;
    if (!mqtt.ping()) {
      Serial.println(F("Failed to ping"));
      mqtt.disconnect();
    } else {
      lastwill.publish(now);
    }
  }
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  if (nextConnectionAttempt < now) {
    Serial.print(F("Connecting to MQTT... "));

    int delaySecs = (2 << failedConnectionAttempts); // Delay for 2, 4, 8 .. seconds
    if (ret = mqtt.connect() != 0) {
      Serial.print(F("Failed: "));
      Serial.println(mqtt.connectErrorString(ret));
      //mqtt.disconnect();

      nextConnectionAttempt = now + delaySecs * 1000;
      ++failedConnectionAttempts;
    }
  
    if (0 == ret) {
      connectedSince = millis();
      failedConnectionAttempts = 0;
      Serial.println(F("Connected!"));
    } else if (failedConnectionAttempts > MQTT_CONNECT_RETRY_MAX) {
      connectedSince = 0;
      resetFunc(F("Max retries exhausted!"), 2000); // Reset and try again
    } else {
      Serial.print(F("Retrying in "));

      Serial.print(delaySecs);
      Serial.println(F("s"));
    }
  }
}

void handleHttpClientRequest() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println(F("New http client"));
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: text/html"));
          client.println(F("Connection: close"));  // the connection will be closed after completion of the response
          client.println(F("Refresh: 10"));  // refresh the page automatically every 5 sec
          client.println();
          client.println(F("<!DOCTYPE HTML>"));
          client.println(F("<html>"));
          // output the value of each analog input pin

          client.print(F("<h1>"));
          client.print(VERSION_MESSAGE);
          client.println(F("</h1>"));
          client.print(F("<br />Last ping "));
          client.print(lastPing);
          client.print(F("<br />Uptime "));
          client.print(now);
          client.print(F("<br />Connected since "));
          client.print(connectedSince);

          client.println(F("<br />"));

          client.println(F("</html>"));
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println(F("Http client disconnected"));
  }
}
