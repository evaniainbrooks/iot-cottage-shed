
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

#define VERSION_MESSAGE F("Shed Console v0.30 25/06/19")

#define AIO_SERVER      "raspberry.home"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "mosquitto"
#define AIO_KEY         "qq211"

#define GAS_SENSOR_PIN A0

#define OUTSIDE_DHT_PIN 4
#define DHT_PIN 7
#define LIGHT_PIN 5
#define LEDLIGHT_PIN 6
#define OUTSIDE_MOTION_SENSOR_PIN 20
#define MOTION_SENSOR_PIN 21
#define DOOR_PIN 8
#define BUTTON_1_PIN 10
#define BUTTON_2_PIN 11

#define LED_RED_PIN = 44
#define LED_GREEN_PIN = 55
#define LED_BLUE_PIN = 46

#define WILL_FEED AIO_USERNAME "/feeds/nodes.shed"
#define SERVER_LISTEN_PORT 80
#define MQTT_CONNECT_RETRY_MAX 5
#define MQTT_PING_INTERVAL_MS 60000
#define GAS_SENSOR_READ_INTERVAL_MS 5000

#define EDGE_NONE (int)-1
#define TOGGLE_COMMAND (uint32_t)2

byte mac[] = {0xBE, 0xBD, 0xEE, 0xEF, 0xFE, 0xFD};

#define DHT_TYPE DHT11     // DHT 22 (AM2302)
DHT_Unified dht(DHT_PIN, DHT_TYPE);
DHT_Unified outsidedht(OUTSIDE_DHT_PIN, DHT_TYPE);

uint32_t sensorDelayMs;
uint32_t lastSensorRead = 0;
uint32_t lastGasSensorRead = 0;
uint32_t lastPing = 0; // timestamp
uint32_t connectedSince = 0; // timestamp
uint32_t now = 0; // timestamp
uint32_t nextConnectionAttempt = 0; // timestamp
uint32_t failedConnectionAttempts = 0;

EthernetClient client;
EthernetServer server(SERVER_LISTEN_PORT);

int lastState[50] = {0};
int lastGasSensor;
float lastTemp;
float lastHumid;
float lastOutsideTemp;
float lastOutsideHumid;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish door = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/doors.shed");
Adafruit_MQTT_Publish gas = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/smoke.shed");

Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/shed.temperature");
Adafruit_MQTT_Publish humid = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/shed.humidity");

Adafruit_MQTT_Publish outsidetemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/outside.temperature");
Adafruit_MQTT_Publish outsidehumid = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/outside.humidity");

Adafruit_MQTT_Publish motion = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/shed.motion");
Adafruit_MQTT_Publish outsidemotion = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/outside.motion");

Adafruit_MQTT_Publish shedled_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.shedled");
Adafruit_MQTT_Publish shedlight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.shedlight");
Adafruit_MQTT_Subscribe shedled = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.shedled");
Adafruit_MQTT_Subscribe shedlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.shedlight");

Adafruit_MQTT_Publish lastwill = Adafruit_MQTT_Publish(&mqtt, WILL_FEED);

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

void initSensor(DHT_Unified& dht) {
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

  lastTemp = event.temperature;
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

  lastHumid = event.relative_humidity;
  humid.publish(event.relative_humidity);
}

void readOutsideSensor() {

  sensors_event_t event;
  outsidedht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading outside temperature!"));
  }
  else {
    Serial.print(F("Outside Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F(" *C"));
  }

  lastOutsideTemp = event.temperature;
  outsidetemp.publish(event.temperature);

  // Get humidity event and print its value.
  outsidedht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Outside Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }

  lastOutsideHumid = event.relative_humidity;
  outsidehumid.publish(event.relative_humidity);
}


void publishEdge(int pin, Adafruit_MQTT_Publish* pub, uint32_t fallingValue = LOW, uint32_t risingValue = HIGH) {
  int result = detectEdge(pin);
  if (result != EDGE_NONE) {
    pub->publish(result == RISING ? risingValue : fallingValue);
  }
}

int detectEdge(int pin) {
  int state = digitalRead(pin);
  int result = -1;
  
  if (state != lastState[pin]) {
    result = state == HIGH ? RISING : FALLING;
    Serial.print("State change on pin ");
    Serial.print(result);
    Serial.println(result == RISING ? " rising" : " falling");
  }

  lastState[pin] = state;
  return result;
}

void setup() {
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, HIGH);

  pinMode(LEDLIGHT_PIN, OUTPUT);
  digitalWrite(LEDLIGHT_PIN, HIGH);

  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(OUTSIDE_MOTION_SENSOR_PIN, INPUT);
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  delay(100);    
  
  lastState[DOOR_PIN] = digitalRead(DOOR_PIN);
  lastState[MOTION_SENSOR_PIN] = digitalRead(MOTION_SENSOR_PIN);
  lastState[OUTSIDE_MOTION_SENSOR_PIN] = digitalRead(OUTSIDE_MOTION_SENSOR_PIN);
  lastState[BUTTON_1_PIN] = digitalRead(BUTTON_1_PIN);
  lastState[BUTTON_2_PIN] = digitalRead(BUTTON_2_PIN);
  lastGasSensor = analogRead(GAS_SENSOR_PIN);
  
  delay(1000);

  Serial.begin(115200);
  Serial.println(VERSION_MESSAGE);
  Serial.println(F("Joining the network..."));
  Ethernet.begin(mac);
  delay(2000);

  Serial.println(Ethernet.localIP());
  if (Ethernet.localIP() == IPAddress(0,0,0,0)) {
    resetFunc(F("DHCP resolution failed"), 30000);
  }

  Serial.println(F("MQTT subscribe"));
  mqtt.subscribe(&shedled, &onSubscriptionEvent);
  mqtt.subscribe(&shedlight, &onSubscriptionEvent);
  mqtt.will(WILL_FEED, "0");

  server.begin();

  initSensor(dht);
  initSensor(outsidedht);
}


void readGasSensor(bool force = false) {
  if (force || now - lastGasSensorRead > GAS_SENSOR_READ_INTERVAL_MS) {
    uint32_t sensor = analogRead(GAS_SENSOR_PIN);
    bool sensorChanged = abs(lastGasSensor - sensor) > 50;
    bool sensorAboveThreshold = sensor > 300;

    lastGasSensorRead = now;
    lastGasSensor = sensor;

    if (force || sensorChanged || sensorAboveThreshold) {
      gas.publish(sensor);
    }
  }
}

void loop() {
  now = millis();
  Ethernet.maintain();
  mqttConnect();
  
  mqtt.process(100);
  
  // Get temperature event and print its value.
  if (now - lastSensorRead > sensorDelayMs) {
    Serial.println(F("sensorRead"));
    lastSensorRead = now;
    readSensor();
    readOutsideSensor();
  }

  readGasSensor();

  publishEdge(BUTTON_1_PIN, &shedlight_pub, TOGGLE_COMMAND, TOGGLE_COMMAND);
  publishEdge(BUTTON_2_PIN, &shedled_pub, TOGGLE_COMMAND, TOGGLE_COMMAND);  

  publishEdge(OUTSIDE_MOTION_SENSOR_PIN, &outsidemotion);
  publishEdge(MOTION_SENSOR_PIN, &motion);
  publishEdge(DOOR_PIN, &door);

  

  handleHttpClientRequest();

  mqttPing();
  delay(100);
}



void onPing(bool result) {
  readGasSensor(true);
  Serial.println("On async ping!");
  if (!lastwill.publish(now)) {
    Serial.println("Failed to publish last will!");
  }
}

void onSubscriptionEvent(Adafruit_MQTT_Subscribe* subscription) {
  
  Serial.print(F("Got: "));
  Serial.println((char *)subscription->lastread);
  if (subscription == &shedled) {
    if (strcmp((char *)subscription->lastread, "1") == 0) {
      digitalWrite(LEDLIGHT_PIN, LOW);
    }
    if (strcmp((char *)subscription->lastread, "0") == 0) {
      digitalWrite(LEDLIGHT_PIN, HIGH);
    }
    if (strcmp((char *)subscription->lastread, "2") == 0) {
      int val = digitalRead(LEDLIGHT_PIN);
      digitalWrite(LEDLIGHT_PIN,  val == HIGH ? LOW : HIGH);
      shedled_pub.publish(val == HIGH ? "1" : "0");
    }
  } else if (subscription == &shedlight) {
    if (strcmp((char *)subscription->lastread, "1") == 0) {
      digitalWrite(LIGHT_PIN, LOW);
    }
    if (strcmp((char *)subscription->lastread, "0") == 0) {
      digitalWrite(LIGHT_PIN, HIGH);
    }
    if (strcmp((char *)subscription->lastread, "2") == 0) {
      int val = digitalRead(LIGHT_PIN);
      digitalWrite(LIGHT_PIN, val == HIGH ? LOW : HIGH);
      shedlight_pub.publish(val == HIGH ? "1" : "0");
    }
  }
}

void mqttPing() {
  if (!mqtt.connected()) {
    return;
  }

  if (now - lastPing > MQTT_PING_INTERVAL_MS) {
    Serial.println(F("Ping"));
    lastPing = now;
    mqtt.pingAsync(onPing);
  }
}

void mqttConnect() {
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
          client.print(F("<br />"));
          client.print(F("<br />Door "));
          client.print(lastState[DOOR_PIN]);
          client.print(F("<br />Motion "));
          client.print(lastState[MOTION_SENSOR_PIN]);
          client.print(F("<br />LED Light "));
          client.print(lastState[LEDLIGHT_PIN]);
          client.print(F("<br />Light "));
          client.print(lastState[LIGHT_PIN]);
          client.print(F("<br />Temperature "));
          client.print(lastTemp);
          client.print(F("<br />Humidity "));
          client.print(lastHumid);
          client.print(F("<br />Outside Temperature "));
          client.print(lastOutsideTemp);
          client.print(F("<br />Outside Humidity "));
          client.print(lastOutsideHumid);

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
