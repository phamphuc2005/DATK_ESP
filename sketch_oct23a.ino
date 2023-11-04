
/**************** Import library ******************/
#include <Wire.h>
#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Arduino_JSON.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <MQUnifiedsensor.h>
#include <LiquidCrystal_I2C.h>
#include <HttpClient.h>

/**************** List constant ******************/
#define DHT_TYPE DHT11
#define LCD_ADDR 0x27 // address I2C
#define LCD_HEIGHT 2
#define LCD_WIDTH 16
#define STEP 100
#define DATA_CYCLE 3000 // send data after 5s

/**************** List GPIO ******************/
#define BUZZER_PIN 13
#define DHT_PIN 12
#define FIRE_PIN 14
#define GAS_PIN 2

/**************** Parameter use ******************/
// param wifi
const char *ssid = "Phuc";
const char *password = "01234567";

// param broker
String mqtt_server = "broker.hivemq.com";
const uint16_t mqtt_port = 1883;
String dataTopic = "mqtt-fire_alarm_system-data";
String commandTopic = "mqtt-fire_alarm_system-command";
String stateTopic = "mqtt-fire_alarm_system-state";

// state of system
int isOn = 1;

// id device
const String deviceId = "123456";
// const String deviceId = "652a843b5e036bfeb3ad54fc";
const String chipId = String(ESP.getChipId());

// param enviroment
float humi, temp;
int fire, gas;
int warn = 0;
const int fireThreshold = 3500, gasThreshold = 2375;
boolean hasFire, hasGas;

// time stamp
int lastUpdate = 0;

/**************** Initilize data structure ******************/
DHT dht(DHT_PIN, DHT_TYPE);                             // read value of sensor temperature and humidity DHT_PIN: GPIO, DHT_TYPE: type sensor
WiFiClient wifiClient;                                  // connect Wifi
PubSubClient mqttClient(wifiClient);                    // connect, send, recive data from MQTT broker
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_WIDTH, LCD_HEIGHT); // control LCD screen throw communicate with I2C

/**************** Define function ******************/

// turn on buzzer
void warning()
{
  lcd.setCursor(4, 1);
  lcd.print("        ");
  lcd.setCursor(4, 1);
  lcd.print("!DANGER!");
  digitalWrite(BUZZER_PIN, LOW);  // turn on buzzer
  delay(1000);                    //  delay 1s
  digitalWrite(BUZZER_PIN, HIGH); // turn off buzzer
}




// Display data on screen
void displayInfo(float temp, float humi)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.setCursor(8, 0);
  lcd.print("F:");
  lcd.setCursor(8, 1);
  lcd.print("G:");
  lcd.setCursor(4, 0);
  lcd.write(1);
  lcd.print("C");
  lcd.setCursor(4, 1);
  lcd.print("%");

  // Value of enviroment
  lcd.setCursor(2, 0);
  lcd.print((int)round(temp));
  lcd.setCursor(2, 1);
  lcd.print((int)round(humi));
  lcd.setCursor(10, 0);
  lcd.print((int)round(fire));
  lcd.setCursor(10, 1);
  lcd.print((int)round(gas));
}

// Dispay data from server send back
void displayStatus(float temp, float humi)
{
  if (fire == 0 && gas >500)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FIRE!");
    lcd.setCursor(7, 0);
    lcd.print("GAS LEAK!");
    warn = 1;
    warning();
  }
  else if (fire == 0)
  {
    lcd.clear();
    lcd.setCursor(5, 0);
    lcd.print("FIRE!!!");
    warn = 1;
    warning();
  }
  else if (gas > 500)
  {
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("GAS LEAK!!!");
    warn = 1;
    warning();
  }
  else if (temp > 35)
  {
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("HIGH TEMP!!!");
    warn = 1;
    warning();
  }
  else if (temp < 10)
  {
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("LOW TEMP!!!");
    warn = 1;
    warning();
  }
  else if (humi < 20 || humi == 100)
  {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("BAD HUMIDITY!!!");
    warn = 1;
    warning();
  }
  else {
    warn = 0;
    return;
  }
}

// Update status to server
void stateReport(int state)
{
  Serial.print("Update trạng thái: ");
  Serial.println(state);
  JSONVar json;
  json["deviceid"] = deviceId;
  json["state"] = state;
  mqttClient.publish(stateTopic.c_str(), JSON.stringify(json).c_str());
}

// Call when has data from Broker send to
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("New message from topic: ");
  Serial.println(topic);
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // handle data
  if ((String)topic == commandTopic)
  {
    Serial.println("ok");
    JSONVar command = JSON.parse((String)((const char *)payload));
    Serial.println(command);

    // check if signal control belong to present device
    Serial.println((String)((const char *)command["deviceid"]));
    Serial.println(deviceId);
    if ((String)((const char *)command["deviceid"]) == deviceId)
    {
      Serial.println("Right ID");

      // Signal recive is control
      if ((String)((const char *)command["type"]) == "control")
      {
        Serial.println("Control Signal");
        int signal;
        if ((String)((const char *)command["state"]) == "1")
        {
          signal = 1;
        }
        else
        {
          signal = 0;
        }
        // int signal = (int)((String)((const char *)command["state"]));
        Serial.print("signal: ");
        Serial.println(signal);
        Serial.print("is ON: ");
        Serial.println(isOn);

        // Signal control different signal of present
        if (signal != isOn)
        {
          Serial.println("Different!");
          isOn = signal;

          // update do state to server
          stateReport(isOn);
          Serial.println(isOn);

          // handle
          if (isOn)
          {
            Serial.println("Signal: Turn on");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("System is ON!");
          }
          else
          {

            Serial.println("Signal: Turn off");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("System is OFF!");
          }

          delay(1000);
        }
        else
          Serial.println("Same!");
      }

      // Signal recive is warning
      else
      {
        Serial.println("Warning Signal");

        // get param from command object
        int hasFire = command["hasFire"];
        int hasGas = command["hasGas"];
        int danger = command["danger"];

        // Display warning on screen and turn on buzzer
        // if (isOn)
        // {
          
        //   // warning();
        // }
      }
    }
  }
}

// function reconnect broker
void reconnectBroker()
{
  Serial.println("Connecting to Broker ... ");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting to");
  lcd.setCursor(0, 1);
  lcd.print("Broker ...");

  // not connect
  while (!mqttClient.connect("ESP8266_ID1", "ESP_OFFLINE", 0, 0, "ESP8266_ID1_OFFLINE"))
  {
    Serial.print("Error, rc = ");
    Serial.print(mqttClient.state());
    Serial.println("Try again in 5 seconds");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error!!!");
    lcd.setCursor(0, 1);
    lcd.print("Try again in 5s");

    delay(5000);
  }

  // connected success
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connected to");
  lcd.setCursor(0, 1);
  lcd.print("Broker");

  // Subscribe to topic command to get signal control
  mqttClient.subscribe(commandTopic.c_str());

  Serial.println("Connected to Broker!");
  delay(1000);
}

// function reconnect wifi
void reconnectWifi()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting to");
  lcd.setCursor(0, 1);
  lcd.print("Wifi ...");

  WiFi.begin(ssid,password);

  // not connect
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Connecting to WiFi ... ");
    // WiFi.disconnect();
    // WiFi.reconnect();

    delay(500);
  }

  // connected success
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connected to");
  lcd.setCursor(0, 1);
  lcd.print("Wifi!");
  lcd.setCursor(7, 1);
  lcd.print(ssid);

  Serial.print("Connected to Wifi: ");
  Serial.println(ssid);
  delay(1000);
}

/**************** Config system ******************/
void setup()
{
  Serial.begin(9600);
  // Wire.begin(5, 4);

  // set up mode for pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FIRE_PIN, INPUT);
  pinMode(GAS_PIN, INPUT);

  // start sensor DHT
  dht.begin();

  // Config LCD
  Serial.println("Starting LCD ... ");
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hello! Device:");
  lcd.setCursor(0, 1);
  lcd.print(deviceId);
  delay(5000);

  // Config Wifi
  WiFi.begin(ssid, password);

  // Config MQTT server and port
  mqttClient.setServer(mqtt_server.c_str(), mqtt_port);
  mqttClient.setCallback(callback);

    byte degreeChar[8] = {
    B00110,
    B01001,
    B01001,
    B00110,
    B00000,
    B00000,
    B00000,
    B00000
  };
  lcd.createChar(1, degreeChar);
}

/**************** Loop main ******************/
void loop()
{
  digitalWrite(BUZZER_PIN, HIGH);
  /**************** 1. Check connection ******************/
  // lost connect to Wifi
  if (WiFi.status() != WL_CONNECTED)
  {
    reconnectWifi();
    return;
  }

  // connect again MQTT broker
  if (!mqttClient.connected())
  {
    reconnectBroker();

    // connected success
    lcd.clear();
    lcd.setCursor(0, 0);
    if (isOn)
    {
      lcd.print("System is ON!");
    }
    else
    {
      lcd.print("System is OFF!");
    }
    delay(500);
  }

  // Read data from mqtt queue
  mqttClient.loop();

  /**************** 2. Send data to server ******************/
  // System is on
  if (isOn)
  {
    // enough 1 period
    if (millis() - lastUpdate > DATA_CYCLE)
    {
      // reassign the timeline
      lastUpdate = millis();

      // read data from temperaure and humidity sensor
      float humi = dht.readHumidity();
      float temp = dht.readTemperature();

      Serial.print(F("Độ ẩm: "));
      Serial.print(humi);
      Serial.print(F("%  Nhiệt độ: "));
      Serial.print(temp);
      Serial.print(F("°C "));

      // read data from fire sensor
      fire = digitalRead(FIRE_PIN);
      Serial.print("Fire: ");
      Serial.print(fire);

      // read data from gas sensor
      gas = analogRead(A0);
      Serial.print(" Gas: ");
      Serial.println(gas);

      displayStatus(temp, humi);

      // data encapsulation
      JSONVar json;
      json["chipId"] = chipId;
      json["deviceid"] = deviceId;
      json["temperature"] = temp;
      json["humidity"] = humi;
      json["fire"] = fire;
      json["gas"] = gas;
      json["warn"] = warn;

      // send data to broker
      mqttClient.publish(dataTopic.c_str(), JSON.stringify(json).c_str());
      
      // display data
      displayInfo(temp, humi);

    }
  }

  // Wait a few seconds between measurements.
  delay(STEP);
}